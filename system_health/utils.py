# system_health/system_health/utils.py
import logging
import re
import subprocess
from typing import List

import psutil


def platform_model_str() -> str:
    """Get the platform model string from the device tree.

    Returns:
        str: The model name of the platform (e.g., "Raspberry Pi 4 Model B Rev 1.1").
             Returns empty string if file not found or on non-compatible systems.
    """
    try:
        with open("/proc/device-tree/model", "r") as f:
            return str(f.read().strip("\x00"))  # Remove null terminator
    except FileNotFoundError:
        logging.warning("Device tree model not available")
        return ""


def ip_address(interface: str) -> str:
    """Get the IP address for a specified network interface.

    Args:
        interface (str): Network interface name (e.g., "eth0", "wlan0")

    Returns:
        str: IPv4 address (e.g., "192.168.1.100") or empty string if not available
    """
    try:
        if network_interface_state(interface) == "down":
            return ""
        cmd = (
            f"ifconfig {interface} | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | "
            "grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'"
        )
        return subprocess.check_output(cmd, shell=True).decode("ascii").strip()
    except subprocess.CalledProcessError:
        logging.error(f"Failed to get IP address for {interface}")
        return ""


def network_interface_state(interface: str) -> str:
    """Check the operational state of a network interface.

    Args:
        interface (str): Network interface name (e.g., "eth0", "wlan0")

    Returns:
        str: "up" if interface is active, "down" if inactive or not found
    """
    try:
        with open(f"/sys/class/net/{interface}/operstate", "r") as f:
            return f.read().strip()
    except FileNotFoundError:
        return "down"


def get_core_count() -> int:
    """Get the number of CPU cores available on the system.

    Returns:
        int: Number of CPU cores (e.g., 4 for quad-core)
    """
    return psutil.cpu_count() or 0  # Return 0 if not available


def cpu_usage_per_core_percentage() -> List[float]:
    """Get the CPU usage for each core as a percentage.

    Returns:
        List[float]: List of CPU usage percentages (0.0 to 100.0) for each core
    """
    try:
        return psutil.cpu_percent(
            interval=1, percpu=True
        )  # Returns list of per-core usage
    except Exception as e:
        logging.error(f"Failed to get per-core CPU usage: {e}")
        return [0.0] * psutil.cpu_count()  # type: ignore # Return zeros for all cores on failure


def memory_usage_percentage() -> float:
    """Get the current RAM usage as a percentage.

    Returns:
        float: RAM usage percentage (0.0 to 100.0), where 100.0 represents full usage
    """
    try:
        output = subprocess.check_output(
            "free -m | awk 'NR==2{printf \"%.2f\", $3*100/$2 }'", shell=True
        ).decode("utf-8")
        return float(output)  # Already in percentage
    except subprocess.CalledProcessError:
        logging.error("Failed to get memory usage")
        return 0.0


def disk_usage_percentage() -> float:
    """Get the current disk usage percentage for the root filesystem.

    Returns:
        float: Disk usage percentage (0.0 to 100.0), where 100.0 represents full disk
    """
    try:
        output = (
            subprocess.check_output(
                'df -h | awk \'$NF=="/"{printf "%s", $5}\'', shell=True
            )
            .decode("utf-8")
            .strip("%")
        )
        return float(output)  # Already in percentage
    except subprocess.CalledProcessError:
        logging.error("Failed to get disk usage")
        return 0.0


def temp() -> float:
    """Get the CPU temperature in Celsius.

    Returns:
        float: Temperature in degrees Celsius (e.g., 45.2)
               Returns 0.0 if not available (non-compatible systems)
    """
    try:
        output = subprocess.check_output(
            "cat /sys/class/thermal/thermal_zone0/temp", shell=True
        ).decode("utf-8")
        return float(output) / 1000.0  # Convert from millidegrees to degrees C
    except (subprocess.CalledProcessError, FileNotFoundError):
        logging.warning("Temperature reading not available")
        return 0.0


def cpu_voltage() -> float:
    """Get the CPU voltage in volts."""
    try:
        output = subprocess.check_output(
            "vcgencmd measure_volts core", shell=True
        ).decode("utf-8")
        return float(
            output.split("=")[1].strip().split(" ")[0]
        )  # Extract voltage value
    except (subprocess.CalledProcessError, FileNotFoundError):
        logging.warning("CPU Voltage reading not available")
        return 0.0


# Mapping of bit positions to their meanings for the 'get_throttled' command.
# Each bit represents a specific hardware status.
_THROTTLED_MESSAGES = {
    # Current (Live) Status
    0: "Under-voltage detected",
    1: "ARM frequency capped",
    2: "Currently throttled",
    3: "Soft temperature limit active",
    # Historical (Occurred since boot) Status
    16: "Under-voltage has occurred",
    17: "ARM frequency capping has occurred",
    18: "Throttling has occurred",
    19: "Soft temperature limit has occurred",
}


class SystemHealthError:
    """Custom exception for system health errors."""

    code: int
    messages: List[str]

    def __init__(self, code: int, messages: List[str]):
        self.code = code
        self.messages = messages


def get_throttled_status(hex_code) -> SystemHealthError:
    """
    Parses the hexadecimal status code from 'vcgencmd get_throttled'
    and returns a list of human-readable messages.

    Args:
        hex_code (str): The hexadecimal string (e.g., '0x50000') from the command.

    Returns:
        dict: A dictionary containing the integer code and a list of status messages.
              Returns a clean status message if no issues are detected.
    """
    # Convert the hexadecimal string to an integer
    try:
        code = int(hex_code, 16)
    except (ValueError, TypeError):
        return SystemHealthError(
            code=-1, messages=[f"Error: Invalid hexadecimal code '{hex_code}'"]
        )

    # Check for a healthy status
    if code == 0:
        return SystemHealthError(
            code=0, messages=["✅ System status is OK. No issues detected."]
        )

    # Decode the bitmask
    active_messages = []
    for bit, message in _THROTTLED_MESSAGES.items():
        # Use a bitwise AND to check if the specific bit is set in the code
        if code & (1 << bit):
            # Add a warning emoji for easier identification
            active_messages.append(f"⚠️  {message}")

    return SystemHealthError(code=code, messages=active_messages)


def get_pi_status_directly() -> SystemHealthError:
    """
    Runs the 'vcgencmd get_throttled' command on a Raspberry Pi
    and returns the parsed status.

    Returns:
        dict: The parsed status dictionary, or an error message if the
              command fails (e.g., not running on a Pi).
    """
    try:
        # Execute the command and capture the output
        process = subprocess.run(
            ["vcgencmd", "get_throttled"], capture_output=True, text=True, check=True
        )
        # The output is in the format "throttled=0x..."
        hex_code = process.stdout.strip().split("=")[1]
        return get_throttled_status(hex_code)
    except FileNotFoundError:
        return SystemHealthError(
            code=-1,
            messages=[
                "Error: 'vcgencmd' command not found. Are you running this on a Raspberry Pi?"
            ],
        )
    except Exception as e:
        return SystemHealthError(code=-1, messages=[f"An error occurred: {e}"])


def get_controller_battery(mac_address) -> float:
    """
    Uses UPower to find the battery percentage of a connected gaming device.

    Args:
        mac_address (str): The MAC address of the Bluetooth device.

    Returns:
        float: The battery percentage (0.0 to 100.0) or -1.0 if not found.
    """
    try:
        # Find the UPower device path for the given MAC address.
        # UPower replaces ':' with '_' in the device path.
        mac_path_format = mac_address.replace(":", "_")

        # Command to find the full device path
        find_device_cmd = f"upower -e | grep 'gaming_input.*{mac_path_format}'"

        # Run the command in a shell to allow for the pipe
        device_path_process = subprocess.run(
            find_device_cmd, shell=True, capture_output=True, text=True
        )

        # Check if a device path was found
        if device_path_process.returncode != 0 or not device_path_process.stdout:
            return -1.0

        device_path = device_path_process.stdout.strip()

        # Get the detailed info for the found device
        info_process = subprocess.run(
            ["upower", "-i", device_path], capture_output=True, text=True, check=True
        )
        output = info_process.stdout

        # Parse for the percentage line, e.g., "percentage:         79%"
        match = re.search(r"percentage:\s+(\d+)%", output)

        if match:
            percentage = match.group(1)
            return float(percentage)
        else:
            return -1.0

    except FileNotFoundError:
        return -1.0
    except Exception:
        return -1.0


def wifi_signal_strength() -> float:
    "Gets the wi-fi signal strength in percentage."
    try:
        # Use iwconfig to get the signal strength of the Wi-Fi interface
        output = subprocess.check_output(
            "iwconfig wlan0 | grep -i --color signal", shell=True
        ).decode("utf-8")
        # Extract the signal strength value
        signal_strength = int(output.split("=")[1].split()[0])
        # Convert to percentage (assuming -100 dBm is 0% and -30 dBm is 100%)
        return max(0.0, min(100.0, (signal_strength + 100) * 100 / 70))
    except subprocess.CalledProcessError:
        logging.error("Failed to get Wi-Fi signal strength")
        return 0.0
