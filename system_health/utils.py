# system_health/system_health/utils.py
import logging
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
