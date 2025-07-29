#!/usr/bin/env python3
# system_health/system_health/system_monitor.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from system_health.utils import cpu_voltage
from .utils import (
    cpu_usage_per_core_percentage,
    get_controller_battery,
    get_core_count,
    # get_pi_status_directly,
    memory_usage_percentage,
    disk_usage_percentage,
    temp,
    platform_model_str,
    ip_address,
    wifi_signal_strength,
)


class SystemMonitor(Node):
    def __init__(self):
        super().__init__("system_monitor")
        self.declare_parameter("controller_mac_address", "")
        self.controller_mac = (
            self.get_parameter("controller_mac_address")
            .get_parameter_value()
            .string_value
        )
        if not self.controller_mac:
            self.get_logger().warn(
                "No controller MAC address provided. Battery monitoring will be disabled."
            )

        # Publishers with units in topic names (percentages for usage)
        self.cpu_pub = self.create_publisher(
            Float32, "system_health/cpu_usage/percent", 10
        )
        self.memory_pub = self.create_publisher(
            Float32, "system_health/memory_usage/percent", 10
        )
        self.disk_pub = self.create_publisher(
            Float32, "system_health/disk_usage/percent", 10
        )
        self.temp_pub = self.create_publisher(
            Float32, "system_health/temperature/celsius", 10
        )

        self.cpu_voltage_pub = self.create_publisher(
            Float32, "system_health/cpu/voltage", 10
        )

        # self.throttling_pub = self.create_publisher(
        #     String, "system_health/throttling/status", 10
        # )

        self.platform_pub = self.create_publisher(String, "system_health/platform", 10)

        self.ip_pub = self.create_publisher(
            String, "system_health/ip_address_string", 10
        )

        if self.controller_mac != "":
            self.controller_battery_pub = self.create_publisher(
                Float32, "system_health/controller/battery/percent", 10
            )

        self.wifi_signal_pub = self.create_publisher(
            Float32, "system_health/wifi/signal/percent", 10
        )

        # Publishers for each CPU core (dynamic based on number of cores)
        self.num_cores = get_core_count()
        self.core_pubs = [
            self.create_publisher(
                Float32, f"system_health/cpu_core_{i}_usage_percent", 10
            )
            for i in range(self.num_cores)
        ]
        self.core_msgs = [
            Float32() for _ in range(self.num_cores)
        ]  # Pre-allocated messages

        # Timer for periodic updates (1 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Publish static info once at startup
        self.publish_static_info()
        self.get_logger().info("System Health Monitor started")

    def publish_static_info(self) -> None:
        """Publish static platform and network information."""
        platform_msg = String()
        platform_msg.data = platform_model_str()
        self.platform_pub.publish(platform_msg)

        ip_msg = String()
        interface = "eth0"  # You can make this configurable
        ip_msg.data = ip_address(interface)
        self.ip_pub.publish(ip_msg)

    def timer_callback(self) -> None:
        """Publish system metrics periodically on individual topics with units."""
        # Per-core CPU Usage (percent, 0.0-100.0)
        core_usages = cpu_usage_per_core_percentage()
        # Publish per-core CPU usage
        for i, usage in enumerate(core_usages):
            self.core_msgs[i].data = usage
            self.core_pubs[i].publish(self.core_msgs[i])

        # CPU Usage (percent, 0.0-100.0)
        cpu_msg = Float32()
        cpu_msg.data = sum(core_usages) / self.num_cores  # Average for overall CPU
        self.cpu_pub.publish(cpu_msg)

        # Memory Usage (percent, 0.0-100.0)
        memory_msg = Float32()
        memory_msg.data = memory_usage_percentage()
        self.memory_pub.publish(memory_msg)

        # Disk Usage (percent, 0.0-100.0)
        disk_msg = Float32()
        disk_msg.data = disk_usage_percentage()
        self.disk_pub.publish(disk_msg)

        # CPU Voltage (volts)
        cpu_voltage_msg = Float32()
        cpu_voltage_msg.data = cpu_voltage()
        self.cpu_voltage_pub.publish(cpu_voltage_msg)

        # Temperature (degrees Celsius)
        temp_msg = Float32()
        temp_msg.data = temp()
        self.temp_pub.publish(temp_msg)

        if self.controller_mac != "":
            # Controller Battery (percent, 0.0-100.0)
            controller_battery_msg = Float32()
            controller_battery_msg.data = get_controller_battery(self.controller_mac)
            self.controller_battery_pub.publish(controller_battery_msg)

        # WiFi Signal (percent, 0.0-100.0)
        wifi_signal_msg = Float32()
        wifi_signal_msg.data = wifi_signal_strength()
        self.wifi_signal_pub.publish(wifi_signal_msg)

        # TODO: publish throttling info. Needs custom message
        # throttling_msg = get_pi_status_directly()
        # self.throttling_pub.publish(throttling_msg)


def main(args=None):
    rclpy.init(args=args)
    system_monitor = SystemMonitor()
    try:
        rclpy.spin(system_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        system_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
