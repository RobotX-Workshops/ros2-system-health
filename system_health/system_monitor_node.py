#!/usr/bin/env python3
# system_health/system_health/system_monitor.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from .utils import (
    cpu_usage_per_core_percentage,
    get_core_count,
    memory_usage_percentage,
    disk_usage_percentage,
    temp,
    platform_model_str,
    ip_address,
)


class SystemMonitor(Node):
    def __init__(self):
        super().__init__("system_monitor")
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
        self.platform_pub = self.create_publisher(String, "system_health/platform", 10)
        self.ip_pub = self.create_publisher(
            String, "system_health/ip_address_string", 10
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

    def publish_static_info(self):
        """Publish static platform and network information."""
        platform_msg = String()
        platform_msg.data = platform_model_str()
        self.platform_pub.publish(platform_msg)

        ip_msg = String()
        interface = "eth0"  # You can make this configurable
        ip_msg.data = ip_address(interface)
        self.ip_pub.publish(ip_msg)

    def timer_callback(self):
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

        # Temperature (degrees Celsius)
        temp_msg = Float32()
        temp_msg.data = temp()
        self.temp_pub.publish(temp_msg)

        self.get_logger().info(
            f"Published: CPU: {cpu_msg.data:.1f}%, "
            f"RAM: {memory_msg.data:.1f}%, "
            f"Disk: {disk_msg.data:.1f}%, "
            f"Temp: {temp_msg.data:.1f}°C"
        )


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
