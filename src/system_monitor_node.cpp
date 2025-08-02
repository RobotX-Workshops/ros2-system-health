#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <regex>
#include <sstream>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

// Utility function to execute a shell command and get its output
std::string exec_command(const char *cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, int (*)(FILE *)> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
    {
        // Using RCLCPP_ERROR for logging within a ROS context if possible,
        // but for a general utility, std::runtime_error is fine.
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    return result;
}

// === Utility Functions (translation of utils.py) ===

int get_core_count()
{
    return std::thread::hardware_concurrency();
}

std::vector<long> get_cpu_times()
{
    std::ifstream proc_stat("/proc/stat");
    std::string line;
    std::vector<long> times;
    if (!proc_stat.is_open())
        return times;

    while (std::getline(proc_stat, line))
    {
        if (line.substr(0, 3) == "cpu")
        {
            std::stringstream ss(line);
            std::string cpu_label;
            ss >> cpu_label; // "cpu", "cpu0", etc.
            long time;
            while (ss >> time)
            {
                times.push_back(time);
            }
        }
    }
    return times;
}

float memory_usage_percentage()
{
    try
    {
        std::string result = exec_command("free -m | awk 'NR==2{printf \"%.2f\", $3*100/$2 }'");
        return std::stof(result);
    }
    catch (...)
    {
        return 0.0f;
    }
}

float disk_usage_percentage()
{
    try
    {
        std::string result = exec_command("df -h | awk '$NF==\"/\"{printf \"%s\", $5}' | sed 's/%//'");
        return std::stof(result);
    }
    catch (...)
    {
        return 0.0f;
    }
}

float temp()
{
    std::ifstream temp_file("/sys/class/thermal/thermal_zone0/temp");
    if (temp_file.is_open())
    {
        double temp_val;
        temp_file >> temp_val;
        return static_cast<float>(temp_val / 1000.0);
    }
    return 0.0f;
}

float cpu_voltage()
{
    try
    {
        std::string result = exec_command("vcgencmd measure_volts core");
        // Output is "volt=1.2345V"
        size_t start = result.find('=') + 1;
        size_t end = result.find('V');
        return std::stof(result.substr(start, end - start));
    }
    catch (...)
    {
        return 0.0f;
    }
}

std::string platform_model_str()
{
    std::ifstream model_file("/proc/device-tree/model");
    if (model_file.is_open())
    {
        std::string model;
        std::getline(model_file, model);
        // Remove null characters
        model.erase(std::remove(model.begin(), model.end(), '\0'), model.end());
        return model;
    }
    return "N/A";
}

std::string ip_address(const std::string &interface)
{
    try
    {
        std::string cmd = "ip -4 addr show " + interface + " | grep -oP '(?<=inet\\s)\\d+(\\.\\d+){3}'";
        return exec_command(cmd.c_str());
    }
    catch (...)
    {
        return "N/A";
    }
}

float wifi_signal_strength()
{
    try
    {
        std::string result = exec_command("iwconfig wlan0");
        std::smatch match;
        std::regex quality_regex("Quality=(\\d+)/(\\d+)");
        if (std::regex_search(result, match, quality_regex) && match.size() == 3)
        {
            float current = std::stof(match[1].str());
            float total = std::stof(match[2].str());
            return (current / total) * 100.0f;
        }
    }
    catch (...)
    {
        // Fallthrough
    }
    return 0.0f;
}

float get_controller_battery(const std::string &mac_address)
{
    if (mac_address.empty())
        return -1.0f;
    try
    {
        std::string mac_path = mac_address;
        std::replace(mac_path.begin(), mac_path.end(), ':', '_');
        std::string cmd_find = "upower -e | grep 'gaming_input.*" + mac_path + "'";
        std::string device_path = exec_command(cmd_find.c_str());
        device_path.erase(std::remove(device_path.begin(), device_path.end(), '\n'), device_path.end());

        if (device_path.empty())
            return -1.0f;

        std::string cmd_info = "upower -i " + device_path;
        std::string info = exec_command(cmd_info.c_str());

        std::smatch match;
        std::regex percent_regex("percentage:\\s+(\\d+)%");
        if (std::regex_search(info, match, percent_regex) && match.size() == 2)
        {
            return std::stof(match[1].str());
        }
    }
    catch (...)
    {
        return -1.0f;
    }
    return -1.0f;
}

class SystemMonitorNode : public rclcpp::Node
{
public:
    SystemMonitorNode() : Node("system_monitor_node")
    {
        // Parameters
        this->declare_parameter<std::string>("controller_mac_address", "");
        this->get_parameter("controller_mac_address", controller_mac_);

        // Publishers
        cpu_pub_ = this->create_publisher<std_msgs::msg::Float32>("system_health/cpu_usage/percent", 10);
        memory_pub_ = this->create_publisher<std_msgs::msg::Float32>("system_health/memory_usage/percent", 10);
        disk_pub_ = this->create_publisher<std_msgs::msg::Float32>("system_health/disk_usage/percent", 10);
        temp_pub_ = this->create_publisher<std_msgs::msg::Float32>("system_health/temperature/celsius", 10);
        cpu_voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>("system_health/cpu/voltage", 10);
        platform_pub_ = this->create_publisher<std_msgs::msg::String>("system_health/platform", 10);
        ip_pub_ = this->create_publisher<std_msgs::msg::String>("system_health/ip_address_string", 10);
        wifi_signal_pub_ = this->create_publisher<std_msgs::msg::Float32>("system_health/wifi/signal/percent", 10);

        if (!controller_mac_.empty())
        {
            controller_battery_pub_ = this->create_publisher<std_msgs::msg::Float32>("system_health/controller/battery/percent", 10);
        }

        // Dynamic per-core publishers
        num_cores_ = get_core_count();
        for (int i = 0; i < num_cores_; ++i)
        {
            auto pub = this->create_publisher<std_msgs::msg::Float32>("system_health/cpu_core_" + std::to_string(i) + "_usage_percent", 10);
            core_pubs_.push_back(pub);
        }

        // Initialize CPU times for usage calculation
        prev_cpu_times_ = get_cpu_times();

        // Timer
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&SystemMonitorNode::timer_callback, this));

        // Publish static info at startup
        publish_static_info();
        RCLCPP_INFO(this->get_logger(), "C++ System Health Monitor started.");
    }

private:
    void publish_static_info()
    {
        auto platform_msg = std_msgs::msg::String();
        platform_msg.data = platform_model_str();
        platform_pub_->publish(platform_msg);

        auto ip_msg = std_msgs::msg::String();
        ip_msg.data = ip_address("eth0"); // Or make configurable
        ip_pub_->publish(ip_msg);
    }

    void timer_callback()
    {
        // CPU Usage Calculation
        std::vector<long> current_cpu_times = get_cpu_times();
        float total_cpu_usage = 0.0f;
        if (current_cpu_times.size() == prev_cpu_times_.size())
        {
            // Per-core CPU
            for (int i = 0; i < num_cores_; ++i)
            {
                // Each core has 10 values in /proc/stat
                int offset = (i + 1) * 10;
                long prev_idle = prev_cpu_times_[offset + 3] + prev_cpu_times_[offset + 4];
                long current_idle = current_cpu_times[offset + 3] + current_cpu_times[offset + 4];

                long prev_total = 0;
                long current_total = 0;
                for (int j = 0; j < 10; ++j)
                {
                    prev_total += prev_cpu_times_[offset + j];
                    current_total += current_cpu_times[offset + j];
                }

                long total_diff = current_total - prev_total;
                long idle_diff = current_idle - prev_idle;

                float cpu_percent = total_diff > 0 ? (1.0f - static_cast<float>(idle_diff) / total_diff) * 100.0f : 0.0f;

                auto core_msg = std_msgs::msg::Float32();
                core_msg.data = cpu_percent;
                core_pubs_[i]->publish(core_msg);
                total_cpu_usage += cpu_percent;
            }
        }
        prev_cpu_times_ = current_cpu_times;

        // Overall CPU
        auto cpu_msg = std_msgs::msg::Float32();
        cpu_msg.data = total_cpu_usage / num_cores_;
        cpu_pub_->publish(cpu_msg);

        // Other metrics
        auto mem_msg = std_msgs::msg::Float32();
        mem_msg.data = memory_usage_percentage();
        memory_pub_->publish(mem_msg);

        auto disk_msg = std_msgs::msg::Float32();
        disk_msg.data = disk_usage_percentage();
        disk_pub_->publish(disk_msg);

        auto temp_msg = std_msgs::msg::Float32();
        temp_msg.data = temp();
        temp_pub_->publish(temp_msg);

        auto voltage_msg = std_msgs::msg::Float32();
        voltage_msg.data = cpu_voltage();
        cpu_voltage_pub_->publish(voltage_msg);

        auto wifi_msg = std_msgs::msg::Float32();
        wifi_msg.data = wifi_signal_strength();
        wifi_signal_pub_->publish(wifi_msg);

        if (!controller_mac_.empty())
        {
            auto battery_msg = std_msgs::msg::Float32();
            battery_msg.data = get_controller_battery(controller_mac_);
            if (battery_msg.data >= 0)
            {
                controller_battery_pub_->publish(battery_msg);
            }
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cpu_pub_, memory_pub_, disk_pub_, temp_pub_, cpu_voltage_pub_, wifi_signal_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr platform_pub_, ip_pub_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> core_pubs_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr controller_battery_pub_;

    std::string controller_mac_;
    int num_cores_;
    std::vector<long> prev_cpu_times_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SystemMonitorNode>());
    rclcpp::shutdown();
    return 0;
}