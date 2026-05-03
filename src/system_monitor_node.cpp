#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/statvfs.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

// exec_command: kept only for optional functions (vcgencmd, upower, hostnamectl)
// that have no /proc equivalent. Memory, disk, and WiFi no longer use it.
std::string exec_command(const char * cmd)
{
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, int (*)(FILE *)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

// === Utility Functions ===

int get_core_count() { return std::thread::hardware_concurrency(); }

std::vector<long> get_cpu_times()
{
  std::ifstream proc_stat("/proc/stat");
  std::string line;
  std::vector<long> times;
  if (!proc_stat.is_open()) {
    return times;
  }

  while (std::getline(proc_stat, line)) {
    if (line.substr(0, 3) == "cpu") {
      std::stringstream ss(line);
      std::string cpu_label;
      ss >> cpu_label;
      long time;
      while (ss >> time) {
        times.push_back(time);
      }
    }
  }
  return times;
}

// Read memory usage directly from /proc/meminfo — no fork/exec.
float memory_usage_percentage()
{
  std::ifstream meminfo("/proc/meminfo");
  if (!meminfo.is_open()) {
    return 0.0f;
  }

  long total_kb = 0, available_kb = 0;
  std::string line;
  while (std::getline(meminfo, line)) {
    if (line.rfind("MemTotal:", 0) == 0) {
      std::sscanf(line.c_str(), "MemTotal: %ld kB", &total_kb);
    } else if (line.rfind("MemAvailable:", 0) == 0) {
      std::sscanf(line.c_str(), "MemAvailable: %ld kB", &available_kb);
    }
    if (total_kb > 0 && available_kb > 0) {
      break;
    }
  }
  if (total_kb <= 0) {
    return 0.0f;
  }
  return static_cast<float>((total_kb - available_kb) * 100.0 / total_kb);
}

// Read disk usage via statvfs — no fork/exec.
float disk_usage_percentage()
{
  struct statvfs fs
  {
  };
  if (statvfs("/", &fs) != 0) {
    return 0.0f;
  }
  const uint64_t total = fs.f_blocks * fs.f_frsize;
  const uint64_t avail = fs.f_bavail * fs.f_frsize;
  if (total == 0) {
    return 0.0f;
  }
  return static_cast<float>((total - avail) * 100.0 / total);
}

float temp()
{
  std::ifstream temp_file("/sys/class/thermal/thermal_zone0/temp");
  if (temp_file.is_open()) {
    double temp_val;
    temp_file >> temp_val;
    return static_cast<float>(temp_val / 1000.0);
  }
  return 0.0f;
}

float cpu_voltage()
{
  if (!exec_command("command -v vcgencmd").empty()) {
    try {
      std::string result = exec_command("vcgencmd measure_volts core");
      size_t start = result.find('=') + 1;
      size_t end = result.find('V');
      return std::stof(result.substr(start, end - start));
    } catch (...) {
      return -1.0f;
    }
  }
  return -1.0f;
}

std::string platform_model_str()
{
  std::ifstream model_file("/proc/device-tree/model");
  if (model_file.is_open()) {
    std::string model;
    std::getline(model_file, model);
    model.erase(std::remove(model.begin(), model.end(), '\0'), model.end());
    return model;
  }
  try {
    return exec_command("hostnamectl | grep 'Hardware Model' | sed 's/.*: //'");
  } catch (...) {
    return "N/A";
  }
}

// Read IP address via getifaddrs() — no fork/exec.
std::string ip_address(const std::string & interface)
{
  if (interface.empty()) {
    return "N/A";
  }

  struct ifaddrs * ifaddr = nullptr;
  if (getifaddrs(&ifaddr) != 0) {
    return "N/A";
  }

  std::string result = "N/A";
  for (struct ifaddrs * ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) {
      continue;
    }
    if (interface != ifa->ifa_name) {
      continue;
    }
    char buf[INET_ADDRSTRLEN];
    auto * addr_in = reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
    if (inet_ntop(AF_INET, &addr_in->sin_addr, buf, sizeof(buf))) {
      result = buf;
    }
    break;
  }
  freeifaddrs(ifaddr);
  return result;
}

// Read WiFi link quality from /proc/net/wireless — no fork/exec.
// Falls back to 0 if the interface is not present (e.g. Ethernet-only host).
float wifi_signal_strength(const std::string & interface)
{
  if (interface.empty()) {
    return 0.0f;
  }

  std::ifstream proc_wireless("/proc/net/wireless");
  if (!proc_wireless.is_open()) {
    return 0.0f;
  }

  std::string line;
  while (std::getline(proc_wireless, line)) {
    if (line.find(interface) == std::string::npos) {
      continue;
    }
    // Format: <iface>: <status> <link>. <level>. <noise>. ...
    // "link" is quality 0–70 on most drivers; normalise to 0–100 %.
    std::istringstream ss(line);
    std::string iface_col;
    int status = 0;
    float link = 0.0f;
    ss >> iface_col >> std::hex >> status >> std::dec >> link;
    return std::min(link / 70.0f * 100.0f, 100.0f);
  }
  return 0.0f;
}

float get_controller_battery(const std::string & mac_address)
{
  if (mac_address.empty()) {
    return -1.0f;
  }
  try {
    std::string mac_path = mac_address;
    std::replace(mac_path.begin(), mac_path.end(), ':', '_');
    std::string cmd_find = "upower -e | grep 'gaming_input.*" + mac_path + "'";
    std::string device_path = exec_command(cmd_find.c_str());
    device_path.erase(std::remove(device_path.begin(), device_path.end(), '\n'), device_path.end());

    if (device_path.empty()) {
      return -1.0f;
    }

    std::string cmd_info = "upower -i " + device_path;
    std::string info = exec_command(cmd_info.c_str());

    std::smatch match;
    std::regex percent_regex("percentage:\\s+(\\d+)%");
    if (std::regex_search(info, match, percent_regex) && match.size() == 2) {
      return std::stof(match[1].str());
    }
  } catch (...) {
    return -1.0f;
  }
  return -1.0f;
}
struct HealthCheck
{
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher;
  double min_threshold;
  double max_threshold;
  bool is_healthy = true;

  HealthCheck(rclcpp::Node * parent_node, const std::string & metric_name)
  {
    // Use very large/small numbers as defaults to effectively disable a check if not set
    double default_min = std::numeric_limits<double>::lowest();
    double default_max = std::numeric_limits<double>::max();

    // Declare parameters with a "health." prefix for organization
    min_threshold = parent_node->declare_parameter("health." + metric_name + ".min", default_min);
    max_threshold = parent_node->declare_parameter("health." + metric_name + ".max", default_max);

    // Create a publisher for this specific health status
    publisher = parent_node->create_publisher<std_msgs::msg::Bool>(
      "system_health/" + metric_name + "/health", 10);
  }

  void update(double value)
  {
    is_healthy = (value >= min_threshold && value <= max_threshold);
    auto msg = std_msgs::msg::Bool();
    msg.data = is_healthy;
    publisher->publish(msg);
  }
};

class SystemMonitorNode : public rclcpp::Node
{
public:
  SystemMonitorNode() : Node("system_monitor_node")
  {
    this->declare_parameter<std::string>("controller_mac_address", "");
    this->declare_parameter<std::string>("wifi_interface", "wlan0");
    this->declare_parameter<std::string>("ip_interface", "eth0");
    this->declare_parameter<double>("update_frequency", 1.0);

    this->get_parameter("controller_mac_address", controller_mac_);
    this->get_parameter("wifi_interface", wifi_interface_);
    this->get_parameter("ip_interface", ip_interface_);
    this->get_parameter("update_frequency", update_frequency_);

    RCLCPP_INFO(this->get_logger(), "Monitoring WiFi Interface: '%s'", wifi_interface_.c_str());
    RCLCPP_INFO(this->get_logger(), "Monitoring IP Interface: '%s'", ip_interface_.c_str());
    RCLCPP_INFO(this->get_logger(), "Timer frequency set to: %.2f Hz", update_frequency_);

    // Value Publishers
    cpu_usage_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("system_health/cpu/usage/percent", 10);
    cpu_temp_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("system_health/cpu/temperature/celsius", 10);
    cpu_voltage_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("system_health/cpu/voltage", 10);
    memory_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("system_health/memory/usage/percent", 10);
    disk_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("system_health/disk/usage/percent", 10);
    platform_pub_ = this->create_publisher<std_msgs::msg::String>("system_health/platform", 10);
    ip_pub_ = this->create_publisher<std_msgs::msg::String>("system_health/ip_address_string", 10);
    wifi_signal_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("system_health/wifi/signal/percent", 10);

    if (!controller_mac_.empty()) {
      controller_battery_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "system_health/controller/battery/percent", 10);
    }

    // Health Publishers
    overall_health_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("system_health/overall/is_healthy", 10);

    // Initialize HealthCheck objects for each metric
    health_checks_.emplace("cpu", HealthCheck(this, "cpu"));
    health_checks_.emplace("memory", HealthCheck(this, "memory"));
    health_checks_.emplace("disk", HealthCheck(this, "disk"));
    health_checks_.emplace("temperature", HealthCheck(this, "temperature"));
    health_checks_.emplace("wifi_signal", HealthCheck(this, "wifi_signal"));

    // Only create controller battery health check if controller MAC is provided
    if (!controller_mac_.empty()) {
      health_checks_.emplace("controller_battery", HealthCheck(this, "controller_battery"));
    }

    num_cores_ = get_core_count();
    for (int i = 0; i < num_cores_; ++i) {
      auto pub = this->create_publisher<std_msgs::msg::Float32>(
        "system_health/cpu_core_" + std::to_string(i) + "_usage_percent", 10);
      core_pubs_.push_back(pub);
    }

    if (update_frequency_ <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Update frequency must be positive. Defaulting to 1.0 Hz.");
      update_frequency_ = 1.0;
    }
    auto timer_period = std::chrono::duration<double>(1.0 / update_frequency_);
    timer_ =
      this->create_wall_timer(timer_period, std::bind(&SystemMonitorNode::timer_callback, this));

    prev_cpu_times_ = get_cpu_times();
    publish_static_info();
    RCLCPP_INFO(this->get_logger(), "System Health Monitor started with health checks.");
  }

private:
  void publish_static_info()
  {
    auto platform_msg = std_msgs::msg::String();
    platform_msg.data = platform_model_str();
    platform_pub_->publish(platform_msg);

    auto ip_msg = std_msgs::msg::String();
    ip_msg.data = ip_address(ip_interface_);
    ip_pub_->publish(ip_msg);
  }

  void timer_callback()
  {
    // --- Get and Publish Metric Values ---
    std::vector<long> current_cpu_times = get_cpu_times();
    float total_cpu_usage = 0.0f;
    if (current_cpu_times.size() >= prev_cpu_times_.size() && !prev_cpu_times_.empty()) {
      for (int i = 0; i < num_cores_; ++i) {
        int offset = (i + 1) * 10;
        if (static_cast<size_t>(offset + 4) >= current_cpu_times.size()) {
          continue;
        }
        long prev_idle = prev_cpu_times_[offset + 3] + prev_cpu_times_[offset + 4];
        long current_idle = current_cpu_times[offset + 3] + current_cpu_times[offset + 4];
        long prev_total = 0;
        long current_total = 0;
        for (int j = 0; j < 10; ++j) {
          prev_total += prev_cpu_times_[offset + j];
          current_total += current_cpu_times[offset + j];
        }
        long total_diff = current_total - prev_total;
        long idle_diff = current_idle - prev_idle;
        float cpu_percent =
          total_diff > 0 ? (1.0f - static_cast<float>(idle_diff) / total_diff) * 100.0f : 0.0f;
        auto core_msg = std_msgs::msg::Float32();
        core_msg.data = cpu_percent;
        core_pubs_[i]->publish(core_msg);
        total_cpu_usage += cpu_percent;
      }
    }
    prev_cpu_times_ = current_cpu_times;

    float avg_cpu_usage = num_cores_ > 0 ? total_cpu_usage / num_cores_ : 0.0f;
    auto cpu_msg = std_msgs::msg::Float32();
    cpu_msg.data = avg_cpu_usage;
    cpu_usage_pub_->publish(cpu_msg);

    float mem_usage = memory_usage_percentage();
    auto mem_msg = std_msgs::msg::Float32();
    mem_msg.data = mem_usage;
    memory_pub_->publish(mem_msg);

    float disk_usage = disk_usage_percentage();
    auto disk_msg = std_msgs::msg::Float32();
    disk_msg.data = disk_usage;
    disk_pub_->publish(disk_msg);

    float temp_val = temp();
    auto temp_msg = std_msgs::msg::Float32();
    temp_msg.data = temp_val;
    cpu_temp_pub_->publish(temp_msg);

    float voltage = cpu_voltage();
    if (voltage >= 0.0f) {
      auto voltage_msg = std_msgs::msg::Float32();
      voltage_msg.data = voltage;
      cpu_voltage_pub_->publish(voltage_msg);
    }

    float wifi_val = wifi_signal_strength(wifi_interface_);
    auto wifi_msg = std_msgs::msg::Float32();
    wifi_msg.data = wifi_val;
    wifi_signal_pub_->publish(wifi_msg);

    float controller_battery_val = -1.0f;
    if (!controller_mac_.empty()) {
      auto battery_msg = std_msgs::msg::Float32();
      controller_battery_val = get_controller_battery(controller_mac_);
      battery_msg.data = controller_battery_val;
      if (battery_msg.data >= 0) {
        controller_battery_pub_->publish(battery_msg);
      }
    }

    // --- Update and Publish Health Statuses ---
    health_checks_.at("cpu").update(avg_cpu_usage);
    health_checks_.at("memory").update(mem_usage);
    health_checks_.at("disk").update(disk_usage);
    health_checks_.at("temperature").update(temp_val);
    health_checks_.at("wifi_signal").update(wifi_val);

    // Skip the health update when the reading is unknown (-1 sentinel from
    // get_controller_battery) — e.g. xpad-bound pads expose no UPower entry.
    // Leaving is_healthy untouched keeps overall health green rather than
    // failing the threshold compare on the sentinel.
    if (!controller_mac_.empty() && controller_battery_val >= 0) {
      health_checks_.at("controller_battery").update(controller_battery_val);
    }

    // Calculate and publish overall health
    bool overall_status = true;
    for (auto const & [name, check] : health_checks_) {
      if (!check.is_healthy) {
        overall_status = false;
        break;  // One failure makes the whole system unhealthy
      }
    }
    auto health_msg = std_msgs::msg::Bool();
    health_msg.data = overall_status;
    overall_health_pub_->publish(health_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cpu_usage_pub_, memory_pub_, disk_pub_,
    cpu_temp_pub_, cpu_voltage_pub_, wifi_signal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr platform_pub_, ip_pub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> core_pubs_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr controller_battery_pub_;

  // Health check members
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr overall_health_pub_;
  std::map<std::string, HealthCheck> health_checks_;

  std::string controller_mac_, wifi_interface_, ip_interface_;
  double update_frequency_;
  int num_cores_;
  std::vector<long> prev_cpu_times_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
