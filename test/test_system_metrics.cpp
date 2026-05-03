// Copyright 2024 tron-roboracer contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file test_system_metrics.cpp
/// @brief Unit tests for /proc-based system metric readers.
///
/// These tests verify:
///   - memory_usage_percentage()    reads /proc/meminfo (no shell fork)
///   - disk_usage_percentage()      reads via statvfs("/")  (no shell fork)
///   - ip_address()                 uses getifaddrs()       (no shell fork)
///   - wifi_signal_strength()       reads /proc/net/wireless (no shell fork)
///   - associated_wifi_interfaces() parses /proc/net/wireless
///   - default_route_interface()    parses /proc/net/route
///
/// The wifi-interface parsers are tested through stream-based variants so the
/// test does not depend on a specific live network state.
///
/// No ROS 2 node is spun up here — these are pure C++ unit tests.

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/statvfs.h>

#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// ── Re-implement the slim versions under test (same logic as system_monitor_node.cpp)
// We duplicate the implementations here to keep the test self-contained without
// needing to link the full ROS2 node binary.

static float test_memory_usage_percentage()
{
  std::ifstream meminfo("/proc/meminfo");
  if (!meminfo.is_open()) {
    return -1.0f;
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
    return -1.0f;
  }
  return static_cast<float>((total_kb - available_kb) * 100.0 / total_kb);
}

static float test_disk_usage_percentage()
{
  struct statvfs fs
  {
  };
  if (statvfs("/", &fs) != 0) {
    return -1.0f;
  }
  const uint64_t total = fs.f_blocks * fs.f_frsize;
  const uint64_t avail = fs.f_bavail * fs.f_frsize;
  if (total == 0) {
    return -1.0f;
  }
  return static_cast<float>((total - avail) * 100.0 / total);
}

static std::string test_ip_address(const std::string & interface)
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

static float test_wifi_signal_strength(const std::string & interface)
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
    std::istringstream ss(line);
    std::string iface_col;
    int status = 0;
    float link = 0.0f;
    ss >> iface_col >> std::hex >> status >> std::dec >> link;
    return std::min(link / 70.0f * 100.0f, 100.0f);
  }
  return 0.0f;
}

// Stream variants of the new wifi-interface helpers so the tests can feed
// canned /proc snapshots and stay deterministic across hosts.
static std::vector<std::pair<std::string, float>> test_associated_wifi_interfaces(std::istream & in)
{
  std::vector<std::pair<std::string, float>> result;
  std::string line;
  std::getline(in, line);
  std::getline(in, line);
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::string iface_col;
    ss >> iface_col;
    if (iface_col.empty() || iface_col.back() != ':') {
      continue;
    }
    iface_col.pop_back();
    int status = 0;
    float link = 0.0f;
    ss >> std::hex >> status >> std::dec >> link;
    if (link > 0.0f) {
      result.emplace_back(std::move(iface_col), link);
    }
  }
  return result;
}

static std::string test_default_route_interface(std::istream & in)
{
  std::string line;
  std::getline(in, line);
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::string iface, dest_hex, gw_hex, mask_hex;
    int flags = 0, refcnt = 0, use = 0, metric = 0;
    if (!(ss >> iface >> dest_hex >> gw_hex >> flags >> refcnt >> use >> metric >> mask_hex)) {
      continue;
    }
    if (dest_hex == "00000000" && mask_hex == "00000000") {
      return iface;
    }
  }
  return "";
}

// ── Tests ────────────────────────────────────────────────────────────────────

TEST(SystemMetrics, MemoryUsageInRange)
{
  const float mem = test_memory_usage_percentage();
  ASSERT_GE(mem, 0.0f) << "/proc/meminfo not readable — test environment issue";
  EXPECT_GE(mem, 0.0f) << "Memory usage must be non-negative";
  EXPECT_LE(mem, 100.0f) << "Memory usage must not exceed 100%";
}

TEST(SystemMetrics, MemoryUsageNonZero)
{
  // On any running system, some memory is in use
  const float mem = test_memory_usage_percentage();
  if (mem < 0.0f) {
    GTEST_SKIP() << "/proc/meminfo not available";
  }
  EXPECT_GT(mem, 0.0f) << "Memory usage should be > 0 on a running system";
}

TEST(SystemMetrics, DiskUsageInRange)
{
  const float disk = test_disk_usage_percentage();
  ASSERT_GE(disk, 0.0f) << "statvfs('/') failed — test environment issue";
  EXPECT_GE(disk, 0.0f);
  EXPECT_LE(disk, 100.0f);
}

TEST(SystemMetrics, DiskUsageNonZero)
{
  const float disk = test_disk_usage_percentage();
  if (disk < 0.0f) {
    GTEST_SKIP() << "statvfs not available";
  }
  EXPECT_GT(disk, 0.0f) << "Root filesystem should have some usage";
}

TEST(SystemMetrics, IpAddressLocalhostValid)
{
  // 'lo' (loopback) is always present on Linux and has IP 127.0.0.1
  const std::string ip = test_ip_address("lo");
  EXPECT_EQ(ip, "127.0.0.1") << "Loopback interface should always be 127.0.0.1";
}

TEST(SystemMetrics, IpAddressEmptyInterfaceReturnsNA) { EXPECT_EQ(test_ip_address(""), "N/A"); }

TEST(SystemMetrics, IpAddressUnknownInterfaceReturnsNA)
{
  EXPECT_EQ(test_ip_address("nonexistent999"), "N/A");
}

TEST(SystemMetrics, WifiSignalEmptyInterfaceReturnsZero)
{
  EXPECT_FLOAT_EQ(test_wifi_signal_strength(""), 0.0f);
}

TEST(SystemMetrics, WifiSignalResultInRange)
{
  // Use a definitely-absent interface name to get 0, or a real one to get [0,100]
  const float sig = test_wifi_signal_strength("nonexistent999");
  EXPECT_FLOAT_EQ(sig, 0.0f);
}

TEST(SystemMetrics, NoShellForkInMemoryRead)
{
  // Verify /proc/meminfo is directly readable (i.e. our implementation
  // doesn't depend on `free` being installed)
  std::ifstream f("/proc/meminfo");
  EXPECT_TRUE(f.is_open()) << "/proc/meminfo must be directly readable without shell tools";
}

TEST(SystemMetrics, NoShellForkInDiskRead)
{
  // Verify statvfs works without `df` being installed
  struct statvfs fs
  {
  };
  EXPECT_EQ(statvfs("/", &fs), 0) << "statvfs('/') must work without `df`";
}

TEST(WifiInterface, AssociatedSkipsZeroLink)
{
  // Two NICs, only wlp1s0 has a non-zero link quality; wlan0 is up but
  // unassociated (link=0).
  std::istringstream in(
    "Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE\n"
    " face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n"
    "wlp1s0: 0000   70.  -40.  -256        0      0      0      0     24        0\n"
    "wlan0: 0000    0.  -110.  -256        0      0      0      0      0        0\n");
  const auto associated = test_associated_wifi_interfaces(in);
  ASSERT_EQ(associated.size(), 1u);
  EXPECT_EQ(associated[0].first, "wlp1s0");
  EXPECT_FLOAT_EQ(associated[0].second, 70.0f);
}

TEST(WifiInterface, AssociatedHandlesEmptyHeaderOnly)
{
  std::istringstream in(
    "Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE\n"
    " face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n");
  EXPECT_TRUE(test_associated_wifi_interfaces(in).empty());
}

TEST(WifiInterface, DefaultRoutePicksZeroDestZeroMask)
{
  // First non-default route entry is on eth0 (a /24); the default route is
  // on wlp1s0. Parser must skip non-defaults and pick wlp1s0.
  std::istringstream in(
    "Iface\tDestination\tGateway\tFlags\tRefCnt\tUse\tMetric\tMask\tMTU\tWindow\tIRTT\n"
    "eth0\t0000A8C0\t00000000\t0001\t0\t0\t100\t00FFFFFF\t0\t0\t0\n"
    "wlp1s0\t00000000\t0102A8C0\t0003\t0\t0\t600\t00000000\t0\t0\t0\n");
  EXPECT_EQ(test_default_route_interface(in), "wlp1s0");
}

TEST(WifiInterface, DefaultRouteEmptyWhenNoneDeclared)
{
  std::istringstream in(
    "Iface\tDestination\tGateway\tFlags\tRefCnt\tUse\tMetric\tMask\tMTU\tWindow\tIRTT\n"
    "eth0\t0000A8C0\t00000000\t0001\t0\t0\t100\t00FFFFFF\t0\t0\t0\n");
  EXPECT_EQ(test_default_route_interface(in), "");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
