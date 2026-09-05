/* SPDX-License-Identifier: Apache-2.0
 * Several independent raw ROS topics -> RTSP streams -> raw ROS topics.
 * Measures scaling and checks pixel identity and isolation when input 0 pauses.
 * No hardware-dependent timing thresholds: the output is a benchmark report.
 */
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <spawn.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

extern char **environ;
using namespace std::chrono_literals;

namespace {
constexpr std::array<std::array<unsigned char, 3>, 4> COLORS{
    {{32, 32, 224}, {32, 224, 32}, {224, 32, 32}, {224, 224, 32}}};

double processCpu(pid_t pid) {
  std::ifstream input("/proc/" + std::to_string(pid) + "/stat");
  std::string line;
  std::getline(input, line);
  if (line.empty())
    return 0;
  std::istringstream fields(line.substr(line.rfind(')') + 2));
  std::string field;
  unsigned long long user = 0, system = 0;
  for (int i = 0; i <= 12; ++i) {
    fields >> field;
    if (i == 11)
      user = std::stoull(field);
    if (i == 12)
      system = std::stoull(field);
  }
  return static_cast<double>(user + system) / sysconf(_SC_CLK_TCK);
}

struct Relays {
  std::vector<pid_t> pids;
  ~Relays() {
    for (pid_t pid : pids)
      kill(pid, SIGINT);
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    for (pid_t pid : pids) {
      while (waitpid(pid, nullptr, WNOHANG) == 0) {
        if (std::chrono::steady_clock::now() >= deadline) {
          kill(pid, SIGKILL);
          waitpid(pid, nullptr, 0);
          break;
        }
        std::this_thread::sleep_for(10ms);
      }
    }
  }

  void start(unsigned i, bool hevc, bool hardware, const std::string &prefix) {
    const std::string input = prefix + "/input" + std::to_string(i);
    const std::string output = prefix + "/output" + std::to_string(i) + "/rtsp";
    std::vector<std::string> args{
        RTSP_REPUBLISH_EXECUTABLE,
        "--ros-args",
        "-r",
        "__node:=rtsp_bench_relay_" + std::to_string(i),
        "-p",
        "in_transport:=raw",
        "-p",
        "out_transport:=rtsp",
        "-p",
        std::string("out.rtsp.codec:=") + (hevc ? "H265" : "H264"),
        "-p",
        "out.rtsp.target_bitrate:=8000000",
        "-p",
        "out.rtsp.expected_framerate:=30",
        "-p",
        "out.rtsp.udp_port:=0",
        "-p",
        "out.rtsp.use_hw_encoder:=" + std::string(hardware ? "true" : "false"),
        "-p",
        "qos_overrides." + input + ".subscription.reliability:=best_effort",
        "-p",
        "qos_overrides." + input + ".subscription.history:=keep_last",
        "-p",
        "qos_overrides." + input + ".subscription.depth:=1",
        "-r",
        "in:=" + input,
        "-r",
        "out/rtsp:=" + output};
    spawn(args);
  }

  void startReceiver(unsigned i, bool hardware, const std::string &prefix) {
    const std::string input = prefix + "/output" + std::to_string(i) + "/rtsp";
    const std::string output = prefix + "/decoded" + std::to_string(i);
    spawn(
        {RTSP_REPUBLISH_EXECUTABLE,
         "--ros-args",
         "-r",
         "__node:=rtsp_bench_receiver_" + std::to_string(i),
         "-p",
         "in_transport:=rtsp",
         "-p",
         "out_transport:=raw",
         "-p",
         "in.rtsp.use_hw_decoder:=" + std::string(hardware ? "true" : "false"),
         "-p",
         "in.rtsp.hw_device:=" + std::string(hardware ? "cuda" : "none"),
         "-p",
         "in.rtsp.timestamp_source:=0",
         "-p",
         "in.rtsp.timeout:=0.0",
         "-p",
         "qos_overrides." + output + ".publisher.reliability:=best_effort",
         "-p",
         "qos_overrides." + output + ".publisher.history:=keep_last",
         "-p",
         "qos_overrides." + output + ".publisher.depth:=1",
         "-r",
         "in/rtsp:=" + input,
         "-r",
         "out:=" + output});
  }

  void spawn(std::vector<std::string> args) {
    std::vector<char *> argv;
    for (auto &arg : args)
      argv.push_back(arg.data());
    argv.push_back(nullptr);
    pid_t pid;
    const int error =
        posix_spawn(&pid, argv[0], nullptr, nullptr, argv.data(), environ);
    if (error)
      throw std::runtime_error("cannot start image_transport republish: " +
                               std::to_string(error));
    pids.push_back(pid);
  }

  double cpu() const {
    double total = 0;
    for (pid_t pid : pids)
      total += processCpu(pid);
    return total;
  }
};

sensor_msgs::msg::Image makeImage(unsigned width, unsigned height,
                                  unsigned stream, unsigned seq) {
  sensor_msgs::msg::Image image;
  image.width = width;
  image.height = height;
  image.encoding = sensor_msgs::image_encodings::BGR8;
  image.step = width * 3;
  image.data.resize(static_cast<std::size_t>(image.step) * height);
  for (unsigned y = 0; y < height; ++y)
    for (unsigned x = 0; x < width; ++x)
      for (unsigned c = 0; c < 3; ++c)
        image.data[y * image.step + x * 3 + c] =
            x < 64 && y < 64 ? COLORS[stream][c]
                             : static_cast<unsigned char>(
                                   ((x * 7 + y * 3 + seq * 11) ^
                                    ((x >> 2) * (y >> 2) + seq * 5)) >>
                                   (c * 2));
  return image;
}

struct Received {
  std::string url;
  std::size_t total = 0, wrong_pixels = 0, steady = 0, isolated = 0;
  std::vector<double> latencies;
};

double percentile(std::vector<double> values, double quantile) {
  if (values.empty())
    return 0;
  std::sort(values.begin(), values.end());
  return values[std::min(values.size() - 1,
                         static_cast<std::size_t>(values.size() * quantile))];
}
} // namespace

int main(int argc, char **argv) {
  const unsigned count = argc > 1 ? std::stoul(argv[1]) : 2;
  const unsigned width = argc > 2 ? std::stoul(argv[2]) : 1920;
  const unsigned height = argc > 3 ? std::stoul(argv[3]) : 1080;
  const unsigned seconds = argc > 4 ? std::stoul(argv[4]) : 10;
  const std::string mode = argc > 5 ? argv[5] : "h264";
  const bool mixed = mode == "mixed";
  const bool all_hevc = mode == "h265";
  const bool hardware = !(argc > 6 && std::string(argv[6]) == "software");
  const bool ros_output = argc > 7 && std::string(argv[7]) == "ros";
  if (count < 1 || count > 4 || width < 64 || height < 64 || seconds < 2 ||
      (mode != "h264" && mode != "h265" && mode != "mixed"))
    return 2;
  // Each invocation has its own topic namespace, including its real RTSP URL
  // topics.
  const std::string prefix = "/rtsp_bench_" + std::to_string(getpid());
  Relays relays;
  for (unsigned i = 0; i < count; ++i)
    relays.start(i, all_hevc || (mixed && i % 2), hardware, prefix);
  Relays receivers;
  if (ros_output)
    for (unsigned i = 0; i < count; ++i)
      receivers.startReceiver(i, hardware, prefix);
  rclcpp::init(argc, argv);
  int result = 0;
  {
    rclcpp::NodeOptions options;
    for (unsigned i = 0; i < count; ++i) {
      std::string base =
          prefix.substr(1) + ".output" + std::to_string(i) + ".rtsp.";
      options.append_parameter_override(base + "use_hw_decoder", hardware);
      options.append_parameter_override(base + "hw_device",
                                        hardware ? "cuda" : "none");
      options.append_parameter_override(base + "timestamp_source", 0);
      options.append_parameter_override(base + "timeout", 0.0);
    }
    auto node =
        std::make_shared<rclcpp::Node>("rtsp_multi_stream_bench", options);
    rclcpp::Clock wall(RCL_SYSTEM_TIME);
    std::mutex mutex;
    std::vector<Received> received(count);
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
        publishers;
    std::vector<image_transport::Subscriber> subscribers;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
        raw_subscribers;
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> urls;
    std::vector<std::vector<sensor_msgs::msg::Image>> patterns(count);
    std::atomic<int> phase{0};
    for (unsigned i = 0; i < count; ++i) {
      publishers.push_back(node->create_publisher<sensor_msgs::msg::Image>(
          prefix + "/input" + std::to_string(i),
          rclcpp::SensorDataQoS().keep_last(1)));
      const std::string output = prefix + "/output" + std::to_string(i);
      auto collect = [&, i](sensor_msgs::msg::Image::ConstSharedPtr image) {
        const auto now = wall.now().nanoseconds();
        bool correct = image->width == width && image->height == height &&
                       image->encoding == sensor_msgs::image_encodings::BGR8;
        if (correct)
          for (unsigned c = 0; c < 3; ++c)
            correct = correct &&
                      std::abs(static_cast<int>(
                                   image->data[16 * image->step + 16 * 3 + c]) -
                               COLORS[i][c]) < 40;
        std::lock_guard<std::mutex> lock(mutex);
        auto &stats = received[i];
        ++stats.total;
        if (!correct)
          ++stats.wrong_pixels;
        if (phase.load() == 1) {
          ++stats.steady;
          stats.latencies.push_back(
              (now - rclcpp::Time(image->header.stamp).nanoseconds()) / 1e6);
        }
        if (phase.load() == 2)
          ++stats.isolated;
      };
      if (ros_output) {
        raw_subscribers.push_back(
            node->create_subscription<sensor_msgs::msg::Image>(
                prefix + "/decoded" + std::to_string(i),
                rclcpp::SensorDataQoS().keep_last(1), collect));
      } else {
#if CURRENT_IMAGE_TRANSPORT_VERSION < (7 * 65536)
        subscribers.push_back(image_transport::create_subscription(
            node.get(), output, collect, "rtsp"));
#else
        subscribers.push_back(image_transport::create_subscription(
            *node, output, collect, "rtsp", rclcpp::QoS(1)));
#endif
      }
      urls.push_back(node->create_subscription<std_msgs::msg::String>(
          output + "/rtsp", rclcpp::QoS(1).reliable().transient_local(),
          [&, i](std_msgs::msg::String::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex);
            received[i].url = msg->data;
          }));
      for (unsigned frame = 0; frame < 8; ++frame)
        patterns[i].push_back(makeImage(width, height, i, frame));
    }
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      count + 1);
    executor.add_node(node);
    std::thread spinner([&] { executor.spin(); });
    const auto start = std::chrono::steady_clock::now();
    auto next = start;
    double relay_cpu = 0, harness_cpu = 0, relay_end = 0, harness_end = 0;
    double receiver_cpu = 0, receiver_end = 0;
    std::vector<unsigned> sent(count, 0);
    unsigned seq = 0;
    // Warm up five seconds, measure, then pause input 0 for three seconds.
    const auto steady_start = start + 5s;
    const auto steady_end = steady_start + std::chrono::seconds(seconds);
    const auto end = steady_end + 3s;
    while (std::chrono::steady_clock::now() < end) {
      auto now = std::chrono::steady_clock::now();
      if (phase.load() == 0 && now >= steady_start) {
        relay_cpu = relays.cpu();
        receiver_cpu = receivers.cpu();
        harness_cpu = processCpu(getpid());
        phase = 1;
      }
      if (phase.load() == 1 && now >= steady_end) {
        relay_end = relays.cpu();
        receiver_end = receivers.cpu();
        harness_end = processCpu(getpid());
        phase = 2;
      }
      for (unsigned i = 0; i < count; ++i) {
        if (phase.load() == 2 && i == 0)
          continue;
        auto &image = patterns[i][seq % patterns[i].size()];
        image.header.stamp = wall.now();
        publishers[i]->publish(image);
        if (phase.load() == 1)
          ++sent[i];
      }
      ++seq;
      next += 33333333ns;
      if (next < std::chrono::steady_clock::now())
        next = std::chrono::steady_clock::now();
      std::this_thread::sleep_until(next);
    }
    phase = 3;
    executor.cancel();
    spinner.join();
    std::lock_guard<std::mutex> lock(mutex);
    for (unsigned i = 0; i < count; ++i) {
      const auto &stats = received[i];
      bool unique = !stats.url.empty();
      for (unsigned j = 0; j < i; ++j)
        unique = unique && stats.url != received[j].url;
      std::printf(
          "STREAM count=%u index=%u codec=%s sent=%u received=%zu fps=%.2f "
          "p50_ms=%.2f p95_ms=%.2f p99_ms=%.2f wrong_pixels=%zu unique_url=%d "
          "other_input_paused_fps=%.2f\n",
          count, i, all_hevc || (mixed && i % 2) ? "H265" : "H264", sent[i],
          stats.steady, static_cast<double>(stats.steady) / seconds,
          percentile(stats.latencies, .5), percentile(stats.latencies, .95),
          percentile(stats.latencies, .99), stats.wrong_pixels, unique,
          static_cast<double>(stats.isolated) / 3);
      if (!unique || stats.total == 0 || stats.wrong_pixels ||
          (i > 0 && stats.isolated == 0))
        result = 1;
    }
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    std::printf(
        "TOTAL streams=%u size=%ux%u mode=%s hardware=%d output=%s "
        "ros_to_rtsp_cpu_cores=%.3f "
        "rtsp_to_ros_cpu_cores=%.3f harness_cpu_cores=%.3f "
        "harness_peak_rss_mib=%.1f\n",
        count, width, height, mode.c_str(), hardware,
        ros_output ? "ros" : "direct", (relay_end - relay_cpu) / seconds,
        (receiver_end - receiver_cpu) / seconds,
        (harness_end - harness_cpu) / seconds, usage.ru_maxrss / 1024.0);
    for (auto &subscriber : subscribers)
      subscriber.shutdown();
  }
  rclcpp::shutdown();
  return result;
}
