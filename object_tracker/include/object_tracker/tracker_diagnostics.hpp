#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace object_tracker {

struct DiagnosticsNodeInterfaces
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics;
};

struct DiagnosticsConfig
{
  double input_warn;
  double input_error;
  double observation_warn;
  double observation_error;
  double processing_warn;
  double startup_grace;
};

struct TrackerSnapshot
{
  size_t registered{0};
  size_t initialized{0};
  double best_observation_age{0.0};
  double worst_observation_age{0.0};
  double average_observation_age{0.0};
};

// Called on the node's mutually exclusive callback group; no worker or tracker mutation.
class TrackerDiagnostics
{
public:
  using Clock = std::chrono::steady_clock;
  TrackerDiagnostics(DiagnosticsNodeInterfaces interfaces, DiagnosticsConfig config);
  void record_frame(Clock::time_point started, bool usable, bool empty, bool backwards, bool repeated = false);
  void record_invalid_object_frames(uint64_t count);
  void record_acquisition();
  void record_loss();
  void record_timeout();
  void record_failure();
  void publish(const TrackerSnapshot & snapshot);

private:
  DiagnosticsNodeInterfaces m_interfaces;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_publisher;
  Clock::time_point m_created{Clock::now()}, m_interval{m_created}, m_received{}, m_usable{};
  DiagnosticsConfig m_config;
  double m_min_duration{0.0}, m_max_duration{0.0}, m_total_duration{0.0}, m_max_gap{0.0};
  uint64_t m_frames{0}, m_interval_frames{0}, m_empty{0}, m_malformed{0}, m_backwards{0}, m_repeated{0};
  uint64_t m_invalid_object_frames{0};
  uint64_t m_acquired{0}, m_lost{0}, m_timed_out{0}, m_failures{0};
  bool m_interval_failure{false};
  bool m_interval_loss{false};
};

}  // namespace object_tracker
