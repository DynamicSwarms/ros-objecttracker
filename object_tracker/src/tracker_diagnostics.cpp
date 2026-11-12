#include "object_tracker/tracker_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace object_tracker {
namespace {
using Status = diagnostic_msgs::msg::DiagnosticStatus;
template<typename T>
void add(Status & status, const std::string & key, T value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = std::to_string(value);
  status.values.push_back(std::move(item));
}
void raise(Status & status, uint8_t level, const std::string & message)
{
  if (level > status.level) {
    status.level = level;
    status.message = message;
  }
}
}  // namespace

TrackerDiagnostics::TrackerDiagnostics(DiagnosticsNodeInterfaces interfaces, DiagnosticsConfig config)
: m_interfaces(std::move(interfaces)), m_config(config)
{
  for (const double value : {m_config.input_warn, m_config.input_error,
      m_config.observation_warn, m_config.observation_error,
      m_config.processing_warn, m_config.startup_grace}) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("diagnostics thresholds must be finite and positive");
    }
  }
  if (m_config.input_warn >= m_config.input_error ||
    m_config.observation_warn >= m_config.observation_error) {
    throw std::invalid_argument("diagnostics warning must precede error threshold");
  }
  m_publisher = rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    m_interfaces.topics, "/diagnostics", rclcpp::QoS(10));
}

void TrackerDiagnostics::record_frame(
  Clock::time_point started,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & markers,
  const std::vector<const libobjecttracker::Object*> & objects,
  bool backwards, bool repeated)
{
  if (m_received != Clock::time_point{}) {
    m_max_gap = std::max(m_max_gap, std::chrono::duration<double>(started - m_received).count());
  }
  m_received = started;
  ++m_frames;
  ++m_interval_frames;
  if (markers && !backwards && !repeated) m_usable = started;
  if (!markers) {++m_malformed; m_interval_failure = true;}
  if (markers) {
    if (markers->empty()) ++m_empty;
    m_invalid_object_frames += std::count_if(
      objects.begin(), objects.end(),
      [](const libobjecttracker::Object* object) {return !object->lastTransformationValid();});
  }
  if (backwards) {++m_backwards; m_interval_failure = true;}
  if (repeated) {++m_repeated; m_interval_failure = true;}
  const double duration = std::chrono::duration<double>(Clock::now() - started).count();
  m_total_duration += duration;
  m_max_duration = std::max(m_max_duration, duration);
  m_min_duration = m_interval_frames == 1 ? duration : std::min(m_min_duration, duration);
  if (duration > m_config.processing_warn) {
    RCLCPP_WARN(
      m_interfaces.logging->get_logger(), "Tracking latency high: %.4f seconds", duration);
  }
}

void TrackerDiagnostics::record_acquisition() {++m_acquired;}
void TrackerDiagnostics::record_loss() {++m_lost; m_interval_loss = true;}
void TrackerDiagnostics::record_timeout() {++m_timed_out; m_interval_failure = true;}

void TrackerDiagnostics::record_failure() {++m_failures; m_interval_failure = true;}

void TrackerDiagnostics::check_input_timeout()
{
  const auto now = Clock::now();
  const auto age = [now](Clock::time_point stamp) {
      return std::chrono::duration<double>(now - stamp).count();
    };
  const double usable_age = age(m_usable == Clock::time_point{} ? m_created : m_usable);
  const bool starting = age(m_created) < m_config.startup_grace && m_usable == Clock::time_point{};
  const bool input_stale = !starting && usable_age >= m_config.input_warn;
  if (input_stale && !m_input_stale) {
    RCLCPP_WARN(
      m_interfaces.logging->get_logger(),
      "Point-cloud input missing or unusable: no usable frame for %.3f seconds", usable_age);
  } else if (!input_stale && m_input_stale) {
    RCLCPP_INFO(m_interfaces.logging->get_logger(), "Usable point-cloud input resumed");
  }
  m_input_stale = input_stale;
}

void TrackerDiagnostics::publish(const TrackerSnapshot & snapshot)
{
  const auto now = Clock::now();
  const auto age = [now](Clock::time_point stamp) {
      return std::chrono::duration<double>(now - stamp).count();
    };
  const double input_age = age(m_received == Clock::time_point{} ? m_created : m_received);
  const double usable_age = age(m_usable == Clock::time_point{} ? m_created : m_usable);
  const bool starting = age(m_created) < m_config.startup_grace && m_usable == Clock::time_point{};
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = m_interfaces.clock->get_clock()->now();
  Status summary;
  summary.name = std::string(m_interfaces.base->get_fully_qualified_name()) + "/tracker";
  summary.hardware_id = "object_tracker";
  summary.message = starting ? "Waiting for point cloud" : "Running";
  if (!starting && usable_age >= m_config.input_warn) {
    raise(summary, usable_age >= m_config.input_error ? Status::ERROR : Status::WARN,
      "Point-cloud input missing or unusable");
  }
  if (m_interval_failure) raise(summary, Status::WARN, "Input or service failure in interval");
  if (m_max_duration > m_config.processing_warn) raise(summary, Status::WARN, "Tracking processing slow");
  if (snapshot.initialized > 0) {
    // Source-time ages are gathered by the node; steady time also accounts for input silence.
    const double worst_age = snapshot.worst_observation_age + usable_age;
    add(summary, "Best observation age (s)", snapshot.best_observation_age + usable_age);
    add(summary, "Worst observation age (s)", worst_age);
    add(summary, "Average observation age (s)", snapshot.average_observation_age + usable_age);
    if (worst_age >= m_config.observation_warn) {
      raise(summary, worst_age >= m_config.observation_error ? Status::ERROR : Status::WARN,
        "Tracking observations stale");
    }
  }
  if (m_interval_loss) raise(summary, Status::ERROR, "Object tracking lost in interval");
  if (snapshot.registered == 0 && summary.level == Status::OK && !starting) {
    summary.message = "Running; no registered objects";
  }
  add(summary, "Input warning threshold (s)", m_config.input_warn);
  add(summary, "Input error threshold (s)", m_config.input_error);
  add(summary, "Processing warning threshold (ms)", 1000 * m_config.processing_warn);
  add(summary, "Registered objects", snapshot.registered);
  add(summary, "Initialized objects", snapshot.initialized);
  add(summary, "Initializing objects", snapshot.registered - snapshot.initialized);
  add(summary, "Input rate (Hz)", m_interval_frames / age(m_interval));
  add(summary, "Input received", m_received != Clock::time_point{});
  add(summary, "Last input age (s)", input_age);
  add(summary, "Last usable input age (s)", usable_age);
  add(summary, "Maximum input gap in interval (s)", std::max(m_max_gap, input_age));
  add(summary, "Best processing duration (ms)", 1000 * m_min_duration);
  add(summary, "Average processing duration (ms)", m_interval_frames ? 1000 * m_total_duration / m_interval_frames : 0);
  add(summary, "Worst processing duration (ms)", 1000 * m_max_duration);
  add(summary, "Frames total", m_frames);
  add(summary, "Empty frames total", m_empty);
  add(summary, "Malformed frames total", m_malformed);
  add(summary, "Backward timestamps total", m_backwards);
  add(summary, "Repeated timestamps total", m_repeated);
  add(summary, "Invalid object-frames total", m_invalid_object_frames);
  add(summary, "Acquisitions total", m_acquired);
  add(summary, "Tracking losses total", m_lost);
  add(summary, "Initialization timeouts total", m_timed_out);
  add(summary, "Service failures total", m_failures);
  message.status.push_back(std::move(summary));
  m_publisher->publish(message);
  m_interval = now;
  m_interval_frames = 0;
  m_total_duration = m_min_duration = m_max_duration = m_max_gap = 0;
  m_interval_loss = false;
  m_interval_failure = false;
}

}  // namespace object_tracker
