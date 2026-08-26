#include "object_tracker/tracker_logger.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <ios>
#include <stdexcept>

#include <Eigen/Geometry>
#include <rclcpp/logging.hpp>

namespace object_tracker
{

TrackerLogger::TrackerLogger(bool enabled)
{
  if (!enabled) {
    return;
  }

  const auto log_directory = rclcpp::get_log_directory() / "object_tracker";
  std::filesystem::create_directories(log_directory);
  const auto log_file = log_directory / "object_tracker_log.log";

  callback_log_.open(log_file, std::ios::out | std::ios::trunc);
  if (!callback_log_.is_open()) {
    throw std::runtime_error("Failed to open tracker log file");
  }
}

void TrackerLogger::log_callback(
  const sensor_msgs::msg::PointCloud2 & message,
  const pcl::PointCloud<pcl::PointXYZ> & markers,
  libobjecttracker::ObjectTracker & tracker)
{
  if (!callback_log_.is_open()) {
    return;
  }

  // JSON Lines keeps every callback self-contained and supports a variable
  // number of markers and tracked objects without losing frame boundaries.
  callback_log_ << std::setprecision(9)
                << "{\"stamp_ns\":"
                << (static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL +
                    static_cast<std::int64_t>(message.header.stamp.nanosec))
                << ",\"frame_id\":\"" << message.header.frame_id
                << "\",\"points\":[";

  for (std::size_t i = 0; i < markers.size(); ++i) {
    if (i != 0) {
      callback_log_ << ',';
    }
    const auto & point = markers[i];
    callback_log_ << '[' << point.x << ',' << point.y << ',' << point.z << ']';
  }

  callback_log_ << "],\"objects\":[";
  const auto objects = tracker.objects();
  for (std::size_t i = 0; i < objects.size(); ++i) {
    if (i != 0) {
      callback_log_ << ',';
    }
    const auto* object = objects[i];
    const auto & position = object->transformation().translation();
    callback_log_ << "{\"name\":\"" << object->name()
                  << "\",\"valid\":"
                  << (object->lastTransformationValid() ? "true" : "false")
                  << ",\"tracking_valid\":"
                  << (tracker.trackingValid(*object) ? "true" : "false")
                  << ",\"seconds_since_valid\":"
                  << object->timeSinceLastValidUpdate().count()
                  << ",\"position\":[" << position.x() << ',' << position.y()
                  << ',' << position.z() << ']';
    if (object->orientationValid()) {
      const Eigen::Quaternionf orientation(object->transformation().linear());
      callback_log_ << ",\"orientation\":[" << orientation.x() << ','
                    << orientation.y() << ',' << orientation.z() << ','
                    << orientation.w() << ']';
    }
    callback_log_ << '}';
  }
  callback_log_ << "]}\n";
  callback_log_.flush();
}

}  // namespace object_tracker
