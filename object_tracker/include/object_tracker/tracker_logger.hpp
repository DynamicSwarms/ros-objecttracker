#pragma once

#include <fstream>
#include <string>

#include <libobjecttracker/object_tracker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace object_tracker
{

class TrackerLogger
{
public:
  void open(const std::string & path, const rclcpp::Logger & logger);

  void log_callback(
    const sensor_msgs::msg::PointCloud2 & message,
    const pcl::PointCloud<pcl::PointXYZ> & markers,
    libobjecttracker::ObjectTracker & tracker);

private:
  std::ofstream callback_log_;
};

}  // namespace object_tracker
