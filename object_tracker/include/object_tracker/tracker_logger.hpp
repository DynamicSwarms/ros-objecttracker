#pragma once

#include <fstream>
#include <memory>

#include <libobjecttracker/object_tracker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace object_tracker
{

class TrackerLogger
{
public:
  explicit TrackerLogger(bool enabled);

  void log_callback(
    const std::shared_ptr<sensor_msgs::msg::PointCloud2> & message,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & markers,
    const libobjecttracker::ObjectTracker & tracker);

private:
  std::ofstream callback_log_;
};

}  // namespace object_tracker
