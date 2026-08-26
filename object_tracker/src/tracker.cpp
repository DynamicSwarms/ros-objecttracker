#define BOOST_BIND_NO_PLACEHOLDERS
#include <cstdio>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>


// Object tracker
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <libobjecttracker/object_tracker.h>
#include <libobjecttracker/cloudlog.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// TF Broadcasting
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

// Named position broadcasting
#include "object_tracker_interfaces/msg/position_named_array.hpp"

// Add/Remove Services
#include "object_tracker_interfaces/srv/add_tracker_object.hpp"
#include "object_tracker_interfaces/srv/remove_tracker_object.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "object_tracker/tracker_logger.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ObjectTracker : public rclcpp::Node
{
  using AddObjectService = object_tracker_interfaces::srv::AddTrackerObject;

  public: 
    ObjectTracker(const rclcpp::NodeOptions & options) 
        : Node("object_tracker",options)
        , frame_id("world")
        , last_valid_timeout(0.25)
    { 
      bool log_enabled;
      this->get_parameter_or("logEnabled", log_enabled, false);
      tracker_logger = std::make_unique<object_tracker::TrackerLogger>(log_enabled);

      this->get_parameter_or(
        "publishSingleMarkerObjectsToTf",
        this->publish_single_marker_objects_to_tf,
        false);
      std::int32_t broadcast_frequency = this->get_parameter("tfBroadcastRate").as_int();
      std::string pc2_topicName = this->get_parameter("pointCloud2TopicName").as_string();
      
      point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pc2_topicName,
        rclcpp::QoS(2).reliability(rclcpp::ReliabilityPolicy::BestEffort), 
        std::bind(&ObjectTracker::pc_callback, this, _1));

      initializeTrackerFromParameters();

      tracking_lost_publisher = this->create_publisher<std_msgs::msg::String>("~/object_tracking_lost", 10);
      tracking_supervisor = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&ObjectTracker::supervise_tracking, this));

      int rate = (int)((1.0 / (double)broadcast_frequency) * 1000);
      position_broadcast_timer = this->create_wall_timer(
        std::chrono::milliseconds(rate),
        std::bind(&ObjectTracker::broadcast_positions, this));

      add_object_service = this->create_service<AddObjectService>(
        "~/add_object",
        std::bind(&ObjectTracker::add_object_call, this, _1, _2, std::placeholders::_3));
      remove_object_service = this->create_service<object_tracker_interfaces::srv::RemoveTrackerObject>("~/remove_object",std::bind(&ObjectTracker::remove_object_call, this, _1, _2));
      remove_all_objects_service = this->create_service<std_srvs::srv::Trigger>("~/remove_all_objects", std::bind(&ObjectTracker::remove_all_objects_call, this, _1, _2));

      tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      position_broadcaster = this->create_publisher<object_tracker_interfaces::msg::PositionNamedArray>("~/positions", 10); 
    }

    ~ObjectTracker() override
    {
      complete_all_acquisitions(false);
    }

    
  private:    
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
    {
      auto start_time = std::chrono::steady_clock::now();
     
      pcl::PointCloud<pcl::PointXYZ>::Ptr markers(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *markers);
      this->frame_id = msg->header.frame_id;
      const rclcpp::Time update_stamp(msg->header.stamp);
      libobjecttracker::TrackerDuration dt = libobjecttracker::TrackerDuration::zero();
      if (last_update_stamp) {
        const rclcpp::Duration elapsed = update_stamp - *last_update_stamp;
        if (elapsed < rclcpp::Duration::from_seconds(0.0)) {
          RCLCPP_WARN(this->get_logger(), "Point-cloud timestamp moved backwards");
        } else {
          dt = elapsed.to_chrono<libobjecttracker::TrackerDuration>();
        }
      }
      last_update_stamp = update_stamp;
      m_tracker->update(dt, markers);
      tracker_logger->log_callback(*msg, *markers, *m_tracker);
     
      auto end_time = std::chrono::steady_clock::now(); // End timing
      std::chrono::duration<double> elapsed = end_time - start_time;
      if (elapsed.count() > this->get_parameter("latencyThreshold").as_double())
      {
        RCLCPP_WARN(this->get_logger(), "Tracking latency high: %.4f seconds", elapsed.count());
      }
    }

    void supervise_tracking()
    {
      m_tracker->removeStaleObjects();
    }

    void broadcast_positions()
    {
      std::vector<const libobjecttracker::Object*> single_marker_objects;
      std::vector<const libobjecttracker::Object*> multi_marker_objects;
      
      for (const auto& object : m_tracker->singleMarkerObjects()) {
        if (object.timeSinceLastValidUpdate() < last_valid_timeout) {
          single_marker_objects.push_back(&object);
        }
      }
      for (const auto& object : m_tracker->multiMarkerObjects()) {
        if (object.timeSinceLastValidUpdate() < last_valid_timeout) {
          multi_marker_objects.push_back(&object);
        }
      }

      if (this->publish_single_marker_objects_to_tf) {
        multi_marker_objects.insert(
          multi_marker_objects.end(), single_marker_objects.begin(),
          single_marker_objects.end());
        sendPosesToTF(multi_marker_objects, this->frame_id);
      } else {
        sendPosesToTF(multi_marker_objects, this->frame_id);
        sendNamedPositions(single_marker_objects, this->frame_id);
      }
    }

    void add_object_call(
      const std::shared_ptr<rclcpp::Service<AddObjectService>> service,
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<AddObjectService::Request> request)
    {
      Eigen::Affine3d initial_pose;
      tf2::fromMsg(request->initial_pose, initial_pose);

      auto acquisition_callback =
        [this, service, request_header](bool success) {
          AddObjectService::Response response;
          response.success = success;
          try {
            service->send_response(*request_header, response);
          } catch (const std::exception& exception) {
            RCLCPP_WARN(
              this->get_logger(), "Could not send add-object response: %s",
              exception.what());
          }
        };

      if (m_tracker->addObject(
          request->marker_configuration_idx,
          request->dynamics_configuration_idx,
          initial_pose.cast<float>(),
          request->tf_name.data,
          request->max_initial_deviation,
          request->initialization_timeout)) 
      {
        // If another request is already pending for this object, the callback will be replaced with the new one.
        complete_acquisition(request->tf_name.data, false); 
        m_pending_acquisitions[request->tf_name.data] = std::move(acquisition_callback);
      } else {
        acquisition_callback(false);
      }
    }

    
    void remove_object_call(const std::shared_ptr<object_tracker_interfaces::srv::RemoveTrackerObject::Request> request,
                                  std::shared_ptr<object_tracker_interfaces::srv::RemoveTrackerObject::Response> response)
    {
      for (const auto* object : m_tracker->objects()) {
        if (object->name() == request->tf_name.data){
          response->success = true;
          complete_acquisition(request->tf_name.data, false);
          m_tracker->removeObject(request->tf_name.data);
          return;
        } 
      }
    }

    void remove_all_objects_call(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void) request;
      complete_all_acquisitions(false);
      response->success = m_tracker->removeAllObjects();
    }

    void sendPosesToTF(
      const std::vector<const libobjecttracker::Object*>& objects,
      const std::string frame_id)
    {
      std::vector<geometry_msgs::msg::TransformStamped> transforms;
      for (const auto* object : objects)
      {
        auto transform = tf2::eigenToTransform(
          object->transformation().cast<double>());
        transform.header.stamp = lastValidStamp(*object);
        transform.header.frame_id = frame_id;
        transform.child_frame_id = object->name();
        transforms.push_back(std::move(transform));
      }
      tf_broadcaster->sendTransform(transforms);
    }

    void sendNamedPositions(
      const std::vector<const libobjecttracker::Object*>& objects,
      const std::string frame_id)
    {
      auto position_array = object_tracker_interfaces::msg::PositionNamedArray();
      position_array.header.stamp = last_update_stamp
        ? static_cast<builtin_interfaces::msg::Time>(*last_update_stamp)
        : static_cast<builtin_interfaces::msg::Time>(this->get_clock()->now());
      position_array.header.frame_id = frame_id;
      for (const auto* object : objects)
      {
        const Eigen::Affine3f& transform = object->transformation();
        const auto& translation = transform.translation();

        object_tracker_interfaces::msg::PositionNamed position;

        position.name = object->name();
        position.header = position_array.header;
        position.header.stamp = lastValidStamp(*object);
        position.position = tf2::toMsg(
          Eigen::Vector3d(translation.cast<double>()));
        position_array.positions.push_back(position);
      }
      position_broadcaster->publish(position_array);
    }

    builtin_interfaces::msg::Time lastValidStamp(
      const libobjecttracker::Object& object) const
    {
      if (!last_update_stamp) {
        return this->get_clock()->now();
      }
      return *last_update_stamp -
        rclcpp::Duration(object.timeSinceLastValidUpdate());
    }


    /**
    * Reads Ros Parameters and build m_tracker from it.
    * Reads markerConfigurations and dynamicsConfigurations (defined in YAML File)
    * Initializes with no object at startup
    */
    void initializeTrackerFromParameters() 
    {
      std::vector<libobjecttracker::MarkerConfiguration> markerConfigurations;
      readMarkerConfigurations(markerConfigurations);
      std::vector<libobjecttracker::DynamicsConfiguration> dynamicsConfigurations;
      readDynamicsConfigurations(dynamicsConfigurations);
      m_tracker = std::make_unique<libobjecttracker::ObjectTracker>(
        dynamicsConfigurations, markerConfigurations);
      m_tracker->setLogWarningCallback(std::bind(&ObjectTracker::logWarn, this, _1));
      m_tracker->setEventCallback(
        std::bind(&ObjectTracker::tracking_event, this, _1, _2));
    }

    void tracking_event(
      libobjecttracker::TrackingEvent event,
      const libobjecttracker::Object& object)
    {
      if (event == libobjecttracker::TrackingEvent::acquired) 
      {
        complete_acquisition(object.name(), true);
      } else if (event == libobjecttracker::TrackingEvent::initialization_timed_out)
      {
        complete_acquisition(object.name(), false);
        RCLCPP_WARN(
          this->get_logger(), "Initialization timed out for object '%s'",
          object.name().c_str());
      } else if (event == libobjecttracker::TrackingEvent::lost) 
      {
        auto msg = std_msgs::msg::String();
        msg.data = object.name();
        tracking_lost_publisher->publish(msg);
        RCLCPP_WARN(
          this->get_logger(), "Lost tracking for object '%s'",
          object.name().c_str());
      } else 
      {
        RCLCPP_WARN(
          this->get_logger(), "Unknown tracking event for object '%s'",
          object.name().c_str());
      }
    }

    void complete_acquisition(const std::string& name, bool success)
    {
      const auto callback = m_pending_acquisitions.find(name);
      if (callback == m_pending_acquisitions.end()) {
        return;
      }
      auto completion = std::move(callback->second);
      m_pending_acquisitions.erase(callback);
      completion(success);
    }

    void complete_all_acquisitions(bool success)
    {
      auto pending = std::move(m_pending_acquisitions);
      m_pending_acquisitions.clear();
      for (auto& acquisition : pending) {
        acquisition.second(success);
      }
    }


    void readMarkerConfigurations(
      std::vector<libobjecttracker::MarkerConfiguration>& markerConfigurations)
    {
      markerConfigurations.clear();
      int numMarkerConfigurations = this->get_parameter("numMarkerConfigurations").as_int();

      for (int i = 0; i < numMarkerConfigurations; i++ ) {
          markerConfigurations.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
          std::stringstream paramName;
          paramName << "markerConfigurations." << i << ".numPoints";
          int numPoints = this->get_parameter(paramName.str()).as_int();;
          paramName.str("");
          paramName << "markerConfigurations." << i << ".offset";
          std::vector<double> offset = this->get_parameter(paramName.str()).as_double_array();
          
          for (int j = 0; j < numPoints; j++) {
            paramName.str("");
            paramName << "markerConfigurations." << i << ".points." << j;
            std::vector<double> points = this->get_parameter(paramName.str()).as_double_array();  
            markerConfigurations.back()->push_back(pcl::PointXYZ(points[0] + offset[0], points[1] + offset[1], points[2] + offset[2]));
          }
          
      }
    }

    void readDynamicsConfigurations(
      std::vector<libobjecttracker::DynamicsConfiguration>& dynamicsConfigurations)
    {
      int numDynamicsConfigurations = this->get_parameter("numDynamicsConfigurations").as_int();
      dynamicsConfigurations.resize(numDynamicsConfigurations);
      for (int i = 0; i < numDynamicsConfigurations; i++) {
        std::stringstream paramName;
        paramName << "dynamicsConfigurations." << i;
        dynamicsConfigurations[i].maxXVelocity = this->get_parameter(paramName.str() + ".maxXVelocity").as_double(); 
        dynamicsConfigurations[i].maxYVelocity = this->get_parameter(paramName.str() + ".maxYVelocity").as_double();
        dynamicsConfigurations[i].maxZVelocity = this->get_parameter(paramName.str() + ".maxZVelocity").as_double();
        dynamicsConfigurations[i].maxPitchRate = this->get_parameter(paramName.str() + ".maxPitchRate").as_double();
        dynamicsConfigurations[i].maxRollRate = this->get_parameter(paramName.str() + ".maxRollRate").as_double();
        dynamicsConfigurations[i].maxYawRate = this->get_parameter(paramName.str() + ".maxYawRate").as_double();
        dynamicsConfigurations[i].maxRoll = this->get_parameter(paramName.str() + ".maxRoll").as_double();
        dynamicsConfigurations[i].maxPitch = this->get_parameter(paramName.str() + ".maxPitch").as_double();
        dynamicsConfigurations[i].maxFitnessScore = this->get_parameter(paramName.str() + ".maxFitnessScore").as_double(); 
      }
    }


    void logWarn(
      const std::string& msg)
    {
      RCLCPP_WARN(this->get_logger(),"%s", msg.empty() ? "" : msg.c_str());
    }

  private:
    std::string frame_id;
    libobjecttracker::TrackerDuration last_valid_timeout;
    std::optional<rclcpp::Time> last_update_stamp;
    std::unique_ptr<object_tracker::TrackerLogger> tracker_logger;
   
    bool publish_single_marker_objects_to_tf;

    std::unique_ptr<libobjecttracker::ObjectTracker> m_tracker;
    std::unordered_map<std::string, std::function<void(bool)>> m_pending_acquisitions;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::Publisher<object_tracker_interfaces::msg::PositionNamedArray>::SharedPtr position_broadcaster;
    rclcpp::TimerBase::SharedPtr position_broadcast_timer; 

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_lost_publisher;
    rclcpp::TimerBase::SharedPtr tracking_supervisor; 

    rclcpp::Service<AddObjectService>::SharedPtr add_object_service;
    rclcpp::Service<object_tracker_interfaces::srv::RemoveTrackerObject>::SharedPtr remove_object_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_all_objects_service;
};


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true); // Allows the Parameters from YAML file to be available in Node
  rclcpp::spin(std::make_shared<ObjectTracker>(options));
  rclcpp::shutdown();
  return 0;
}
