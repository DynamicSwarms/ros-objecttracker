#define BOOST_BIND_NO_PLACEHOLDERS
#include <cstdio>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>


// Object tracker
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <libobjecttracker/object_tracker.h>
#include <libobjecttracker/cloudlog.hpp>
#include <pcl_conversions/pcl_conversions.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// TF Broadcasting
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

// Add/Remove Services
#include "object_tracker_interfaces/srv/add_tracker_object.hpp"
#include "object_tracker_interfaces/srv/remove_tracker_object.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ObjectTracker : public rclcpp::Node
{
  public: 
    ObjectTracker(const rclcpp::NodeOptions & options) 
        : Node("object_tracker",options)
        , latestPCL(new pcl::PointCloud<pcl::PointXYZ>)
        , frame_id("world")
    {
      std::int32_t broadcast_frequency = this->get_parameter("tfBroadcastRate").as_int();
      std::string pc2_topicName = this->get_parameter("pointCloud2TopicName").as_string();
      point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pc2_topicName, 10, std::bind(&ObjectTracker::pc_callback, this, _1));

      initializeTrackerFromParameters();

      tracking_lost_publisher = this->create_publisher<std_msgs::msg::String>("~/object_tracking_lost", 10);
      tracking_supervisor = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&ObjectTracker::supervise_tracking, this));

      int rate = (int)((1.0 / (double)broadcast_frequency) * 1000);
      position_broadcast_timer = this->create_wall_timer(
        std::chrono::milliseconds(rate),
        std::bind(&ObjectTracker::broadcast_positions, this));

      add_object_service = this->create_service<object_tracker_interfaces::srv::AddTrackerObject>("~/add_object", std::bind(&ObjectTracker::add_object_call, this, _1, _2));
      remove_object_service = this->create_service<object_tracker_interfaces::srv::RemoveTrackerObject>("~/remove_object",std::bind(&ObjectTracker::remove_object_call, this, _1, _2));
      remove_all_objects_service = this->create_service<std_srvs::srv::Trigger>("~/remove_all_objects", std::bind(&ObjectTracker::remove_all_objects_call, this, _1, _2));


      tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    
  private:    
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr markers(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *markers);
      pcl::fromROSMsg(*msg, *latestPCL); // Save latest point Cloud for checking in addObject
      this->frame_id = msg->header.frame_id;
      m_tracker->update(markers);     
    }

    void supervise_tracking()
    {
      std::vector<libobjecttracker::Object> objects;
      for (auto obj : m_tracker->objects())
      {
        if (!m_tracker->trackingValid(obj))
        {
          objects.push_back(obj);
        }
      }

      for (auto obj : objects)
      {
        m_tracker->removeObject(obj.name());
        auto msg = std_msgs::msg::String();
        msg.data = obj.name();
        tracking_lost_publisher->publish(msg);
      }
    }

    void broadcast_positions()
    {
      std::vector<libobjecttracker::Object> objects;
      for (size_t i = 0; i < m_tracker->objects().size(); i++) {
          if (m_tracker->objects()[i].lastTransformationValid())
          {
            objects.push_back(m_tracker->objects()[i]);
          }          
      }
      sendPosesToTF(objects, this->frame_id);
    }

    void add_object_call(const std::shared_ptr<object_tracker_interfaces::srv::AddTrackerObject::Request> request,
                                std::shared_ptr<object_tracker_interfaces::srv::AddTrackerObject::Response> response)
    {
      // Checks if obj with ame tf_name already exists, will not add object with duplicate names
      for (auto obj :  m_tracker->objects()) {
        if (obj.name() == request->tf_name.data) return; // Returns success = false
      }
      // Checks if there is a point in last point cloud which satisfies max intial deviation constraint
      bool deviation_ok = false;
      for (auto point : latestPCL->points) {
        auto p = point;
        auto o = request->initial_pose.position;
        if (sqrt(pow(p.x - o.x, 2) + pow(p.z - o.z, 2) + pow(p.z - o.z, 2)) < request->max_initial_deviation) deviation_ok = true;
      }
      if (!deviation_ok) return; // Returns success = false


      Eigen::Affine3f initialPose;
      
      initialPose = Eigen::Translation3f(request->initial_pose.position.x,request->initial_pose.position.y,request->initial_pose.position.z);
      float x = request->initial_pose.orientation.x;
      float y = request->initial_pose.orientation.y;
      float z = request->initial_pose.orientation.z;
      float w = request->initial_pose.orientation.w;    
      initialPose.rotate(Eigen::Quaternionf(w,x, y, z));

      m_tracker->addObject(libobjecttracker::Object(request->marker_configuration_idx, request->dynamics_configuration_idx, initialPose, request->tf_name.data));
      response->success = true;
    }

    
    void remove_object_call(const std::shared_ptr<object_tracker_interfaces::srv::RemoveTrackerObject::Request> request,
                                  std::shared_ptr<object_tracker_interfaces::srv::RemoveTrackerObject::Response> response)
    {
      for (auto obj :  m_tracker->objects()) {
        if (obj.name() == request->tf_name.data){
          response->success = true;
          m_tracker->removeObject(request->tf_name.data);
          return;
        } 
      }
    }

    void remove_all_objects_call(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      response->success = m_tracker->removeAllObjects();
    }

    void sendPosesToTF(const std::vector<libobjecttracker::Object>& objects, const std::string frame_id) 
    {
      std::vector<geometry_msgs::msg::TransformStamped> transforms;
      for (const auto& object : objects) 
      {
        const Eigen::Affine3f& transform = object.transformation();
        Eigen::Quaternionf q(transform.rotation());
        const auto& translation = transform.translation();

        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = frame_id;                     // No this needs to be set somwhow differently (same as pointcloud)
        t.child_frame_id = object.name().c_str();

        t.transform.translation.x = translation.x();
        t.transform.translation.y = translation.y();
        t.transform.translation.z = translation.z();

        t.transform.rotation.x = q.x();      
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        transforms.push_back(t);
      }
      tf_broadcaster->sendTransform(transforms);
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
      std::vector<libobjecttracker::Object> objects;
      //readObjects(objects);

      m_tracker = new libobjecttracker::ObjectTracker(dynamicsConfigurations, markerConfigurations, objects);
      m_tracker->setLogWarningCallback(std::bind(&ObjectTracker::logWarn, this, _1));
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
      RCLCPP_WARN(this->get_logger(),"%s", msg.c_str());
    }

    std::string frame_id;

    libobjecttracker::ObjectTracker* m_tracker; // non-owning pointer
    pcl::PointCloud<pcl::PointXYZ>::Ptr latestPCL;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr position_broadcast_timer; 

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_lost_publisher;
    rclcpp::TimerBase::SharedPtr tracking_supervisor; 

    rclcpp::Service<object_tracker_interfaces::srv::AddTrackerObject>::SharedPtr add_object_service;
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
