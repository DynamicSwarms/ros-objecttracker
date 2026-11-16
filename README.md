# ROS Objecttracker

Packages libobjecttracker as Ros2 Node. 
Objects can dynamically be added and removed via services.

Single-marker and multi-marker objects can be tracked side by side. Both
trackers read the same point cloud and do not reserve markers from each other.

# Usage

Configure a tracker_config.yaml (as in ros_ws/src/object_tracker/launch).
This allows for configuration of:
#### MarkerConfigurations: 
The markerlayout on the object to track. 
#### DynamicsConfiguration: 
Dynaics of object to track (knowing the maximum speed / acceleration of objects helps to increase accuracy)
#### PointCloud topic name:
The point cloud used for tracking

Launch object_tracker node.

Objects can then be added or remove with the add_tracker_object and remove_tracker_object services.

# AddTrackerObject
~~~text
std_msgs/String tf_name             # The name of the object 
uint8 marker_configuration_idx      # The index of to be used marker config from yaml
uint8 dynamics_configuration_idx    # The index of to be used dynamics config from yaml
geometry_msgs/Pose initial_pose     # The initial position of the object
float32 max_initial_deviation       # Maximum distance accepted during initial acquisition
float32 initialization_timeout     # Seconds allowed for initial acquisition
---
bool success                        # True after the first valid tracking update
~~~

The add service uses a deferred response. A valid request registers the object
and waits for a matching point-cloud observation. It returns false if the
configuration is invalid, the object is removed or updated before acquisition,
or initialization times out.
# RemoveTrackerObject
~~~text
std_msgs/String tf_name             # The object to be removed
---
bool success                        # If no object with given name was found return will be false
~~~
# Installation instructions
sudo apt install -y libpcl-dev

## If ROS is built from sources following dependencies need to be build:

git clone https://github.com/ros-perception/perception_pcl.git
git clone https://github.com/ros-perception/pcl_msgs.git -b ros2
colcon build --packages-select pcl_msgs
colcon build --packages-select pcl_conversions
