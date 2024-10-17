# ROS Objecttracker

Packages libobjecttracker as Ros2 Node. 
Objects can dynamically be added and removed via services.

It is only possible to either single marker track or multi marker track. 
As soon as one object is added with single marker tracking everything is switched to this mode.

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
float32 max_initial_deviation       # If no point is in this deviation distance in last point cloud object wont get added
---
bool success                        # If add succeded this will be true
~~~
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
