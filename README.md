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

## Diagnostics

`TrackerDiagnostics` is a separate module; the node only records frame timing,
tracking events and service failures. Its bound `publish_diagnostics` callback gathers
aggregate data from `m_tracker` and passes a plain `TrackerSnapshot` once per second.
It does not modify tracker state or output behavior. No tracking-library changes
are required. Frame recording uses constant-size counters and steady-clock timing.

The module publishes `diagnostic_msgs/DiagnosticArray` on `/diagnostics`, with
a single `<node>/tracker` status. Object statistics are best/worst/average
observation age across initialized objects, plus aggregate object counts. No
individual object names, statuses or bookkeeping are kept by the module. It has
no dependency on the tracking library. It receives only base, clock and topics interfaces rather than the full node. All callbacks use the node's default
mutually exclusive callback group, so snapshot gathering and recording need no locks.

| Condition | WARN | ERROR |
| --- | --- | --- |
| No usable, timestamp-advancing point cloud | 100 ms | 250 ms |
| Initialized object's last observation age | 100 ms | `lastValidTimeout` (default 250 ms) |
| Processing duration | Above `latencyThreshold` | — |
| Backward/repeated timestamp, points with empty data, rejected add or add-response failure | Reporting interval containing failure | — |
| Initialization timeout | Reporting interval containing timeout | — |
| Tracking loss | — | Reporting interval containing loss |

Silence and frozen source timestamps age diagnostics using steady time even when
tracker time stops advancing. Object age combines tracker age and elapsed time
since the last usable input. Empty clouds are usable input and are counted; they
do not by themselves cause an input warning. No registered objects is an OK idle
state when input is healthy. Losses and timeouts affect the reporting interval
in which they occur; lifetime counters retain their history. Observation-age
statistics are omitted when no initialized objects exist.

Optional parameters (seconds, finite and positive; warning must precede error):

```yaml
diagnostics_input_warn_seconds: 0.1
diagnostics_input_error_seconds: 0.25
diagnostics_observation_warn_seconds: 0.1
diagnostics_startup_grace_seconds: 2.0
```

Scalar parameter reads live in the tracker constructor; marker and dynamics
configuration retain the existing initialization helpers. Scalar defaults use
`get_parameter_or`; YAML overrides are automatically declared by the node options.
Diagnostics receives a plain `DiagnosticsConfig` and never reads parameters.
`lastValidTimeout` controls both the output freshness cutoff and the observation
diagnostic error threshold (default 0.25 seconds). The observation warning
threshold must remain below it.
The startup grace applies until the first usable input. Parameters are read at
startup. Statistics include input Hz, input age and maximum gap,
processing duration (best/worst/average), registered/initializing/initialized object
counts, and lifetime
frame, empty/malformed-frame, timestamp-anomaly, acquisition, loss, timeout and
service-failure counts. `Invalid object-frames total` counts each registered
object with no valid transformation after each tracker update, including objects
still initializing. Two invalid objects in one processed frame add two. Skipped
malformed input and periods with no input do not add to this counter. It does not
change warning/error criteria. Interval statistics reset after publication; lifetime
counts do not. Recovery appears on the next 1 Hz publication.

This initial module does not measure transport latency, pose fitness, nonfinite
points, output delivery or executor stalls. Its timer shares the node executor;
a stalled process requires an external diagnostic heartbeat monitor.
