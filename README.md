<img src="./docs/imgs/pyx4.png" width="100%">

# Pyx4

Python ROS2 interface with PX4 through a Fast-RTPS bridge.

## Requirements

You need to install the required packages for PX4, ROS, and XRCE-DDS before running the Pyx4 examples, as instructed in this [guide](https://docs.px4.io/main/en/ros/ros2_comm.html).

<div align='center'>
  <img src="./docs/imgs/architecture_xrce-dds_ros2.svg" width="90%">
</div>

## Getting Started

Setting up `Pyx4`:

```bash
cd dev_ws/src
git clone https://github.com/kirinslab/pyx4.git
cd dev_ws
colcon build --packages-select pyx4
source install/setup.bash
```

## Sample

Run the example to read information about `GPS Position`:

```bash
ros2 run pyx4 pyx4_gps
```

Output:

```
RECEIVED VEHICLE GPS POSITION DATA
==================================
[Lat] :  473977440
[lon] :  85455944
[Alt]:  487624
[Heading] :  nan
RECEIVED VEHICLE GPS POSITION DATA
==================================
[Lat] :  473977472
[lon] :  85455960
[Alt]:  487939
[Heading] :  nan
```

## QoS for PX4 Python

PX4 QoS settings for publishers are incompatible with the default QoS settings for ROS 2 subscribers. So if ROS 2 code needs to subscribe to a uORB topic, it will need to use compatible QoS settings.

PX4 uses the following QoS settings for `Sensor Data` subscribers:

```C++
rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
```

PX4 defines `qos_profile` as `rmw_qos_profile_sensor_data`. You can look up this profile in [QoS Profiles](https://github.com/ros2/rmw/blob/foxy/rmw/include/rmw/qos_profiles.h) as follows:

```C++
static const rmw_qos_profile_t rmw_qos_profile_sensor_data =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  5,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
}
```

Looking up the values of `rmw_qos_profile_sensor_data` corresponding to the classes defined in the [QoS rclpy](https://docs.ros2.org/foxy/api/rclpy/api/qos.html#) library. We get the following results:

```python
qos = QoSProfile(
            history=HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )
```

With QoS defined in `Python`, we can use it to read the packets that `PX4` publishes using `ROS2` and `Python`.

## Author

```
haiquantran2897@gmail.com
```

## Reference

- [PX4 Documentation](https://docs.px4.io/main/en/ros/ros2_comm.html)
- [PX4 ROS COM](https://github.com/PX4/px4_ros_com)
- [About Quality of Service Setting](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html)
- [QoS Profiles](https://github.com/ros2/rmw/blob/foxy/rmw/include/rmw/qos_profiles.h)
- [QoS rclpy](https://docs.ros2.org/foxy/api/rclpy/api/qos.html#)

## License

Copyright (c) Kirinslab. All rights reserved. Licensed under the [GPT-3](LICENSE) license.