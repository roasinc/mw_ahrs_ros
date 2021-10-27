MW-AHRSv1 ROS Driver
===============================

Overview
--------

ROS driver for MW-AHRSv1 sensor

ROS Interface
-----------

| Topic Name   | Type                             | Description             |
|--------------|----------------------------------|-------------------------|
| ``imu/data`` | ``sensor_msgs/Imu``              | IMU values              |
| ``imu/rpy``  | ``geometry_msgs/Vector3Stamped`` | Roll, Pitch, Yaw values |
| ``imu/mag``  | ``sensor_msgs/MagneticField``    | Magnetic Field values   |

| Service Name  | Type             | Description      |
|---------------|------------------|------------------|
| ``imu/reset`` | ``mi_ros/Reset`` | Reset the sensor |

Installation
------------

```
cd ~/catkin_ws/src/
git clone https://github.com/roasinc/mw_ahrs_ros.git
cd ~/catkn_ws/
rosdep install --from-paths src --ignore-src -y
catkin_make
```
