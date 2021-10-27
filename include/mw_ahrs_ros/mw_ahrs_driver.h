/*
Software License Agreement (BSD License)

Authors : Brighten Lee <shlee@roas.co.kr>

Copyright (c) 2021, ROAS Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MW_AHRS_ROS__MW_AHRS_DRIVER_H_
#define MW_AHRS_ROS__MW_AHRS_DRIVER_H_

#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "serial/serial.h"
#include "realtime_tools/realtime_publisher.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "mw_ahrs_ros/Reset.h"

using namespace std;

class MwAhrsDriver
{
public:
  MwAhrsDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  virtual ~MwAhrsDriver();

  /**
   * \brief Initialize
   */
  bool init();

  /**
   * \brief Read message
   */
  void read();

  /**
   * \brief Start streaming IMU sensor data
   */
  void start();

  /**
   * \brief Reset IMU
   * \param req Request
   * \param resp Response
   */
  bool reset(mw_ahrs_ros::Reset::Request& req, mw_ahrs_ros::Reset::Response& resp);

  /**
   * \brief Publish IMU sensor data
   */
  void publishData();

private:
  /// ROS parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  ros::ServiceServer srv_reset_;

  realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped> rp_rpy_;
  realtime_tools::RealtimePublisher<sensor_msgs::Imu> rp_imu_;
  realtime_tools::RealtimePublisher<sensor_msgs::MagneticField> rp_mag_;

  /// For serial communication
  serial::Serial serial_;
  string port_;
  int32_t baud_;

  /// Publishing rate
  double rate_;

  /// Frame ID
  string frame_id_;
};

#endif  // MW_AHRS_ROS__MW_AHRS_DRIVER_H_