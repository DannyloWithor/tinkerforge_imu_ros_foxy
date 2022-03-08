// Copyright (c) 2020 Steve Macenski
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include <string.h>
#include "../include/ip_connection.h"
#include "../include/brick_imu_v2.h"

#define HOST "localhost"
#define PORT 4223

class TinkforgeImu : public rclcpp::Node
{
  public:
    TinkforgeImu()
    : Node("tinkerforge_node")
    {
      imuPub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

      zeros_ = { 0, 0, 0,
                0, 0, 0,
                0, 0, 0 };

      IMUV2 imu;
      IPConnection ipcon;
      ipcon_create(&ipcon);
      imu_v2_create(&imu, uid.c_str(), &ipcon);
      if(ipcon_connect(&ipcon, HOST, PORT) < 0)
      {
        RCLCPP_INFO(this->get_logger(), "Bricks IMU V2: Could not connect!");
        return ;
      }

      imu_v2_register_callback(&imu,
                           IMU_V2_CALLBACK_QUATERNION,
                           (void *)&TinkforgeImu::cb_quaternion,
                           NULL);
      imu_v2_set_quaternion_period(&imu, period_ms);

    }

    private:
      rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;
      std::array<double, 9> zeros_ ;
      std::string frameId_ = "imu_link";
      std::string uid = "6Det55";
      int period_ms = 10;

      void cb_quaternion(int16_t w, int16_t x, int16_t y, int16_t z, void * user_data)
      {
        (void)user_data;

        sensor_msgs::msg::Imu imuMsg;
        imuMsg.header.frame_id = frameId_;
        imuMsg.header.stamp = this->get_clock()->now();

        imuMsg.orientation.w = w/16383.0;
        imuMsg.orientation.x = x/16383.0;
        imuMsg.orientation.y = y/16383.0;
        imuMsg.orientation.z = z/16383.0;

        imuMsg.orientation_covariance = zeros_;
        imuMsg.orientation_covariance[0] = 1e-8;
        imuMsg.orientation_covariance[4] = 1e-8;
        imuMsg.orientation_covariance[8] = 1e-8;

        //publish the message
        imuPub_->publish(imuMsg);
      }

};



int main(int argc, char ** argv)
{
  // initialize ROS node
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TinkforgeImu>());
  rclcpp::shutdown();

  //ipcon_destroy(&ipcon);
  return 0;
}
