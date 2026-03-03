// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("ackermann_steering_test_node");

  auto publisher = node->create_publisher<geometry_msgs::msg::TwistStamped>("/ackermann_steering_controller/reference", 10);

  RCLCPP_INFO(node->get_logger(), "node created");

  geometry_msgs::msg::TwistStamped command;

  using namespace std::chrono_literals;

  command.header.frame_id = "map";
  command.header.stamp = node->get_clock()->now();
  command.twist.linear.x = 0.0;
  command.twist.angular.z = 0.0;
  publisher->publish(command);
  std::this_thread::sleep_for(1s);

  command.header.stamp = node->get_clock()->now();
  command.twist.linear.x = 1.0;
  command.twist.angular.z = 0.0;
  publisher->publish(command);
  std::this_thread::sleep_for(1s);

  command.header.stamp = node->get_clock()->now();
  command.twist.linear.x = 0.0;
  command.twist.angular.z = 1.0;
  publisher->publish(command);
  std::this_thread::sleep_for(1s);

  std::this_thread::sleep_for(1s);
  rclcpp::shutdown();

  return 0;
}
