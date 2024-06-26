// Copyright 2021, Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "domain_bridge/component_manager.hpp"
#include "domain_bridge/domain_bridge.hpp"
#include "domain_bridge/parse_domain_bridge_yaml_config.hpp"
#include "domain_bridge/process_cmd_line_arguments.hpp"

#include "interface_package/srv/order_info.hpp"
#include "interface_package/srv/order_tracking.hpp"
#include "interface_package/srv/robot_arrival.hpp"

int main(int argc, char ** argv)
{
  auto arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);

  auto config_rc_pair = domain_bridge::process_cmd_line_arguments(arguments);
  if (!config_rc_pair.first || 0 != config_rc_pair.second) {
    return config_rc_pair.second;
  }
  domain_bridge::DomainBridge domain_bridge(*config_rc_pair.first);

  // Add component manager node and domain bridge to single-threaded executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  domain_bridge.add_to_executor(*executor);

  domain_bridge::ServiceBridgeOptions options;
  domain_bridge.bridge_service<interface_package::srv::OrderInfo>("order_info", 71, 0, options.remap_name("order_info_1"));
  domain_bridge.bridge_service<interface_package::srv::OrderInfo>("order_info", 2, 0, options.remap_name("order_info_2"));
  domain_bridge.bridge_service<interface_package::srv::OrderInfo>("order_info", 69, 0, options.remap_name("order_info_3"));
  domain_bridge.bridge_service<interface_package::srv::OrderTracking>("order_tracking_1", 0, 71, options.remap_name("order_tracking"));
  domain_bridge.bridge_service<interface_package::srv::OrderTracking>("order_tracking_2", 0, 2, options.remap_name("order_tracking"));
  domain_bridge.bridge_service<interface_package::srv::OrderTracking>("order_tracking_3", 0, 69, options.remap_name("order_tracking"));
  domain_bridge.bridge_service<interface_package::srv::RobotArrival>("robotArrival", 0, 71);
  domain_bridge.bridge_service<interface_package::srv::RobotArrival>("robotArrival", 0, 2);
  domain_bridge.bridge_service<interface_package::srv::RobotArrival>("robotArrival", 0, 69);

  executor->spin();

  rclcpp::shutdown();
  return 0;
}
