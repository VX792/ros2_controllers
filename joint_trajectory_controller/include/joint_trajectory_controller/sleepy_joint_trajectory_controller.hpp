// Copyright (c) 2021 ros2_control Development Team
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

#ifndef JOINT_TRAJECTORY_CONTROLLER__SLEEPY_TRAJECTORY_CONTROLLER_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__SLEEPY_TRAJECTORY_CONTROLLER_HPP_

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"


#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "control_toolbox/pid.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_trajectory_controller/interpolation_methods.hpp"
#include "joint_trajectory_controller/tolerances.hpp"
#include "joint_trajectory_controller/visibility_control.h"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/types.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_server_goal_handle.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;  // NOLINT


namespace joint_trajectory_controller
{


class SleepyTrajectoryController : public JointTrajectoryController {

   controller_interface::return_type update(
   const rclcpp::Time & time, const rclcpp::Duration & period) override;

   int sleep_base = 10;
	
 };
} //namespace joint_trajectory_controller
#endif //JOINT_TRAJECTORY_CONTROLLER__SLEEPY_TRAJECTORY_CONTROLLER_HPP_