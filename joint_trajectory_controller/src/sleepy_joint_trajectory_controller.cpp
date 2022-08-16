#include "joint_trajectory_controller/sleepy_joint_trajectory_controller.hpp"


#include <stddef.h>
#include <chrono>
#include <functional>
#include <memory>
#include <ostream>
#include <ratio>
#include <string>
#include <vector>
#include <fstream>
#include <random>

#include "angles/angles.h"
#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/header.hpp"

namespace joint_trajectory_controller
{
	
	controller_interface::return_type SleepyTrajectoryController::update(
	  const rclcpp::Time & time, const rclcpp::Duration & period)
	{ 
		if (sleep_base != 0) {
		 RCLCPP_WARN(get_node()->get_logger(), "Inside sleepy update");
	   std::mt19937_64 eng{std::random_device{}()};
     std::uniform_int_distribution<> dist{10, 300};
     
     std::this_thread::sleep_for(std::chrono::milliseconds(sleep_base + dist(eng)));
    }
	  
	  auto now = std::chrono::system_clock::now();
	  update_call_date.push_back(std::make_pair(std::chrono::system_clock::to_time_t(now)
	                                          , std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000));

	  if (update_call_date.size() == 300) 
	  {
	    std::ofstream generated_csv;
	    generated_csv.open("/tmp/bar.csv");
	    generated_csv << "sleepy JTC\n";
	    for (auto& element : update_call_date) 
	    {
	      generated_csv << std::put_time(localtime(&element.first), "%H:%M:%S") << '.' << std::setfill('0') << std::setw(3) << element.second.count() << ",\n";
	    }
	    generated_csv.close();
	    //exit(1);
	  }

	  return joint_trajectory_controller::JointTrajectoryController::update(time, period);
	}
	
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_trajectory_controller::SleepyTrajectoryController, controller_interface::ControllerInterface)
