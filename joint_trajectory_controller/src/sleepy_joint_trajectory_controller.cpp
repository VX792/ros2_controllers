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
	  //std::mt19937_64 eng{std::random_device{}()};  // or seed however you want
    //  std::uniform_int_distribution<> dist{10, 100};
      std::this_thread::sleep_for(std::chrono::milliseconds{500}); //dist(eng)}
	  
	  auto now = std::chrono::system_clock::now();
	  update_call_date.push_back(std::make_pair(std::chrono::system_clock::to_time_t(now)
	                                          , std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000));

	  if (update_call_date.size() == 300) 
	  {
	    std::ofstream generated_csv;
	    generated_csv.open("/home/vx792/bar.csv");
	    generated_csv << "sleepy JTC\n";
	    for (auto& element : update_call_date) 
	    {
	      generated_csv << std::put_time(localtime(&element.first), "%H:%M:%S") << '.' << std::setfill('0') << std::setw(3) << element.second.count() << ",\n";
	    }
	    generated_csv.close();
	    //exit(1);
	  }
	  
	  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
	  {
	    return controller_interface::return_type::OK;
	  }

	  auto compute_error_for_joint = [&](
	                                   JointTrajectoryPoint & error, int index,
	                                   const JointTrajectoryPoint & current,
	                                   const JointTrajectoryPoint & desired) {
	    // error defined as the difference between current and desired
	    error.positions[index] =
	      angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
	    if (has_velocity_state_interface_ && has_velocity_command_interface_)
	    {
	      error.velocities[index] = desired.velocities[index] - current.velocities[index];
	    }
	    if (has_acceleration_state_interface_ && has_acceleration_command_interface_)
	    {
	      error.accelerations[index] = desired.accelerations[index] - current.accelerations[index];
	    }
	  };

	  // Check if a new external message has been received from nonRT threads
	  auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
	  auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
	  if (current_external_msg != *new_external_msg)
	  {
	    fill_partial_goal(*new_external_msg);
	    sort_to_local_joint_order(*new_external_msg);
	    // TODO(denis): Add here integration of position and velocity
	    traj_external_point_ptr_->update(*new_external_msg);
	  }

	  // TODO(anyone): can I here also use const on joint_interface since the reference_wrapper is not
	  // changed, but its value only?
	  auto assign_interface_from_point =
	    [&](auto & joint_interface, const std::vector<double> & trajectory_point_interface) {
	      for (size_t index = 0; index < dof_; ++index)
	      {
	        joint_interface[index].get().set_value(trajectory_point_interface[index]);
	      }
	    };

	  // current state update
	  state_current_.time_from_start.set__sec(0);
	  read_state_from_hardware(state_current_);

	  // currently carrying out a trajectory
	  if (traj_point_active_ptr_ && (*traj_point_active_ptr_)->has_trajectory_msg())
	  {
	    bool first_sample = false;
	    // if sampling the first time, set the point before you sample
	    if (!(*traj_point_active_ptr_)->is_sampled_already())
	    {
	      first_sample = true;
	      if (open_loop_control_)
	      {
	        (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, last_commanded_state_);
	      }
	      else
	      {
	        (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, state_current_);
	      }
	    }

	    // find segment for current timestamp
	    TrajectoryPointConstIter start_segment_itr, end_segment_itr;
	    const bool valid_point =
	      (*traj_point_active_ptr_)
	        ->sample(time, interpolation_method_, state_desired_, start_segment_itr, end_segment_itr);

	    if (valid_point)
	    {
	      bool tolerance_violated_while_moving = false;
	      bool outside_goal_tolerance = false;
	      bool within_goal_time = true;
	      double time_difference = 0.0;
	      const bool before_last_point = end_segment_itr != (*traj_point_active_ptr_)->end();

	      // Check state/goal tolerance
	      for (size_t index = 0; index < dof_; ++index)
	      {
	        compute_error_for_joint(state_error_, index, state_current_, state_desired_);

	        // Always check the state tolerance on the first sample in case the first sample
	        // is the last point
	        if (
	          (before_last_point || first_sample) &&
	          !check_state_tolerance_per_joint(
	            state_error_, index, default_tolerances_.state_tolerance[index], false))
	        {
	          tolerance_violated_while_moving = true;
	        }
	        // past the final point, check that we end up inside goal tolerance
	        if (
	          !before_last_point &&
	          !check_state_tolerance_per_joint(
	            state_error_, index, default_tolerances_.goal_state_tolerance[index], false))
	        {
	          outside_goal_tolerance = true;

	          if (default_tolerances_.goal_time_tolerance != 0.0)
	          {
	            // if we exceed goal_time_tolerance set it to aborted
	            const rclcpp::Time traj_start = (*traj_point_active_ptr_)->get_trajectory_start_time();
	            const rclcpp::Time traj_end = traj_start + start_segment_itr->time_from_start;

	            time_difference = get_node()->now().seconds() - traj_end.seconds();

	            if (time_difference > default_tolerances_.goal_time_tolerance)
	            {
	              within_goal_time = false;
	            }
	          }
	        }
	      }

	      // set values for next hardware write() if tolerance is met
	      if (!tolerance_violated_while_moving && within_goal_time)
	      {
	        if (use_closed_loop_pid_adapter_)
	        {
	          // Update PIDs
	          for (auto i = 0ul; i < dof_; ++i)
	          {
	            tmp_command_[i] = (state_desired_.velocities[i] * ff_velocity_scale_[i]) +
	                              pids_[i]->computeCommand(
	                                state_desired_.positions[i] - state_current_.positions[i],
	                                state_desired_.velocities[i] - state_current_.velocities[i],
	                                (uint64_t)period.nanoseconds());
	          }
	        }

	        // set values for next hardware write()
	        if (has_position_command_interface_)
	        {
	          assign_interface_from_point(joint_command_interface_[0], state_desired_.positions);
	        }
	        if (has_velocity_command_interface_)
	        {
	          if (use_closed_loop_pid_adapter_)
	          {
	            assign_interface_from_point(joint_command_interface_[1], tmp_command_);
	          }
	          else
	          {
	            assign_interface_from_point(joint_command_interface_[1], state_desired_.velocities);
	          }
	        }
	        if (has_acceleration_command_interface_)
	        {
	          assign_interface_from_point(joint_command_interface_[2], state_desired_.accelerations);
	        }
	        if (has_effort_command_interface_)
	        {
	          if (use_closed_loop_pid_adapter_)
	          {
	            assign_interface_from_point(joint_command_interface_[3], tmp_command_);
	          }
	          else
	          {
	            assign_interface_from_point(joint_command_interface_[3], state_desired_.effort);
	          }
	        }

	        // store the previous command. Used in open-loop control mode
	        last_commanded_state_ = state_desired_;
	      }

	      const auto active_goal = *rt_active_goal_.readFromRT();
	      if (active_goal)
	      {
	        // send feedback
	        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
	        feedback->header.stamp = time;
	        feedback->joint_names = joint_names_;

	        feedback->actual = state_current_;
	        feedback->desired = state_desired_;
	        feedback->error = state_error_;
	        active_goal->setFeedback(feedback);

	        // check abort
	        if (tolerance_violated_while_moving)
	        {
	          set_hold_position();
	          auto result = std::make_shared<FollowJTrajAction::Result>();

	          RCLCPP_WARN(get_node()->get_logger(), "Aborted due to state tolerance violation");
	          result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
	          active_goal->setAborted(result);
	          // TODO(matthew-reynolds): Need a lock-free write here
	          // See https://github.com/ros-controls/ros2_controllers/issues/168
	          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

	          // check goal tolerance
	        }
	        else if (!before_last_point)
	        {
	          if (!outside_goal_tolerance)
	          {
	            auto res = std::make_shared<FollowJTrajAction::Result>();
	            res->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
	            active_goal->setSucceeded(res);
	            // TODO(matthew-reynolds): Need a lock-free write here
	            // See https://github.com/ros-controls/ros2_controllers/issues/168
	            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

	            RCLCPP_INFO(get_node()->get_logger(), "Goal reached, success!");
	          }
	          else if (!within_goal_time)
	          {
	            set_hold_position();
	            auto result = std::make_shared<FollowJTrajAction::Result>();
	            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
	            active_goal->setAborted(result);
	            // TODO(matthew-reynolds): Need a lock-free write here
	            // See https://github.com/ros-controls/ros2_controllers/issues/168
	            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
	            RCLCPP_WARN(
	              get_node()->get_logger(), "Aborted due goal_time_tolerance exceeding by %f seconds",
	              time_difference);
	          }
	          // else, run another cycle while waiting for outside_goal_tolerance
	          // to be satisfied or violated within the goal_time_tolerance
	        }
	      }
	    }
	  }

	  publish_state(state_desired_, state_current_, state_error_);

	  return controller_interface::return_type::OK;
	}
	
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_trajectory_controller::SleepyTrajectoryController, controller_interface::ControllerInterface)
