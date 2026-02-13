#include "cartesian_motion_base/cartesian_motion_base.hpp"

namespace cartesian_motion_base
{

void MotionBase::on_init()
{
  // Parsing configuration
  if (!robot_init()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse configuration");
    return;
  }
  // Initialize the node
  ros_init();
  // Initialize user-defined components
  custom_init();
  tasks_init();
}

bool MotionBase::robot_init()
{
  robot_count_ = robot_configs_.size();   // get robot count

  // Initialize robot states
  for (const auto & config : robot_configs_) {
    RobotHandles handles;
    robots_[config.name] = RobotHandles();
  }

  return true;
}

void MotionBase::ros_init()
{
  // clock
  my_clock_ = rclcpp::Clock(RCL_ROS_TIME);

  // Initialize ROS interfaces for each robot
  for (const auto & config : robot_configs_) {
    RCLCPP_INFO(this->get_logger(), "Robot '%s' initialized", config.name.c_str());
    RobotHandles & robot_handles = robots_[config.name];
    ROSHandles & handles = robots_[config.name].ros_handles;
    MotionState & motion_state = robots_[config.name].motion_state;

    // publishers
    handles.target_pose_pub =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
      config.cmd_pub,
      rclcpp::SystemDefaultsQoS());
    handles.target_wrench_pub =
      this->create_publisher<geometry_msgs::msg::WrenchStamped>(
      config.wrench_pub,
      rclcpp::SystemDefaultsQoS());


    // subscribers
    handles.current_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      config.pose_sub, rclcpp::SystemDefaultsQoS(),
      [this, &robot_handles, &motion_state](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        motion_state.current_pose = *msg;
        if (!robot_handles.initialized) {
          motion_state.target_pose = *msg;
          motion_state.start_pose = *msg;
          system_state_.start_time = my_clock_.now();
          robot_handles.initialized = true;
        }
      });
    handles.target_monitor_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      config.cmd_monitor_sub, rclcpp::SystemDefaultsQoS(),
      [this, &motion_state](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        motion_state.target_monitor = *msg;
      });
    handles.wrench_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      config.wrench_sub, rclcpp::SystemDefaultsQoS(),
      [this, &motion_state](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        motion_state.current_wrench = *msg;
      });

    // services
    handles.joint_move_client =
      std::make_shared<MotionClient<cartesian_controller_msgs::srv::JointMove>>(
      config.joint_service,
      [this](MotionClient<cartesian_controller_msgs::srv::JointMove>::ResponseSharedPtr response) {
        // print result
        RCLCPP_INFO(this->get_logger(), "Joint left send complete");
        RCLCPP_INFO(this->get_logger(), "success: %d", response->success);
        bool success = response->success;
        if (!success) {
          RCLCPP_ERROR(this->get_logger(), "Joint left Service Failed");
          rclcpp::shutdown();
        }
      },
      this->shared_from_this()
      );
  }
}

/* Predefined Functions */

bool MotionBase::move(PoseMap target_poses, double duration)
{
  // Update target poses
  double current_duration = (system_state_.current_time - system_state_.start_time).seconds();
  if (current_duration >= duration) {
    return true;
  }

  // Check target names
  for (auto & [name, pose] : target_poses) {
    if (robots_.find(name) == robots_.end()) {
      RCLCPP_ERROR(this->get_logger(), "Target for robot '%s' not found", name.c_str());
      RCLCPP_ERROR(this->get_logger(), "Available targets are:");
      for (const auto & [robot_name, robot_handles] : robots_) {
        RCLCPP_ERROR(this->get_logger(), " - %s", robot_name.c_str());
      }
      deactivate();
      return true;
    }

    // Interpolate and set target pose
    auto pose_msg = intepolation(
      robots_[name].motion_state.start_pose, pose, duration,
      (system_state_.current_time - system_state_.start_time).seconds());
    set_target_pose(PoseMap{{name, pose_msg}});
  }
  return false;
}

bool MotionBase::move_wrench(
  PoseMap target_poses,
  WrenchMap target_wrenches,
  double duration)
{
  // Update target poses
  double current_duration = (system_state_.current_time - system_state_.start_time).seconds();
  if (current_duration >= duration) {
    return true;
  }

  // Check target names
  for (auto & [name, pose] : target_poses) {
    if (robots_.find(name) == robots_.end() ||
      target_wrenches.find(name) == target_wrenches.end())
    {
      RCLCPP_ERROR(this->get_logger(), "Target for robot '%s' not found", name.c_str());
      RCLCPP_ERROR(this->get_logger(), "Available targets are:");
      for (const auto & [robot_name, robot_handles] : robots_) {
        RCLCPP_ERROR(this->get_logger(), " - %s", robot_name.c_str());
      }
      deactivate();
      return true;
    }

    // Interpolate and set target pose
    auto pose_msg = intepolation(
      robots_[name].motion_state.start_pose, pose, duration,
      (system_state_.current_time - system_state_.start_time).seconds());
    set_target_pose(PoseMap{{name, pose_msg}});

    // Interpolate and set target wrench
    auto wrench_msg = intepolation_wrench(
      robots_[name].motion_state.start_wrench, target_wrenches[name], duration,
      (system_state_.current_time - system_state_.start_time).seconds());
    set_target_wrench(WrenchMap{{name, wrench_msg}});
  }

  return false;
}

bool MotionBase::sleep(double duration)
{
  double current_duration = (system_state_.current_time - system_state_.start_time).seconds();
  if (current_duration >= duration) {
    return true;
  }
  return false;
}

bool MotionBase::joint_move(
  std::map<std::string, std::vector<double>> target_joints,
  double time)
{
  for (const auto & [name, joints] : target_joints) {
    if (robots_.find(name) == robots_.end()) {
      RCLCPP_ERROR(this->get_logger(), "Target for robot '%s' not found", name.c_str());
      RCLCPP_ERROR(this->get_logger(), "Available targets are:");
      for (const auto & [robot_name, robot_handles] : robots_) {
        RCLCPP_ERROR(this->get_logger(), " - %s", robot_name.c_str());
      }
      deactivate();
      return true;
    }

    auto client = robots_[name].ros_handles.joint_move_client;
    if (!client->get_service_called()) {
      auto request = std::make_shared<cartesian_controller_msgs::srv::JointMove::Request>();
      request->cmd.data = joints;
      request->duration = time;
      client->call(request);
    }

    model_joint_ = true;
  }

  double current_duration = (system_state_.current_time - system_state_.start_time).seconds();
  // Finish condition
  if (current_duration >= time + 0.5) {
    for (const auto & [name, joints] : target_joints) {
      auto client = robots_[name].ros_handles.joint_move_client;
      client->reset();         // reset service call flag
      this->active();        // update target pose to avoid sudden jump
      model_joint_ = false;
    }
    return true;
  }
  return false;
}

// Execute the task

void MotionBase::task_execute()
{
  auto current_task = tasks_vector_[system_state_.task_num];
  system_state_.current_time = my_clock_.now();
  // The task state for executing the task
  switch (current_task->get_state()) {
    case TaskState::INIT:
      {
        RCLCPP_INFO(
          this->get_logger(), "Init task [%zu]: %s", system_state_.task_num,
          current_task->get_name().c_str());

        for (auto & [name, handles] : robots_) {
          handles.motion_state.start_pose = handles.motion_state.target_monitor;
          handles.motion_state.start_wrench = handles.motion_state.target_wrench;
        }

        system_state_.start_time = my_clock_.now();

        current_task->init();

        current_task->set_state(TaskState::EXECUTE);
        break;
      }

    case TaskState::EXECUTE:
      {
        current_task->execute();
        break;
      }

    case TaskState::FINISH:
      {

        current_task->set_state(TaskState::INIT);
        RCLCPP_INFO(
          this->get_logger(), "The task [%zu]: %s finished", system_state_.task_num,
          current_task->get_name().c_str());
        // Check if the task is the last task
        if (system_state_.task_num == tasks_vector_.size() - 1) {
          RCLCPP_INFO(this->get_logger(), "All tasks finished");
          active_ = false;
          return;
        }
        system_state_.task_num = system_state_.task_num + next_task_offset_;
        break;
      }

    default:
      break;
  }
}

uint8_t MotionBase::task_pushback(std::shared_ptr<cartesian_motion_base::MotionTask> task)
{
  tasks_vector_.push_back(task);
  return tasks_vector_.size() - 1;
}

// Start the control loop
void MotionBase::start()
{
  on_init();
  control_loop_thread_ = std::thread(&MotionBase::control_loop, this);
}

void MotionBase::control_loop()
{
  rclcpp::Rate rate(rate_);
  RCLCPP_INFO(this->get_logger(), "Waiting for initialization");
  while (rclcpp::ok()) {
    if (check_init()) {
      task_execute();

      if (!model_joint_ && active_) {
        for (auto & [name, robot] : robots_) {
          robot.motion_state.target_pose.header.stamp = my_clock_.now();
          robot.motion_state.target_wrench.header.stamp = my_clock_.now();
          robot.ros_handles.target_pose_pub->publish(robot.motion_state.target_pose);
          robot.ros_handles.target_wrench_pub->publish(robot.motion_state.target_wrench);
        }
      }
    }
    rate.sleep();
  }
}

}  // namespace cartesian_motion_base
