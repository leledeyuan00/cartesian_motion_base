#ifndef CARTESIAN_MOTION_BASE_HPP__
#define CARTESIAN_MOTION_BASE_HPP__

// std
#include <stdlib.h>
#include <sstream>
#include <thread>
#include <cstring>

// ros
#include <rclcpp/rclcpp.hpp>

#include "kdl/frames.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

// custom
#include "cartesian_motion_base/cartesian_motion_tasks.hpp"
#include "cartesian_motion_base/cartesian_motion_type.hpp"

// eigen
#include <eigen3/Eigen/Dense>

namespace cartesian_motion_base
{

/**
 * @brief Base class for robot motion tasks.
 * @param node_name Name of the node.
 * @param robot_configs Vector of robot configurations.
 * @param rate Rate of the task execution.
 */
class CartesianMotionBase : public rclcpp::Node
{

public:
  explicit CartesianMotionBase(
    const std::string & node_name,
    std::vector<RobotConfig> robot_configs = {RobotConfig()},
    uint16_t rate = 125)
  : Node(node_name), robot_configs_(robot_configs), rate_(rate)
  {
    // Initialize the node
    RCLCPP_INFO(this->get_logger(), "CartesianMotionBase initializing...");
  }
  virtual ~CartesianMotionBase() = default;

  void on_init();
  void start();
  void active()
  {
    for (auto & [name, handles] : robots_) {
      handles.motion_state.target_pose = handles.motion_state.target_monitor;
      handles.motion_state.start_pose = handles.motion_state.target_monitor;
    }
    active_ = true;
  }
  void deactivate() {active_ = false;}

protected:
  /* functions */
  void init();
  bool robot_init();
  virtual void custom_init() {}

  /**
   * @brief Interpolation between two poses. Default is linear interpolation for position and slerp for orientation.
   * @param init_pose Initial pose.
   * @param target_pose Target pose.
   * @param duration Duration of the interpolation.
   * @param current_duration Current time in the interpolation.
   * @return Interpolated pose.
   */
  virtual geometry_msgs::msg::PoseStamped intepolation(
    geometry_msgs::msg::PoseStamped init_pose, geometry_msgs::msg::PoseStamped target_pose,
    double duration, double current_duration)
  {
    Eigen::Vector3d p_start = toEigen(init_pose.pose.position);
    Eigen::Vector3d p_target = toEigen(target_pose.pose.position);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = init_pose.header.frame_id;
    pose.pose.position = toMsgPoint(p_start + (p_target - p_start) * current_duration / duration);


    Eigen::Quaterniond init_q, target_q;
    tf2::fromMsg(init_pose.pose.orientation, init_q);
    tf2::fromMsg(target_pose.pose.orientation, target_q);

    Eigen::Quaterniond q = init_q.slerp(current_duration / duration, target_q);
    pose.pose.orientation = tf2::toMsg(q);

    return pose;
  }

  /**
   * @brief Interpolation between two wrenches. Default is linear interpolation.
   * @param init_wrench Initial wrench.
   * @param target_wrench Target wrench.
   * @param duration Duration of the interpolation.
   * @param current_duration Current time in the interpolation.
   * @return Interpolated wrench.
   */
  virtual geometry_msgs::msg::WrenchStamped intepolation_wrench(
    geometry_msgs::msg::WrenchStamped init_wrench, geometry_msgs::msg::WrenchStamped target_wrench,
    double duration, double current_duration)
  {
    Eigen::Vector3d f_start = toEigen(init_wrench.wrench.force);
    Eigen::Vector3d f_target = toEigen(target_wrench.wrench.force);
    Eigen::Vector3d t_start = toEigen(init_wrench.wrench.torque);
    Eigen::Vector3d t_target = toEigen(target_wrench.wrench.torque);

    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header = init_wrench.header;
    wrench.wrench.force = toMsgVec(f_start + (f_target - f_start) * current_duration / duration);
    wrench.wrench.torque = toMsgVec(t_start + (t_target - t_start) * current_duration / duration);

    return wrench;
  }


  // robot state
  /**
   * @brief get robot names
   * @return vector of robot names
   */
  std::vector<std::string> get_robot_names()
  {
    std::vector<std::string> names;
    for (const auto & config : robot_configs_) {
      names.push_back(config.name);
    }
    return names;
  }

  /**
   * @brief get current pose for a robot by name
   * @param robot_name
   * @return current pose
   */
  const geometry_msgs::msg::PoseStamped get_current_pose(std::string name)
  {
    return robots_[name].motion_state.current_pose;
  }

  /**
   * @brief get current wrench for a robot by name
   * @param robot_name
   * @return current wrench
   */
  const geometry_msgs::msg::WrenchStamped get_current_wrench(std::string name)
  {
    return robots_[name].motion_state.current_wrench;
  }

  /**
   * @brief get start pose for a robot by name
   * @param robot_name
   * @return start pose
   */
  const geometry_msgs::msg::PoseStamped get_start_pose(std::string name)
  {
    return robots_[name].motion_state.start_pose;
  }

  /**
   * @brief get start wrench for a robot by name
   * @param robot_name
   * @return start wrench
   */
  const geometry_msgs::msg::WrenchStamped get_start_wrench(std::string name)
  {
    return robots_[name].motion_state.start_wrench;
  }

  /**
   * @brief get current target monitor pose for a robot by name
   * @param robot_name
   * @return target monitor pose
   */
  const geometry_msgs::msg::PoseStamped get_target_monitor(std::string name)
  {
    return robots_[name].motion_state.target_monitor;
  }

  /**
   * @brief get system state
   * @return system state.
   * The system state includes current task number, start time, and current time.
   */
  SystemState get_system_state() {return system_state_;}


  /**
   * @brief set target pose for a robot by name
   * @param robot_name
   * @param target_pose
   */
  inline void set_target_pose(PoseMap target_pose)
  {
    for (auto & [name, pose] : target_pose) {
      robots_[name].motion_state.target_pose = pose;
    }
  }

  /**
   * @brief set target wrench for a robot by name
   * @param robot_name
   * @param target_wrench
   */
  inline void set_target_wrench(WrenchMap target_wrench)
  {
    for (auto & [name, wrench] : target_wrench) {
      robots_[name].motion_state.target_wrench = wrench;
    }
  }

  /* FSM tasks */
  /**
   * @brief from start_pose(initialized when this task running) to target_pose
   * @param target_poses map of robot name and target poses
   * @param duration
   * @return finish or not
   */
  bool move(
    PoseMap target_poses,
    double duration);

  /**
   * @brief from start_pose(initialized when this task running) to target_pose
   * @param target_poses map of robot name and target poses
   * @param target_wrenches map of robot name and target wrenches
   * @param duration
   * @return finish or not
   */
  bool move_wrench(
    PoseMap target_poses,
    WrenchMap target_wrenches,
    double duration);
  /**
   * @brief sleep for a while
   * @param duration
   * @return finish or not
   */
  bool sleep(double duration);

  /**
   * @brief joint move
   * @param target_joints map of robot name and target joint positions
   * @param time
   * @return finish or not
   */
  bool joint_move(
    JointMap target_joints,
    double time);


  /**
   * @brief Push back the task, the task is derived from GarmentTaskBase
   * @param task
   * @return the task number
   */
  uint8_t task_pushback(std::shared_ptr<cartesian_motion_base::MotionTask> task);
  virtual void tasks_init() = 0;
  void task_execute();

  /**
   * @brief Set the task finished, go to the next task
   */
  void set_task_finished()
  {
    next_task_offset_ = 1;
    tasks_vector_[system_state_.task_num]->set_state(TaskState::FINISH);
  }

  /**
   * @brief Go to a specific task number. You can get the task number when you push back the task.
   * @param task_num
   */
  void goto_specific_task(size_t task_num)
  {
    if (task_num >= tasks_vector_.size()) {
      RCLCPP_WARN(
        this->get_logger(), "Task number %zu is out of range, max task number is %zu", task_num,
        tasks_vector_.size() - 1);
      return;
    }
    next_task_offset_ = task_num - system_state_.task_num;
    tasks_vector_[system_state_.task_num]->set_state(TaskState::FINISH);
  }

  /**
   * @brief Go to the initial task
   */
  void goto_init_task()
  {
    goto_specific_task(0);
  }

  /**
   * @brief Go to the last task
   */
  void goto_last_task()
  {
    goto_specific_task(tasks_vector_.size() - 1);
  }

  // get initialized states
  inline bool check_init()
  {
    bool init = true;
    for (const auto & [name, handles] : robots_) {
      init = init && handles.initialized;
    }
    return init;
  }

  // states
  SystemState system_state_;   // current task number, current time, start time
  uint8_t next_task_offset_ = 1;

  // std
  std::thread control_loop_thread_;

  // time
  rclcpp::Clock my_clock_;

  // system state
  // create a vector for TaskWrapper
  std::vector<std::shared_ptr<cartesian_motion_base::MotionTask>> tasks_vector_;


  bool active_ = true;

private:
  /* function */
  void control_loop();
  // ros
  void ros_init();


  /* variable */
  bool model_joint_ = false;

  // robot state
  std::vector<RobotConfig> robot_configs_;
  std::map<std::string, RobotHandles> robots_;

  uint16_t rate_;
  size_t robot_count_ = 0;


}; // class CartesianMotionBase


} // namespace cartesian_motion_base


#endif // CARTESIAN_MOTION_BASE_HPP__
