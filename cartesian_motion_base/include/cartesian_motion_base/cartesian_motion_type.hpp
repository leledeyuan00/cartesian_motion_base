#ifndef CARTESIAN_MOTION_TYPE_HPP__
#define CARTESIAN_MOTION_TYPE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <kdl/frames.hpp>
#include <eigen3/Eigen/Dense>


namespace cartesian_motion_base
{


// enum class for task status
enum class TaskState
{
    INIT = 0,
    EXECUTE,
    FINISH,
};

// struct for service flags
struct ServiceFlags{
    bool service_called = false;
    bool model_joint = false; // when moving by the joint, stop the cartesian motion
};

// struct for robot motion states
struct MotionState
{
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::PoseStamped target_pose;
    geometry_msgs::msg::PoseStamped target_monitor; // for getting the latest command
    geometry_msgs::msg::PoseStamped start_pose;
    geometry_msgs::msg::WrenchStamped current_wrench;
    geometry_msgs::msg::WrenchStamped target_wrench;
};

// struct for System States Machine
struct SystemState
{
    uint8_t task_num = 0;
    rclcpp::Time start_time;
    rclcpp::Time current_time;
};


/**
 * @brief Convert geometry_msgs::msg::Vector3 to Eigen::Vector3d
 * @param v geometry_msgs::msg::Vector3
 * @return Eigen::Vector3d
 */
inline Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3& v) {
    return Eigen::Vector3d(v.x, v.y, v.z);
}

/**
 * @brief Convert geometry_msgs::msg::Point to Eigen::Vector3d
 * @param v geometry_msgs::msg::Vector3
 * @return Eigen::Vector3d
 */
inline Eigen::Vector3d toEigen(const geometry_msgs::msg::Point& v) {
    return Eigen::Vector3d(v.x, v.y, v.z);
}


/**
 * @brief Convert Eigen::Vector3d to geometry_msgs::msg::Vector3
 * @param v Eigen::Vector3d
 * @return geometry_msgs::msg::Vector3
 */
inline geometry_msgs::msg::Vector3 toMsgVec(const Eigen::Vector3d& v) {
    geometry_msgs::msg::Vector3 msg;
    msg.x = v.x(); msg.y = v.y(); msg.z = v.z();
    return msg;
}

/**
 * @brief Convert Eigen::Vector3d to geometry_msgs::msg::Point
 * @param v Eigen::Vector3d
 * @return geometry_msgs::msg::Point
 */
inline geometry_msgs::msg::Point toMsgPoint(const Eigen::Vector3d& v) {
    geometry_msgs::msg::Point msg;
    msg.x = v.x(); msg.y = v.y(); msg.z = v.z();
    return msg;
}

} // namespace cartesian_motion_base


#endif // CARTESIAN_MOTION_TYPE_HPP__