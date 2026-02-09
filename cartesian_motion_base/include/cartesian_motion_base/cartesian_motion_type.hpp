#ifndef CARTESIAN_MOTION_TYPE_HPP__
#define CARTESIAN_MOTION_TYPE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "cartesian_controller_msgs/srv/joint_move.hpp"
#include <kdl/frames.hpp>
#include <eigen3/Eigen/Dense>


#define JointMap std::map<std::string, std::vector<double>>
#define PoseMap std::map<std::string, geometry_msgs::msg::PoseStamped>
#define WrenchMap std::map<std::string, geometry_msgs::msg::WrenchStamped>

namespace cartesian_motion_base
{

// A wrapper class for garment motion base to create a service client
// Template of the client service
template<class ServiceT>
class MotionClient
{
public:
    using RequestSharedPtr = typename ServiceT::Request::SharedPtr;
    using ResponseSharedPtr = typename ServiceT::Response::SharedPtr;
    using SharedFuture = typename rclcpp::Client<ServiceT>::SharedFuture;
    RCLCPP_SMART_PTR_DEFINITIONS(MotionClient)

    MotionClient(const std::string& service_name, std::function<void(ResponseSharedPtr)> response_func, const rclcpp::Node::SharedPtr &node) :
        node_(node), response_func_(response_func)
    {
        client_ = node_->create_client<ServiceT>(service_name);
    }

    // return true if the service is responsed
    void call(RequestSharedPtr request){
        
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(node_->get_logger(), "Service %s not available", client_->get_service_name());
            return;
        }

        if (!client_->service_is_ready()) {
            RCLCPP_WARN(node_->get_logger(), "Service %s not ready", client_->get_service_name());
            return;
        }
            
        auto result = client_->async_send_request(request, [this](const SharedFuture future) {
            auto response = future.get();
            if (response) {
                RCLCPP_INFO(node_->get_logger(), "Service %s called successfully", client_->get_service_name());
                response_func_(response);
                service_finished_ = true; // set the service finished flag
                return;
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Service %s call failed", client_->get_service_name());
            }
        });
        service_called_ = true; // make sure the service is called only once
        return;
    }

    bool get_service_called()
    {
        return service_called_;
    }

    bool get_service_finished()
    {
        return service_finished_;
    }

    void reset()
    {
        service_called_ = false;
        service_finished_ = false;
    }

    typename rclcpp::Client<ServiceT>::SharedPtr get_client()
    {
        return client_;
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool service_called_ = false;
    bool service_finished_ = false;
    
    typename rclcpp::Client<ServiceT>::SharedPtr client_;
    // callback function to handle the request
    std::function<void(ResponseSharedPtr)> response_func_;
};


// struct for Robot Config
struct RobotConfig
{
    std::string name = "ur" ; // robot name
    std::string pose_sub = "/cartesian_compliance_controller/current_pose"; // topic name for current pose subscriber
    std::string cmd_monitor_sub = "/cartesian_compliance_controller/target_frame_monitor"; // topic name for target pose monitor subscriber
    std::string wrench_sub = "/cartesian_compliance_controller/current_wrench"; // topic name for current wrench subscriber
    std::string wrench_pub = "/cartesian_compliance_controller/target_wrench"; // topic name for target wrench publisher
    std::string cmd_pub = "/cartesian_compliance_controller/target_frame"; // topic name for target pose publisher
    std::string joint_service = "/cartesian_compliance_controller/target_joint"; // service name for joint move
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
    geometry_msgs::msg::WrenchStamped start_wrench;
};

// struct for ROS handles
struct ROSHandles
{
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_monitor_sub;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr target_wrench_pub;
    MotionClient<cartesian_controller_msgs::srv::JointMove>::SharedPtr joint_move_client;
};

// struct for robot handles
struct RobotHandles
{
    MotionState motion_state;
    ROSHandles ros_handles;
    bool initialized = false;
};

// enum class for task status
enum class TaskState
{
    INIT = 0,
    EXECUTE,
    FINISH,
};

// struct for System States Machine
struct SystemState
{
    size_t task_num = 0;
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

/**
 * @brief Get default robot configurations
 * @return std::vector<RobotConfig>
 */
std::vector<cartesian_motion_base::RobotConfig> get_default_robot_configs()
{
    std::vector<cartesian_motion_base::RobotConfig> robot_configs;

    cartesian_motion_base::RobotConfig robot;
    robot.name = "ur";
    robot.pose_sub = "/cartesian_compliance_controller/current_pose";
    robot.cmd_monitor_sub = "/cartesian_compliance_controller/target_frame_monitor";
    robot.wrench_sub = "/cartesian_compliance_controller/current_wrench";
    robot.wrench_pub = "/cartesian_compliance_controller/target_wrench";
    robot.cmd_pub = "/cartesian_compliance_controller/target_frame";
    robot.joint_service = "/cartesian_compliance_controller/target_joint";

    robot_configs.push_back(robot);

    return robot_configs;
}

} // namespace cartesian_motion_base


#endif // CARTESIAN_MOTION_TYPE_HPP__