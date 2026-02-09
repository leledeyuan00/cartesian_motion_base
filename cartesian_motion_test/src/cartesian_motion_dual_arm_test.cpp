#include "cartesian_motion_test/cartesian_motion_dual_arm_test.hpp"

using namespace std::chrono_literals;

namespace cartesian_motion_test
{
// For custom initialization
void MotionDualArmTest::custom_init()
{
    // service client
    test_client_ = this->create_client<std_srvs::srv::SetBool>("test_service");

    // load parameters
    std::vector<double> robot_camera_trans, robot_camera_quat;
    // Declare parameters that could be set on this node
    this->declare_parameter("robot_camera.translation", std::vector<double>{0.0, 0.0, 0.0});
    if (!this->get_parameter("robot_camera.translation", robot_camera_trans))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter robot_camera.translation");
    }
    
    this->declare_parameter("robot_camera.quaternion", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    if (!this->get_parameter("robot_camera.quaternion", robot_camera_quat))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter robot_camera.rotation");
    }

    Eigen::Quaterniond robot_camera_q(robot_camera_quat[0], robot_camera_quat[1], robot_camera_quat[2], robot_camera_quat[3]);
    calibration_matrix_ = Eigen::Isometry3d(
        Eigen::Translation3d(robot_camera_trans[0], robot_camera_trans[1], robot_camera_trans[2]) *
        (robot_camera_q ));
    
    std::cout << "calibration_matrix_r_:" << std::endl << calibration_matrix_.matrix() << std::endl;
}


void MotionDualArmTest::tasks_init()
{

    // A fake service test. Please realize the service in your own node.
    task_pushback(TaskPtr("Go Home", [this](){
        std::vector<double> left_home_joints = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
        std::vector<double> right_home_joints = {3.14, -1.57, -1.57, -1.57, 1.57, 0.0};
        
        // Create joint map
        JointMap joint_map;
        joint_map["left"] = left_home_joints;
        joint_map["right"] = right_home_joints;
        if(joint_move(joint_map, 5.0)){
            set_task_finished();
        }
    }));

    // Move up
    task_pushback(TaskPtr("Move up", [this](){
        geometry_msgs::msg::PoseStamped left_target_pose, right_target_pose;
        left_target_pose = get_start_pose("left");
        right_target_pose = get_start_pose("right");
        left_target_pose.pose.position.z += 0.1;
        right_target_pose.pose.position.z += 0.1;
        // Set the target pose
        PoseMap pose_map;
        pose_map["left"] = left_target_pose;
        pose_map["right"] = right_target_pose;
        if (move(pose_map, 1.0)){
            set_task_finished();
        }
    }));

    // Move forward
    task_pushback(TaskPtr("Move forward", [this](){
        geometry_msgs::msg::PoseStamped left_target_pose, right_target_pose;
        left_target_pose = get_start_pose("left");
        right_target_pose = get_start_pose("right");
        left_target_pose.pose.position.x += 0.1;
        right_target_pose.pose.position.x += 0.1;
        // Set the target pose
        PoseMap pose_map;
        pose_map["left"] = left_target_pose;
        pose_map["right"] = right_target_pose;
        if (move(pose_map, 1.0)){
            set_task_finished();
        }
    }));

    // Move down
    task_pushback(TaskPtr("Move down", [this](){
        geometry_msgs::msg::PoseStamped left_target_pose, right_target_pose;
        left_target_pose = get_start_pose("left");
        right_target_pose = get_start_pose("right");
        left_target_pose.pose.position.z -= 0.1;
        right_target_pose.pose.position.z -= 0.1;
        // Set the target pose
        PoseMap pose_map;
        pose_map["left"] = left_target_pose;
        pose_map["right"] = right_target_pose;
        if (move(pose_map, 1.0)){
            set_task_finished();
        }
    }));

    // Move back
    task_pushback(TaskPtr("Move back", [this](){
        geometry_msgs::msg::PoseStamped left_target_pose, right_target_pose;
        left_target_pose = get_start_pose("left");
        right_target_pose = get_start_pose("right");
        left_target_pose.pose.position.x -= 0.1;
        right_target_pose.pose.position.x -= 0.1;
        // Set the target pose
        PoseMap pose_map;
        pose_map["left"] = left_target_pose;
        pose_map["right"] = right_target_pose;
        if (move(pose_map, 1.0)){
            set_task_finished();
        }
    }));

    // Goto the specific task
    task_pushback(TaskPtr("This is an example for goto specific task", [this](){
        // Goto the task22
        goto_specific_task(test_task_num2_);
    }));

    test_task_num1_ = task_pushback(TaskPtr("Test task num 1", [this](){
        // Goto the task3
        goto_specific_task(test_task_num3_);
    }));

    test_task_num2_ = task_pushback(TaskPtr("Test task num 2", [this](){
        // Goto the task1
        goto_specific_task(test_task_num1_);
    }));

    test_task_num3_ = task_pushback(TaskPtr("Test task num 3", [this](){
        // Goto the next task
        set_task_finished();
    }));
    
    // Sleep for 1 seconds
    task_pushback(TaskPtr("Sleep for 1 seconds", [this](){
        if (sleep(1.0)){
            set_task_finished();
        }
    }));
    
    // Log Test
    task_pushback(TaskPtr("Log Test", [this](){
        RCLCPP_INFO(this->get_logger(), "Log Test");
        auto current_pose_l = get_current_pose("left");
        auto current_pose_r = get_current_pose("right");
        auto system_state = get_system_state();

        RCLCPP_INFO(this->get_logger(), "Current Pose Left is: [%f, %f, %f]", 
            current_pose_l.pose.position.x, current_pose_l.pose.position.y, current_pose_l.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Current Pose Right is: [%f, %f, %f]", 
            current_pose_r.pose.position.x, current_pose_r.pose.position.y, current_pose_r.pose.position.z);

        RCLCPP_INFO(this->get_logger(), "Current system task number is: %zu", system_state.task_num);
        RCLCPP_INFO(this->get_logger(), "Current system start time is: %f", system_state.start_time.seconds());
        RCLCPP_INFO(this->get_logger(), "Current system current time is: %f", system_state.current_time.seconds());

        set_task_finished();
    }));

    // An example for task with init function and sending the target pose command in the loop
    task_pushback(TaskPtr("Custom Test", 
    // init function
    [this](){
        RCLCPP_INFO(this->get_logger(), "This function will be called only once when the task is initialized. You could do some initialization here.");
    },
    // execute function
    [this](){
        geometry_msgs::msg::PoseStamped target_pose_l, target_pose_r;
        target_pose_l = get_start_pose("left");
        target_pose_r = get_start_pose("right");

        double current_duration = (get_system_state().current_time - get_system_state().start_time).seconds();

        target_pose_l.pose.position.z += current_duration * 0.02;
        target_pose_r.pose.position.z += current_duration * 0.02;

        if (current_duration <= 5.0)
        {
            // Set the target pose
            PoseMap pose_map;
            pose_map["left"] = target_pose_l;
            pose_map["right"] = target_pose_r;
            set_target_pose(pose_map);
        }
        else
        {
            set_task_finished();
        }
    }));

    // // Shut down
    task_pushback(TaskPtr("Shut down", [this](){
        //initialize the task
        rclcpp::shutdown();
    }));
}
} // namespace cartesian_motion_test


int main(int argc, char const *argv[])
{
    RCLCPP_INFO(rclcpp::get_logger("cartesian_motion_test"), "Garment Motion Test by FSM Node Started");
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<cartesian_motion_test::MotionDualArmTest>("cartesian_motion_test",
                                             cartesian_motion_test::get_test_dual_robot_configs(),
                                             125); // node name, robot configs, hz

    node->start();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("cartesian_motion_test"), "Garment Motion Test by FSM Node Stopped");
    return 0;
}