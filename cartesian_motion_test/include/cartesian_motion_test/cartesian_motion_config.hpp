#ifndef CARTESIAN_MOTION_CONFIG_HPP__
#define CARTESIAN_MOTION_CONFIG_HPP__

#include "cartesian_motion_base/cartesian_motion_type.hpp"

namespace cartesian_motion_test
{

std::vector<cartesian_motion_base::RobotConfig> get_test_robot_configs()
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

std::vector<cartesian_motion_base::RobotConfig> get_test_dual_robot_configs()
{
    std::vector<cartesian_motion_base::RobotConfig> robot_configs;

    cartesian_motion_base::RobotConfig robot_l;
    robot_l.name = "left";
    robot_l.pose_sub = "/left_cartesian_compliance_controller/current_pose";
    robot_l.cmd_monitor_sub = "/left_cartesian_compliance_controller/target_frame_monitor";
    robot_l.wrench_sub = "/left_cartesian_compliance_controller/current_wrench";
    robot_l.wrench_pub = "/left_cartesian_compliance_controller/target_wrench";
    robot_l.cmd_pub = "/left_cartesian_compliance_controller/target_frame";
    robot_l.joint_service = "/left_cartesian_compliance_controller/target_joint";
    robot_configs.push_back(robot_l);

    cartesian_motion_base::RobotConfig robot_r;
    robot_r.name = "right";
    robot_r.pose_sub = "/right_cartesian_compliance_controller/current_pose";
    robot_r.cmd_monitor_sub = "/right_cartesian_compliance_controller/target_frame_monitor";
    robot_r.wrench_sub = "/right_cartesian_compliance_controller/current_wrench";
    robot_r.wrench_pub = "/right_cartesian_compliance_controller/target_wrench";
    robot_r.cmd_pub = "/right_cartesian_compliance_controller/target_frame";
    robot_r.joint_service = "/right_cartesian_compliance_controller/target_joint";
    robot_configs.push_back(robot_r);

    return robot_configs;
}



} // namespace cartesian_motion_test

#endif // CARTESIAN_MOTION_CONFIG_HPP__