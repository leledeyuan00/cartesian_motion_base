#ifndef CARTESIAN_MOTION_TEST_HPP__
#define CARTESIAN_MOTION_TEST_HPP__

// std
#include <stdlib.h>
#include <sstream>
#include <ctime>
#include <fstream>
#include <thread>

// ros
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "kdl/frames.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

// custom
#include "cartesian_motion_base/cartesian_motion_base.hpp"
#include "cartesian_motion_test/cartesian_motion_config.hpp"

using namespace cartesian_motion_base;

namespace cartesian_motion_test
{
class MotionTest : public MotionBase
{
public:
    MotionTest(const std::string &node_name,
               std::vector<RobotConfig> robot_configs,
               uint16_t rate) :
        MotionBase(node_name, robot_configs, rate){}; // pass to base class

private:
    /* function */
    virtual void custom_init(); // must override for custom initialization
    //tasks
    virtual void tasks_init(); // must override for task initialization
    
    void test_service(bool on);
    /* variable */
    // service client
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr test_client_;

    // load parameters test
    Eigen::Isometry3d calibration_matrix_;

    size_t test_task_num1_;
    size_t test_task_num2_;
    size_t test_task_num3_;
};
}

#endif // CARTESIAN_MOTION_TEST_HPP__