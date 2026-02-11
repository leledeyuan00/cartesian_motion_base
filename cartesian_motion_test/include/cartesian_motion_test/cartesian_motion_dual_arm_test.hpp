#ifndef CARTESIAN_MOTION_DUAL_ARM_TEST_HPP__
#define CARTESIAN_MOTION_DUAL_ARM_TEST_HPP__

// ros
#include <std_srvs/srv/set_bool.hpp>

// custom
#include "cartesian_motion_base/cartesian_motion_base.hpp"
#include "cartesian_motion_test/cartesian_motion_config.hpp"

using namespace cartesian_motion_base;

namespace cartesian_motion_test
{
class MotionDualArmTest : public MotionBase
{
public:
    MotionDualArmTest(const std::string &node_name,
               std::vector<RobotConfig> robot_configs,
               uint16_t rate) :
        MotionBase(node_name, robot_configs, rate){}; // pass to base class

private:
    /* function */
    void custom_init() override; // must override for custom initialization
    //tasks
    void tasks_init() override; // must override for task initialization
    
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