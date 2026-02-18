#include <gtest/gtest.h>

#include "cartesian_motion_base/cartesian_motion_base.hpp"

const std::vector<cartesian_motion_base::RobotConfig> get_test_robot_config()
{
  std::vector<cartesian_motion_base::RobotConfig> robot_configs;
  cartesian_motion_base::RobotConfig config1;
  config1.name = "ur";
  config1.pose_sub = "/ur_cartesian_compliance_controller/current_pose";
  config1.cmd_monitor_sub = "/ur_cartesian_compliance_controller/target_frame_monitor";
  config1.wrench_sub = "/ur_cartesian_compliance_controller/current_wrench";
  config1.wrench_pub = "/ur_cartesian_compliance_controller/target_wrench";
  config1.cmd_pub = "/ur_cartesian_compliance_controller/target_frame";
  config1.joint_service = "/ur_cartesian_compliance_controller/target_joint";
  robot_configs.push_back(config1);

  cartesian_motion_base::RobotConfig config2;
  config2.name = "franka";
  config2.pose_sub = "/franka_cartesian_compliance_controller/current_pose";
  config2.cmd_monitor_sub = "/franka_cartesian_compliance_controller/target_frame_monitor";
  config2.wrench_sub = "/franka_cartesian_compliance_controller/current_wrench";
  config2.wrench_pub = "/franka_cartesian_compliance_controller/target_wrench";
  config2.cmd_pub = "/franka_cartesian_compliance_controller/target_frame";
  config2.joint_service = "/franka_cartesian_compliance_controller/target_joint";
  robot_configs.push_back(config2);

  return robot_configs;
}

// Define a test motion base class
class TestMotionBase : public cartesian_motion_base::CartesianMotionBase
{
public:
  TestMotionBase(
    const std::string & node_name,
    const std::vector<cartesian_motion_base::RobotConfig> & robot_configs, double rate)
  : cartesian_motion_base::CartesianMotionBase(node_name, robot_configs, rate) {}
  void on_init() override {}
  void tasks_init() override {}

  void test()
  {
    EXPECT_EQ(get_robot_names().size(), 2);
    EXPECT_EQ(get_robot_names()[0], "ur");
    EXPECT_EQ(get_robot_names()[1], "franka");
  }
};

// Test fixture for ROS2 lifecycle management
class MyNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    auto configs = get_test_robot_config();
    node_ = std::make_shared<TestMotionBase>("test_motion_base", configs, 125);
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<TestMotionBase> node_;
};

// Test robot motion base initialization
TEST_F(MyNodeTest, Initialization)
{
  ASSERT_NE(node_, nullptr);
  node_->on_init();
  node_->test();
}
