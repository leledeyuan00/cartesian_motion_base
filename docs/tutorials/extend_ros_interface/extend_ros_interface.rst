Extend ROS Interface on Cartesian Motion Base
===============================================

This tutorial will show how to realize a topic and a service interface on Cartesian Motion Base (CMB).

1. Create a new header file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code:: bash

    cd ~/cmb_ws/src/my_cartesian_motion/include/my_cartesian_motion

Create a ``my_cartesian_interface.hpp`` file.

.. code:: c++

    #ifndef MY_CARTESIAN_INTERFACE_HPP
    #define MY_CARTESIAN_INTERFACE_HPP

    #include "cartesian_motion_base/cartesian_motion_base.hpp"
    #include "my_cartesian_motion/my_cartesian_motion_config.hpp"
    #include "std_msgs/msg/string.hpp"
    #include "std_srvs/srv/set_bool.hpp"

    namespace my_cartesian_motion
    {
    class MyCartesianInterface : public cartesian_motion_base::CartesianMotionBase
    {
    public:
        MyCartesianInterface(const std::string &node_name,
           std::vector<RobotConfig> robot_configs,
           uint16_t rate)
        : cartesian_motion_base::CartesianMotionBase(
              node_name, robot_configs, rate){};

    private:
        void custom_init() override; // must override for custom initialization
        void tasks_init() override; // must override for task initialization

        // ROS interface
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

        bool service_flag_ = false;
    }
    } // namespace my_cartesian_motion
    #endif // MY_CARTESIAN_INTERFACE_HPP

2. Implement the interface in cpp file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code:: bash

    cd ~/cmb_ws/src/my_cartesian_motion/src

Create a ``my_cartesian_interface.cpp`` file.

.. code:: c++

    #include "my_cartesian_motion/my_cartesian_interface.hpp"

    using namespace std::chrono_literals;

    namespace my_cartesian_motion
    {
    void MyCartesianInterface::custom_init()
    {
        // Subscriber example
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "my_topic", 10,
            [this](const std_msgs::msg::String::SharedPtr msg){
                RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
            });

        // Service example
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "my_service",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response){
                service_flag_ = request->data;
                response->success = true;
                response->message = "Trigger flag set to " + std::to_string(service_flag_);;
            });
    }

    void MyCartesianInterface::tasks_init()
    {
        // Go Home by joint space
        task_pushback(TaskPtr("Go Home", [this](){
            std::vector<double> home_joints = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};

            // Create joint map
            JointMap joint_map;
            joint_map["ur"] = home_joints;
            if(joint_move(joint_map, 5.0)){
                set_task_finished();
            }
        }));

        // Wait for service trigger
        task_pushback(TaskPtr("Wait for Service Trigger", [this](){
            if(service_flag_){
                RCLCPP_INFO(this->get_logger(), "Service triggered, proceeding to next task.");
                set_task_finished();
            }
        }));

        // Move up with 0.1m in z direction by cartesian space
        task_pushback(TaskPtr("Move Up", [this](){
            geometry_msgs::msg::PoseStamped current_pose = get_current_pose("ur");
            geometry_msgs::msg::PoseStamped target_pose = current_pose;
            target_pose.pose.position.z += 0.1;
            // Create pose map
            PoseMap pose_map;
            pose_map["ur"] = target_pose;
            if(move(pose_map, 1.0)){
                set_task_finished();
            }
        }));

        // Shut down task
        task_pushback(TaskPtr("shutdown", [this](){
            rclcpp::shutdown();
        }));
    }
    } // namespace my_cartesian_motion

    int main(int argc, char const *argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::executors::SingleThreadedExecutor executor;
        auto robot_configs = my_cartesian_motion::get_test_robot_configs();
        auto my_interface_node = std::make_shared<my_cartesian_motion::MyCartesianInterface>(
            "my_cartesian_interface_node", robot_configs, 125);

        my_interface_node->start();

        executor.add_node(my_interface_node);
        executor.spin();
        rclcpp::shutdown();
        return 0;
    }

3. Build and run the node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1 Modyfy CMakeLists.txt
--------------------------

Add the following lines in ``CMakeLists.txt`` of ``my_cartesian_interface`` package:

.. code:: cmake
    
    add_executable(my_cartesian_interface src/my_cartesian_interface.cpp)
    ament_target_dependencies(my_cartesian_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})
    target_include_directories(my_cartesian_interface PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    install(
        TARGETS 
        my_cartesian_interface
        DESTINATION lib/${PROJECT_NAME}
    )

.. code:: bash

    cd ~/cmb_ws
    colcon build --packages-select my_cartesian_motion
    source ~/cmb_ws/install/setup.bash
    ros2 run my_cartesian_motion my_cartesian_interface

.. note::
    Keep your simulation environment running while you run this node.

Now, you have created a topic subscriber and a service server on Cartesian Motion Base.

Open a new terminal and send a test message to the topic:

.. code-block:: bash

   source ~/cmb_ws/install/setup.bash
   ros2 topic pub /my_topic std_msgs/msg/String "{data: 'Hello from topic!'}"

You should see the message printed in the terminal running the ``my_cartesian_interface`` node.

Then, call the service to set the trigger flag:

.. code-block:: bash

   source ~/cmb_ws/install/setup.bash
   ros2 service call /my_service std_srvs/srv/SetBool "{data: true}"

The robot should move up by 0.1m in the z direction after the service is called.

.. tip::
    You can further extend publisher and client interfaces similarly by following the ROS2 communication patterns.

Congratulations! You have successfully extended the ROS interface on Cartesian Motion Base.