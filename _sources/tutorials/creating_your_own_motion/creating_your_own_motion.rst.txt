Creating Your Own Motion
===========================

In this tutorial, we will create a new ros2 package to implement the motion program.

1 Crate a new package
^^^^^^^^^^^^^^^^^^^^^

.. code:: bash

   cd ~/cmb_ws/src
   ros2 pkg create --build-type ament_cmake my_cartesian_motion --dependencies rclcpp cartesian_motion_base cartesian_controller_msgs geometry_msgs 

2 Writing the configuration file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code:: bash

    cd ~/cmb_ws/src/my_cartesian_motion/include/my_cartesian_motion

Create a ``my_cartesian_motion_config.hpp`` file. In this configuration file, we will define the parameters of the robot for CMB.
Here is an example configuration for a single-arm robot:

.. code:: c++

    #ifndef MY_CARTESIAN_MOTION_CONFIG_HPP
    #define MY_CARTESIAN_MOTION_CONFIG_HPP

    #include "cartesian_motion_base/cartesian_motion_type.hpp"

    namespace my_cartesian_motion
    {
    std::vector<cartesian_motion_base::RobotConfig> get_test_robot_configs()
    {
        std::vector<cartesian_motion_base::RobotConfig> robot_configs;

        cartesian_motion_base::RobotConfig robot;
        robot.name = "ur"; // robot name which used for CMB API
        robot.pose_sub = "/cartesian_compliance_controller/current_pose"; // topic to subscribe current end-effector pose
        robot.cmd_monitor_sub = "/cartesian_compliance_controller/target_frame_monitor"; // topic to subscribe the target pose monitor 
        robot.wrench_sub = "/cartesian_compliance_controller/current_wrench"; // topic to subscribe current end-effector wrench
        robot.wrench_pub = "/cartesian_compliance_controller/target_wrench"; // topic to publish target end-effector wrench
        robot.cmd_pub = "/cartesian_compliance_controller/target_frame"; // topic to publish target end-effector pose
        robot.joint_service = "/cartesian_compliance_controller/target_joint"; // service to set target joint positions

        robot_configs.push_back(robot);

        return robot_configs;
    }

    } // namespace my_cartesian_motion

    #endif // MY_CARTESIAN_MOTION_CONFIG_HPP

2.1 Examine the code
---------------------

The top of the code include the ``cartesian_motion_type.hpp`` file that defines the ``RobotConfig`` struct.

Then, we define a function ``get_test_robot_configs()`` that returns a vector of ``RobotConfig``.

The ``RobotConfig`` struct contains CMB needed parameters, the CMB class needs a vector of ``RobotConfig`` to initialize itself.
You can define multiple robots by push_back multiple ``RobotConfig`` into the vector.

3 Writing the motion program
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1 Writing the header file
----------------------------

.. code:: bash

    cd ~/cmb_ws/src/my_cartesian_motion/include/my_cartesian_motion

Create a ``my_cartesian_motion.hpp`` file. 

.. code:: c++

    #ifndef MY_CARTESIAN_MOTION_HPP
    #define MY_CARTESIAN_MOTION_HPP

    #include "cartesian_motion_base/cartesian_motion_base.hpp"
    #include "my_cartesian_motion/my_cartesian_motion_config.hpp"

    namespace my_cartesian_motion
    {
    class MyCartesianMotion : public cartesian_motion_base::CartesianMotionBase
    {
    public:
        MyCartesianMotion(const std::string &node_name,
               std::vector<RobotConfig> robot_configs,
               uint16_t rate)
            : cartesian_motion_base::CartesianMotionBase(
                  node_name, robot_configs, rate){};
    
    private:
        void custom_init() override; // must override for custom initialization
        void tasks_init() override; // must override for task initialization
    };
    } // namespace my_cartesian_motion
    #endif // MY_CARTESIAN_MOTION_HPP

3.2 Examine the hpp file
----------------------------

The CartesianMotionBase class needs to be inherited to create a new motion class with ``node_name``, ``robot_configs``, and ``rate`` parameters in the constructor.

.. code::c++

    MyCartesianMotion(const std::string &node_name,
           std::vector<RobotConfig> robot_configs,
           uint16_t rate)
        : cartesian_motion_base::CartesianMotionBase(
              node_name, robot_configs, rate){};

Then, we must override two functions: ``custom_init()`` and ``tasks_init()`` to implement our motion program.
These two functions will initialize the extra ros components and the task sequence, respectively.
They will be called when you call the ``start()`` method of the CMB class.

.. code::c++

    void custom_init() override;
    void tasks_init() override;

3.3 Writing the cpp file
----------------------------

.. code:: bash

    cd ~/cmb_ws/src/my_cartesian_motion/src

Create a ``my_cartesian_motion.cpp`` file.

.. code:: c++

    #include "my_cartesian_motion/my_cartesian_motion.hpp"

    using namespace std::chrono_literals;

    namespace my_cartesian_motion
    {
    void MyCartesianMotion::custom_init()
    {
        // Custom initialization code here
        // This time we leave it empty
    }

    void MyCartesianMotion::tasks_init()
    {
        // Define the task sequence here. The tasks will be executed in order.
        // Here we create a log task just show how to create a task.
        task_pushback(TaskPtr("log_task", [this](){
            RCLCPP_INFO(this->get_logger(), "Log Test");
            auto current_pose = get_current_pose("ur");
            auto system_state = get_system_state();

            RCLCPP_INFO(this->get_logger(), "Current Pose is: [%f, %f, %f]", 
                current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

            RCLCPP_INFO(this->get_logger(), "Current system task number is: %zu", system_state.task_num);
            RCLCPP_INFO(this->get_logger(), "Current system start time is: %f", system_state.start_time.seconds());
            RCLCPP_INFO(this->get_logger(), "Current system current time is: %f", system_state.current_time.seconds());
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
        auto my_motion_node = std::make_shared<my_cartesian_motion::MyCartesianMotion>(
            "my_cartesian_motion_node", robot_configs, 125);

        my_motion_node->start();

        executor.add_node(my_motion_node);
        executor.spin();
        rclcpp::shutdown();
        return 0;
    }

3.4 Examine the cpp file
-------------------------------

In the ``custom_init()`` function, we can add any customized initialization. In this example, we leave it empty.

In the ``tasks_init()`` function, we define a sequence of tasks for the robot to execute.

The tasks are defined using the predefined ``task_pushback()`` method, which takes a ``TaskPtr`` as input.
``TaskPtr`` is a smart pointer that points to a task.

.. code::c++

    task_pushback(TaskPtr("TASK NAME", 
    // lambda function that defines the task
    [this](){
        // task code here
    }));

    // or you could also defined a TASK with initialization and loop function
    task_pushback(TaskPtr("TASK NAME",
    // initialization function
    [this](){
        // initialization code here
    },
    // loop function
    [this](){
        // loop code here
    }));

In this example, we create a log task that prints the current end-effector pose and system state to the console.
We provide APIs like ``get_current_pose()`` and ``get_system_state()`` to get the robot's current state.

Finally, we add a shutdown task to safely shut down the node.

4 Build and run the motion program
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.1 Modify the CMakeLists.txt
-------------------------------

Edit the ``CMakeLists.txt`` file in the ``my_cartesian_motion`` package to include the new source files.

.. code:: cmake

    cmake_minimum_required(VERSION 3.8)
    project(my_cartesian_motion)

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    # Find dependencies
    find_package(ament_cmake REQUIRED)
    set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        cartesian_motion_base
        cartesian_controller_msgs
        geometry_msgs
    )

    foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
        find_package(${Dependency} REQUIRED)
    endforeach()

    # Add executable
    add_executable(my_cartesian_motion src/my_cartesian_motion.cpp)
    ament_target_dependencies(my_cartesian_motion ${THIS_PACKAGE_INCLUDE_DEPENDS})
    target_include_directories(my_cartesian_motion PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )

    install(TARGETS
      my_cartesian_motion
      DESTINATION lib/${PROJECT_NAME}
    )

    ament_package()

4.2 Build the package
----------------------

.. code:: bash

   cd ~/cmb_ws
   colcon build --symlink-install

4.3 Run the motion program
----------------------------

Make sure running the simulation in Gazebo as shown in :doc:`Simulation in Gazebo </tutorials/simulating_robot/simulating_robot>`.

.. code:: bash

   source ~/cmb_ws/install/setup.bash
   ros2 run my_cartesian_motion my_cartesian_motion

You will see the log messages printed in the console.

5 Next Steps
^^^^^^^^^^^^^
Next, we will try to :doc:`move the robot using Cartesian Motion Base </tutorials/move_your_robot/move_your_robot>`.