Real-Time Servo Control
===========================

In manipulator control, real-time servo control is the most basic and important control method, which is widely used in various scenarios. 
And it is also what Cartesian Motion Base (CMB) is designed for.

.. tip::

    Even ``move()`` and ``joint_move()`` are also implemented using real-time servo control under the hood.

1. Move robot in real-time servo mode
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When you need to generate a target pose in real-time and send it to the robot, you can use the ``set_target_pose()`` method provided by CMB.

1. Open the source code file ``cartesian_motion_test/src/cartesian_motion_test.cpp`` in your favorite editor.
2. Add the following code in the ``tasks_init()`` function to add joint space motion and cartesian space motion:


.. code:: c++

    void MyCartesianMotion::tasks_init()
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
        
        // Real-time servo control example
        task_pushback(TaskPtr("Real-time Servo Control", [this](){
            geometry_msgs::msg::PoseStamped start_pose, target_pose;
            start_pose = get_start_pose("ur");
            target_pose = start_pose;

            double move_distance = 0.1; // 0.1 m
            double move_duration = 2.0; // 2 seconds to move forward
            double current_duration = (get_system_state().current_time - get_system_state().start_time).seconds();

            target_pose.pose.position.x = start_pose.pose.position.x + move_distance * (current_duration / move_duration);

            // Task finish condition: move forward for move_duration seconds
            if (current_duration <= move_duration)
            {
                // Set the target pose
                PoseMap pose_map;
                pose_map["ur"] = target_pose;
                set_target_pose(pose_map);
            }
            else
            {
                set_task_finished();
            }
        }));

        // Real-time servo move back
        task_pushback(TaskPtr("Real-time Servo move back", [this](){
            geometry_msgs::msg::PoseStamped start_pose, target_pose;
            start_pose = get_start_pose("ur");
            target_pose = start_pose;

            double move_distance = -0.1; // 0.1 m
            double move_duration = 2.0; // 2 seconds to move back
            double current_duration = (get_system_state().current_time - get_system_state().start_time).seconds();

            target_pose.pose.position.x = start_pose.pose.position.x + move_distance * (current_duration / move_duration);

            // Task finish condition: move back for move_duration seconds
            if (current_duration <= move_duration)
            {
                // Set the target pose
                PoseMap pose_map;
                pose_map["ur"] = target_pose;
                set_target_pose(pose_map);
            }
            else
            {
                set_task_finished();
            }
        }));

        // Shut down task
        task_pushback(TaskPtr("shutdown", [this](){
            rclcpp::shutdown();
        }));
    }

1.1 Explanation of the code
----------------------------

In this example, we generate a target pose in real-time based on the current time.

- First, we get the start pose of the robot using ``get_start_pose()`` method.
    This pose is recorded when the task is switched to this real-time servo control task.

    Other apis like:

    * ``get_current_pose()``: Get the current pose of the robot.
    * ``get_current_wrench()``: Get the current wrench of the robot.
    * ``get_start_wrench()``: Get the start wrench of the robot.
    * ``get_target_monitor()``: Get the target pose, usually for different progress commands synchronization.
    
- Second, we get the current duration from the start of the system using ``get_system_state()`` method.
    It will return a struct that contains the current task_num, start_time and current_time;

    .. code:: c++

        struct SystemState
        {
            size_t task_num; // current executing task number
            rclcpp::Time start_time; // start time will be recorded and fixed when task switched
            rclcpp::Time current_time; // return current time
        };

- Third, we calculate the target pose based on the current duration, send the command using ``set_target_pose()`` method.
    In this example, we move the robot 0.01m in x direction in 2 seconds.

    Other apis like:

    * ``set_target_wrench()``: Set the target wrench of the robot.

1.2 Build and Run the example
------------------------------

Open a new terminal and source your workspace:

.. code-block:: bash

   cd ~/cmb_ws
   colcon build --symlink-install

.. code-block:: bash

   source ~/cmb_ws/install/setup.bash
   ros2 run my_cartesian_motion my_cartesian_motion

.. tip::

    Now pleae trying to realize dual-arm motion by yourself.

2. Next Steps
^^^^^^^^^^^^^^
Next, we will show how to :doc:`extend the ROS interface </tutorials/extend_ros_interface/extend_ros_interface>`.