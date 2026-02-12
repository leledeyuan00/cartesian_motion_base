Move Your Robot
=================

In this tutorial, we will show you how to move your robot by joints and cartesian using the API provided by CMB.

1. Prerequisites
^^^^^^^^^^^^^^^^^

Make sure you have completed the :doc:`creating your own motion program tutorial </tutorials/creating_your_own_motion/creating_your_own_motion>` and the :doc:`simulating robot tutorial </tutorials/simulating_robot/simulating_robot>`.

2. Control the robot using Cartesian Motion Base API
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This time, we will edit the source code to move the robot using CMB API.

1. Open the source code file ``cartesian_motion_test/src/cartesian_motion_test.cpp`` in your favorite editor.
2. Add the following code in the ``tasks_init()`` function to add joint space motion and cartesian space motion:

.. code:: c++

    // into tasks_init() function
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

    // Move down with 0.1m in z direction by cartesian space
    task_pushback(TaskPtr("Move Down", [this](){
        geometry_msgs::msg::PoseStamped current_pose = get_current_pose("ur");
        geometry_msgs::msg::PoseStamped target_pose = current_pose;
        target_pose.pose.position.z -= 0.1;
        // Create pose map
        PoseMap pose_map;
        pose_map["ur"] = target_pose;
        if(move(pose_map, 1.0)){
            set_task_finished();
        }
    }));

2.1 Explanation of the code
----------------------------

- First, we provide ``joint_move()`` method to move the robot to a specified joint position.
    This method takes a ``JointMap`` as input, which is a map that associates robot names with their target joint positions.
    In this example, we create a ``JointMap`` called ``joint_map``, and set the target joint positions for the robot named "ur" to ``home_joints``.
    We then call ``joint_move(joint_map, 5.0)`` to move the robot to the target joint positions in 5 seconds.

- Second, we provide ``move()`` method to move the robot to a specified cartesian pose.
    This method takes a ``PoseMap`` as input, which is a map that associates robot names with their target cartesian poses.
    In this example, we create a ``PoseMap`` called ``pose_map``, and set the target cartesian pose for the robot named "ur" to ``target_pose``.
    We then call ``move(pose_map, 1.0)`` to move the robot to the target cartesian pose in 1 second.

- Finally, the ``set_task_finished()`` method is called to indicate that the current task is finished, swithching to the next task in the sequence as a default behavior.
    ``joint_move()`` and ``move()`` methods return true when the motion is completed successfully.

3. Build and run the motion program
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   cd ~/cmb_ws
   colcon build --symlink-install

.. code-block:: bash

   source ~/cmb_ws/install/setup.bash
   ros2 run my_cartesian_motion my_cartesian_motion

4. Next Steps
^^^^^^^^^^^^^^
Next, we will show how to use set_target_pose() method to  :doc:`realize a simple real-time servo control </tutorials/real_time_servo/real_time_servo>`.