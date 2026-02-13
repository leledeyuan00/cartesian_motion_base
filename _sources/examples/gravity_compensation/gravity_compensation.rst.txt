Gravity Compensation
=====================

We implemented a gravity compensation algorithm on cartesian controllers.
Gravity compenstation is important when the robot needs to work in different orientations.

This example shows how to calculate the parameters and how to configure the contollers.

1. Prequisites
^^^^^^^^^^^^^^^

* Make sure you have a F/T sensor mounted on the robot flange, and publish its raw data on ``cartesian_compliance_controller/ft_sensor_wrench`` topic.
* Check topic ``cartesian_compliance_controller/current_wrench`` to see the transformed wrench data is aligned with robot frame.

.. note::
    * The cartesian controller will automatically transform the raw F/T sensor data to the base frame based on the urdf.
    * For dual-arm, please check their topic names.
    * Your robot base frame z axis should be aligned with the gravity direction.

2. Recording the gravity pose and wrench
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We provide a example script to record the data needed for gravity compensation see ``cartesian_motion_tools/gravity_pose_record.py``.

.. note::
    This script will move the robot to 7 different poses and record the end-effector pose and wrench data.
    So please make sure these poses are reachable and safe for your robot, you can check it first on simulation.

2.1 Run the recording script
------------------------------

Open a new terminal and source your workspace:

.. code-block:: bash

   source ~/cmb_ws/install/setup.bash
   ros2 run cartesian_motion_tools gravity_pose_record --sim True  # if you are using simulation

2.2 Calculate the gravity compensation parameters
--------------------------------------------------

.. code-block:: bash

   ros2 run cartesian_motion_tools gravity_compute

It will generates a file named ``gravity_result.txt`` under ``~/.gravity_compensation/`` directory.
Its content looks like:

.. code-block:: yaml

    force: 
    [-12.26039,-9.02906,-2.64165,0.52321] # [gravity on z axis reference to world frame, offset x, offset y, offset z]
    torque: 
    [0.04930,-0.08519,-0.10053,0.23747]   # [center of mass reference to end-effector, offset x, offset y, offset z]

.. note::
    You can find this algorithm on this paper:    
    D. Chen, Y. Zhang, W. He, A. E. Petrilli Barcel´o, J. V. Salazar Luces, and Y. Hirata, “Simplified offset and gravity compensation for wrist-mounted force/torque sensor with a garment manipulation application,” in 2025 IEEE Int. Conf. on Robot. and Biomimetics (ROBIO), 2025

3. Configure the cartesian controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Reference the example controller configuration file: ``cartesian_motion_sim/cartesian_sim_bringup/config/cartesian_gazebo_controllers_sa.yaml``
Make changes to your robot controller configuration file:

.. code:: yaml

    cartesian_compliance_controller:
        ros__parameters:
            ik_solver: "forward_dynamics"
            end_effector_link: "tool0"
            robot_base_link: "robot_base_link"
            ft_sensor_ref_link: "tool0"
            compliance_ref_link: "tool0"
            force_enable: false # Make sure you got the correct F/T sensor data
            gravity_compensation: false  # Set to true to enable gravity compensation
                # Copy your results here
                force: [-12.26039,-9.02906,-2.64165,0.52321]
                torque: [0.04930,-0.08519,-0.10053,0.23747]
            force_state_pub: false # Set to true when use the real robot
            emergency_stop_threshold: 200.0
            joints:
            - shoulder_pan_joint
            - shoulder_lift_joint
            - elbow_joint
            - wrist_1_joint
            - wrist_2_joint
            - wrist_3_joint

            # Choose: position or velocity.
            command_interfaces:
            - position
                #- velocity

            stiffness:  # w.r.t. compliance_ref_link
                trans_x: 500.0
                trans_y: 500.0
                trans_z: 500.0
                rot_x: 100.0
                rot_y: 100.0
                rot_z: 100.0
            
            damping:
                trans_x: 1.0
                trans_y: 1.0
                trans_z: 1.0
                rot_x: 0.0
                rot_y: 0.0
                rot_z: 0.0

            solver:
                forward_dynamics:
                link_mass: 100.0
                error_scale: 0.5
                iterations: 1
                publish_state_feedback: true

            pd_gains:
                trans_x: {p: 0.1, d: 0.005}
                trans_y: {p: 0.1, d: 0.005}
                trans_z: {p: 0.1, d: 0.005}
                rot_x: {p: 0.5, d: 0.0001}
                rot_y: {p: 0.5, d: 0.0001}
                rot_z: {p: 0.5, d: 0.0001}

.. caution::
    For recording:
        When using the real robot, first set ``force_enable`` to false, ``gravity_compensation`` to false, and ``force_state_pub`` to true.
    After computed the gravity compensation:
        Set ``gravity_compensation`` to true to enable it, check the topic ``/cartesian_compliance_controller/current_wrench`` to see if it is working.
        When using the real robot, set ``force_enable`` to true to enable the F/T sensor, and ``force_state_pub`` to true.

.. note::
    For dual-arm robot:
        Please reference the example scripts, then recording the dual-arm pose and wrench to calculate their gravity compensation parameters respectively.
        

4. Application: Dragging the robot by hand
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After the gravity compensation is enabled, you can easily drag the robot even rotating arbitrarily.
For realizing this function, we just need set the target pose as same as the current pose in a loop, due to the compliance controller, the robot has a passive compliance behavior.

.. code:: c++

    // Some other tasks ...

    // Trigger the dragging task by service flag
    task_pushback(TaskPtr("Wait for Service Trigger", [this](){
        if(service_flag_){
            RCLCPP_INFO(this->get_logger(), "Service triggered, proceeding to next task.");
            set_task_finished();
        }
    }));

    // Dragging the robot by hand
    task_pushback(TaskPtr("Drag by hand", [this](){
        geometry_msgs::msg::PoseStamped current_pose = get_current_pose("ur");
        // Create pose map
        PoseMap pose_map;
        pose_map["ur"] = current_pose;
        set_target_pose(pose_map);

        if(!service_flag_){
            // Keep this task until the service flag is false
            set_task_finished();
        }
    }));