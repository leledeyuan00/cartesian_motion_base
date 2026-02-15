Trying our Example Motion
===========================

In this tutorial, we will give you a quick start on using or provided example to moving the robot in simulation.

1. Prerequisites
^^^^^^^^^^^^^^^^^
Make sure you have completed the :doc:`getting started tutorial </tutorials/getting_started/getting_started>` and the :doc:`simulating robot tutorial </tutorials/simulating_robot/simulating_robot>`.
And the simulation is running.

2. Control the robot using Cartesian Motion Base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open a new terminal and source your workspace:

.. code-block:: bash

   source ~/cmb_ws/install/setup.bash

Run the example to move the robot and see how the task sequence is running:

.. code-block:: bash

    ros2 launch cartesian_motion_test cartesian_motion_test.launch.py

.. tip::
    For dual-arm robot simulation:

    .. code-block:: bash

        ros2 launch cartesian_motion_test cartesian_motion_dual_arm_test.launch.py

You should see the robot moving in the Gazebo as a point to point sequence. Then the terminal will output the informations of the each motion.

3. Quickly explaination of the example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can see the source code of this example in ``cartesian_motion_test/src/cartesian_motion_test.cpp`` 
with its hpp file in ``cartesian_motion_test/include/cartesian_motion_test/cartesian_motion_test.hpp``,
as well as the configuration file in ``cartesian_motion_test/include/cartesian_motion_config.hpp``.

1. First, in the configuration file, we define the parameters of the robot for CMB. Including the robot name, the topics and client names for cartesian controllers.
2. Second, in the hpp file, we define a class ``CartesianMotionTest`` inherited from ``CartesianMotionBase``.
   In the constructor, we initialize the CMB with the parameters defined in the configuration file. 
   And there are two functions that we must override two functions: ``custom_init`` and ``tasks_init``.
3. Third, in the cpp file, we implement the two functions.
   In ``custom_init``, we can add any customized initialization. In this example, we create a simple client and a ROS parameters parsing from a yaml file.
   In ``tasks_init``, we define a sequence of tasks for the robot to execute. In this example, we define 4 tasks using predefined intepolation method ``move()``, 
   showing how to jump from each tasks, how to get current robot and system status and showing a simple real-time servo control intepolation task using ``set_target_pose()`` method.


4. Next Steps
^^^^^^^^^^^^^

Next, we will step by step create your own motion program and learn more about the API provided by CMB in :doc:`Creating Your Own Motion Program </tutorials/creating_your_own_motion/creating_your_own_motion>`.