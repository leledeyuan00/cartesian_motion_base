Simulation in Gazebo
================================

In this tutorial, we provide two method to simulate a robot in Gazebo. 
You can easily run the simulation directly in :ref:`Docker <using-docker>`, or install Gazebo on your local machine and run the simulation there.

.. _using-docker:

1. Running the Simulation in Docker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1 Install Docker and Nvidia drivers for Docker:
--------------------------------------------------


    * `Docker Installation for Ubuntu <https://docs.docker.com/engine/install/ubuntu/>`_
    * `Nvidia drivers for Docker <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html>`_
    * `Linux Post Installation Steps <https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user>`_

1.2 Launch container and run simulation:
-----------------------------------------

Launch the Docker container with GUI and GPU support:

.. code-block:: bash

   cd ~/cmb_ws/src/cartesian_motion_base/cartesian_motion_sim/
   ./set_display.sh # For display forwarding
   docker compose up -d

Access the Docker container:

.. code-block:: bash

   docker container exec -it cartesian_motion_sim /bin/bash

.. note::
    You need restart the container if you restart your host machine. ``docker container start cartesian_motion_sim``.

.. _launch-simulation:

Launch the simulation:

.. code-block:: bash

   ros2 launch cartesian_sim_bringup cartesian_sim_bringup.launch.py

(or launch the dual-arm simulation)

.. code-block:: bash

   ros2 launch cartesian_sim_bringup cartesian_sim_bringup.launch.py config_type:=dual_arm

.. tip::
    This simulation uses `cartesian_controllers <https://github.com/leledeyuan00/cartesian_controllers.git>`_ to realize a cartesian control.
    You can reference ``cartesian_motion_sim/cartesian_sim_bringup/config/cartesian_gazebo_controllers_*.yaml`` to see how to config this controller.

Based on the Docker configuration, you can also communicate with the docker environment from your host machine.
For example, you can run ``ros2 topic list`` in your host machine terminal to see the topics published in the Docker container.

2. Running the Simulation on Your Local Machine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1 Install Gazebo and its plugins(if you haven't already):
------------------------------------------------------------

.. code:: bash

    sudo apt-get install -y ignition-fortress ros-${ROS_DISTRO}-gazebo-plugins ros-${ROS_DISTRO}-gz-ros2-control ros-${ROS_DISTRO}-ros-gz

Launch the simulation on your local machine (same as in :ref:`Docker <launch-simulation>`):

.. code-block:: bash

   ros2 launch cartesian_sim_bringup cartesian_sim_bringup.launch.py

or launch the dual-arm simulation

.. code-block:: bash

   ros2 launch cartesian_sim_bringup cartesian_sim_bringup.launch.py config_type:=dual_arm

3. Next Setps:
^^^^^^^^^^^^^^^

Nice job! Next, we will try to :doc:`control the robot in simulation using Cartesian Motion Base</tutorials/your_first_motion/your_first_motion>`.
