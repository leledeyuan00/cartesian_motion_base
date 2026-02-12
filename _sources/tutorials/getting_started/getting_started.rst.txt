Getting Started
=================

Here, we will setup your environment to use Cartesian Motion Base (CMB) and run a simple example.
CMB is designed to work with `cartesian controllers <https://github.com/leledeyuan00/cartesian_controllers.git>`_ in ROS 2.
But, you can also use other controllers that support Cartesian commands.

1. Installation ROS 2 distribution and tools:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Make sure you have ROS 2 Humble or later installed on your system.

* `Humble Hawksbill <https://docs.ros.org/en/humble/Installation.html>`_
* `Jazzy Jalisco <https://docs.ros.org/en/jazzy/Installation.html>`_

Install rosdep to install system dependencies:

.. code-block:: bash

   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update

Install vcstool:

.. code-block:: bash

   sudo apt install python3-vcstool

Install cyclonedds for ROS 2 communication(if you haven't already):

.. code-block:: bash

   sudo apt install ros-humble-rmw-cyclonedds-cpp
   echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

2. Download the cartesian_motion_base source code:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash
    
    mkdir -p ~/cmb_ws/src
    cd ~/cmb_ws/src
    git clone https://github.com/leledeyuan00/cartesian_motion_base.git
    vcs import < cartesian_motion_base/cartesian_motion_base.repos

3. Build your colcon workspace:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   cd ~/cmb_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install

4. Setup your environment:
^^^^^^^^^^^^^^^^^^^^^^^^^^
Source the workspace:

.. code-block:: bash

   source ~/cmb_ws/install/setup.bash

Optional: add the previous command to your ``~/.bashrc``: ::

   echo "source ~/cmb_ws/install/setup.bash" >> ~/.bashrc

.. note:: Sourcing the ``setup.bash`` automatically in your ``~/.bashrc`` is
   not required and often skipped by advanced users who use more than one
   Colcon workspace at a time, but we recommend it for simplicity.

5. Next Steps
^^^^^^^^^^^^^

Nice job! Next, we will :doc:`visualize a simple Cartesian motion example using Gazebo </tutorials/simulating_robot/simulating_robot>`