FROM osrf/ros:humble-desktop-full

# Install necessary packages
RUN apt-get update && apt-get install -y lsb-release wget gnupg vim

# Install Gazebo and ros-gz packages
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
apt-get update && \
apt-get install -y ignition-fortress ros-humble-gz-ros2-control ros-humble-ros-gz ros-humble-ur-description

# Install CycloneDDS
RUN apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp -y

# Create workspace and copy source files
RUN mkdir -p /root/cmb_ws/src
WORKDIR /root/cmb_ws/src
COPY . /root/cmb_ws/src/cartesian_motion_sim

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \ 
    cd /root/cmb_ws/src/cartesian_motion_sim && \
    rm COLCON_IGNORE 2>/dev/null || true && \
    cd /root/cmb_ws/src/ && \
    git clone https://github.com/leledeyuan00/cartesian_controllers.git -b garment && \
    cd /root/cmb_ws && \
    colcon build --symlink-install"

# ADD ROS2 Environment Config
COPY ROS2_config /root/ROS2_config
RUN touch /root/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ROS2_config/ROS2_environment.bash" >> /root/.bashrc && \
    echo "source /root/cmb_ws/install/setup.bash" >> /root/.bashrc