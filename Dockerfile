FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    git \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-$ROS_DISTRO-gz-ros2-control \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-servo \
    ros-$ROS_DISTRO-moveit-task-constructor-capabilities \
    ros-$ROS_DISTRO-moveit-task-constructor-core \
    ros-$ROS_DISTRO-moveit-task-constructor-msgs \
    ros-$ROS_DISTRO-moveit-task-constructor-visualization \
    ros-$ROS_DISTRO-robotiq-description \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-ur-description \
    ros-$ROS_DISTRO-ur-moveit-config \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /franka

RUN mkdir -p src && \
    git clone https://github.com/frankarobotics/franka_description.git -b jazzy src/franka_description

RUN source /opt/ros/jazzy/setup.bash && colcon build

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /franka/install/setup.bash" >> ~/.bashrc
RUN echo "source /vgdtas/install/setup.bash" >> ~/.bashrc
RUN echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:/vgdtas/install/vgdtas_description/share" >> ~/.bashrc

WORKDIR /vgdtas

CMD ["bash"]