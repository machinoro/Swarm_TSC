FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Setting up user
ARG UNAME=user
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} -o ${UNAME}
RUN useradd -m -u ${UID} -g ${GID} -o -s /bin/bash ${UNAME}
RUN usermod -aG sudo ${UNAME}
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER ${UNAME}

ENV ROS2_WS=/home/user/ros_ws
RUN mkdir -p ${ROS2_WS}/src

SHELL ["/bin/bash", "-c"]

COPY docker/requirements.txt .
RUN sudo apt-get update && sudo apt-get install -y $(cat requirements.txt)

# COPY docker/nvim_setup.sh /tmp/nvim_setup.sh 
# RUN /tmp/nvim_setup.sh 

RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get update
RUN sudo apt-get install ignition-fortress -y

RUN sudo echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc 
RUN export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:/opt/ros/humble/lib

WORKDIR /home/user/ros_ws/ros2_ws

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
