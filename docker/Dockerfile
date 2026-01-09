FROM osrf/ros:humble-desktop-full 

ENV DEBIAN_FRONTEND=noninteractive

# Setting up user
ARG UNAME=ivsr
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} -o ${UNAME}
RUN useradd -m -u ${UID} -g ${GID} -o -s /bin/bash ${UNAME}
RUN usermod -aG sudo ${UNAME}
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER ${UNAME}

ENV ROS2_WS=/home/${UNAME}/ros_ws
RUN mkdir -p ${ROS2_WS}/src

SHELL ["/bin/bash", "-c"]

COPY docker/requirements.txt .
RUN sudo apt-get update && sudo apt-get install -y $(cat requirements.txt)

COPY docker/nvim_setup.sh /tmp/nvim_setup.sh 
RUN /tmp/nvim_setup.sh 

RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update
RUN sudo apt-get install gz-harmonic -y

RUN sudo echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc 
RUN export GZ_SIM_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:/opt/ros/humble/lib

WORKDIR /home/${UNAME}
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive 
# RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh 
# WORKDIR /home/${UNAME}/PX4-Autopilot/
# RUN make px4_sitl

WORKDIR /home/${UNAME}
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /home/${UNAME}/Micro-XRCE-DDS-Agent/
RUN mkdir build 
# WORKDIR /home/${UNAME}/Micro-XRCE-DDS-Agent/build/
# RUN cmake ..
# RUN make 
# RUN sudo make install 
# RUN sudo ldconfig /usr/local/lib/

WORKDIR /home/${UNAME}/
RUN curl https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage -o QGroundControl-x86_64.AppImage
RUN chmod +x QGroundControl-x86_64.AppImage
RUN ./QGroundControl-x86_64.AppImage --appimage-extract
RUN rm -rf QGroundControl-x86_64.AppImage
RUN echo "alias qgc='./$HOME/squashfs-root/AppRun'" > /home/${UNAME}/.bashrc
RUN source ~/.bashrc

WORKDIR /home/user/ros_ws/ros2_ws
USER ${UNAME}
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
