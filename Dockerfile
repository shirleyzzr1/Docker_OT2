FROM kwelbeck/base-ros2-with-empty-overlay:latest

# # Install system packages : python
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     python3.9 \
#     && rm -rf /var/lib/apt/lists/*


# Creating directory for decoded yamls
RUN mkdir -p /root/config/temp

# Downloading ot2_driver, installing dependencies and changing ownership from root
# Could alterntively change permissions instead
WORKDIR /root
RUN git clone -b kojo https://github.com/KPHippe/ot2_driver.git \
    && pip3 install -r ot2_driver/requirements.txt \
    && pip3 install -e ot2_driver \
    && pip install numpy --upgrade \
    && useradd user \
    && chown user:user ot2_driver \
    && mkdir -p /root/

# Downloading ros packages and Creating an overlay
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS/src
COPY ros-packages .
RUN vcs import < repos
WORKDIR $ROS_WS
SHELL ["/bin/bash", "-c"]
RUN source $ROS_ROOT/setup.bash && colcon build --symlink-install && source $ROS_WS/install/setup.bash

#Download image_tools
WORKDIR $ROS_WS/src
RUN git clone -b galactic https://github.com/ros2/demos.git \
    && apt-get update && apt-get -y install libopencv-dev
WORKDIR $ROS_WS
SHELL ["/bin/bash", "-c"]
RUN source $ROS_ROOT/setup.bash && colcon build --symlink-install --packages-select image_tools && source $ROS_WS/install/setup.bash

# On image run, source overlay and launch node
COPY ros_entrypoint.sh /
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ros2 launch demo demo.launch.py
