FROM kwelbeck/base-ros2-with-empty-overlay:latest

# Creating, downloading resource directories, ros packages and sourcing as overlay
RUN mkdir -p /root/config/temp

#downloading ot2_driver package
WORKDIR /root
RUN git clone -b dev-kyle https://github.com/kjwelbeck3/ot2_driver.git \
    && pip3 install -r ot2_driver/requirements.txt \
    && useradd user \
    && chown user:user ot2_driver \
    && mkdir -p /root/

# creating, downloading resource directories ros packages and sourcing an overlay
WORKDIR $ROS_WS
COPY demo src/demo/
RUN git clone -b demo https://github.com/kjwelbeck3/demo_interfaces.git \
    && mv demo_interfaces src

SHELL ["/bin/bash", "-c"]
RUN source $ROS_ROOT/setup.bash && colcon build --symlink-install && source $ROS_WS/install/setup.bash

# Prepping entrypoint, running ros node
COPY ros_entrypoint.sh /
RUN chown user:user /ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]

CMD ["ros2", "run", "demo", "action_server"]
