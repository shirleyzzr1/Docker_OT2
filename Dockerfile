FROM waggle/plugin-base:1.1.1-base
LABEL description="Waggle base image containing ROS2 foxy."
LABEL maintainer="Sage Waggle Team <sage-waggle@sagecontinuum.org>"
LABEL url="https://github.com/waggle-sensor/plugin-base-images/tree/master/base-ros2"

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3
ENV ROS_WS=/root/overlay_ws

# install system packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    openssh-client \
    && rm -rf /var/lib/apt/lists/*

# install ros2 foxy-base
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt-get update \
    && apt-get install -y ros-foxy-ros-base \
    && rm -rf /var/lib/apt/lists/*

# sourcing underlay
RUN echo "source $ROS_ROOT/setup.bash" >> ~/.bashrc

# creating, downloading resource directories ros packages and sourcing an overlay
RUN mkdir -p $ROS_WS/src/demo \
    && mkdir -p $ROS_WS/src/demo_interfaces \
    && mkdir -p /root/config/temp
WORKDIR /root
COPY resources/test_config.yaml /root/test_config.yaml
RUN git clone -b dev-kyle https://github.com/KPHippe/ot2_driver.git \
    && pip3 install -r ot2_driver/requirements.txt \
    && useradd user \
    && chown user:user ot2_driver \
    && mkdir -p /root/
WORKDIR $ROS_WS
COPY demo/ src/demo
COPY demo_interfaces/ src/demo_interfaces

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    build-essential \
    && rm -rf /var/lib/apt/lists/*
SHELL ["/bin/bash", "-c"]
RUN source $ROS_ROOT/setup.bash && colcon build --symlink-install && source $ROS_WS/install/setup.bash
RUN echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc


# Installing network and ping tools
RUN apt-get update && apt-get install --no-install-recommends -y \ 
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

CMD ["bash"]