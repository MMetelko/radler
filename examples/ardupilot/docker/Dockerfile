# Use Ubuntu 20.04 (Focal) base image
FROM ubuntu:focal

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# Prevent user interaction during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Generate locales
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
    
# Update system packages and install necessary tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libbullet-dev \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-setuptools \
    wget

RUN python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest

RUN apt-get install -y --no-install-recommends \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev

RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release

# Install ROS2
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - 
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' 

RUN apt-get update && apt-get install -y ros-foxy-desktop
RUN apt-get install -y ros-foxy-mavros
RUN apt-get install -y \
    ros-foxy-rqt \
    ros-foxy-rqt-common-plugins
RUN apt-get install -y \
    mesa-utils \
    libgl1-mesa-glx
RUN apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Install GeographicLib datasets
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh -O ~/install_geographiclib_datasets.sh
RUN chmod a+x ~/install_geographiclib_datasets.sh
RUN sudo ~/install_geographiclib_datasets.sh
RUN echo "\n# Setup for ROS2" >> ~/.bashrc && \
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "\n# Setup for ROS2" >> ~/.profile && \
    echo "source /opt/ros/foxy/setup.bash" >> ~/.profile

#beepbeep
RUN apt-get update && apt-get install -y \
    default-jdk \
    ant

# Download and extract Log4J 
# Orginally requested version 2.17.2 which is no longer available - moved to 2.24.1 (which is a .zip file)
RUN apt-get install -y unzip
RUN cd ~ && \
    wget https://dlcdn.apache.org/logging/log4j/2.24.1/apache-log4j-2.24.1-bin.zip && \
    unzip apache-log4j-2.24.1-bin.zip

#daikon
RUN apt-get install -y \
    netpbm \
    texinfo
RUN cd ~ && \
    wget http://plse.cs.washington.edu/daikon/history/daikon-5.8.10/daikon-5.8.10.tar.gz && \
    tar zxf daikon-5.8.10.tar.gz && \
    cd daikon-5.8.10 && \
    make rebuild-everything
RUN echo "\n# Setup for BeepBepp/Log4J/Daikon" >> ~/.profile && \
    echo "source ~/radler/examples/ardupilot/vagrant/setup.bash" >> ~/.profile

#ros2 build
RUN pip3 install colcon-common-extensions colcon-mixin
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml 
RUN colcon mixin update
RUN colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml 
RUN colcon metadata update
#sros2
RUN sudo apt install -y libssl-dev

RUN pip3 install tarjan pyyaml pydot
RUN sudo add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && sudo apt-get install -y \
    python3.6 \
    python3.6-distutils
RUN python3.6 -m pip install tarjan

RUN cd ~ && git clone https://github.com/SRI-CSL/radler.git && \
    cd radler && \ 
    git submodule update --init --recursive && \
    git checkout ros2
RUN sudo mv /opt/ros/foxy/share/mavros/launch /opt/ros/foxy/share/mavros/launch.bak
RUN sudo cd ~/radler && \
    cp -r examples/ardupilot/launch /opt/ros/foxy/share/mavros
RUN mkdir -p ~/ros2_ws/src

# From vagrant/setup.bash
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/jvm/java-11-openjdk-amd64/lib/server
ENV DAIKONDIR=~/daikon-5.8.10
RUN source $DAIKONDIR/scripts/daikon.bashrc
ENV CLASSPATH=$CLASSPATH:~/daikon-5.8.10/java/daikon:~/radler/examples/ardupilot/jni:~/beepbeep-3-examples/beepbeep-3-examples.jar:~/daikon-5.8.10/daikon.jar:~/daikon-5.8.10/java/dcomp_rt.jar:~/daikon-5.8.10/java/ChicoryPremain.jar:~/daikon-5.8.10/java/dcomp_premain.jar:~/apache-log4j-2.17.2-bin/log4j-api-2.17.2.jar:~/apache-log4j-2.17.2-bin/log4j-core-2.17.2.jar
ENV PATH=$PATH:~/apache-log4j-2.17.2-bin

# From vagrant sros_env.bash
ENV ROS_SECURITY_KEYSTORE=~/ros2_ws/sros2_keys
ENV ROS_SECURITY_ENABLE=true
ENV ROS_SECURITY_STRATEGY=Enforce
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# setup ros2_ws 
ENV ROS2_WS="/tmp/ros2_ws"
RUN mkdir -p $ROS2_WS/src

# setup entrypoint
ENV ROS2_DISTRO="foxy"
ENV ROS2_PREFIX="/opt/ros"
WORKDIR $ROS2_WS

COPY ./ros2_entrypoint.sh /
ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/bin/bash"]

# MM: What did not convert .... keep for debugging
#===========================
#  config.ssh.forward_x11 = true
      # Make some effort to avoid clock skew
      #vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-threshold", "5000"]
      #vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-start"]
      #vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-on-restore", "1"]
      #vb.name = "DesCert Phase2 ArduPilot (focal)"
      #timesynch
      #sudo apt-get install -y virtualbox-guest-dkms virtualbox-guest-utils

