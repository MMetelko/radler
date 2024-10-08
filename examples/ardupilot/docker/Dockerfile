# Use Ubuntu 20.04 (Focal) base image
FROM ubuntu:focal
WORKDIR /ardupilot

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata 

# Prevent user interaction during package installation
ARG DEBIAN_FRONTEND=noninteractive
# From https://github.com/ArduPilot/ardupilot/blob/master/Dockerfile
ARG USER_NAME=ardupilot
ARG USER_UID=1000
ARG USER_GID=1000
ARG SKIP_AP_EXT_ENV=0
ARG SKIP_AP_GRAPHIC_ENV=1
ARG SKIP_AP_COV_ENV=1
ARG SKIP_AP_GIT_CHECK=1
ARG DO_AP_STM_ENV=1

RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash

RUN apt-get update && apt-get install --no-install-recommends -y \
    lsb-release \
    sudo \
    tzdata \
    bash-completion

COPY Tools/environment_install/install-prereqs-ubuntu.sh /ardupilot/Tools/environment_install/
COPY Tools/completion /ardupilot/Tools/completion/

ENV USER=${USER_NAME}

# Create non root user for pip
RUN echo "ardupilot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}
RUN chmod 0440 /etc/sudoers.d/${USER_NAME}

RUN chown -R ${USER_NAME}:${USER_NAME} /${USER_NAME}

USER ${USER_NAME}

# Check that local/bin are in PATH for pip --user installed package
RUN echo "if [ -d \"\$HOME/.local/bin\" ] ; then\nPATH=\"\$HOME/.local/bin:\$PATH\"\nfi" >> ~/.ardupilot_env

# Set the buildlogs directory into /tmp as other directory aren't accessible
ENV BUILDLOGS=/tmp/buildlogs

# Converted from examples/ardupilot/vagrant
# Generate locales
RUN sudo apt-get update && sudo apt-get install -y locales
RUN sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
    
# Update system packages and install necessary tools
RUN sudo apt-get update && sudo apt-get install -y \
    binutils-dev \
    build-essential \
    cmake \
    git \
    libbullet-dev \
    nano \
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

RUN sudo apt-get install -y --no-install-recommends \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev

RUN sudo apt-get update && sudo apt-get install -y \
    curl \
    gnupg2 \
    lsb-release

RUN SKIP_AP_EXT_ENV=$SKIP_AP_EXT_ENV SKIP_AP_GRAPHIC_ENV=$SKIP_AP_GRAPHIC_ENV SKIP_AP_COV_ENV=$SKIP_AP_COV_ENV SKIP_AP_GIT_CHECK=$SKIP_AP_GIT_CHECK \
    DO_AP_STM_ENV=$DO_AP_STM_ENV \
    AP_DOCKER_BUILD=1 \
    USER=${USER_NAME} \
    Tools/environment_install/install-prereqs-ubuntu.sh -y

RUN python3 -m pip install --upgrade pip setuptools wheel packaging importlib-metadata
RUN pip3 install --user -U empy==3.3.4

# Install ROS2
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - 
RUN sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' 

RUN sudo apt-get update && sudo -E apt-get install -y ros-foxy-desktop
RUN sudo apt-get install -y ros-foxy-mavros
RUN sudo apt-get install -y \
    ros-foxy-rqt \
    ros-foxy-rqt-common-plugins
RUN sudo apt-get install -y \
    mesa-utils \
    libgl1-mesa-glx
RUN sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Install GeographicLib datasets
RUN sudo wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh -O /root/install_geographiclib_datasets.sh
RUN sudo chmod a+x /root/install_geographiclib_datasets.sh
RUN sudo -E /root/install_geographiclib_datasets.sh

#beepbeep
RUN sudo apt-get update && sudo apt-get install -y \
    default-jdk \
    ant

# Download and extract Log4J 
# Orginally requested version 2.17.2 which is no longer available - moved to 2.24.1 (which is a .zip file)
RUN sudo apt-get install -y unzip
ENV APACHELOG4J_VERSION=2.24.1
ENV APACHELOG4J=apache-log4j-$APACHELOG4J_VERSION-bin
ENV APACHELOG4JDIR=/root/$APACHELOG4J
RUN wget https://dlcdn.apache.org/logging/log4j/$APACHELOG4J_VERSION/$APACHELOG4J.zip
RUN sudo mkdir $APACHELOG4JDIR
RUN sudo mv $APACHELOG4J.zip $APACHELOG4JDIR/. 
RUN sudo -E sh -c 'cd $APACHELOG4JDIR && unzip $APACHELOG4J.zip'

#daikon
RUN sudo apt-get install -y \
    netpbm \
    texinfo \
    texlive-full

# From docker/setup.bash
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/jvm/java-11-openjdk-amd64/lib/server
ENV DAIKON_VERSION=5.8.10
ENV DAIKONDIR=/root/daikon-$DAIKON_VERSION
# From source $DAIKONDIR/scripts/daikon.bashrc
ENV DAIKONSCRIPTS=$DAIKONDIR/scripts
ENV PLUMESCRIPTS=$DAIKONDIR/utils/plume-scripts
ENV DAIKON_CLASSPATH=$DAIKONDIR/daikon.jar:$JAVA_HOME/jre/lib/rt.jar:$JAVA_HOME/lib/tools.jar:$DAIKONDIR/java:$DAIKONDIR/java/lib/*
ENV PATH=$DAIKONSCRIPTS:$PLUMESCRIPTS:$JAVA_HOME/bin:$PATH
# PERLLIB is already set to these values, so leaving this out for now
#ENV PERLLIB=$DAIKONSCRIPTS:${PLUMESCRIPTS}:$PERLLIB
# From docker/setup.bash (cont.)
ENV RADLERDIR=/root/radler
ENV CLASSPATH=$CLASSPATH:$DAIKONDIR/java/daikon:$RADLERDIR/examples/ardupilot/jni:/root/beepbeep-3-examples/beepbeep-3-examples.jar:$DAIKONDIR/daikon.jar:$DAIKONDIR/java/dcomp_rt.jar:$DAIKONDIR/java/ChicoryPremain.jar:$DAIKONDIR/java/dcomp_premain.jar:$APACHELOG4JDIR/log4j-api-$APACHELOG4J_VERSION.jar:$APACHELOG4JDIR/log4j-core-$APACHELOG4J_VERSION.jar
ENV PATH=$PATH:$APACHELOG4JDIR

#RUN sudo mkdir $DAIKONDIR
RUN wget http://plse.cs.washington.edu/daikon/history/daikon-$DAIKON_VERSION/daikon-$DAIKON_VERSION.tar.gz
RUN tar zxf daikon-$DAIKON_VERSION.tar.gz
RUN sudo mv daikon-$DAIKON_VERSION /root/. 
RUN sudo -E sh -c 'cd $DAIKONDIR && make rebuild-everything'
# MM - done above
#RUN echo "\n# Setup for BeepBepp/Log4J/Daikon" >> ~/.profile && \
#    echo "source $RADLERDIR/examples/ardupilot/docker/setup.bash" >> ~/.profile

#ros2 build
RUN pip3 install colcon-common-extensions colcon-mixin
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml 
RUN colcon mixin update
RUN colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml 
RUN colcon metadata update
#sros2
RUN sudo apt install -y libssl-dev

RUN pip3 install tarjan pyyaml pydot
RUN sudo apt-get install -y software-properties-common
RUN sudo add-apt-repository ppa:deadsnakes/ppa
RUN sudo apt-get update && sudo apt-get install -y \
    python3.6 \
    python3.6-distutils
RUN python3.6 -m pip install tarjan

RUN sudo -E sh -c 'cd /root && git clone https://github.com/SRI-CSL/radler.git && \
    cd radler && \ 
    git submodule update --init --recursive && \
    git checkout ros2'
RUN sudo mv /opt/ros/foxy/share/mavros/launch /opt/ros/foxy/share/mavros/launch.bak
RUN sudo -E sh -c 'cd $RADLERDIR && \
    sudo cp -r examples/ardupilot/launch /opt/ros/foxy/share/mavros'

RUN sudo -E sh -c 'cd /root && \
    git clone https://github.com/ArduPilot/ardupilot.git && \
    cd ardupilot && \
    git submodule update --init --recursive'

RUN sudo mkdir -p ~/ros2_ws/src

# From vagrant sros_env.bash
ENV ROS_SECURITY_KEYSTORE=~/ros2_ws/sros2_keys
ENV ROS_SECURITY_ENABLE=true
ENV ROS_SECURITY_STRATEGY=Enforce
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# setup ros2_ws 
ENV ROS2_WS="/tmp/ros2_ws"
RUN sudo mkdir -p $ROS2_WS/src

# setup entrypoint
ENV ROS2_DISTRO="foxy"
ENV ROS2_PREFIX="/opt/ros"
WORKDIR $ROS2_WS

# Cleanup
RUN sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV CCACHE_MAXSIZE=1G
COPY ./ardupilot_entrypoint.sh /
ENTRYPOINT ["/ardupilot_entrypoint.sh"]
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

