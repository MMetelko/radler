ARG BASE_IMAGE="ubuntu"
ARG TAG="20.04"
FROM ${BASE_IMAGE}:${TAG}

# Prevent user interaction during package installation
ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=ardupilot
ARG USER_UID=1000
ARG USER_GID=1000
ARG SKIP_AP_EXT_ENV=0
ARG SKIP_AP_GRAPHIC_ENV=1
ARG SKIP_AP_COV_ENV=1
ARG SKIP_AP_GIT_CHECK=1
ARG DO_AP_STM_ENV=1


# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata 

RUN apt-get update && apt-get install --no-install-recommends -y \
    lsb-release \
    sudo \
    bash-completion \
    git \
    ca-certificates

# Create non root user for pip (per ardupilot approach)
RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash
RUN echo "ardupilot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}
RUN chmod 0440 /etc/sudoers.d/${USER_NAME}

# Packages for JSB sim build
RUN apt-get install -y \
    libtool \
    automake \
    autoconf \
    libexpat1-dev \
    cmake

# Generate locales
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Clone ardupilot repository
RUN git clone https://github.com/ArduPilot/ardupilot.git /${USER_NAME}

RUN chown -R ${USER_NAME}:${USER_NAME} /${USER_NAME}

# Setup the ardupilot folder to be the ardupilot repository
WORKDIR /ardupilot

# Move to the ardupilot user
USER ${USER_NAME}

RUN SKIP_AP_EXT_ENV=$SKIP_AP_EXT_ENV SKIP_AP_GRAPHIC_ENV=$SKIP_AP_GRAPHIC_ENV SKIP_AP_COV_ENV=$SKIP_AP_COV_ENV SKIP_AP_GIT_CHECK=$SKIP_AP_GIT_CHECK \
    DO_AP_STM_ENV=$DO_AP_STM_ENV \
    AP_DOCKER_BUILD=1 \
    USER=${USER_NAME} \
    Tools/environment_install/install-prereqs-ubuntu.sh -y

# Check that local/bin are in PATH for pip --user installed package
RUN echo "if [ -d \"\$HOME/.local/bin\" ] ; then\nPATH=\"\$HOME/.local/bin:\$PATH\"\nfi" >> ~/.ardupilot_env

# Set the buildlogs directory into /tmp as other directory aren't accessible
ENV BUILDLOGS=/tmp/buildlogs

# Configuration files from ardupilot/Tools/vagrant/initvagrant.sh
RUN ln -fs /${USER_NAME}/Tools/vagrant/.screenrc /home/${USER_NAME}/.screenrc
RUN ln -fs /${USER_NAME}/Tools/vagrant/.mavinit.scr /home/${USER_NAME}/.mavinit.scr

# Build JSB sim
RUN /${USER_NAME}/Tools/scripts/build-jsbsim.sh

# Move to root for apt package installs
USER root

# Update system packages and install necessary tools
RUN apt-get update && apt-get install -y \
    binutils-dev \
    build-essential \
    cmake \
    libbullet-dev \
    nano \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-setuptools \
    wget

RUN apt-get install -y --no-install-recommends \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev

RUN apt-get install -y \
    curl \
    gnupg2 \
    libssl-dev \
    unzip

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

# for beepbeep
RUN apt-get install -y \
    default-jdk \
    ant

# for daikon
RUN apt-get install -y \
    netpbm \
    texinfo \
    texlive-full

USER ${USER_NAME}
ENV USER=${USER_NAME}

# setup ros2_ws 
ENV ROS2_DISTRO="foxy"
ENV ROS2_PREFIX="/opt/ros"
ENV ROS2_WS="/home/${USER_NAME}/ros2_ws"
RUN mkdir -p /home/${USER_NAME}/ros2_ws/src

# Set the buildlogs directory into /tmp as other directory aren't accessible
ENV BUILDLOGS=/tmp/buildlogs

RUN python3 -m pip install -U --user \
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

# Install GeographicLib datasets
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh -O ~/install_geographiclib_datasets.sh
RUN chmod a+x ~/install_geographiclib_datasets.sh
RUN sudo ~/install_geographiclib_datasets.sh

RUN sh -c '(echo ; echo "# Setup for ROS2" ; echo "source /opt/ros/foxy/setup.bash") >> ~/.bashrc'
RUN sh -c '(echo ; echo "# Setup for ROS2" ; echo "source /opt/ros/foxy/setup.bash") >> ~/.profile'

# Download and extract Log4J 
# Originally requested version 2.17.2 which is no longer available - moved to 2.24.1 (which is a .zip file)
ENV APACHELOG4J_VERSION=2.24.1
ENV APACHELOG4J=apache-log4j-$APACHELOG4J_VERSION-bin
ENV APACHELOG4JDIR=/home/${USER_NAME}/$APACHELOG4J
RUN wget https://dlcdn.apache.org/logging/log4j/$APACHELOG4J_VERSION/$APACHELOG4J.zip
RUN mkdir -p $APACHELOG4JDIR
RUN mv $APACHELOG4J.zip $APACHELOG4JDIR/$APACHELOG4J.zip
RUN cd $APACHELOG4JDIR && unzip $APACHELOG4J.zip
RUN cd $APACHELOG4JDIR && rm $APACHELOG4J.zip

# From docker/setup.bash
ENV JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${JAVA_HOME}/lib/server
ENV DAIKON_VERSION=5.8.10
ENV DAIKONDIR=/home/${USER_NAME}/daikon-$DAIKON_VERSION
# From source $DAIKONDIR/scripts/daikon.bashrc
ENV DAIKONSCRIPTS=$DAIKONDIR/scripts
ENV PLUMESCRIPTS=$DAIKONDIR/utils/plume-scripts
ENV DAIKON_CLASSPATH=$DAIKONDIR/daikon.jar:${JAVA_HOME}/jre/lib/rt.jar:${JAVA_HOME}/lib/tools.jar:$DAIKONDIR/java:$DAIKONDIR/java/lib/*
ENV PATH=$DAIKONSCRIPTS:$PLUMESCRIPTS:${JAVA_HOME}/bin:${PATH}
ENV PERLLIB=$DAIKONSCRIPTS:$PLUMESCRIPTS
# From docker/setup.bash (cont.)
ENV RADLERDIR=/home/${USER_NAME}/radler
ENV CLASSPATH=${CLASSPATH}:$DAIKONDIR/java/daikon:$RADLERDIR/examples/ardupilot/jni:/home/${USER_NAME}/beepbeep-3-examples/beepbeep-3-examples.jar:$DAIKONDIR/daikon.jar:$DAIKONDIR/java/dcomp_rt.jar:$DAIKONDIR/java/ChicoryPremain.jar:$DAIKONDIR/java/dcomp_premain.jar:$APACHELOG4JDIR/log4j-api-$APACHELOG4J_VERSION.jar:$APACHELOG4JDIR/log4j-core-$APACHELOG4J_VERSION.jar
ENV PATH=$PATH:/home/${USER_NAME}/jsbsim/build/src:$APACHELOG4JDIR

RUN wget http://plse.cs.washington.edu/daikon/history/daikon-$DAIKON_VERSION/daikon-$DAIKON_VERSION.tar.gz
RUN mv daikon-$DAIKON_VERSION.tar.gz ~/daikon-$DAIKON_VERSION.tar.gz
RUN cd ~/ && tar zxf daikon-$DAIKON_VERSION.tar.gz
RUN cd ~/ && rm daikon-$DAIKON_VERSION.tar.gz
#RUN sudo -E sh -c 'cd $DAIKONDIR && make rebuild-everything'
RUN cd $DAIKONDIR && make rebuild-everything
# MM - done above
#RUN echo "\n# Setup for BeepBepp/Log4J/Daikon" >> ~/.profile && \
#    echo "source $RADLERDIR/examples/ardupilot/docker/setup.bash" >> ~/.profile

#ros2 build
RUN pip3 install colcon-common-extensions colcon-mixin
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml 
RUN colcon mixin update
RUN colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml 
RUN colcon metadata update

RUN pip3 install tarjan pyyaml pydot scipy
RUN sudo apt-get install -y software-properties-common
RUN sudo add-apt-repository ppa:deadsnakes/ppa
RUN sudo apt-get update && sudo apt-get install -y python3.6
RUN sudo apt-get install -y python3.6-distutils
RUN sudo python3.6 -m pip install tarjan

# Put all radler build setup in ~/radler
#RUN git clone https://github.com/SRI-CSL/radler.git
RUN git clone https://github.com/MMetelko/radler.git
RUN mv radler /home/${USER_NAME}
RUN cd $RADLERDIR && \ 
    git submodule update --init --recursive && \
    git checkout thermo-example

RUN sudo mv /opt/ros/foxy/share/mavros/launch /opt/ros/foxy/share/mavros/launch.bak
RUN sudo cp -r ${RADLERDIR}/examples/ardupilot/launch /opt/ros/foxy/share/mavros

# Remove ModemManger due to warning during vehicle sim run
RUN sudo apt-get remove -y modemmanager

# Cleanup
RUN sudo apt-get autoremove -y
RUN sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV CCACHE_MAXSIZE=1G

# Setup entrypoint file
COPY ardupilot_entrypoint.sh /ardupilot_entrypoint.sh
RUN sudo chown ${USER_NAME}:${USER_NAME} /ardupilot_entrypoint.sh
ENTRYPOINT ["/ardupilot_entrypoint.sh"]
CMD ["bash"]


# Things to consider adding that are in Tools/vagrant/initvagrant.sh
# apt install gdb, automake, autoconf, libexpat1-dev, libtool (libtool-bin is installed)
# enable permissive ptrace perl commands
# DEFINITELY CONSIDER THIS: add Tools/scripts/build-jsbsim.sh and run it - but consider how to clone repo first
# Look at DOT_PROFILE section and shellinit.sh
# Not sure what is needed for the bashrc code set - this section extend .bashrc file for git formatting (put later, if needed)
# touch /ardupilot.vagrant - not sure needed