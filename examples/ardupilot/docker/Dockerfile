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

RUN apt-get install -y \
    valgrind \
    gdb \
    perl

# enable permissive ptrace: (from Tools/vagrant/initvagrant.sh) - did not build so left out (MM)
#RUN perl -pe 's/kernel.yama.ptrace_scope = ./kernel.yama.ptrace_scope = 0/' -i /etc/sysctl.d/10-ptrace.conf
#RUN echo 0 > /proc/sys/kernel/yama/ptrace_scope

# Installs for MAVproxy
RUN apt-get install -y \
    python3-dev \
    python3-opencv \
    python3-wxgtk4.0 \ 
    python3-matplotlib \
    python3-lxml \
    python3-pygame

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

# Setup shell configurations
RUN sh -c '(echo ; echo "# Setup shell configurations" ; echo "source /ardupilot/Tools/completion/completion.bash") >> ~/.bashrc'
RUN sh -c '(echo "source /ardupilot/Tools/vagrant/bashrc_git") >> ~/.bashrc'
RUN sh -c '(echo ; echo "# Setup for ROS2" ; echo "source /opt/ros/foxy/setup.bash") >> ~/.bashrc'
RUN sh -c '(echo ; echo "# Setup for ROS2" ; echo "source /opt/ros/foxy/setup.bash") >> ~/.profile'

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

# Download and extract Log4J 
# Originally requested version 2.17.2 which is no longer available - moved to 2.24.1 (which is a .zip file)
ENV APACHELOG4J_VERSION=2.24.3
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
RUN cd $DAIKONDIR && make rebuild-everything

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

# For script running of Ardupilot
RUN pip3 install pymavlink dronekit

# Put all radler build setup in ~/radler
#RUN git clone https://github.com/SRI-CSL/radler.git
RUN git clone https://github.com/MMetelko/radler.git
RUN mv radler /home/${USER_NAME}
RUN cd $RADLERDIR && \ 
    git submodule update --init --recursive && \
    git checkout mm-ardupilot-descert

RUN sudo mv /opt/ros/foxy/share/mavros/launch /opt/ros/foxy/share/mavros/launch.bak
RUN sudo cp -r ${RADLERDIR}/examples/ardupilot/launch /opt/ros/foxy/share/mavros

# Remove ModemManger due to warning during vehicle sim run
RUN sudo apt-get remove -y modemmanager

# Additional tools for DesCert demo
RUN echo "Installing Mono"
RUN sudo apt install -y gnupg ca-certificates
RUN yes | sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF
RUN echo "deb https://download.mono-project.com/repo/ubuntu stable-bionic main" | sudo tee /etc/apt/sources.list.d/mono-official-stable.list
RUN sudo apt update
RUN sudo apt install -y mono-devel

RUN mkdir -p ~/MissionPlanner
RUN /bin/bash -c "cd ~/MissionPlanner && \
    wget https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.zip && \
    unzip MissionPlanner-latest.zip"

RUN /bin/bash -c 'echo "Installing Vicon (aka Vision Positioning) for GPS/Non-GPS Transitions on EKF Navigation"'
RUN cp /app/radler/examples/ardupilot/sitl_config/ViconDataStreamSDK_1.7.1_96542h.zip ~/ViconDataStreamSDK_1.7.1_96542h.zip
RUN /bin/bash -c "cd ~/ && \
    unzip ~/ViconDataStreamSDK_1.7.1_96542h.zip && \
    git clone https://github.com/tridge/pyvicon.git && \
    cp ~/ViconDataStreamSDK_1.7.1_96542h/Linux64-boost-1.55.0/20170125_96542h.x64/*.cpp ~/pyvicon/pyvicon/ && \
    cp ~/ViconDataStreamSDK_1.7.1_96542h/Linux64-boost-1.55.0/20170125_96542h.x64/*.h ~/pyvicon/pyvicon/ && \
    sudo cp ~/ViconDataStreamSDK_1.7.1_96542h/Linux64-boost-1.55.0/20170125_96542h.x64/lib* /usr/lib/"
RUN /bin/bash -c "cd ~/pyvicon && \
    sudo python3 setup.py install"

    RUN sudo apt-get install -y libgtk-3-dev
    RUN sudo -H pip3 install scikit-build --verbose
    RUN sudo -H pip3 install opencv-python --verbose
    RUN python3 -m pip install --verbose --user pathlib2 wxpython
    RUN /bin/bash -c 'echo "Updating startup script"'
    RUN /bin/bash -c '(echo ; echo "# Setup for VICON" ; echo "set moddebug 3"; echo "module load vicon"; echo "vicon start"; echo "rc 8 1000" ) >> /ardupilot/Tools/vagrant/mavinit.scr'
    RUN /bin/bash -c 'echo "Completed DesCert Special Build"'

# Setup NoVNC
#Install git, supervisor, VNC, & X11 packages
RUN sudo apt-get install -y \
    bash \
    dbus-x11 \
    fluxbox \
    git \
    gpg \
    net-tools \
    novnc \
    procps \
    supervisor \
    vim \
    wget \
    xdg-utils \
    x11vnc \
    xterm \
    xvfb

RUN mkdir -p /home/${USER_NAME}/.fluxbox
RUN sudo mkdir -p /usr/local/share/fluxbox/styles
RUN sudo mkdir -p /app-novnc

COPY fluxbox/config /home/${USER_NAME}/.fluxbox
COPY fluxbox/styles /usr/local/share/fluxbox/styles
COPY app /app-novnc
RUN sudo chown -R ${USER_NAME}:${USER_NAME} /app-novnc
RUN sudo chmod +x /app-novnc/scripts/run_with_dronekit.py

# Setup demo environment variables
ENV DISPLAY=:0.0 \
    DISPLAY_WIDTH=1920 \
    DISPLAY_HEIGHT=1080 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=C.UTF-8 \
    RUN_FLUXBOX=yes


# Setup entrypoint file
COPY ardupilot_entrypoint.sh /ardupilot_entrypoint.sh
RUN sudo chown ${USER_NAME}:${USER_NAME} /ardupilot_entrypoint.sh

EXPOSE 8090 8501 14550

# Build the ardupilot SITL simulator
# Finish pulling the ardupilot repository
RUN git submodule update --init --recursive
RUN ./waf configure --board sitl && ./waf build --target bin/arducopter
ENV PATH=$PATH:/ardupilot/build/sitl/bin

# setup ardupilot environment - build Radler
RUN /bin/bash -c "source /home/${USER_NAME}/.ardupilot_env && \
    # setup ros2 environment
    source ${ROS2_PREFIX}/${ROS2_DISTRO}/setup.bash && \
    # build radler house_thermo example
    cd /home/${USER_NAME}/radler && \
    git pull && \
    ./radler.sh --ws_dir ${ROS2_WS}/src compile examples/ardupilot/afs.radl --plant plant --ROS && \
    cd ${ROS2_WS} && \
    colcon build --cmake-args -DSECURITY=ON --no-warn-unused-cli --symlink-install && \
    source ${ROS2_WS}/install/local_setup.bash && \
    # From vagrant/sros_keystore.bash
    cd /home/${USER_NAME} && \
    ros2 security create_keystore sros2_keys && \
    ros2 security create_key sros2_keys /afs/mavros && \
    ros2 security create_key sros2_keys /afs/afs_gateway && \
    ros2 security create_key sros2_keys /afs/afs_function"

# From vagrant sros_env.bash
RUN mkdir -p ${ROS2_WS}/install/afs/bin
RUN cd ${ROS2_WS}/install/afs/bin && \
    ln -sf ../../../build/afs/afs_gateway . && \
    ln -sf ../../../build/afs/afs_function . 
    
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