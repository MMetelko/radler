FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install git, supervisor, VNC, & X11 packages
RUN set -ex; \
    mkdir /root/Downloads; \
    apt-get update; \
    apt-get upgrade -y; \
    apt-get install -y \
      bash \
      dbus-x11 \
      fluxbox \
      git \
      gpg \
      libarchive-tools \
      libswt-gtk-4-jni \
      net-tools \
      novnc \
      openjdk-17-jre-headless \
      procps \
      supervisor \
      unzip \
      vim \
      wget \
      xdg-utils \
      x11vnc \
      xvfb \
      zip; \
    wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | gpg --dearmor > /usr/share/keyrings/google-chrome.gpg; \
    echo "deb [signed-by=/usr/share/keyrings/google-chrome.gpg, arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" > /etc/apt/sources.list.d/google-chrome.list; \
    apt-get update; \
    cp /etc/apt/sources.list.d/google-chrome.list /etc/apt/sources.list.d/google-chrome.list.bak; \
    apt-get install -y google-chrome-stable; \
    mv /etc/apt/sources.list.d/google-chrome.list.bak /etc/apt/sources.list.d/google-chrome.list;

COPY Files/ chromePreferences /root/Downloads/

RUN mkdir -p /root/.fluxbox && \
    mkdir -p /usr/local/share/fluxbox/styles

COPY fluxbox/config/* /root/.fluxbox/
COPY fluxbox/styles/ /usr/local/share/fluxbox/styles/
COPY app /app/

# Setup demo environment variables
ENV HOME=/root \
    DISPLAY=:0.0 \
    DISPLAY_WIDTH=1920 \
    DISPLAY_HEIGHT=1080 \
    JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=C.UTF-8 \
    RUN_FLUXBOX=yes

ENV PATH="$JAVA_HOME/bin:$PATH"

RUN cp -r /root/.fluxbox /home/ardupilot && \
    useradd -d /home/ardupilot -s /bin/bash ardupilot && \
    chown -R ardupilot:ardupilot /home/ardupilot && \
    mkdir -p /home/ardupilot/.config/google-chrome/Default && \
    mv /root/Downloads/chromePreferences /home/ardupilot/.config/google-chrome/Default/Preferences

CMD ["/app/entrypoint.sh"]
EXPOSE 8090
