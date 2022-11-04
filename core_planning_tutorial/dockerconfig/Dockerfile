FROM osrf/ros:melodic-desktop

ARG DOCKER_LANG=en_US
ARG DOCKER_TIMEZONE=America/New_York

ENV LANG=$DOCKER_LANG.UTF-8 \
    LANGUAGE=$DOCKER_LANG:UTF-8 \
    LC_ALL=$DOCKER_LANG.UTF-8

WORKDIR /tmp

ARG DEBIAN_FRONTEND=noninteractive

# Install some required system tools and packages for X Windows and ssh
# Also remove the message regarding unminimize
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        apt-utils \
        apt-file \
        locales \
        language-pack-en && \
    locale-gen $LANG && \
    dpkg-reconfigure -f noninteractive locales && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        curl \
        less \
        vim \
        psmisc \
        runit \
        apt-transport-https ca-certificates \
        software-properties-common \
        man \
        sudo \
        rsync \
        bsdtar \
        net-tools \
        gpg-agent \
        inetutils-ping \
        csh \
        tcsh \
        zsh \
        build-essential \
        libssl-dev \
        git \
        dos2unix \
        dbus-x11 \
        \
        openssh-server \
        python \
        python3 \
        python3-distutils \
        python3-tk \
        python3-dbus \
        \
        xserver-xorg-video-dummy \
        lxde \
        x11-xserver-utils \
        xterm \
        gnome-themes-standard \
        gtk2-engines-pixbuf \
        gtk2-engines-murrine \
        libcanberra-gtk-module libcanberra-gtk3-module \
        ttf-ubuntu-font-family \
        xfonts-base xfonts-100dpi xfonts-75dpi xfonts-scalable xfonts-cyrillic \
        libopengl0 mesa-utils libglu1-mesa libgl1-mesa-dri libjpeg8 libjpeg62 \
        xauth \
        x11vnc \
        \
        firefox \
        xpdf && \
    chmod 755 /usr/local/share/zsh/site-functions && \
    apt-get -y autoremove && \
    ssh-keygen -A && \
    ln -s -f /lib64/ld-linux-x86-64.so.2 /lib64/ld-lsb-x86-64.so && \
    perl -p -i -e 's/#?X11Forwarding\s+\w+/X11Forwarding yes/g; \
        s/#?X11UseLocalhost\s+\w+/X11UseLocalhost no/g; \
        s/#?PasswordAuthentication\s+\w+/PasswordAuthentication no/g; \
        s/#?PermitEmptyPasswords\s+\w+/PermitEmptyPasswords no/g' \
        /etc/ssh/sshd_config && \
    rm -f /etc/update-motd.d/??-unminimize && \
    rm -f /etc/xdg/autostart/lxpolkit.desktop && \
    chmod a-x /usr/bin/lxpolkit && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install websokify and noVNC
RUN curl -O https://bootstrap.pypa.io/get-pip.py && \
    python3 get-pip.py && \
    pip3 install --no-cache-dir \
        setuptools && \
    pip3 install -U https://github.com/novnc/websockify/archive/60acf3c.tar.gz && \
    mkdir /usr/local/noVNC && \
    curl -s -L https://github.com/x11vnc/noVNC/archive/master.tar.gz | \
         bsdtar zxf - -C /usr/local/noVNC --strip-components 1 && \
    rm -rf /tmp/* /var/tmp/*

# Install x11vnc from source
# Install X-related to compile x11vnc from source code.
# https://bugs.launchpad.net/ubuntu/+source/x11vnc/+bug/1686084
# Also, fix issue with Shift-Tab not working
# https://askubuntu.com/questions/839842/vnc-pressing-shift-tab-tab-only
RUN apt-get update && \
    apt-get install -y libxtst-dev libssl-dev libjpeg-dev && \
    \
    mkdir -p /tmp/x11vnc-0.9.14 && \
    curl -s -L http://x11vnc.sourceforge.net/dev/x11vnc-0.9.14-dev.tar.gz | \
        bsdtar zxf - -C /tmp/x11vnc-0.9.14 --strip-components 1 && \
    cd /tmp/x11vnc-0.9.14 && \
    ./configure --prefix=/usr/local CFLAGS='-O2 -fno-stack-protector -Wall' && \
    make && \
    make install && \
    perl -e 's/,\s*ISO_Left_Tab//g' -p -i /usr/share/X11/xkb/symbols/pc && \
    apt-get -y remove libxtst-dev libssl-dev libjpeg-dev && \
    ldconfig && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

########################################################
# Customization for user and location
########################################################
# Set up user so that we do not run as root in DOCKER
ENV DOCKER_USER=ubuntu \
    DOCKER_UID=9999 \
    DOCKER_GID=9999 \
    DOCKER_SHELL=/bin/bash

ENV DOCKER_GROUP=$DOCKER_USER \
    DOCKER_HOME=/home/$DOCKER_USER \
    SHELL=$DOCKER_SHELL


# Change the default timezone to $DOCKER_TIMEZONE
# Run ldconfig so that /usr/local/lib etc. are in the default
# search path for dynamic linker
RUN groupadd -g $DOCKER_GID $DOCKER_GROUP && \
    useradd -m -u $DOCKER_UID -g $DOCKER_GID -s $DOCKER_SHELL -G sudo $DOCKER_USER && \
    echo "$DOCKER_USER:"`openssl rand -base64 12` | chpasswd && \
    echo "$DOCKER_USER ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    echo "$DOCKER_TIMEZONE" > /etc/timezone && \
    ln -s -f /usr/share/zoneinfo/$DOCKER_TIMEZONE /etc/localtime

ADD image/etc /etc
ADD image/usr /usr
ADD image/sbin /sbin
ADD image/home $DOCKER_HOME

RUN mkdir -p $DOCKER_HOME/.config/mozilla && \
    ln -s -f .config/mozilla $DOCKER_HOME/.mozilla && \
    touch $DOCKER_HOME/.sudo_as_admin_successful && \
    mkdir -p $DOCKER_HOME/shared && \
    mkdir -p $DOCKER_HOME/.ssh && \
    mkdir -p $DOCKER_HOME/.log && touch $DOCKER_HOME/.log/vnc.log && \
    chown -R $DOCKER_USER:$DOCKER_GROUP $DOCKER_HOME



# Basic tools and dependencies and nodejs LTS PPA.
RUN apt-get update && apt-get install -y ssh curl apt-utils && curl -sL https://deb.nodesource.com/setup_10.x | bash -

#Install packages:

RUN apt-get update && apt-get install -y ros-melodic-octomap ros-melodic-octomap-msgs ros-melodic-octomap-rviz-plugins ros-melodic-dynamic-edt-3d  ros-melodic-rosmon python-catkin-tools gcc g++ make nodejs

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xAB17C654
RUN apt-get update
RUN apt-get install -y python3-vcstool libpcl-dev ros-melodic-pcl-ros emacs wget

# Install rvizweb. Not needed if we are doing x11vnc
#WORKDIR /rvizweb_ws
#ARG rvizweb_branch=master
#RUN git clone https://github.com/osrf/rvizweb/ src/rvizweb -b ${rvizweb_branch}
#RUN rosdep install --from-paths src --ignore-src -r -y
#RUN . /opt/ros/melodic/setup.sh && catkin_make install

# Clear apt cache.
RUN apt-get clean

# Build the motion planning tutorial code:
WORKDIR /tutorial_ws/src
RUN mkdir -p /tutorial_ws/src
RUN cd  /tutorial_ws/src
RUN git clone https://bitbucket.org/castacks/core_planning_tutorial.git
RUN cp core_planning_tutorial/planning_tutorial.repos /tutorial_ws/src/
RUN vcs import < planning_tutorial.repos
WORKDIR /tutorial_ws/
RUN . /opt/ros/melodic/setup.sh && catkin build
RUN echo "source /opt/ros/melodic/setup.bash" >> /home/ubuntu/.bashrc
RUN echo "source /tutorial_ws/devel/setup.bash" >> /home/ubuntu/.bashrc
RUN sudo chown -R ubuntu:ubuntu /tutorial_ws


#Install VS Code to have a nice IDE:
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
RUN sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
RUN sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
RUN sudo apt-get install -y apt-transport-https
RUN sudo apt-get update
RUN sudo apt-get install -y code 

WORKDIR $DOCKER_HOME

ENV DOCKER_CMD=start_vnc

USER root
ENTRYPOINT ["/sbin/my_init", "--quiet", "--", "/sbin/setuser", "ubuntu"]
CMD ["$DOCKER_CMD"]

