FROM osrf/ros:noetic-desktop-full

# install opencv 
RUN apt update && apt install libopencv-dev -y 

# Install packages without prompting the user to answer any questions
ENV DEBIAN_FRONTEND noninteractive  

# Minimal setup
# Install packages
RUN apt-get update && apt-get install -y \
locales \
lsb-release \
mesa-utils \
git \
wget \
curl \
gnupg2 \
dirmngr \
libssl-dev \
build-essential \
dbus-x11 \
libtool \
software-properties-common 
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install python3-osrf-pycommon python3-catkin-tools -y

# =========> INSTALL ceres library <=============
RUN apt-get update && apt-get install -y \
    libgflags-dev \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# =========> INSTALL ceres <=============
RUN wget -O ceres.tar.gz https://github.com/ceres-solver/ceres-solver/archive/refs/tags/1.14.0.tar.gz && tar -xvzf ceres.tar.gz \
    && cd ceres-solver-1.14.0 && mkdir build && cd build && cmake .. \
    && make install -j8 && cd / && rm ceres.tar.gz && rm -r ceres-solver-1.14.0/

RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.5/zsh-in-docker.sh)" -- \
    -t ys \
    -p git \
	-p z \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting \
    && sudo echo "set-option -g default-shell /bin/zsh" >> ~/.tmux.conf \
    && sudo chsh -s /bin/zsh

# Locale
RUN locale-gen en_US.UTF-8  
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en  
ENV LC_ALL en_US.UTF-8 
ENV ROS_DISTRO noetic
ENV ROBOT_ID 0

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.zsh" >> /root/.zshrc
RUN echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.zshrc

RUN mkdir -p /home/catkin_ws/src/lvt2calib
WORKDIR /home/catkin_ws

COPY ros_entrypoint.sh /
RUN sudo chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["zsh"]

