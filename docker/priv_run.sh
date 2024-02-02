#!/bin/sh
# Need to expose host linux x11 server for GUI to be displayed on host (unsafe if on shared system. comment out and use xauth is needed)
xhost +local:root

cd ..

sudo docker run -it --network=host --privileged --name lvt2calib \
    -v ${PWD}:/home/catkin_ws/src/lvt2calib \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    -v $HOME/.ssh:$HOME/.ssh \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    clothooo/lvt2calib:noetic