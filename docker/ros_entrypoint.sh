#!/bin/zsh
set -e

source /opt/ros/noetic/setup.zsh

cd /home/catkin_ws
catkin_make
source devel/setup.zsh 
echo "source /home/catkin_ws/devel/setup.zsh" >> ~/.zshrc
source /opt/ros/noetic/setup.zsh

exec "$@"