#!/bin/bash

# Auto update rpi branch
cd catkin_ws/src/hop && git pull origin rpi
echo Updated Git Repo

# Link ROS Master
export ROS_MASTER_URI=http://kghite.local:11311
export ROS_IP=mahri.local
echo Set up ROS Master link to kghite.local

# Start robot scripts
echo Finished Mahri Startup
