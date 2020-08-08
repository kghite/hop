#!/bin/bash

# Auto update rpi branch
cd catkin_ws/src/hop && git pull origin rpi
echo Updated Git Repo

# Start robot scripts
echo Finished Mahri Startup
