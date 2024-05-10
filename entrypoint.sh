#!/bin/bash
# Source the ROS setup for every new shell
source /opt/ros/noetic/setup.bash

# Execute the command provided to the docker run command
exec "$@"
