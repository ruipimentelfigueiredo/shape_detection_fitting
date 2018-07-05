#!/bin/bash

#if [ -z "$1" ] 
#  then
#    echo "No folder supplied!"
#    echo "Usage: bash `basename "$0"` mot_video_folder"
#    exit
#fi

ROSBAG_DIR="/home/rui/rosbags/"

ROSBAG_NAME="noise.bag"

ITERATIONS=200

# Define noise levels
HEIGHTS="1.0,2.0,3.0,4.0,5.0,6.0"

# Choose segmentation threshold
RADII="0.5"

# Define noise levels
SIGMAS="0.001,0.05,0.10,0.15,0.20,0.25,0.30,0.35,0.40,0.45,0.50"

# Define clutter levels
CLUTTER="0"

# Define clutter levels
HEIGHT_SAMPLES=30

# Define clutter levels
ANGLE_SAMPLES=30

rosrun shape_detection_fitting cylinder_generation $ROSBAG_DIR$ROSBAG_NAME $ITERATIONS $HEIGHTS $RADII $SIGMAS $CLUTTER $HEIGHT_SAMPLES $ANGLE_SAMPLES



