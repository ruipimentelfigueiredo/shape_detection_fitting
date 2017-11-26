#!/bin/bash

#if [ -z "$1" ] 
#  then
#    echo "No folder supplied!"
#    echo "Usage: bash `basename "$0"` mot_video_folder"
#    exit
#fi

DIR=$PWD; # Foveated_YOLT

# Choose set mode for caffe  [CPU | GPU]
SET_MODE=GPU

# Choose which GPU the detector runs on
GPU_ID=0

# Choose number of top prediction that you want
TOP=5

# Size of the images received by the network
SIZE_MAP=227

# Define number of kernel levels
LEVELS=5

# Define size of the fovea
#SIGMAs={10,20,30,40,50,60,70,80,90,100,110,120,130,140}
SIGMAS="1,10,20,30,40,50,60,70,80,90,100"

# Choose segmentation threshold
THRESHOLDS="0.0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5,0.55,0.6,0.65,0.7,0.75,0.80,0.85,0.90,0.95,1.0"

# Choose method (1-CARTESIAN 2-FOVEATION 3-HYBRID)
MODE=2

# change this path to the absolute location of the network related files
FILES_FOLDER_ABSOLUTE_PATH=$PWD"/files/"
MODEL_FILE="deploy_caffenet.prototxt"
WEIGHTS_FILE="bvlc_reference_caffenet.caffemodel"
MEAN_FILE="imagenet_mean.binaryproto"
LABELS_FILE="synset_words_change.txt"
DATASET="/media/Data/filipa/ILSVRC2012_img_test"
#DATASET="/home/rui/ILSVR2007"
#GROUND_TRUTH_LABELS=$FILES_FOLDER_ABSOLUTE_PATH"ground_truth_labels_ilsvrc12.txt"
RESULTS_FOLDER_ABSOLUTE_PATH=$PWD"/results/"
DEBUG=1
TOTAL_IMAGES=100

build/yolt $FILES_FOLDER_ABSOLUTE_PATH $MODEL_FILE $WEIGHTS_FILE $MEAN_FILE $LABELS_FILE  $DATASET $TOP $THRESHOLDS $SIZE_MAP $LEVELS $SIGMAS $RESULTS_FOLDER_ABSOLUTE_PATH $MODE $DEBUG $TOTAL_IMAGES $SET_MODE $GPU_ID



