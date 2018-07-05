#!/bin/bash

#if [ -z "$1" ] 
#  then
#    echo "No folder supplied!"
#    echo "Usage: bash `basename "$0"` mot_video_folder"
#    exit
#fi

DIR=$PWD; # MUST BE RUN FROM shape_detection_fitting ROOT !!!!!!!!!!!!!!!!!!!!!!

BASE_DATASET_FOLDER='/run/user/1004/gvfs/smb-share:server=blackhole,share=users/ruifigueiredo'

CAFFE_FOLDER=$DIR'/lib/caffe'
# 1. modify the dataset
#python $DIR/python/modify_dataset.py $BASE_DATASET_FOLDER

# 2. create lmdb data
#python $DIR/python/create_lmdb.py $BASE_DATASET_FOLDER $DIR

# 3. compute mean image
#compute mean.binaryproto 
$DIR/lib/caffe/build/tools/compute_image_mean -backend=lmdb $DIR/Cylinders/dataset_v3/train_lmdb $DIR/Cylinders/dataset_v3/mean.binaryproto

# 4. train the network with the lmdb - mean data
$DIR/lib/caffe/build/tools/caffe train --solver=$DIR/squeezenet/solver.prototxt --weights $DIR/squeezenet/squeezenet_v1.1.caffemodel 2>&1 | tee $DIR/Cylinders/dataset_v3/model_1_train.log
