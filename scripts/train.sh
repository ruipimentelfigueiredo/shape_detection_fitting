#!/bin/bash

#if [ -z "$1" ] 
#  then
#    echo "No folder supplied!"
#    echo "Usage: bash `basename "$0"` mot_video_folder"
#    exit
#fi

DIR=$PWD; # active_semantic_mapping

BASE_DATASET_FOLDER='/run/user/1000/gvfs/smb-share:server=blackhole,share=users/ruifigueiredo'

CAFFE_FOLDER='~/mot/lib/caffe'
# 1. modify the dataset
python $DIR/python/modify_dataset.py $BASE_DATASET_FOLDER

# 2. create lmdb data
python $DIR/python/create_lmdb.py $BASE_DATASET_FOLDER $DIR
