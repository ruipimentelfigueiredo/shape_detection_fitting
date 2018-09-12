#!/bin/bash
PIPELINE_TEST_EXEC=$PWD"/build/planar_top_test"

# Directory where dataset is stored
DATASET_DIR=$PWD"/dataset/shapes2018_all/test"
CONFIG_FILE=$PWD"/config/config.yaml"

$PIPELINE_TEST_EXEC $DATASET_DIR  $CONFIG_FILE 


