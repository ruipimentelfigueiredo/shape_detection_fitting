# shape detection, fitting and tracking ROS pipeline

<p align="center"> 
    <img src=.image/pipeline.png>
</p>

## Install
1. Clone this repo and library dependencies (git submodules) inside your catkin workspace src folder:
```
cd $CATKIN_WORKSPACE_DIR/src && git clone https://github.com/ruipimentelfigueiredo/shape_detection_fitting && cd shape_detection_fitting && git submodule update --init --recursive
```
2. Change directory to catkin_wsand compile
```
cd .. && catkin_make
```

## Setup
2. Download CNN files:

```
cd $SHAPE_DETECTION_FITTING/lib/shape-classification && bash scripts/download_models.sh
```
