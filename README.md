# shape detection and fitting ROS pipeline

<p align="center"> 
    <img src=.image/pipeline.png>
</p>

## Install
1. Clone this repo and library dependencies (git submodules) inside your catkin workspace src folder:
```
cd $CATKIN_WORKSPACE_DIR/src && git clone https://github.com/ruipimentelfigueiredo/shape_detection_fitting && cd shape_detection_fitting && git submodule update --init --recursive
```

2. Change directory to catkin_ws and compile

```
cd $CATKIN_WORKSPACE_DIR && catkin_make
```

## Setup
Follow the instructions in:

Classifier: https://github.com/ruipimentelfigueiredo/shape-classification

Fitting: https://github.com/ruipimentelfigueiredo/shape-fitting

## Config
Set the appropriate parameters and ROS topic names in the files inside config/ and launch/ folders.

## Demo

```
roslaunch shape_detection_fitting shape_detection_fitting.launch
```

## Generating a new dataset

First, set the flag ```dataset_create``` to true, inside ```config/config.yaml``` file, and select desired ```dataset_path``` and ```object_type```

Then, run:

```
roslaunch shape_detection_fitting shape_detection_fitting.launch
```

## Evaluating the dataset

```
roslaunch shape_detection_fitting test.launch
```

## Reference
In case you use our library in your research, please cite our work

```
@article{figueiredo2019facyl,
title = "A robust and efficient framework for fast cylinder detection",
journal = "Robotics and Autonomous Systems",
volume = "117",
pages = "17 - 28",
year = "2019",
issn = "0921-8890",
doi = "https://doi.org/10.1016/j.robot.2019.04.002",
url = "http://www.sciencedirect.com/science/article/pii/S0921889017308710",
author = "Rui Figueiredo and Atabak Dehban and Plinio Moreno and Alexandre Bernardino and José Santos-Victor and Helder Araújo",
}
```

