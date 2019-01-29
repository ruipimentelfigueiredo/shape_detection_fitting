# shape detection, fitting and tracking ROS pipeline

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
https://github.com/ruipimentelfigueiredo/shape-classification

https://github.com/ruipimentelfigueiredo/shape-fitting

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
@inproceedings{figueiredo2017shape,
  title={Shape-based attention for identification and localization of cylindrical objects},
  author={Figueiredo, Rui and Dehban, Atabak and Bernardino, Alexandre and Santos-Victor, Jos{\'e} and Ara{\'u}jo, Helder},
  booktitle={IEEE International Conference on Development and Learning and on Epigenetic Robotics (ICDL-EpiRob)},
  volume={18},
  pages={21},
  year={2017}
}
```

