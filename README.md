<<<<<<< HEAD
# shape detection, fitting and tracking ROS pipeline
=======
<<<<<<< HEAD
# shape detection, fitting and tracking

## Install
1. First clone this repo and submodules git 

´´´
git clone https://github.com/ruipimentelfigueiredo/shape_detection_fitting && submodule update --init --recursive
´´´ 
1. run modify_dataset.py:


2. run create_lmdb.py:
>>>>>>> b4020863b4cbafe7c5c8f4330553fa0046e8e906

<p align="center"> 
    <img src=.image/pipeline.png>
</p>

<<<<<<< HEAD
=======
3. compute mean.binaryproto
/home/rui/mot/lib/caffe/build/tools/compute_image_mean -backend=lmdb /home/rui/Cylinders/dataset_v3/train_lmdb /home/rui/Cylinders/dataset_v3/mean.binaryproto

4. TRAIN


/home/rui/mot/lib/caffe/build/tools/caffe train --solver=/home/rui/ws/src/active_semantic_mapping/squeezenet/solver.prototxt --weights /home/rui/ws/src/active_semantic_mapping/squeezenet/squeezenet_v1.1.caffemodel 2>&1 | tee /home/rui/Cylinders/dataset_v3/model_1_train.log

the .log file contains train and test error (very important for papers!!)
=======
# shape detection, fitting and tracking ROS pipeline

<p align="center"> 
    <img src=.image/pipeline.png>
</p>

>>>>>>> b4020863b4cbafe7c5c8f4330553fa0046e8e906
## Install
1. Clone this repo and library dependencies (git submodules) inside your catkin workspace src folder:
```
cd $CATKIN_WORKSPACE_DIR/src && git clone https://github.com/ruipimentelfigueiredo/shape_detection_fitting && cd shape_detection_fitting && git submodule update --init --recursive
```
<<<<<<< HEAD
2. Change directory to catkin_wsand compile
=======
2. Change directory to catkin_ws and compile
>>>>>>> b4020863b4cbafe7c5c8f4330553fa0046e8e906
```
cd .. && catkin_make
```

## Setup
Follow the instructions in:
https://github.com/ruipimentelfigueiredo/shape-classification
<<<<<<< HEAD
=======


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
>>>>>>> f3bbdfdfd5727cc40ebf8d993b4f5938dae4cf30
>>>>>>> b4020863b4cbafe7c5c8f4330553fa0046e8e906
