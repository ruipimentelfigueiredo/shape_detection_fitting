# shape detection, fitting and tracking

## Install
1. First clone this repo and submodules git 

´´´
git clone https://github.com/ruipimentelfigueiredo/shape_detection_fitting && submodule update --init --recursive
´´´ 
1. run modify_dataset.py:


2. run create_lmdb.py:


3. compute mean.binaryproto
/home/rui/mot/lib/caffe/build/tools/compute_image_mean -backend=lmdb /home/rui/Cylinders/dataset_v3/train_lmdb /home/rui/Cylinders/dataset_v3/mean.binaryproto

4. TRAIN


/home/rui/mot/lib/caffe/build/tools/caffe train --solver=/home/rui/ws/src/active_semantic_mapping/squeezenet/solver.prototxt --weights /home/rui/ws/src/active_semantic_mapping/squeezenet/squeezenet_v1.1.caffemodel 2>&1 | tee /home/rui/Cylinders/dataset_v3/model_1_train.log

the .log file contains train and test error (very important for papers!!)
