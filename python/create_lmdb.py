'''
Title           :create_lmdb.py
Description     :This script divides the training images into 2 sets and stores them in lmdb databases for training and validation.
Author          :Adil Moujahid
Date Created    :20160619
Date Modified   :20160625
version         :0.2
usage           :python create_lmdb.py
python_version  :2.7.11
'''

import os
import glob
import random
import numpy as np

import cv2

import caffe
from caffe.proto import caffe_pb2
import lmdb

#Size of images
IMAGE_WIDTH = 227
IMAGE_HEIGHT = 227

<<<<<<< HEAD

base_dataset = '/run/user/1000/gvfs/smb-share:server=blackhole,share=users/ruifigueiredo'

=======
>>>>>>> 0d7805bc4a83e76e03250a8b6c2ad956ae7444a6
home = os.path.expanduser('~')

def transform_img(img, img_width=IMAGE_WIDTH, img_height=IMAGE_HEIGHT):

    #Histogram Equalization
    img[:, :, 0] = cv2.equalizeHist(img[:, :, 0])
    img[:, :, 1] = cv2.equalizeHist(img[:, :, 1])
    img[:, :, 2] = cv2.equalizeHist(img[:, :, 2])

    #Image Resizing
    img = cv2.resize(img, (img_width, img_height), interpolation = cv2.INTER_CUBIC)

    return img


def make_datum(img, label):
    #image is numpy.ndarray format. BGR instead of RGB
    return caffe_pb2.Datum(
        channels=3,
        width=IMAGE_WIDTH,
        height=IMAGE_HEIGHT,
        label=label,
        data=np.rollaxis(img, 2).tostring())

train_lmdb = home + '/Cylinders/dataset_v3/train_lmdb'
validation_lmdb = home + '/Cylinders/dataset_v3/validation_lmdb'

#os.system('rm -rf  ' + train_lmdb)
#os.system('rm -rf  ' + validation_lmdb)


<<<<<<< HEAD
#train_data = [img for img in glob.glob("../../dataset_v3/train/*jpg")]
train_data = [img for img in glob.glob(base_dataset + '/train/*jpg')]
=======
train_data = [img for img in glob.glob("../../dataset_v3/train/*jpg")]
>>>>>>> 0d7805bc4a83e76e03250a8b6c2ad956ae7444a6
#test_data = [img for img in glob.glob("../input/test1/*jpg")]

#Shuffle train_data
random.shuffle(train_data)

print 'Creating train_lmdb'

in_db = lmdb.open(train_lmdb, map_size=int(1e12))
with in_db.begin(write=True) as in_txn:
    for in_idx, img_path in enumerate(train_data):
        if in_idx %  10 == 0:
            continue
        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        img = transform_img(img, img_width=IMAGE_WIDTH, img_height=IMAGE_HEIGHT)
        if 'cylinder.' in img_path:
            label = 0
<<<<<<< HEAD
        elif 'sphere.':
            label = 1
        else:
            label = 2
=======
        else:
            label = 1
>>>>>>> 0d7805bc4a83e76e03250a8b6c2ad956ae7444a6
        datum = make_datum(img, label)
        in_txn.put('{:0>5d}'.format(in_idx), datum.SerializeToString())
        print '{:0>5d}'.format(in_idx) + ':' + img_path
in_db.close()


print '\nCreating validation_lmdb'

in_db = lmdb.open(validation_lmdb, map_size=int(1e12))
with in_db.begin(write=True) as in_txn:
    for in_idx, img_path in enumerate(train_data):
        if in_idx % 10 != 0:
            continue
        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        img = transform_img(img, img_width=IMAGE_WIDTH, img_height=IMAGE_HEIGHT)
        if 'cylinder.' in img_path:
            label = 0
<<<<<<< HEAD
        elif 'sphere.':
            label = 1
        else:
            label = 2
=======
        else:
            label = 1
>>>>>>> 0d7805bc4a83e76e03250a8b6c2ad956ae7444a6
        datum = make_datum(img, label)
        in_txn.put('{:0>5d}'.format(in_idx), datum.SerializeToString())
        print '{:0>5d}'.format(in_idx) + ':' + img_path
in_db.close()

print '\nFinished processing all images'