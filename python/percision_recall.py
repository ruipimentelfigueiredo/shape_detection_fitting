#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Apr  2 15:50:09 2017

@author: atabak
"""

import os
import glob
import cv2
import caffe
#import lmdb
import numpy as np
from caffe.proto import caffe_pb2
from os.path import expanduser

from itertools import cycle

from sklearn.metrics import precision_recall_curve
from sklearn.metrics import average_precision_score
from matplotlib import pyplot as plt


caffe.set_mode_gpu()
home = expanduser('~')

#Size of images
IMAGE_WIDTH = 227
IMAGE_HEIGHT = 227


def transform_img(img, img_width=IMAGE_WIDTH, img_height=IMAGE_HEIGHT):

    #Histogram Equalization
    img[:, :, 0] = cv2.equalizeHist(img[:, :, 0])
    img[:, :, 1] = cv2.equalizeHist(img[:, :, 1])
    img[:, :, 2] = cv2.equalizeHist(img[:, :, 2])

    #Image Resizing
    img = cv2.resize(img, (img_width, img_height), interpolation = cv2.INTER_CUBIC)

    return img


'''
Reading mean image, caffe model and its weights
'''
#Read mean image
mean_blob = caffe_pb2.BlobProto()
with open(home+'/Cylinders/dataset_v3/mean.binaryproto') as f:
    mean_blob.ParseFromString(f.read())
mean_array = np.asarray(mean_blob.data, dtype=np.float32).reshape(
    (mean_blob.channels, mean_blob.height, mean_blob.width))


#Read model architecture and trained model's weights
net = caffe.Net(home+'/Cylinders/camera_ready/train_val.prototxt',
                home+'/Cylinders/dataset_v3/backup_trained_model/train_iter_200.caffemodel',
                caffe.TEST)

#Define image transformers
transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
transformer.set_mean('data', mean_array)
transformer.set_transpose('data', (2,0,1))

'''
Making predicitions
'''
#Reading image paths


test_img_paths = [img_path for img_path in glob.glob("./test/*jpg")]

#Making predictions
y_score = list()
y_test = list()
for img_path in test_img_paths:
    img = cv2.imread(img_path, cv2.IMREAD_COLOR)
    img = transform_img(img, img_width=IMAGE_WIDTH, img_height=IMAGE_HEIGHT)

    net.blobs['data'].data[...] = transformer.preprocess('data', img)
    out = net.forward()
    pred_probas = out['prob'][0]
    y_score.append(pred_probas.squeeze())
    if 'cylinder.' in img_path.split('/')[-1][:-4]:
      y_test.append(np.array([1,0]))
    else:
      y_test.append(np.array([0,1]))

y_test = np.asarray(y_test)
y_score = np.asarray(y_score)
##
##
n_classes = 1
##
precision = dict()
recall = dict()
average_precision = dict()
for i in range(n_classes):
    precision[i], recall[i], _ = precision_recall_curve(y_test[:, i],
                                                        y_score[:, i])
    average_precision[i] = average_precision_score(y_test[:, i], y_score[:, i])

# Compute micro-average ROC curve and ROC area
precision["micro"], recall["micro"], _ = precision_recall_curve(y_test.ravel(),
    y_score.ravel())
#average_precision["micro"] = average_precision_score(y_test, y_score,
#                                                     average="micro")



second_y_score_cyl = np.genfromtxt('fitting_quality_cylinders_1.txt').reshape(-1,1)
second_y_score_cyl = np.hstack((second_y_score_cyl, 1.0-second_y_score_cyl))
second_y_test_cyl = np.ones_like(second_y_score_cyl)*np.asarray([1,0])

second_y_score_obj = np.genfromtxt('fitting_quality_others_1.txt').reshape(-1,1)
second_y_score_obj = np.hstack((second_y_score_obj, 1.0 - second_y_score_obj))
second_y_test_obj = np.ones_like(second_y_score_obj)*np.asarray([0,1])
second_y_test = np.vstack((second_y_test_cyl, second_y_test_obj))
second_y_score = np.vstack((second_y_score_cyl, second_y_score_obj))
#
#
#
precision_2 = dict()
recall_2 = dict()
average_precision_2 = dict()
for i in range(n_classes):
    precision_2[i], recall_2[i], _ = precision_recall_curve(second_y_test[:, i],
                                                        second_y_score[:, i])
    average_precision_2[i] = average_precision_score(second_y_test[:, i], second_y_score[:, i])

# Compute micro-average ROC curve and ROC area
precision_2["micro"], recall_2["micro"], _ = precision_recall_curve(second_y_test.ravel(),
    second_y_score.ravel())
#average_precision["micro"] = average_precision_score(y_test, y_score,
#                                                     average="micro")
#












# Plot Precision-Recall curve
colors = cycle(['navy', 'turquoise', 'darkorange', 'cornflowerblue', 'teal'])
lw = 2
plt.clf()
plt.plot(recall[0], precision[0], lw=lw, color='cornflowerblue', 
         label='Classifier AUC={0:0.2f}'.format(average_precision[0]))
plt.plot(recall_2[0], precision_2[0], lw=lw, color='darkorange', 
         label='baseline AUC={0:0.2f}'.format(average_precision_2[0]))
plt.xlabel('Recall')
plt.ylabel('Precision')
plt.ylim([0.0, 1.05])
plt.xlim([0.0, 1.0])
plt.title('Precision-recall curve of class cylinders')
plt.legend(loc="lower left")
plt.show()
plt.savefig('percision_recall_total.eps', format='eps', dpi=300)


# Plot Precision-Recall curve for each class
#plt.clf()
#plt.plot(recall["micro"], precision["micro"], color='gold', lw=lw,
#         label='micro-average Precision-recall curve (area = {0:0.2f})'
#               ''.format(average_precision["micro"]))
#for i, color in zip(range(n_classes), colors):
#    plt.plot(recall[i], precision[i], color=color, lw=lw,
#             label='Precision-recall curve of class cylinders (area = {1:0.2f})'.format(average_precision[i]))

#plt.xlim([0.0, 1.0])
#plt.ylim([0.0, 1.05])
#plt.xlabel('Recall')
#plt.ylabel('Precision')
#plt.title('Precision-recall curve of class cylinders (area = {0:0.2f})'.format(average_precision[0]))
#plt.legend(loc="lower right")
#plt.show()