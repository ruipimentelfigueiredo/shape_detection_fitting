import os
import sys

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import PIL
from PIL import Image
import caffe
import cv2

from scipy import ndimage
from skimage.filter import threshold_otsu
import Image
from numpy import array, argwhere
import matplotlib.patches as patches
from glob import glob
from thesis_functions import *
import matplotlib.pyplot as mplot

caffe.set_mode_cpu()        # mode

#########################################################################################
# GLOBAL VARIABLES
imagenetModelFile = 'deploy_' + sys.argv[1] + '.prototxt'
imagenetTrainedModel = 'bvlc_' + sys.argv[1] + '.caffemodel'

imagenetMeanFile = '/home/filipa/caffe/python/caffe/imagenet/ilsvrc_2012_mean.npy'
LABELS_IMAGENET = '/home/filipa/PycharmProjects/Proposal_Code/synset_words_change.txt'
IMAGE_PATH_FOLDER = '/home/filipa/Documents/Validation_Set/'
IMAGE_FOLDER = 'Actual_Results/'
#LABELS_IMAGENET = '/home/filipa/PycharmProjects/Proposal_Code/Labels/val.txt'

#########################################################################################


#########################################################################################
#                                                                                       #
#                                   MAIN PROGRAM                                        #
#                                                                                       #
#########################################################################################
#
# Initialization
localization_error = []

# Create network for Test
net = caffe.Net(imagenetModelFile,     # arquitecture
                imagenetTrainedModel,  # weights
                caffe.TEST)            # phase

# Configure Preprocessing
transformer = config_preprocess(net, imagenetMeanFile)

# For each image
for path in sorted(glob(IMAGE_PATH_FOLDER + "ILSVRC2012_val_*.JPEG")):

    m = path.rstrip('.JPEG')[-5:]

    # Initialization
    final_top_5 = []  # final top 5 classes for each image
    bbox = [[0 for x in range(4)] for y in range(5)]
    BBOX_IMAGENET = '/home/filipa/Documents/Validation_bbox/val/ILSVRC2012_val_000' + str(m) + '.xml'

    # Load image
    im = caffe.io.load_image(path)

    # perform the preprocessing we've set up
    net.blobs['data'].data[...] = transformer.preprocess('data', im)

    # Compute forward
    out = net.forward()

    print '\n Predicted class is (class index):', out['prob'].argmax()

    # Predict first top 5 labels
    top_list, top_k, labels = predicted_first_top_labels(LABELS_IMAGENET, net, out)
    #print top_list

    # for each class
    for k in range(0, 5):

        #########################################################################################
        #########################################################################################
        #                       Weakly Supervised Object Localisation                           #
        #        Class Saliency Extraction + Segmentation Mask + Bounding Box to locate         #
        #########################################################################################


        # Get Saliency map for a given class
        saliency = get_saliency_map(net, top_k, k)

        # Get Segmentation Mask
        bbox = segmentation_mask(saliency, k, bbox)

        #########################################################################################
        #########################################################################################
        #                       Image Re-Classification with Attention                          #
        #                Crop and resize patch + Forward + Predict new classes                  #
        #########################################################################################

        # Crop image by bbox and resize to 227x227
        net = crop_bbox_and_resize(path, net, transformer, bbox, k, m)

        # Compute forward
        out = net.forward()

        # New predicted top 5 classes
        new_top_k = net.blobs['prob'].data[0].flatten().argsort()[-1:-6:-1]

        # Create top_list with 25 predicted labels, 5 labels per class
        y = k
        y += 1
        for i in range(0, 5):
            top_list[i + 5 * y][0] = labels[new_top_k][i]
            top_list[i + 5 * y][1] = out['prob'][0][new_top_k][i]

    #print top_list

    # Rank total 25 labels and pick top 5 as final solution
    final_top_5, top_class, file_top1, file_top5, final_top_5_prob = rank_final_top_solution(top_list, final_top_5)
    #print final_top_5

    # Save original image + 5 bboxes
    #bbox = plot_image_with_5bbox(path, bbox, m)

    # Save input image with final class solution
    #plot_image_final_label(path, top_class, m)


    ######################################################################
    # Plot graphic bar with top 5 and the input image                    #
    # Red bar means that the ground truth label is in the top 5          #
    ######################################################################
    #draw_bar_graph(final_top_5, final_top_5_prob, path, m)


    ######################################################################
    # Read all ground truth bboxes of an image                           #
    # Calculate overlap percentage of ground truth bbox with 5 predicted #
    # bboxes by the network                                              #
    ######################################################################
    get_localization(BBOX_IMAGENET, bbox, path, m, localization_error)

    print localization_error




# Calculate top 1 error rate
#calculate_top1_error()


# Calculate top 5 error rate
#calculate_top5_error()


# Calculate localization error
calculate_localization_error(localization_error)
