import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from PIL import Image
import PIL
import caffe
import cv2
import os
import sys
from scipy import ndimage
from skimage.filter import threshold_otsu
import Image
from numpy import array, argwhere
import matplotlib.patches as patches
from numpy import matrix
import matplotlib.pyplot as mplot
from xml.dom import minidom

#########################################################################################


#########################################################################################
#                                                                                       #
#                                DEFINE FUNCTIONS                                       #
#                                                                                       #
#########################################################################################

#########################################################################################
# FUNCTION: CONFIG_PREPROCESS                                                           #
# GOAL:     Configure Preprocessing                                                     #
# INPUT:    Network and image mean file                                                 #
# RETURN:   Transformer                                                                 #
#########################################################################################

def config_preprocess(net, imagenetMeanFile):
    # Load input and configure preprocessing
    transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
    transformer.set_mean('data', np.load(imagenetMeanFile).mean(1).mean(1))  # subtract the dataset-mean value in each channel
    transformer.set_transpose('data', (2, 0, 1))
    transformer.set_channel_swap('data', (2, 1, 0))  # swap channels from RGB to BGR
    transformer.set_raw_scale('data', 255.0)  # rescale from [0, 1] to [0, 255]

    return transformer


#########################################################################################

#########################################################################################
# FUNCTION: PREDICTED FIRST TOP LABELS                                                  #
# GOAL:     Determine predicted top 5 labels for an image                               #
# INPUT:    LABELS_IMAGENET, net, out                                                   #
# OUTPUT:                                                                               #
# RETURN:   top_list, top_k                                                             #
#########################################################################################

def predicted_first_top_labels(LABELS_IMAGENET, net, out):

    # Print predicted labels
    labels = np.loadtxt(LABELS_IMAGENET, str, delimiter='\t')  # Load ImageNet labels
    top_k = net.blobs['prob'].data[0].flatten().argsort()[-1:-6:-1]  # Choose top 5 labels

    top_list = [[0 for x in range(2)] for y in range(30)]

    for i in range(0, 5):
        top_list[i][0] = labels[top_k][i]
        top_list[i][1] = out['prob'][0][top_k][i]

    return top_list, top_k, labels


#########################################################################################

#########################################################################################
# FUNCTION: GET SALIENCY MAP                                                            #
# GOAL:     Compute saliency map for a given class                                      #
# INPUT:    net, top_k, k                                                               #
# OUTPUT:                                                                               #
# RETURN:   Saliency                                                                    #
#########################################################################################

def get_saliency_map(net, top_k, k):

    label_index = top_k[k]
    caffeLabel = np.zeros((1, 1000))
    caffeLabel[0, label_index] = 1

    bw = net.backward(**{net.outputs[0]: caffeLabel})
    diff = bw['data']

    # Find the saliency map and normalize it
    diff -= diff.min()
    diff /= diff.max()
    diff_sq = np.squeeze(diff)
    saliency = np.amax(diff_sq, axis=0)

    return saliency



#########################################################################################

#########################################################################################
# FUNCTION: SEGMENATION MASK                                                            #
# GOAL:     Choose only most salient pixels                                             #
# INPUT:    Saliency map                                                                #
# OUTPUT:                                                                               #
# RETURN:   Bbox                                                                        #
#########################################################################################

def segmentation_mask(saliency, k, bbox):

    # Make copy of saliency map
    foreground_mask = np.array(saliency)

    # Mask with top ranked pixels
    for i in range(0, 227):
        for j in range(0, 227):
            if foreground_mask[i][j] < 7.5e-01:
                foreground_mask[i][j] = 0

    # Find nonzero elements and set boundaries
    B = argwhere(foreground_mask)
    (ystart, xstart), (ystop, xstop) = B.min(0), B.max(0) + 1

    # Store points to plot final bounding boxes
    bbox[k][0] = xstart
    bbox[k][1] = xstop
    bbox[k][2] = ystart
    bbox[k][3] = ystop

    return bbox


#########################################################################################

#########################################################################################
# FUNCTION: CROP BBOX AND RESIZE                                                        #
# GOAL:     Crop image by bbox and resize it to 227*227                                 #
# INPUT:    Path, net, transformer, bbox, k, m                                          #
# OUTPUT:                                                                               #
# RETURN:   Net                                                                         #
#########################################################################################

def crop_bbox_and_resize(path, net, transformer, bbox, k, m):

    # im = Image.open('/home/filipa/Documents/Validation_Set/ILSVRC2012_val_0000000' + str(m + 1) + '.JPEG')
    im = Image.open(path)
    im = im.resize((227, 227), PIL.Image.ANTIALIAS)

    # Crop image patch and resize it for each of 5 bbox
    crop_im = im.crop((bbox[k][0], bbox[k][2], bbox[k][1], bbox[k][3]))

    resize_im = crop_im.resize((227, 227), PIL.Image.ANTIALIAS)
    resize_im.save('resize_' + str(m) + str(k) + '.jpg')

    # Load new cropped and resized image to the network
    image = 'resize_' + str(m) + str(k) + '.jpg'
    net.blobs['data'].data[...] = transformer.preprocess('data', caffe.io.load_image(image))
    os.remove('resize_' + str(m) + str(k) + '.jpg')  # delete resize images

    return net


#########################################################################################

#########################################################################################
# FUNCTION: RANK FINAL TOP SOLUTION                                                     #
# GOAL:     Rank total 25 labels and pick top 5 as final solution                       #
# INPUT:    Top_list, final_top_5                                                       #
# OUTPUT:                                                                               #
# RETURN:   Final_top_5, top_class                                                      #
#########################################################################################


def rank_final_top_solution(top_list, final_top_5):

    file_top1 = open('top1_final_solution_labels_' + sys.argv[1] + '.txt', 'a')
    file_top5 = open('top5_final_solution_labels_' + sys.argv[1] + '.txt', 'a')

    classes = matrix(top_list).transpose()[0].getA()[0]  # Top class name
    classes = np.delete(classes, [0, 1, 2, 3, 4])  # Delete first 5 elements
    probs = matrix(top_list).transpose()[1].getA()[0]  # Top class probability
    probs = np.delete(probs, [0, 1, 2, 3, 4])  # Delete first 5 elements

    final_top_5_prob = []


    for x in range(0, 5):

        idx = max(range(len(probs)), key=lambda i: float(probs[i]))

        if x == 0:
            top_class = classes[idx]
            file_top1.write("{: <50} {: <15} \n".format(top_class, probs[idx]))

        final_top_5.append(classes[idx])
        final_top_5_prob.append(float(probs[idx]))

        file_top5.write("{: <50} {: <15} \n".format(classes[idx], probs[idx]))
        probs[idx] = 0
    final_top_5 = tuple(final_top_5)

    return final_top_5, top_class, file_top1, file_top5, final_top_5_prob



#########################################################################################

#########################################################################################
# FUNCTION: PLOT IMAGE WITH 5 BBOX                                                      #
# GOAL:     Save original image + 5 bboxes                                              #
# INPUT:    Path, bbox, k, m                                                            #
# OUTPUT:   Image with input image and 5 bboxes                                         #
# RETURN:                                                                               #
#########################################################################################


def plot_image_with_5bbox(path, bbox, m):

    im = Image.open(path)
    im = im.resize((227, 227), PIL.Image.ANTIALIAS)

    fig, ax = plt.subplots(1)  # Create figure and axes
    ax.imshow(im)

    for k in range(0, 5):
        # Create a Rectangle patch
        rect = patches.Rectangle((bbox[k][0], bbox[k][2]), bbox[k][1] - bbox[k][0], bbox[k][3] - bbox[k][2],
                                 linewidth=1, edgecolor='r', facecolor='none')

        # Add the patch to the Axes
        ax.add_patch(rect)

    plt.savefig('local_' + str(m) + '.png')

    return bbox
#########################################################################################

#########################################################################################
# FUNCTION: PLOT IMAGE FINAL LABEL                                                      #
# GOAL:     Save input image with final class solution                                  #
# INPUT:    Path, top_class, m                                                          #
# OUTPUT:   File with image and final class solution                                    #
# RETURN:                                                                               #
#########################################################################################

def plot_image_final_label(path, top_class, m):

    original_image = cv2.imread(path)

    # Swap Red and Blue color channels BGR -> RGB
    red = original_image[:, :, 2].copy()
    blue = original_image[:, :, 0].copy()
    original_image[:, :, 0] = red
    original_image[:, :, 2] = blue

    cv2.putText(original_image, "Label: {}".format(top_class), (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    plt.figure()
    plt.imshow(original_image)
    plt.savefig('final_solution_' + str(m) + '.jpg')

#########################################################################################

#########################################################################################
# FUNCTION: DRAW BAR GRAPH                                                              #
# GOAL:     Draw input image and graph bar with top 5 labels                            #
# INPUT:    final_top_5, final_top_5_prob, path, m                                      #
# OUTPUT:   Graph bar                                                                   #
# RETURN:                                                                               #
#########################################################################################

def draw_bar_graph(final_top_5, final_top_5_prob, path, m):
    ######################################################################
    # Plot graphic bar with top 5 and the input image                    #
    # Red bar means that the ground truth label is in the top 5          #
    ######################################################################

    y_pos = np.arange(len(final_top_5))
    scores = final_top_5_prob

    truth_labels = open('ground_truth_labels_ilsvrc12.txt')
    for k in range(0, int(m)):
        ground_truth = truth_labels.readline()

    original = cv2.imread(path)

    # Swap Red and Blue color channels BGR -> RGB
    red = original[:, :, 2].copy()
    blue = original[:, :, 0].copy()
    original[:, :, 0] = red
    original[:, :, 2] = blue

    plt.figure(figsize=(5, 8))

    plt.subplot(2, 1, 1)
    fig = plt.imshow(original)
    fig.axes.get_yaxis().set_visible(False)  # Delete axes
    fig.axes.get_xaxis().set_visible(False)

    plt.subplot(2, 1, 2)
    barlist = plt.barh(y_pos, scores, 0.3, align='center', alpha=0.8)

    ground_truth = ground_truth.split(',')

    # If ground truth in Top 5 -> Red Bar
    for i in range(len(final_top_5)):
        if str(ground_truth[0].rstrip()) in str(final_top_5[i]):
            barlist[i].set_color('r')

    plt.yticks(y_pos, final_top_5)  # final_top_5
    plt.title(ground_truth[0], fontweight="bold")
    cur_axes = mplot.gca()
    # plt.xlabel('Score')
    # cur_axes.axes.get_yaxis().set_ticks([])
    cur_axes.axes.get_xaxis().set_ticks([])

    plt.savefig('bar_graph_' + str(m) + '.png')


#########################################################################################

#########################################################################################
# FUNCTION: CALCULATE_OVERLAP                                                           #
# GOAL:     Calculate if any of predicted bbox overlap at least 50% with ground truth   #
# INPUT:    Bbox, xmin, ymin, xmax, ymax, image_width, image_height, k                  #
# OUTPUT:   Print overlap percentage                                                    #
# RETURN:   Overlap list                                                                #
#########################################################################################

def calculate_overlap(bbox, xmin, ymin, xmax, ymax, image_width, image_height, overlap_list):

    #####################################################################################
    #      Check if top-1 class bbox overlap at least 50% with the ground truth bbox    #
    #                                                                                   #
    #####################################################################################
    x_g = float(227 * int(xmin) / int(image_width))
    y_g = float(227 * int(ymin) / int(image_height))
    width_g = float(227 * int(xmax) / int(image_width)) - float(227 * int(xmin) / int(image_width))
    height_g = float(227 * int(ymax) / int(image_height)) - float(227 * int(ymin) / int(image_height))
    ground_truth_bbox = [x_g, y_g, width_g, height_g]

    for k in range(0,5):

        x_p = bbox[k][0]
        y_p = bbox[k][2]
        width_p = bbox[k][1] - bbox[k][0]
        height_p = bbox[k][3] - bbox[k][2]
        predicted_bbox = [x_p, y_p, width_p, height_p]

        # # Function which calculates area of intersection of two rectangles.
        intersectionArea = max(0, min(float(227 * int(xmax) / int(image_width)), bbox[k][1]) - max(x_g, x_p)) * max(0, min(
            float(227 * int(ymax) / int(image_height)), bbox[k][3])- max(y_g, y_p))

        # print intersectionArea
        unionCoords = [min(x_g, x_p), min(y_g, y_p), max(x_g + width_g - 1, x_p + width_p - 1),
                       max(y_g + height_g - 1, y_p + height_p - 1)]

        unionArea = (unionCoords[2] - unionCoords[0] + 1) * (unionCoords[3] - unionCoords[1] + 1)
        # print unionArea
        overlapArea = intersectionArea / unionArea  # This should be greater than 0.5 to consider it as a valid detection.

        #print overlapArea*100
        overlap_list.append(overlapArea*100)

    return overlap_list


#########################################################################################

#########################################################################################
# FUNCTION: GET LOCALIZATION                                                            #
# GOAL:     Get all ground truth bboxes and predicted bboxes of an image                #
# INPUT:    BBOX_IMAGENET, bbox, path, m                                                #
# OUTPUT:                                                                               #
# RETURN:                                                                               #
#########################################################################################

def get_localization(BBOX_IMAGENET, bbox, path, m, localization_error):

    file = minidom.parse(BBOX_IMAGENET)

    sizes = file.getElementsByTagName("size")
    for size in sizes:
        image_width = size.getElementsByTagName("width")[0].firstChild.data
        image_height = size.getElementsByTagName("height")[0].firstChild.data

    j = 0
    overlap_list = []

    bboxes = file.getElementsByTagName("bndbox")
    for bndbox in bboxes:
        xmin = bndbox.getElementsByTagName("xmin")[0].firstChild.data
        ymin = bndbox.getElementsByTagName("ymin")[0].firstChild.data
        xmax = bndbox.getElementsByTagName("xmax")[0].firstChild.data
        ymax = bndbox.getElementsByTagName("ymax")[0].firstChild.data

        im = Image.open(path)
        im = im.resize((227, 227), PIL.Image.ANTIALIAS)

        fig, ax = plt.subplots(1)  # Create figure and axes
        ax.imshow(im)

        # Create a Rectangle patch
        rect = patches.Rectangle((float(227 * int(xmin) / int(image_width)), float(227 * int(ymin) / int(image_height))),
                                 float(227 * int(xmax) / int(image_width)) - float(227 * int(xmin) / int(image_width)),
                                 float(227 * int(ymax) / int(image_height)) - float(227 * int(ymin) / int(image_height)),
                                 linewidth=1, edgecolor='r', facecolor='none')

        # Add the patch to the Axes
        ax.add_patch(rect)

        # Have number of ground truth bbox per image
        j += 1
        #plt.savefig('ground_local_resized_' + str(m) + '_' + str(j) + '.png')

        # Calculate overlap between 2 bbox
        overlap_list = calculate_overlap(bbox, xmin, ymin, xmax, ymax, image_width, image_height, overlap_list)

    print overlap_list


    # Calculate localization error
    if any(i >= 50 for i in overlap_list):
        print 'maior'
        localization_error.append(0)
    else:
        localization_error.append(1)


    return localization_error



#########################################################################################

#########################################################################################
# FUNCTION: CALCULATE TOP 1 ERROR                                                       #
# GOAL:     Calculate top 1 error rate                                                  #
# INPUT:    File with top 1 labels of all images                                        #
# OUTPUT:   Top 1 error rate                                                            #
# RETURN:                                                                               #
#########################################################################################

def calculate_top1_error():

    error_rate_1 = []
    final_solution = open('top1_final_solution_labels_' + sys.argv[1] + '.txt')
    truth_labels = open('ground_truth_labels_ilsvrc12.txt')

    k = 0
    for i, ground_truth in enumerate(truth_labels, start=1):

        for j, predicted_label in enumerate(final_solution, start=1):

            if str(ground_truth.rstrip()) in str(predicted_label.rstrip()):
                error_rate_1.append(0)
            else:
                error_rate_1.append(1)
            break

    accur_top_1 = (sum(i for i in error_rate_1) / float(len(error_rate_1))) * float(100.0)

    print 'Error Rate Top 1: ', accur_top_1


#########################################################################################

#########################################################################################
# FUNCTION: CALCULATE TOP 5 ERROR                                                       #
# GOAL:     Calculate top 5 error rate                                                  #
# INPUT:    File with top 5 labels of all images                                        #
# OUTPUT:   Top 5 error rate                                                            #
# RETURN:                                                                               #
#########################################################################################

def calculate_top5_error():

    # top class == target label -> error = 0
    # top class != target label -> error = 1


    final_solution = open('top5_final_solution_labels_' + sys.argv[1] + '.txt')
    truth_labels = open('ground_truth_labels_ilsvrc12.txt')

    error_rate_5 = []
    final_error_rate_5 = []
    top5_predicted_label = []
    k = 0
    for i, ground_truth in enumerate(truth_labels, start=1):

        for j, predicted_label in enumerate(final_solution, start=1):

            if j != 5 * (i + 1):
                top5_predicted_label.append(predicted_label.rstrip())

                if str(ground_truth.rstrip()) in str(top5_predicted_label):
                    error_rate_5.append(0)
                else:
                    error_rate_5.append(1)

                if len(error_rate_5) == 5:
                    if error_rate_5.count(0) != 0:  # ground truth esta no top 5
                        final_error_rate_5.append(0)

                    else:
                        final_error_rate_5.append(1)

                    error_rate_5 = []
                    top5_predicted_label = []
                    break


    accur_top_5 = (sum(i for i in final_error_rate_5) / float(len(final_error_rate_5))) * float(100.0)

    print 'Error Rate Top 5: ', accur_top_5


#########################################################################################

#########################################################################################
# FUNCTION: CALCULATE LOCALIZATION ERROR                                                #
# GOAL:     Calculate localization error of validation set                              #
# INPUT:    Array with 1 and 0 if overlap > 50 %                                        #
# OUTPUT:   Localization error percentage                                               #
# RETURN:                                                                               #
#########################################################################################

def calculate_localization_error(localization_error):

    local_error = (sum(i for i in localization_error) / float(len(localization_error))) * float(100.0)

    print 'Localization Error: ', local_error
