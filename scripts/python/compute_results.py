#!/usr/bin/python
from os import path
import os 
import numpy as np
import sys, getopt
import xml.etree.ElementTree as ET
import pandas as pd
modes=['no_classifier','with_classifier']
#paths=['/home/rui/dataset/results/mix/no_classifier','/home/rui/dataset/results/mix/with_classifier']
#main_path='/home/rui/gt.csv'
#test=pd.read_csv(main_path)

#mean_cyls_gt=np.mean(test['cylinder'])
#std_dev_cyls_gt=np.std(test['cylinder'])

#mean_others_gt=np.mean(test['other'])
#std_dev_others_gt=np.std(test['other'])

#mean_total=np.mean(test['other']+test['cylinder'])
#std_dev_total=np.std(test['other']+test['cylinder'])

#print ('gt cylinders    -     mean: '+str(mean_cyls_gt)               +'    std: '+str(std_dev_cyls_gt))
#print ('gt others       -     mean: '+str(mean_others_gt)             +'    std: '+str(std_dev_others_gt))
#print ('total           -     mean: '+str(mean_total)                 +'    std: '+str(std_dev_total))

main_path='/home/rui/ws/src/shape_detection_fitting/lib/shape-classification/dataset/test/results/mix/'

for mode in modes:
	path=main_path+mode
	print path
	# RESULTS MAIN ABSOLUTE PATH
	times_path = os.path.abspath(path+'/times') #os.path.join(current_file, 'dataset')
	annotations_path = os.path.abspath(path+'/annotations') #os.path.join(current_file, 'dataset')

	plane_fitting_time_results=[]
	cluster_extraction_time_results=[]
	cluster_classification_time_results=[]
	cluster_fitting_time_results=[]

	# open timmings result files
	plane_fitting_time_file=open(times_path + "/plane_fitting_time.txt", "r")
	cluster_extraction_time_file=open(times_path + "/cluster_extraction_time.txt", "r")
	cluster_classification_time_file=open(times_path + "/cluster_classification_time.txt", "r")
	cluster_fitting_time_file=open(times_path + "/cluster_fitting_time.txt", "r")

	for line in plane_fitting_time_file:
		plane_fitting_time_results.append(float(line))

	for line in cluster_extraction_time_file:
		cluster_extraction_time_results.append(float(line))

	for line in cluster_classification_time_file:
		cluster_classification_time_results.append(float(line))

	for line in cluster_fitting_time_file:
		cluster_fitting_time_results.append(float(line))

	total_time=np.array(plane_fitting_time_results)+np.array(cluster_extraction_time_results)+np.array(cluster_classification_time_results)+np.array(cluster_fitting_time_results)

	# compute average and standard deviation
	plane_fitting_time_mean = np.mean(plane_fitting_time_results)
	cluster_extraction_time_mean = np.mean(cluster_extraction_time_results)
	cluster_classification_time_mean = np.mean(cluster_classification_time_results)
	cluster_fitting_time_mean = np.mean(cluster_fitting_time_results)
	total_time_mean = np.mean(total_time)

	plane_fitting_time_std_dev = np.std(plane_fitting_time_results)
	cluster_extraction_time_std_dev = np.std(cluster_extraction_time_results)
	cluster_classification_time_std_dev = np.std(cluster_classification_time_results)
	cluster_fitting_time_std_dev = np.std(cluster_fitting_time_results)
	total_time_std_dev = np.std(total_time)

	#####################
	## Detections info ##
	#####################

	annotations_path=path+'/annotations/'
	cylinder_detections_per_scene=[]
	other_detections_per_scene=[]
	total_sequences=len(os.listdir(annotations_path))
	iterations_per_sequence=[]
	
	# for each sequence
	for sequence_folder in os.listdir(annotations_path): # if os.path.isdir(name)
		iterations_per_sequence.append(len(os.listdir(annotations_path+sequence_folder)))
	# for each scene
	for file_name in os.listdir(annotations_path+sequence_folder):
		file_path=annotations_path+sequence_folder+"/"+file_name
		try:
			tree = ET.parse(file_path)
			annotation = tree.getroot()

			# count detected cylinders and others
			#print(len(annotation.findtext("cylinder")))
			cylinder_counter=0
			other_counter=0
			for obj in annotation.findall('object'):
				name = obj.find('name').text
				if   (name == "other"):   other_counter=other_counter+1
				elif (name == "cylinder"): cylinder_counter=cylinder_counter+1
			cylinder_detections_per_scene.append(cylinder_counter)
			other_detections_per_scene.append(other_counter)

			#detection_classes.append(name)
			#print(name)
		except:
			print('Could not open ' + file_path + '. Exiting...')
			#self.parent.destroy()
			sys.exit(2)


	cylinder_detections_per_scene = np.array(cylinder_detections_per_scene)
	other_detections_per_scene = np.array(other_detections_per_scene)
	total_detections_per_scene = cylinder_detections_per_scene + other_detections_per_scene

	# compute average and standard deviation
	cylinder_detections_per_scene_mean = np.mean(cylinder_detections_per_scene)
	other_detections_per_scene_mean = np.mean(other_detections_per_scene)
	total_detections_per_scene_mean = np.mean(total_detections_per_scene)

	cylinder_detections_per_scene_std = np.std(cylinder_detections_per_scene)
	other_detections_per_scene_std = np.std(other_detections_per_scene)
	total_detections_per_scene_std = np.std(total_detections_per_scene)

	# PRINT STATISTICS
	print(" total sequences: " + str(total_sequences))
	print(" total iterations: " + str(np.sum(iterations_per_sequence)) )
	print(" timming stats:")
	#print("  - plane fitting time:           {:.2f}".format(plane_fitting_time_mean)          + " +- {:.2f}".format(plane_fitting_time_std_dev) + " ms")
	#print("  - cluster extraction time:      {:.2f}".format(cluster_extraction_time_mean)     + " +- {:.2f}".format(cluster_extraction_time_std_dev) + " ms")
	print("  - plane fitting + cluster extraction time:      {:.2f}".format(plane_fitting_time_mean+cluster_extraction_time_mean)     + " +- {:.2f}".format(plane_fitting_time_std_dev+cluster_extraction_time_std_dev) + " ms")
	print("  - cluster classification time:                  {:.2f}".format(cluster_classification_time_mean) + " +- {:.2f}".format(cluster_classification_time_std_dev) + " ms")
	print("  - cluster fitting time:                         {:.2f}".format(cluster_fitting_time_mean)        + " +- {:.2f}".format(cluster_fitting_time_std_dev) + " ms")
	print("  - total time:                                   {:.2f}".format(total_time_mean)                  + " +- {:.2f}".format(total_time_std_dev) + " ms")


	print(" detection stats: ")
	print("  - cylinder detections per scene: {:.2f}".format(cylinder_detections_per_scene_mean) + " +- {:.2f}".format(cylinder_detections_per_scene_std))   
	print("  - other detections per scene:    {:.2f}".format(other_detections_per_scene_mean)    + " +- {:.2f}".format(other_detections_per_scene_std))   
	print("  - total detections per scene:    {:.2f}".format(total_detections_per_scene_mean)    + " +- {:.2f}".format(total_detections_per_scene_std)) 
	
