#
#  Copyright (C) 2018 Rui Pimentel de Figueiredo
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#  
#      http://www.apache.org/licenses/LICENSE-2.0
#      
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#
#    
#    \author Rui Figueiredo : ruipimentelfigueiredo
#



#! /usr/bin/env python
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
from os import path

#plt.rc('text', usetex=True)
import numpy as np

import math
home=path.expanduser('~/ws/src/shape_detection_fitting/dataset/shapes2018_all/results')

plane_fitting_time_file=open(home + "/plane_fitting_time.txt", "r")
cluster_extraction_time_file=open(home + "/cluster_extraction_time.txt", "r")
cluster_classification_fitting_file=open(home + "/cluster_classification_fitting.txt", "r")

plane_fitting_time_results=[]
for line in plane_fitting_time_file:
	plane_fitting_time_results.append(float(line))

cluster_extraction_time_results=[]
for line in cluster_extraction_time_file:
	cluster_extraction_time_results.append(float(line))

cluster_classification_fitting_results=[]
for line in cluster_classification_fitting_file:
	cluster_classification_fitting_results.append(float(line))

# compute average and standard deviation
plane_fitting_time_mean = np.mean(plane_fitting_time_results)
plane_fitting_time_std = np.std(plane_fitting_time_results)

cluster_extraction_time_mean = np.mean(cluster_extraction_time_results)
cluster_extraction_time_std = np.std(cluster_extraction_time_results)

cluster_classification_fitting_mean = np.mean(cluster_classification_fitting_results)
cluster_classification_fitting_std = np.std(cluster_classification_fitting_results)


print "plane fitting avg time = "+str(plane_fitting_time_mean)+" std dev = "+str(plane_fitting_time_std)
print "clustering avg time = "+str(cluster_extraction_time_mean)+" std dev = "+str(plane_fitting_time_std)
print "classification + fitting avg time = "+str(cluster_classification_fitting_mean)+" std dev = "+str(cluster_classification_fitting_std)


