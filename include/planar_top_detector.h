/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/
#ifndef PLANAR_TOP_DETECTOR_H
#define PLANAR_TOP_DETECTOR_H

#include <pcl/point_types.h>
#include "cylinder_classifier.h"
#include "cylinder_fitting_hough.h"
#include "cylinder_fitting_ransac.h"
#include "sphere_fitting_hough.h"
#include "plane_fitting_ransac.h"
#include "shape_detection_manager.h"

template <class cylinder_detector_type, class sphere_detector_type, class plane_detector_type>
class PlanarTopDetector
{
	boost::shared_ptr<ShapeDetectionManager<cylinder_detector_type, sphere_detector_type> > shape_detection_manager;
	boost::shared_ptr<plane_detector_type> plane_fitting;
	pcl::VoxelGrid<PointT> grid_;
	double clustering_voxel_size,z_filter_min,z_filter_max;
	bool with_classifier;
	bool visualize;

	public:
		PlanarTopDetector(
			boost::shared_ptr<plane_detector_type> & plane_fitting_, 
			boost::shared_ptr<CylinderClassifier> & cylinder_classifier_, 
			boost::shared_ptr<cylinder_detector_type> & cylinder_fitting_, 
			boost::shared_ptr<sphere_detector_type> & sphere_fitting_, 
			const Eigen::Matrix4f & cam_projection_, 
			const double & classification_threshold_,
			double padding_=0.1,
			double clustering_voxel_size_=0.01,
			double z_filter_min_=0.05,
			double z_filter_max_=5.0,
			bool with_classifier_=false,
			bool visualize_=true) : 
				shape_detection_manager(new ShapeDetectionManager<cylinder_detector_type, sphere_detector_type>(
					cylinder_classifier_, 
					cylinder_fitting_,
					sphere_fitting_,
		 			cam_projection_,
					classification_threshold_,
					padding_)),
				plane_fitting(plane_fitting_),
				clustering_voxel_size(clustering_voxel_size_),
				z_filter_min(z_filter_min_),
				z_filter_max(z_filter_max_),
				with_classifier(with_classifier_),
				visualize(visualize_)

		{
			// Filtering parameters
			grid_.setLeafSize (clustering_voxel_size, clustering_voxel_size, clustering_voxel_size);
			grid_.setFilterFieldName ("z");
			grid_.setFilterLimits (z_filter_min, z_filter_max);
			grid_.setDownsampleAllData (false);
		};
	
		DetectionData detect(cv::Mat & image, const PointCloudT::Ptr & point_cloud, boost::shared_ptr<VisualizeFittingData> visualizer=NULL, std::vector<long int> & durations=std::vector<long int>())
		{
			grid_.setInputCloud (point_cloud);
			grid_.filter (*point_cloud);

			/* PLANE FITTING */
			std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
			FittingData plane_model_params=plane_fitting->fit(point_cloud);
			std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
			/* END PLANE FITTING */

			long int plane_fitting_duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
			durations.push_back(plane_fitting_duration);

			/* CLUSTER EXTRACTION */
			std::vector<PointCloudT::Ptr> clusters_point_clouds;
			t1 = std::chrono::high_resolution_clock::now();
			plane_fitting->extractTabletopClusters(point_cloud, clusters_point_clouds);
			t2 = std::chrono::high_resolution_clock::now();
			/* END CLUSTER EXTRACTION */

			long int cluster_extraction_duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
			durations.push_back(cluster_extraction_duration);

			/* CLASSIFICATION + FITTING */
			t1 = std::chrono::high_resolution_clock::now();
			DetectionData detections=shape_detection_manager->detect(image, clusters_point_clouds,plane_model_params, durations);
			t2 = std::chrono::high_resolution_clock::now();
			/* END CLASSIFICATION + FITTING */

			long int detection_duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
			durations.push_back(detection_duration);
			
			/* VISUALIZE */
			if(visualize)
			{
				visualizer->removeCoordinateSystems();
				visualizer->addCoordinateSystem(0.3,plane_model_params.computeReferenceFrame(),std::to_string(plane_model_params.id)+"_reference_frame");
				plane_model_params.visualize(visualizer);
				for(unsigned int d=0; d<detections.clusters_fitting_data.size();++d)
				{
					std::string id=std::to_string(d)+"_cluster_point_cloud";
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(detections.clusters_fitting_data[d].inliers,  rand() % 255, rand() % 255, rand() % 255);
					visualizer->addPointCloud(clusters_point_clouds[d],id,rgb);
					detections.clusters_fitting_data[d].visualize(visualizer);
				}
			}
			/* END VISUALIZE */

			return detections;
		}
};

#endif // TABLE_TOP_DETECTOR_H
