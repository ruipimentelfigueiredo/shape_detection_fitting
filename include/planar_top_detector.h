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
#include <pcl/filters/statistical_outlier_removal.h>
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
	double clustering_voxel_size, x_filter_min, x_filter_max, z_filter_min,z_filter_max;
	bool visualize;
	std::string object_type; 
	PointCloudT::Ptr point_cloud;
	pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne_org;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
	pcl::PassThrough<pcl::PointXYZ> pass;

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
				double x_filter_min_=-0.4,
				double x_filter_max_= 0.4,
				double z_filter_min_=0.05,
				double z_filter_max_=5.00,
				bool with_classifier_=false,
				bool visualize_=true,
				bool dataset_create_=false,
				std::string dataset_path_="default_path",
				std::string object_type_="cylinder"
			) : 
				shape_detection_manager(new ShapeDetectionManager<cylinder_detector_type, sphere_detector_type>(
					cylinder_classifier_, 
					cylinder_fitting_,
					sphere_fitting_,
		 			cam_projection_,
					classification_threshold_,
					padding_,
					with_classifier_,
					dataset_create_,
					dataset_path_,
					object_type_)),
				plane_fitting(plane_fitting_),
				clustering_voxel_size(clustering_voxel_size_),
				x_filter_min(x_filter_min_),
				x_filter_max(x_filter_max_),
				z_filter_min(z_filter_min_),
				z_filter_max(z_filter_max_),
				visualize(visualize_),
				point_cloud(new PointCloudT),
				tree(new pcl::search::KdTree<pcl::PointXYZ>()),
				cloud_normals(new pcl::PointCloud<pcl::Normal>())

		{
			// Filtering parameters
			grid_.setLeafSize (clustering_voxel_size, clustering_voxel_size, clustering_voxel_size);
			grid_.setFilterFieldName ("z");
			grid_.setFilterLimits (z_filter_min, z_filter_max);
			grid_.setDownsampleAllData (false);

			// Normal estimation parameters (organized)
			ne_org.setNormalEstimationMethod(ne_org.AVERAGE_3D_GRADIENT); //COVARIANCE_MATRIX, AVERAGE_3D_GRADIENT, AVERAGE_DEPTH_CHANGE 
			ne_org.setMaxDepthChangeFactor(0.02f);
			ne_org.setNormalSmoothingSize(20.0f);

			// Normal estimation parameter (not organized)
			ne.setKSearch (6);
			ne.setSearchMethod(tree);
		};
	
		DetectionData detect(cv::Mat & image, const PointCloudRGB::Ptr & point_cloud_rgb, boost::shared_ptr<VisualizeFittingData> visualizer=NULL, std::vector<long int> & durations=std::vector<long int>())
		{
			pcl::copyPointCloud(*point_cloud_rgb, *point_cloud);

			pass.setInputCloud (point_cloud);
			pass.setKeepOrganized(true);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (z_filter_min, z_filter_max);
			pass.filter (*point_cloud);
			pass.setFilterFieldName ("x");
			pass.setFilterLimits (x_filter_min, x_filter_max);
			pass.filter (*point_cloud);

			// Normal estimation
			std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
			float* distance_map;
			if(point_cloud->isOrganized())
			{
				ne_org.setInputCloud(point_cloud);
				ne_org.compute(*cloud_normals);
				distance_map = ne_org.getDistanceMap ();
			}
			else
			{
				ne.setInputCloud(point_cloud);
				ne.compute(*cloud_normals);
			}
			std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

			FittingData plane_model_params;

			/* PLANE FITTING */
			try
			{
				t1 = std::chrono::high_resolution_clock::now();
				plane_model_params=plane_fitting->fit(point_cloud,cloud_normals,distance_map);
				t2 = std::chrono::high_resolution_clock::now();
			}
			catch (std::exception& e) 
			{
				std::string errorMessage = e.what();
				throw std::runtime_error(errorMessage);
			}
			/* END PLANE FITTING */

			long int plane_fitting_duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
			durations.push_back(plane_fitting_duration);

			/* CLUSTER EXTRACTION */
			std::vector<PointCloudT::Ptr> clusters_point_clouds;
			std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> clusters_;
			t1 = std::chrono::high_resolution_clock::now();
			plane_fitting->extractTabletopClusters(point_cloud,cloud_normals,clusters_point_clouds,clusters_);
			t2 = std::chrono::high_resolution_clock::now();
			long int cluster_extraction_duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
			durations.push_back(cluster_extraction_duration);
			/* END CLUSTER EXTRACTION */

			/* CLASSIFICATION + FITTING */
			DetectionData detections=shape_detection_manager->detect(image,clusters_point_clouds,clusters_,point_cloud_rgb,plane_model_params,durations);
			/* END CLASSIFICATION + FITTING */

			/* VISUALIZE */
			if(visualize && visualizer!=NULL)
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

		DetectionData detect(cv::Mat & image, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & point_cloud, std::vector<PointCloudT::Ptr> & clusters_point_clouds, FittingData & plane_model_params, boost::shared_ptr<VisualizeFittingData> visualizer=NULL, std::vector<long int> & durations=std::vector<long int>())
		{
			try
			{
				/* CLASSIFICATION + FITTING */
				DetectionData detections=shape_detection_manager->detect(image, clusters_point_clouds, plane_model_params, durations);
				/* END CLASSIFICATION + FITTING */

				/* VISUALIZE */
				if(visualize && visualizer!=NULL)
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
			catch (std::exception& e) 
			{
				DetectionData detection_data;
				return detection_data;
			}
		}
};
#endif // TABLE_TOP_DETECTOR_H

