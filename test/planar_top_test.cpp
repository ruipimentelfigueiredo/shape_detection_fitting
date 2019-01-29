/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!   
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include <ctime>
#include <chrono>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <tf/tf.h>

#include "config.h"
#include "helpers.h"
#include "planar_top_detector.h"

// ROS
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "annotations.h"

class GroundTruthDetection 
{
	public:
	cv::Rect bbox;
	cv::Scalar color;
	GroundTruthDetection(cv::Rect & bbox_, cv::Scalar & color_) : bbox(bbox_), color(color_)
	{};
};

class Filenames 
{
	public:
		std::vector<std::string> file_names;

		Filenames(const std::string & dataset_path_)
		{
			boost::filesystem::path p(dataset_path_);
			boost::filesystem::directory_iterator start(p);
			boost::filesystem::directory_iterator end;
			std::transform(start, end, std::back_inserter(file_names), path_leaf_string());
			std::sort(file_names.begin(), file_names.end());
		}
};

std::map<int, Color> id_colors_map_shape;
std::map<int, cv::Scalar> id_colors_map_bb;
const std::string marker_detections_namespace_ = "detections";

ros::Publisher shape_markers_pub; 
ros::Publisher cloud_markers_pub;
ros::Publisher image_pub; 
ros::Publisher image_detections_pub; 

ros::Publisher point_cloud_pub; 
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

visualization_msgs::Marker createCylinderMarker(const Eigen::VectorXf &model_params, int model_type, const std::string &frame, Color &color_, int id, const std::string &marker_namespace_)
{
	// Convert direction vector to quaternion
	tf::Vector3 axis_vector(model_params[3], model_params[4], model_params[5]);

	tf::Vector3 up_vector(0.0, 0.0, 1.0);
	tf::Quaternion q;
	if (axis_vector.dot(up_vector) > 0.99)
	{
		q = tf::createIdentityQuaternion();
	}
	else
	{
		tf::Vector3 right_vector = axis_vector.cross(up_vector);
		right_vector.normalized();
		q = tf::Quaternion(right_vector, -1.0 * acos(axis_vector.dot(up_vector)));
	}

	q.normalize();
	geometry_msgs::Quaternion cylinder_orientation;
	tf::quaternionTFToMsg(q, cylinder_orientation);
	float height = model_params[7];

	visualization_msgs::Marker marker;
	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time();
	marker.ns = marker_namespace_;
	marker.id = id;
	marker.type = model_type;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = model_params[0];
	marker.pose.position.y = model_params[1];
	marker.pose.position.z = model_params[2];
	marker.pose.orientation = cylinder_orientation;

	marker.scale.x = 2 * model_params[6];
	marker.scale.y = 2 * model_params[6];
	marker.scale.z = height;

	marker.color.a = color_.a;
	marker.color.r = color_.r;
	marker.color.g = color_.g;
	marker.color.b = color_.b;

	return marker;
}


std::vector<GroundTruthDetection> parseGroundTruth(std::string file, boost::shared_ptr<TiXmlDocument> doc)
{
	std::vector<GroundTruthDetection> ground_truth;

	// Load ground truth file
	doc->LoadFile(file);
	TiXmlElement *pRoot = doc->FirstChildElement("annotation");
	TiXmlElement *objParam;
	if(pRoot)
	{
		objParam = pRoot->FirstChildElement("object");

		while(objParam)
		{
			TiXmlElement* bndbox = objParam->FirstChildElement("bndbox");
			cv::Rect rect;
			if(bndbox)
			{
				int xmin(0),xmax(0),ymin(0),ymax(0);

				TiXmlElement *xminParam = bndbox->FirstChildElement("xmin");
				if(xminParam)
					xmin=atoi(xminParam->GetText());
				TiXmlElement *yminParam = bndbox->FirstChildElement("ymin");
				if(yminParam)
					ymin=atoi(yminParam->GetText());
				TiXmlElement *xmaxParam = bndbox->FirstChildElement("xmax");
				if(xmaxParam)
					xmax=atoi(xmaxParam->GetText());
				TiXmlElement *ymaxParam = bndbox->FirstChildElement("ymax");
				if(ymaxParam)
					ymax=atoi(ymaxParam->GetText());

				rect=cv::Rect(xmin,ymin,(xmax-xmin),(ymax-ymin));
			}
			else
				continue;

			TiXmlElement* typeParam = objParam->FirstChildElement("name");
			std::string type=std::string(typeParam->GetText());

			cv::Scalar color;
			if(type=="cylinder")
				color=cv::Scalar(0,0,255);
			else if(type=="sphere")
				color=cv::Scalar(0,255,0);
			else if(type=="box")
				color=cv::Scalar(255,0,0);

			objParam = objParam->NextSiblingElement("object");

			ground_truth.push_back(GroundTruthDetection(rect,color));
		}
	}
	return ground_truth;
};

visualization_msgs::Marker getCloudMarker(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	static bool first_time = true;
	if (first_time)
	{
		srand(time(NULL));
		first_time = false;
	}

	//create the marker
	visualization_msgs::Marker marker;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(5);

	marker.type = visualization_msgs::Marker::POINTS;
	marker.scale.x = 0.002;
	marker.scale.y = 0.002;
	marker.scale.z = 0.002;

	marker.color.r = ((double)rand()) / RAND_MAX;
	marker.color.g = ((double)rand()) / RAND_MAX;
	marker.color.b = ((double)rand()) / RAND_MAX;
	marker.color.a = 1.0;

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		geometry_msgs::Point p;
		p.x = cloud->points[i].x;
		p.y = cloud->points[i].y;
		p.z = cloud->points[i].z;
		marker.points.push_back(p);
	}

	//the caller must decide the header; we are done here
	return marker;
}

void visualize(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, DetectionData &detections, cv_bridge::CvImagePtr &cv_ptr)
{
	visualization_msgs::MarkerArray shape_markers_;
	visualization_msgs::MarkerArray cloud_markers_;

	visualization_msgs::Marker clean_shapes_marker_;
	clean_shapes_marker_.action = 3;
	shape_markers_.markers.push_back(clean_shapes_marker_);
	cloud_markers_.markers.push_back(clean_shapes_marker_);

	std::vector<cv::Scalar> colors_cv;
	visualization_msgs::Marker cloud_marker=getCloudMarker(detections.plane_fitting_data.inliers);

	pcl_conversions::fromPCL(cloud->header, cloud_marker.header);
	cloud_marker.header.frame_id="world";

	cloud_marker.pose.orientation.w = 1;
	cloud_marker.ns = "table";
	cloud_markers_.markers.push_back(cloud_marker);

	// Get table contour marker
	visualization_msgs::Marker table_contour_cloud_marker=getCloudMarker(detections.plane_fitting_data.contour);

	table_contour_cloud_marker.header.frame_id="world";
	table_contour_cloud_marker.pose.orientation.w = 1;
	table_contour_cloud_marker.ns = "table_contour";
	cloud_markers_.markers.push_back(table_contour_cloud_marker);

	for (unsigned int ind = 0; ind < detections.clusters_fitting_data.size(); ++ind)
	{	
		// Cylinder shape
		if (detections.clusters_fitting_data[ind].type == FittingData::CYLINDER)
		{
			if (detections.clusters_fitting_data[ind].parameters.size() == 0)
				continue;
				
			// Cluster inliers marker
			visualization_msgs::Marker cluster_marker=getCloudMarker(detections.clusters_fitting_data[ind].inliers);
			cluster_marker.header = cloud_marker.header;
			cluster_marker.pose.orientation.w = 1;
			cluster_marker.ns = "clusters";
			cluster_marker.id=ind;
			cloud_markers_.markers.push_back(cluster_marker);

			// Shape marker
			FittingData fitting_data = detections.clusters_fitting_data[ind];
			Eigen::VectorXf model_params = fitting_data.parameters;

			Color color_;
			cv::Scalar color_cv;
			visualization_msgs::Marker marker;

			// If no classifier and fitting error less than a given threshold -> cylinder
			if(!detections.with_classifier)
			{
				color_cv = id_colors_map_bb.find(2)->second;
				if(detections.clusters_fitting_data[ind].confidence>FittingData::fitting_threshold)
				{
					color_ = id_colors_map_shape.find(0)->second;
				}
				else 
				{
					color_ = id_colors_map_shape.find(1)->second;
				}
			}
			else
			{
				color_ = id_colors_map_shape.find(0)->second;
				color_cv = id_colors_map_bb.find(0)->second;
			}

			marker = createCylinderMarker(model_params, visualization_msgs::Marker::CYLINDER, cloud->header.frame_id, color_, ind, marker_detections_namespace_);
			marker.header = cloud_marker.header;
			colors_cv.push_back(color_cv);

			shape_markers_.markers.push_back(marker);
		}
		else
		{
			cv::Scalar color_cv = id_colors_map_bb.find(1)->second;
			colors_cv.push_back(color_cv);
		}
	}

	// Publish data
	image_pub.publish(cv_ptr);
	cv_bridge::CvImagePtr cv_ptr_det(new cv_bridge::CvImage());
	*cv_ptr_det=*cv_ptr;
	detections.draw_bounding_boxes(cv_ptr_det->image, colors_cv);
	image_detections_pub.publish(cv_ptr_det);
	shape_markers_pub.publish(shape_markers_);
	cloud_markers_pub.publish(cloud_markers_);
	point_cloud_pub.publish(cloud);
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "shape_detection_fitting");
	ros::NodeHandle n;

	// Arguments
	std::string config_file=std::string(argv[1]);

	Config config;
	config.parseConfig(config_file);

	std::vector<std::string> image_files,annotation_files,point_cloud_files;

	std::string dataset_path=config.dataset_path;

	// Test set path
	std::string images_dataset_path=dataset_path+"/images/"+config.object_type+"/";
	std::string point_clouds_dataset_path=dataset_path+"/pointclouds/"+config.object_type+"/";
	std::string annotations_dataset_path=dataset_path+"/annotations/"+config.object_type+"/"; 

	std::string results_path;
	std::string results_images_clusters_path;
	std::string results_annotations_path;
	std::string results_times_path;
	std::string resuls_score_path;

	// Results path
	if(config.with_classifier)
	{
		results_path=dataset_path+"/results/"+config.object_type+"/with_classifier/";
	}
	else
	{
		results_path=dataset_path+"/results/"+config.object_type+"/no_classifier/";
	}

	results_images_clusters_path=results_path+"images_clusters/";
	results_annotations_path    =results_path+"annotations/";
	results_times_path    =results_path+"times/";
	resuls_score_path     =results_path+"scores/";

	boost::shared_ptr<VisualizeFittingData> visualizer;
	
	if(config.visualize)
		visualizer=boost::shared_ptr<VisualizeFittingData>(new VisualizeFittingData());

	boost::shared_ptr<TiXmlDocument> doc(new TiXmlDocument());

	createDirectory(results_images_clusters_path);
	createDirectory(results_annotations_path);
	createDirectory(results_times_path);
	createDirectory(resuls_score_path);

	boost::shared_ptr<CylinderClassifier> cylinder_classifier;
	if(config.with_classifier)
		cylinder_classifier=boost::shared_ptr<CylinderClassifier> (new CylinderClassifier(config.classifier_absolute_path_folder,config.model_file,config.weight_file,config.mean_file,config.device,(unsigned int)config.device_id));

	GaussianSphere gaussian_sphere(config.gmm,config.gaussian_sphere_points_num,config.orientation_accumulators_num);

	boost::shared_ptr<SphereFittingHough> sphere_fitting(new SphereFittingHough(gaussian_sphere,config.position_bins,config.radius_bins,config.min_radius, config.max_radius,config.accumulator_peak_threshold));
	boost::shared_ptr<CylinderFittingHough> cylinder_fitting(new CylinderFittingHough(gaussian_sphere,config.angle_bins,config.radius_bins,config.position_bins,config.min_radius, config.max_radius,config.accumulator_peak_threshold,config.hough_fitting_mode));
	boost::shared_ptr<PlaneFittingRansac> plane_fitting(new PlaneFittingRansac(config.distance_threshold,config.cluster_tolerance,config.min_cluster_size,config.max_cluster_size,config.do_refine,config.table_z_filter_min,config.table_z_filter_max,config.z_filter_min, config.z_filter_max,config.plane_detection_voxel_size, config.cluster_voxel_size,config.inlier_threshold,config.angular_threshold));
	
	FittingData::fitting_threshold=config.fitting_threshold;

	boost::shared_ptr<PlanarTopDetector<CylinderFittingHough, SphereFittingHough, PlaneFittingRansac> > planar_top_detector(new PlanarTopDetector<CylinderFittingHough, SphereFittingHough, PlaneFittingRansac>(plane_fitting, 
		cylinder_classifier, 
		cylinder_fitting, 
		sphere_fitting, 
		config.cam_projection, 
		config.classification_threshold, 
		config.padding, 
		config.cluster_voxel_size, 
		config.x_filter_min, 
		config.x_filter_max,
		config.z_filter_min, 
		config.z_filter_max,
		config.with_classifier, 
		config.visualize, 
		false, 
		config.dataset_path, 
		config.object_type));

	std::fstream plane_fitting_time;
	std::fstream cluster_extraction_time;
	std::fstream cluster_classification_time;
	std::fstream cluster_fitting_time;
	std::fstream cluster_fitting_score;

	plane_fitting_time.open(results_times_path+"plane_fitting_time.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
	cluster_extraction_time.open(results_times_path+"cluster_extraction_time.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
	cluster_classification_time.open(results_times_path+"cluster_classification_time.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
	cluster_fitting_time.open(results_times_path+"cluster_fitting_time.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
	cluster_fitting_score.open(resuls_score_path+"cluster_fitting_score.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);

	// Initialize visualization colors
	id_colors_map_shape.insert(std::pair<int, Color>(0, Color(0, 1, 0, 1.0)));
	id_colors_map_shape.insert(std::pair<int, Color>(1, Color(0, 0, 1, 1.0)));
	id_colors_map_shape.insert(std::pair<int, Color>(2, Color(0.4, 0.1, 0.4)));

	id_colors_map_bb.insert(std::pair<int, cv::Scalar>(0, cv::Scalar(0, 255, 0)));
	id_colors_map_bb.insert(std::pair<int, cv::Scalar>(1, cv::Scalar(0, 0, 255)));
	id_colors_map_bb.insert(std::pair<int, cv::Scalar>(2, cv::Scalar(255, 0, 0)));

	// Initialize advertisers
	shape_markers_pub = n.advertise<visualization_msgs::MarkerArray>("shape_detections_vis", 1);
	cloud_markers_pub = n.advertise<visualization_msgs::MarkerArray>("inlier_detections_vis", 1);
	image_pub         = n.advertise<sensor_msgs::Image>("image", 1);
	image_detections_pub         = n.advertise<sensor_msgs::Image>("image_detections", 1);

	point_cloud_pub   = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("cloud_in", 1);

	unsigned int total_directories=sub_directory_count(images_dataset_path);
	unsigned int total_iters=0;
	unsigned int current_dir=0;

	// For each subdirectory
	for (boost::filesystem::directory_iterator itr(images_dataset_path); itr!=boost::filesystem::directory_iterator() && ros::ok(); ++itr)
	{
		current_dir++;
		std::string images_directory_path	    = images_dataset_path+itr->path().filename().string()+"/";
		std::string pointclouds_directory_path  = point_clouds_dataset_path+itr->path().filename().string()+"/";
		std::string ground_truth_directory_path = annotations_dataset_path+itr->path().filename().string()+"/";

		// Get point clouds
		PointClouds point_clouds(pointclouds_directory_path);
		std::vector<std::string> point_clouds_file_names=Filenames(pointclouds_directory_path).file_names;
		std::vector<std::string> images_file_names=Filenames(images_directory_path).file_names;
		std::vector<std::string> annotation_files=Filenames(ground_truth_directory_path).file_names;

		// Read filenames from directory
		//readDirectory(images_directory_path,images_file_names);
		//readDirectory(pointclouds_directory_path,point_cloud_files);
		//readDirectory(ground_truth_directory_path,annotation_files);

		std::string results_annotations_path_current=results_annotations_path+itr->path().filename().string()+"/";
		createDirectory(results_annotations_path_current);

		// For each file
		for(unsigned int i=0;i<point_clouds_file_names.size() && ros::ok();++i)
		{
			try
			{
				// Load RGB data in OpenCV format
				cv::Mat image=cv::imread(images_directory_path+images_file_names[i], CV_LOAD_IMAGE_COLOR);    // Read the file
				if(! image.data )                              											// Check for invalid input
				{
					std::cout <<  "Could not open or find the image " << image_files[i] << std::endl ;
					return -1;
				}

				// Load 3D point cloud information in PCL format
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>());
				pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
				
				point_cloud_color=point_clouds.loadPointCloudRGB(pointclouds_directory_path+point_clouds.file_names[i]);
				//point_cloud=point_clouds.loadPointCloud(pointclouds_directory_path+point_clouds.file_names[i]);
				pcl::copyPointCloud(*point_cloud_color, *point_cloud);

				point_cloud->header.frame_id=std::string("world");

				// Load ground truth file
				std::vector<GroundTruthDetection> bboxes=parseGroundTruth(ground_truth_directory_path+annotation_files[i],doc);
				std::vector<long int> durations;
				point_cloud_color->header.frame_id="world";

				/* VISUALIZE GROUND TRUTH */
				if(config.visualize_ground_truth)
				{
					visualizer->clear();
					for(unsigned int i = 0; i<bboxes.size(); ++i)
					{
						cv::rectangle(image, bboxes[i].bbox, bboxes[i].color, 2, 4, 0 );	
					}
				}
				/* END VISUALIZE GROUND TRUTH */

				DetectionData detections;//=planar_top_detector->detect(image,point_cloud_color,visualizer,durations);
				try 
				{
					detections=planar_top_detector->detect(image,point_cloud_color,visualizer,durations);
				}
				catch (std::exception& e) 
				{
					std::cout << "iteration: " << i << std::endl;
					std::cout << e.what() << std::endl;
					continue;
				}

				/* VISUALIZE */
				cv_bridge::CvImagePtr cv_img(new cv_bridge::CvImage());
				cv_img->image=image;
				cv_img->encoding="bgr8";
				visualize(point_cloud_color, detections, cv_img);
				/* END VISUALIZE */

				/* RECORD RESULTS DATA */
				// TIMMINGS
				plane_fitting_time              << durations[0] << " \n";
				cluster_extraction_time         << durations[1] << " \n";
				cluster_classification_time 	<< durations[2] << " \n";
				cluster_fitting_time 			<< durations[3] << " \n";
				for(unsigned int i=0; i<detections.clusters_fitting_data.size();++i)
					cluster_fitting_score 	<< detections.clusters_fitting_data[i].confidence << " \n";

				// FITTING SCORE (TODO: ADD FITTING SCORE TO THE XML FILE)

				// DETECTION
				std::vector<cv::Rect> bounding_boxes;
        		std::vector<PointCloudT::Ptr> pcl_clusters;
				annotations::storeTestData(results_images_clusters_path,results_annotations_path_current,annotation_files[i],(i+1),detections,config.with_classifier); 
				/* END RECORD RESULTS DATA */
				++total_iters;
				double total_time;
				total_time = std::accumulate(durations.begin(), durations.end(), 0);
				std::cout 	<< "with_classifier: " << config.with_classifier << " set: " << current_dir << " of " << total_directories 
							<< " iteration: " << (i+1) << " of " << point_clouds.file_names.size() 
							<< " total iterations: " << total_iters  
							<< " plane fitting time: " << durations[0] << " ms "
							<< " cluster extraction time: " << durations[1] << " ms "
							<< " classification: " << durations[2] << " ms "
							<< " fitting: " << durations[3] << " ms "
							<< " total_time: " << total_time << " ms "

							<< std::endl;
			}
			catch (exception& e) 
			{
				std::cout << e.what() << std::endl;
				continue;
			}
			ros::Duration(0.1).sleep(); // sleep for visualization purposes
		}
	}
	plane_fitting_time.close();
	cluster_extraction_time.close();
	cluster_classification_time.close();
	cluster_fitting_time.close();
	return 0;
}
