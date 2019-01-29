/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/
#ifndef SHAPEFITTINGROS_H
#define SHAPEFITTINGROS_H

//#include "multiple_tracker_manager.h"
#include <sensor_msgs/CameraInfo.h>
#include <object_recognition_msgs/TableArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
//#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include "planar_top_detector.h"
#include "config.h"
#include "shape_detection_fitting/Clusters.h"
#include "shape_detection_fitting/Shapes.h"
#include "cylinder_classifier.h"
#include "shape_detection_manager.h"

#include <vision_msgs/Detection2DArray.h>

template <class cylinder_detector_type, class sphere_detector_type, class plane_detector_type>
class ShapeFittingROS
{
	const std::string marker_detections_namespace_ = "detections";
	//const std::string marker_trackers_namespace_ = "trackers";

	std::map<int, Color> id_colors_map_shape;
	std::map<int, cv::Scalar> id_colors_map_bb;

	ros::NodeHandle n;
	ros::NodeHandle n_priv;
	image_transport::ImageTransport it;

	//MultipleTrackerManager tracker_manager;

	ros::Publisher shapes_pub;
	image_transport::Publisher image_pub;
	ros::Publisher shape_markers_pub;
	ros::Publisher cloud_markers_pub;

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

	typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudConstPtr;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, PointCloudRGB> MySyncPolicy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, object_recognition_msgs::TableArray, shape_detection_fitting::Clusters> MySyncPolicy2;

	boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> image_sub;
	boost::shared_ptr<message_filters::Subscriber<PointCloudRGB>> pcl_sub;
	boost::shared_ptr<message_filters::Subscriber<shape_detection_fitting::Clusters>> cluster_sub;
	boost::shared_ptr<message_filters::Subscriber<object_recognition_msgs::TableArray>> plane_sub;

	boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;

	boost::shared_ptr<PlanarTopDetector<cylinder_detector_type, sphere_detector_type, plane_detector_type>> planar_top_detector;
	
	bool dataset_create;
	double x_filter_min, x_filter_max, z_filter_min, z_filter_max;

	boost::shared_ptr<VisualizeFittingData> visualizer;


	void callback_detections(const vision_msgs::Detection2DArray::ConstPtr &detections, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_)
	{
		static int iteration = 0;
		if(!cloud_->isOrganized())
			return;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::copyPointCloud(*cloud_, *cloud);

		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

		// Normal estimation parameters (organized)
		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne_org;

		ne_org.setInputCloud(cloud);
		ne_org.compute(*normals);
		double distance_map = ne_org.getDistanceMap ();
		ne_org.setNormalEstimationMethod(ne_org.COVARIANCE_MATRIX); //COVARIANCE_MATRIX, AVERAGE_3D_GRADIENT, AVERAGE_DEPTH_CHANGE 
		ne_org.setMaxDepthChangeFactor(0.02f);
		ne_org.setNormalSmoothingSize(20.0f);

		// Merge normals and point clouds
		pcl::concatenateFields (*cloud, *normals, *cloud_normals);

		std::vector<cv::Rect> bounding_boxes;
		for(unsigned int i=0; i<detections->detections.size(); ++i)
		{
			// Compute bounding box
			double top_left_x=detections->detections[i].bbox.center.x-detections->detections[i].bbox.size_x*0.5;
			double top_left_y=detections->detections[i].bbox.center.y-detections->detections[i].bbox.size_y*0.5;
			cv::Rect bounding_box(top_left_x,top_left_y, detections->detections[i].bbox.size_x, detections->detections[i].bbox.size_y );
			bounding_boxes.push_back(bounding_box);
		}

		const float bad_point = std::numeric_limits<float>::quiet_NaN();
		
		// Traverse the point cloud and set pixels not within the bounding boxes to NaN
		std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr > clouds_;
		pcl::PassThrough<pcl::PointNormal> pass;
		for(unsigned int b=0; b<detections->detections.size();++b)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud(new pcl::PointCloud<pcl::PointNormal>());
			for(unsigned int col=bounding_boxes[b].x; col<bounding_boxes[b].x+bounding_boxes[b].width; ++col)
			{
				for(unsigned int row=bounding_boxes[b].y; row<bounding_boxes[b].y+bounding_boxes[b].height; ++row)
				{
					// Set value to NaN
					point_cloud->push_back(clouds_[b]->points[row * point_cloud->width + col]);
				}
			}

			// Build a passthrough filter to remove spurious NaNs
			pass.setInputCloud (point_cloud);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (z_filter_min, z_filter_max);
			pass.setKeepOrganized(true);
			pass.filter (*point_cloud);

			// Save point cloud
			clouds_.push_back(point_cloud);
		}
		
		/* PLANE FITTING */
		FittingData plane_model_params;
		try
		{
			//t1 = std::chrono::high_resolution_clock::now();
			plane_model_params=planar_top_detector.plane_fitting->fit(cloud,normals,distance_map);
			//t2 = std::chrono::high_resolution_clock::now();
		}
		catch (std::exception& e) 
		{
			std::cout << e.what() << std::endl;
			std::string errorMessage = e.what();
			throw std::runtime_error(errorMessage);
		}
		/* END PLANE FITTING */
		//long int plane_fitting_duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();

		std::vector<long int> durations;
		std::vector<FittingData> shape_fitting_data; 
		shape_fitting_data.resize(clouds_.size());
		Eigen::Affine3d plane_transform=plane_model_params.computeReferenceFrame();
		for(unsigned int ind=0; ind<clouds_.size(); ++ind)
		{
			// If object type is cylinder
			bool cylinder=true;
			if(cylinder)
			{	
				pcl::PointCloud<pcl::PointNormal>::Ptr cluster_projected(new pcl::PointCloud<pcl::PointNormal>());

				pcl::transformPointCloudWithNormals(*clouds_[ind], *cluster_projected, plane_transform.inverse());

				pcl::PointCloud<pcl::Normal>::Ptr normal_cluster_projected(new pcl::PointCloud<pcl::Normal>);
				pcl::copyPointCloud(*cluster_projected, *normal_cluster_projected);

				pcl::PointCloud<pcl::PointXYZ>::Ptr point_cluster_projected(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::copyPointCloud(*cluster_projected, *point_cluster_projected);

				try
				{
					shape_fitting_data[ind]=planar_top_detector.cylinder_fitting->fit(point_cluster_projected,normal_cluster_projected);
				}
				catch (const std::exception& ex)
				{
					std::cout << ex.what() << std::endl;
					continue;
				}
			
				Eigen::Vector3f position=shape_fitting_data[ind].parameters.segment(0,3);
				Eigen::Vector3f direction=shape_fitting_data[ind].parameters.segment(3,3);

				shape_fitting_data[ind].parameters.segment(0,3)=(plane_transform.linear().cast<float>()*position).cast<float>();
				shape_fitting_data[ind].parameters.segment(3,3)=(plane_transform.rotation().cast<float>()*direction).cast<float>();

				// Convert point cloud back to original frame (for visualization purposes)
				pcl::transformPointCloud (*shape_fitting_data[ind].inliers, *shape_fitting_data[ind].inliers, plane_transform);
			}
			else 
			{
				continue;
			}
		}
		/* END DETECT */
	}

	void callback(const sensor_msgs::Image::ConstPtr &input_image, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		static int iteration = 0;
		try
		{
			//std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());

			pcl::copyPointCloud(*cloud, *cloud_rgb);

			// Build a passthrough filter to remove spurious NaNs
			pcl::PassThrough<pcl::PointXYZRGB> pass;
			pass.setInputCloud (cloud_rgb);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (z_filter_min, z_filter_max);
			pass.setKeepOrganized(true);
			pass.filter (*cloud_rgb);

			pass.setFilterFieldName ("x");
			pass.setFilterLimits (x_filter_min, x_filter_max);
			pass.setKeepOrganized(true);
			pass.filter (*cloud_rgb);

			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(input_image, "bgr8");
			//std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
			//long int filtering_duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
			
			/* DETECT */
			std::vector<long int> durations;
			DetectionData detections = planar_top_detector->detect(cv_ptr->image, cloud_rgb, visualizer, durations);
			/* END DETECT */

			/* VISUALIZE */
			visualize(cloud, detections, cv_ptr);
			/* END VISUALIZE */

			double total_time;
			total_time = std::accumulate(durations.begin(), durations.end(), 0);
			if(dataset_create)
				ROS_INFO_STREAM("iteration: " << (iteration++) << " plane fitting time: " << durations[0] << " ms"
											<< " cluster extraction time: " << durations[1] << " ms"
											<< " classification: " << durations[2] << " ms"
											<< " total_time: " << total_time << " ms");
			else
				ROS_INFO_STREAM("iteration: " << (iteration++) << " plane fitting time: " << durations[0] << " ms"
											<< " cluster extraction time: " << durations[1] << " ms"
											<< " classification: " << durations[2] << " ms"
											<< " fitting: " << durations[3] << " ms"
											<< " total_time: " << total_time << " ms");
		}
		catch (exception &e)
		{
			return;
		}
	}

	visualization_msgs::Marker getCloudMarker(const PointCloud::ConstPtr &cloud)
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
		marker.lifetime = ros::Duration(0.1);

		marker.type = visualization_msgs::Marker::POINTS;
		marker.scale.x = 0.002;
		marker.scale.y = 0.002;
		marker.scale.z = 0.002;

		marker.color.r = ((double)rand()) / RAND_MAX;
		marker.color.g = ((double)rand()) / RAND_MAX;
		marker.color.b = ((double)rand()) / RAND_MAX;
		marker.color.a = 1.0;

		if(cloud!=NULL)
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

	void visualize(const PointCloudRGB::ConstPtr &cloud, DetectionData &detections, cv_bridge::CvImagePtr &cv_ptr)
	{
		visualization_msgs::MarkerArray shape_markers_;
		visualization_msgs::MarkerArray cloud_markers_;

		visualization_msgs::Marker clean_shapes_marker_;
		clean_shapes_marker_.action = 3;
		shape_markers_.markers.push_back(clean_shapes_marker_);

		shape_detection_fitting::Shapes shapes_msg;
		pcl_conversions::fromPCL(cloud->header, shapes_msg.header);
		std::vector<cv::Scalar> colors_cv;
		
		// Get table marker
		visualization_msgs::Marker cloud_marker=getCloudMarker(detections.plane_fitting_data.inliers);

		cloud_marker.header = shapes_msg.header;
		cloud_marker.pose.orientation.w = 1;
		cloud_marker.ns = "table";
		cloud_markers_.markers.push_back(cloud_marker);

		// Get table contour marker
		visualization_msgs::Marker table_contour_cloud_marker=getCloudMarker(detections.plane_fitting_data.inliers);

		table_contour_cloud_marker.header = shapes_msg.header;
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

				// Get Marker
				visualization_msgs::Marker cloud_marker=getCloudMarker(detections.clusters_fitting_data[ind].inliers);
				cloud_marker.header = shapes_msg.header;
				cloud_marker.pose.orientation.w = 1;
				cloud_marker.ns = "clusters";
				cloud_marker.id=ind;
				cloud_markers_.markers.push_back(cloud_marker);
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

				colors_cv.push_back(color_cv);

				shape_markers_.markers.push_back(marker);
				shape_detection_fitting::Shape shape;
				shape.parameters.layout.dim.resize(2);
				shape.parameters.layout.dim[0].label = "shape";
				shape.parameters.layout.dim[0].size = 0;
				shape.parameters.layout.dim[0].stride = 0;
				shape.parameters.layout.dim[1].label = "parameters";
				shape.parameters.layout.dim[1].size = model_params.size();
				shape.parameters.layout.dim[1].stride = model_params.size();

				for (unsigned int p = 0; p < model_params.size(); ++p)
				{
					shape.parameters.data.push_back((float)model_params[p]);
				}
				shapes_msg.shapes.push_back(shape);
			}
			else
			{
				cv::Scalar color_cv = id_colors_map_bb.find(1)->second;
				colors_cv.push_back(color_cv);
			}
		}

		detections.draw_bounding_boxes(cv_ptr->image, colors_cv);
		// Publish data
		image_pub.publish(cv_ptr->toImageMsg());
		shape_markers_pub.publish(shape_markers_);
		shapes_pub.publish(shapes_msg);
		cloud_markers_pub.publish(cloud_markers_);
	}

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
		marker.lifetime=ros::Duration(0.1);

		return marker;
	}

  public:


	ShapeFittingROS(ros::NodeHandle &n_,
					ros::NodeHandle &n_priv_,
					boost::shared_ptr<PlanarTopDetector<cylinder_detector_type, sphere_detector_type, plane_detector_type>> &planar_top_detector_,
					bool dataset_create_=false,
					double x_filter_min_=.0,
					double x_filter_max_=5.0,
					double z_filter_min_=.0,
					double z_filter_max_=5.0
					) : 
						n(n_),
						n_priv(n_priv_),
						it(n_),
						planar_top_detector(planar_top_detector_),
						dataset_create(dataset_create_),
						x_filter_min(x_filter_min_),
						x_filter_max(x_filter_max_),
						z_filter_min(z_filter_min_),
						z_filter_max(z_filter_max_),
						visualizer(boost::shared_ptr<VisualizeFittingData>())						
	{
		// Initialize visualization colors
		id_colors_map_shape.insert(std::pair<int, Color>(0, Color(0, 1, 0, 1.0)));
		id_colors_map_shape.insert(std::pair<int, Color>(1, Color(0, 0, 1, 1.0)));
		id_colors_map_shape.insert(std::pair<int, Color>(2, Color(0.4, 0.1, 0.4)));

		id_colors_map_bb.insert(std::pair<int, cv::Scalar>(0, cv::Scalar(0, 255, 0)));
		id_colors_map_bb.insert(std::pair<int, cv::Scalar>(1, cv::Scalar(0, 0, 255)));
		id_colors_map_bb.insert(std::pair<int, cv::Scalar>(2, cv::Scalar(255, 0, 0)));

		// Initialize advertisers
		shapes_pub = n.advertise<shape_detection_fitting::Shapes>("shape_detections_params", 1);
		shape_markers_pub = n.advertise<visualization_msgs::MarkerArray>("shape_detections_markers", 1);
		cloud_markers_pub = n.advertise<visualization_msgs::MarkerArray>("clusters_inliers_pcl", 1);
		image_pub = it.advertise("shape_detections_image", 1);

		// Initialize subscribers
		pcl_sub = boost::shared_ptr<message_filters::Subscriber<PointCloudRGB>>(new message_filters::Subscriber<PointCloudRGB >(n, "pcl_in", 10));
		image_sub = boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(new message_filters::Subscriber<sensor_msgs::Image>(n, "image_in", 10));
		
		sync = boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>>(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub, *pcl_sub));
		sync->registerCallback(boost::bind(&ShapeFittingROS<cylinder_detector_type, sphere_detector_type, plane_detector_type>::callback, this, _1, _2));
	}
};

#endif // SHAPEFITTINGROS_H
