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
#include "shape_detection_fitting/Clusters.h"
#include "shape_detection_fitting/Shapes.h"
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/filters/voxel_grid.h>
#include <cv_bridge/cv_bridge.h>
//#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include "cylinder_classifier.h"
#include "shape_detection_manager.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include "planar_top_detector.h"
#include "config.h"
class Color
{
	public:
		Color(double r_=0.0,double g_=0.0, double b_=0.0, double a_=1.0) : r(r_),g(g_),b(b_),a(a_)
		{}
		double r,g,b,a;
};

template <class cylinder_detector_type,class sphere_detector_type, class plane_detector_type>
class ShapeFittingROS {

	double fitting_acceptance_threshold;

	ros::Time odom_last_stamp;
	std::string odom_link;

	const std::string marker_detections_namespace_ ="detections";
	const std::string marker_trackers_namespace_ ="trackers";

	std::map<int, Color> id_colors_map;
		
	ros::NodeHandle n;
	ros::NodeHandle n_priv;

	//MultipleTrackerManager tracker_manager;
	
	ros::Publisher shapes_pub;
	ros::Publisher image_pub;
	ros::Publisher cluster_pub;
	ros::Publisher vis_pub;

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudConstPtr;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, PointCloud> MySyncPolicy;

	boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > image_sub;
	boost::shared_ptr<message_filters::Subscriber<PointCloud> > pcl_sub;
    	boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > sync;
	boost::shared_ptr<VisualizeFittingData> visualizer;

	boost::shared_ptr<PlanarTopDetector<cylinder_detector_type, sphere_detector_type, plane_detector_type> > planar_top_detector;

	void callback (const sensor_msgs::Image::ConstPtr & input_image, const PointCloud::ConstPtr & cloud)
	{
		static int iteration=0;
		try
		{
			PointCloudT::Ptr cloud_test(new PointCloudT());
			*cloud_test=*cloud;

			vector<int> indices;
			pcl::removeNaNFromPointCloud (*cloud, *cloud_test, indices);
			cv_bridge::CvImagePtr cv_ptr=cv_bridge::toCvCopy(input_image, "bgr8");
			//cv::Mat image=cv_ptr->image;

			/* DETECT */
			std::vector<long int> durations;
			DetectionData detections=planar_top_detector->detect(cv_ptr->image,cloud_test,visualizer,durations);
			/* END DETECT */

			std::vector<int> shapes_indices;
			shapes_indices.reserve(detections.clusters_fitting_data.size());

			/* VISUALIZE */
			visualize(cloud,detections,cv_ptr);
			/* END VISUALIZE */

			ROS_INFO_STREAM("iteration: " << (iteration+1) << " plane fitting time: " << durations[0] << " ms " << " cluster extraction time: " << durations[1] << " ms" << " classification: " << durations[2] << " ms  fitting: " << durations[3] << " ms");
		}
		catch(exception& e)
		{
			return;
		}
	}

	void clusters_cb (const visualization_msgs::MarkerArray::ConstPtr& input_clusters)
	{	
		if(input_clusters->markers.size()==0) return;

		shape_detection_fitting::Clusters clusters;
		clusters.header=input_clusters->markers[0].header;
		clusters.markers=*input_clusters;
		cluster_pub.publish(clusters);
		return;
	}

	void visualize(const PointCloud::ConstPtr & cloud, DetectionData & detections, cv_bridge::CvImagePtr & cv_ptr)
	{
		visualization_msgs::MarkerArray markers_;
		visualization_msgs::Marker marker_;
		marker_.action = 3;
		markers_.markers.push_back(marker_);
		shape_detection_fitting::Shapes shapes_msg;

		pcl_conversions::fromPCL(cloud->header, shapes_msg.header);
		for(unsigned int ind=0; ind<detections.clusters_fitting_data.size(); ++ind)
		{
			FittingData fitting_data=detections.clusters_fitting_data[ind];
			Eigen::VectorXf model_params=fitting_data.parameters;
			double confidence=detections.clusters_classification_data[ind].confidence;
			Color color_;
			
			if(confidence<fitting_acceptance_threshold)
				color_=id_colors_map.find(0)->second;
			else
				color_=id_colors_map.find(1)->second;

			visualization_msgs::Marker marker;
			if(confidence>0)
			      marker=createCylinderMarker(model_params,visualization_msgs::Marker::CYLINDER,cloud->header.frame_id,color_,ind,marker_detections_namespace_);

			markers_.markers.push_back(marker);

			shape_detection_fitting::Shape shape;
			shape.parameters.layout.dim.resize(2);
			shape.parameters.layout.dim[0].label  = "shape";
			shape.parameters.layout.dim[0].size   = 0;
			shape.parameters.layout.dim[0].stride = 0;
			shape.parameters.layout.dim[1].label  = "parameters";
			shape.parameters.layout.dim[1].size   = model_params.size();
			shape.parameters.layout.dim[1].stride = model_params.size();

			for(unsigned int p=0; p<model_params.size();++p)
			{
				shape.parameters.data.push_back((float)model_params[p]);
			}

			shapes_msg.shapes.push_back(shape);

		}
		detections.draw_bounding_boxes(cv_ptr->image);
		image_pub.publish(cv_ptr);
		vis_pub.publish(markers_);
		shapes_pub.publish(shapes_msg);
	}



	visualization_msgs::Marker createCylinderMarker(const Eigen::VectorXf & model_params, int model_type, const std::string & frame, Color & color_, int id, const std::string & marker_namespace_)
	{
		// Convert direction vector to quaternion
		tf::Vector3 axis_vector(model_params[3], model_params[4], model_params[5]);

		tf::Vector3 up_vector(0.0, 0.0, 1.0);
		tf::Quaternion q;
		if(axis_vector.dot(up_vector)>0.99)
		{
			q=tf::createIdentityQuaternion();
		}
		else
		{
			tf::Vector3 right_vector = axis_vector.cross(up_vector);
			right_vector.normalized();
			q=tf::Quaternion(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
		}

		q.normalize();
		geometry_msgs::Quaternion cylinder_orientation;
		tf::quaternionTFToMsg(q, cylinder_orientation);
		float height=model_params[7];

		visualization_msgs::Marker marker;
		marker.header.frame_id =frame;
		marker.header.stamp = ros::Time();
		marker.ns = marker_namespace_;
		marker.id = id;
		marker.type = model_type;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = model_params[0];
		marker.pose.position.y = model_params[1];
		marker.pose.position.z = model_params[2];
		marker.pose.orientation = cylinder_orientation;

		marker.scale.x = 2*model_params[6];
		marker.scale.y = 2*model_params[6];
		marker.scale.z = height;

		marker.color.a = color_.a;
		marker.color.r = color_.r;
		marker.color.g = color_.g;
		marker.color.b = color_.b;

		//marker.lifetime = ros::Duration(0.05);
		return marker;
	}


public:

	ShapeFittingROS(ros::NodeHandle & n_, 
			ros::NodeHandle & n_priv_,
			boost::shared_ptr<PlanarTopDetector<cylinder_detector_type, sphere_detector_type, plane_detector_type> > & planar_top_detector_) : 
		n(n_), 
		n_priv(n_priv_),
		planar_top_detector(planar_top_detector_)
	{
		n_priv.param<double>("fitting_acceptance_threshold", fitting_acceptance_threshold, 0.5);

		// Initialize visualization colors
		id_colors_map.insert(std::pair<int,Color>(0,Color(0,   0  , 1, 1.0) ) );
		id_colors_map.insert(std::pair<int,Color>(1,Color(0,   1  , 0, 1.0) ) );
		id_colors_map.insert(std::pair<int,Color>(2,Color(0.4, 0.1, 0.4) ) );
		//odom_link="/odom";

		// Initialize advertisers
		shapes_pub=n.advertise<shape_detection_fitting::Shapes>( "shapes_detections", 1);
		image_pub=n.advertise<sensor_msgs::Image >("shape_detections", 1);
		vis_pub=n.advertise<visualization_msgs::MarkerArray>("shape_detections_vis", 1);

		// Initialize subscribers
		image_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > (new message_filters::Subscriber<sensor_msgs::Image>(n, "image_in", 10));
		pcl_sub=boost::shared_ptr<message_filters::Subscriber<PointCloud> > (new message_filters::Subscriber<PointCloud >(n, "pcl_in", 10));

		sync=boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > (new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub, *pcl_sub));
		sync->registerCallback(boost::bind(&ShapeFittingROS<cylinder_detector_type,sphere_detector_type, plane_detector_type>::callback, this, _1, _2));	
	}
};

#endif // SHAPEFITTINGROS_H
