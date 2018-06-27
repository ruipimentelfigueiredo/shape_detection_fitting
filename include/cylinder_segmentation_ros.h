#include "cylinder_segmentation_hough.h"
#include "cylinder_segmentation_ransac.h"
#include "visualization_msgs/MarkerArray.h"
//#include "multiple_tracker_manager.h"
#include "sensor_msgs/CameraInfo.h"
#include "shape_detection_fitting/Clusters.h"
#include "shape_detection_fitting/Cylinders.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/filters/voxel_grid.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include "cylinder_classifier.h"
#include "shape_detection_manager.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

class Color
{

	public:
	Color(double r_=0.0,double g_=0.0, double b_=0.0, double a_=1.0) : r(r_),g(g_),b(b_),a(a_)
	{}
	double r,g,b,a;
};

template <class detector_type>
class CylinderSegmentationROS {

	boost::shared_ptr<ShapeDetectionManager<detector_type> > shape_detection_manager;
	ros::Time odom_last_stamp;
	std::string odom_link;

	const std::string marker_detections_namespace_ ="detections";
	const std::string marker_trackers_namespace_ ="trackers";



	std::map<int, Color> id_colors_map;
	MultipleTrackerManager tracker_manager;
	
	
	ros::NodeHandle n;
	ros::NodeHandle n_priv;
	boost::shared_ptr<tf::TransformListener> listener;
	ros::Subscriber point_cloud_sub;

	double fitting_acceptance_threshold;
	ros::Publisher cylinders_pub;
	ros::Publisher image_pub;

	// Aux pub
	ros::Publisher cluster_pub;
	ros::Subscriber cluster_sub;
	ros::Publisher vis_pub;


	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, active_semantic_mapping::Clusters> MySyncPolicy;
	boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > image_sub;
	boost::shared_ptr<message_filters::Subscriber<active_semantic_mapping::Clusters> > clusters_sub;
    	boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> >sync;

	void callback (const sensor_msgs::Image::ConstPtr& input_image, const active_semantic_mapping::Clusters::ConstPtr & input_clusters)
	{

		static int scene_number=0;
		cv::Mat image_cv;
		image_cv =cv_bridge::toCvCopy(input_image, "bgr8")->image;

		// Get cluster pcl point clouds and opencv bounding boxes

		std::vector<int> cylinder_indices;
		cylinder_indices.reserve(input_clusters->markers.markers.size());


		const clock_t classification_begin_time = clock();
		std::vector<PointCloudT::Ptr> pcl_clusters;
		pcl_clusters.reserve(input_clusters->markers.markers.size());
		for(unsigned int i=0; i<input_clusters->markers.markers.size();++i)
		{

			PointCloudT::Ptr cloud_(new PointCloudT);
			cloud_->header.frame_id = input_clusters->header.frame_id;
			for (unsigned int p=0; p<input_clusters->markers.markers[i].points.size();++p)
			{
				double x_=input_clusters->markers.markers[i].points[p].x;
				double y_=input_clusters->markers.markers[i].points[p].y;
				double z_=input_clusters->markers.markers[i].points[p].z;
				cloud_->points.push_back (pcl::PointXYZ(x_, y_, z_));
				//pcl::PointCloudinput->markers[i].points
			}
			pcl_clusters.push_back(cloud_);
		}

		const clock_t begin_time = clock();
		std::vector<CylinderFitting> detections=shape_detection_manager->detect(image_cv, pcl_clusters);
		ROS_INFO_STREAM("Cylinders fitting time: "<<float( clock () - begin_time ) /  CLOCKS_PER_SEC<< " seconds");

		//const clock_t classification_end_time = clock();
		//scene_number++;
		//std::ofstream outputFile;
		//outputFile.open("/home/rui/classification_time.txt", fstream::out | std::ofstream::app);

		//outputFile <<  std::fixed << std::setprecision(8)<< (double)(classification_end_time-classification_begin_time ) /  CLOCKS_PER_SEC << " "<< input_clusters->markers.markers.size() << std::endl;
		//outputFile.close(); // clear flags
		//outputFile.clear(); // clear flags

		//ROS_INFO_STREAM("Cylinders classification time: "<<float( clock () - classification_begin_time ) /  CLOCKS_PER_SEC<< " seconds");

		sensor_msgs::ImagePtr image_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_cv).toImageMsg();

		image_pub.publish(image_out);

		visualization_msgs::MarkerArray markers_;

		// First delete all markers
		visualization_msgs::Marker marker_;
		marker_.action = 3;
		markers_.markers.push_back(marker_);
		


		// DETECT

		active_semantic_mapping::Cylinders cylinders_msg;
		cylinders_msg.header=input_clusters->header;

		cylinders_msg.header.frame_id=input_clusters->header.frame_id;

		cylinders_msg.cylinders.layout.dim.resize(2);
		cylinders_msg.cylinders.layout.dim[0].label  = "cylinders";
		cylinders_msg.cylinders.layout.dim[0].size   = 0;
		cylinders_msg.cylinders.layout.dim[0].stride = 0;
		cylinders_msg.cylinders.layout.dim[1].label  = "parameters";
		cylinders_msg.cylinders.layout.dim[1].size   = 8;
		cylinders_msg.cylinders.layout.dim[1].stride = 8;



		std::string detections_frame_id=input_clusters->header.frame_id;
		//outputFile.open("/home/rui/fitting_quality_cylinders.txt", fstream::out | std::ofstream::app);

		for(unsigned int ind=0; ind<detections.size();++ind)
		{
			unsigned int i=cylinder_indices[ind];
			CylinderFitting cylinder_fitting=detections[ind];
			Eigen::VectorXf model_params=cylinder_fitting.parameters;
			double confidence=cylinder_fitting.confidence;

			model_params.cast <double> ();
			Color color_;
			ROS_ERROR_STREAM("CONFIDENCE: "<< confidence);
			if(confidence<fitting_acceptance_threshold)
				color_=id_colors_map.find(0)->second;
			else
				color_=id_colors_map.find(1)->second;
			//outputFile <<  std::fixed << std::setprecision(4)<< confidence<< std::endl;
			visualization_msgs::Marker marker=createMarker(model_params,visualization_msgs::Marker::CYLINDER,detections_frame_id, color_, i, marker_detections_namespace_);
			markers_.markers.push_back(marker);

			for(unsigned int p=0; p<8;++p)
			{
				cylinders_msg.cylinders.data.push_back((float)model_params[p]);
			}

			cylinders_msg.cylinders.layout.dim[0].size+=1;
			cylinders_msg.cylinders.layout.dim[0].stride+= 8;
		}

		//outputFile.close(); // clear flags
		//outputFile.clear(); // clear flags
		const clock_t fitting_end_time = clock();

		//outputFile.open("/home/rui/fitting_time.txt", fstream::out | std::ofstream::app);

		//outputFile <<  std::fixed << std::setprecision(8)<< (double)(fitting_end_time-begin_time ) /  CLOCKS_PER_SEC << " "<< cylinder_indices.size() << std::endl;
		//outputFile.close(); // clear flags
		//outputFile.clear(); // clear flags


		vis_pub.publish( markers_ );
		cylinders_pub.publish(cylinders_msg);
	}

	void clusters_cb (const visualization_msgs::MarkerArray::ConstPtr& input_clusters)
	{	
		if(input_clusters->markers.size()==0) return;

		active_semantic_mapping::Clusters clusters;
		clusters.header=input_clusters->markers[0].header;
		clusters.markers=*input_clusters;
		cluster_pub.publish(clusters);
		return;
	}



  	void odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg)
	{
		ROS_ERROR_STREAM("ODOMETRY linear:" << odom_msg->twist.twist.linear);
		ROS_ERROR_STREAM("ODOMETRY angular:" << odom_msg->twist.twist.angular);
	}


	visualization_msgs::Marker createMarker(const Eigen::VectorXf & model_params, int model_type, const std::string & frame, Color & color_, int id, const std::string & marker_namespace_)
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

	CylinderSegmentationROS(ros::NodeHandle & n_, ros::NodeHandle & n_priv_,boost::shared_ptr<ShapeDetectionManager<detector_type> > & shape_detection_manager_) : 
		n(n_), 
		n_priv(n_priv_),
		shape_detection_manager(shape_detection_manager_),
	    	listener(new tf::TransformListener(ros::Duration(3.0)))
	{

		n_priv.param<double>("fitting_acceptance_threshold", fitting_acceptance_threshold, 0.5);

		// INITIALIZE VISUALIZATION COLORS
		id_colors_map.insert(std::pair<int,Color>(0,Color(0,   0  , 1, 1.0) ) );
		id_colors_map.insert(std::pair<int,Color>(1,Color(0,   1  , 0, 1.0) ) );
		id_colors_map.insert(std::pair<int,Color>(2,Color(0.4, 0.1, 0.4) ) );
		//odom_link="/odom";

		

		// Advertise cylinders
		cylinders_pub = n.advertise<active_semantic_mapping::Cylinders>( "cylinders_detections", 1);
		image_pub=n.advertise<sensor_msgs::Image >("cylinders_image", 1);

		vis_pub=n.advertise<visualization_msgs::MarkerArray>("cylinder_detections_vis", 1);

		// Subscribe to point cloud and planar segmentation
		image_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > (new message_filters::Subscriber<sensor_msgs::Image>(n, "image_in", 10));
		clusters_sub=boost::shared_ptr<message_filters::Subscriber<active_semantic_mapping::Clusters> > (new message_filters::Subscriber<active_semantic_mapping::Clusters>(n, "clusters_out_aux", 10));


		sync=boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > (new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub, *clusters_sub));
		sync->registerCallback(boost::bind(&CylinderSegmentationROS<detector_type>::callback, this, _1, _2));

		// Aux subscriber
		cluster_sub=n.subscribe<visualization_msgs::MarkerArray> ("clusters_in", 1, &CylinderSegmentationROS::clusters_cb, this);
		cluster_pub=n.advertise<active_semantic_mapping::Clusters>( "clusters_out_aux", 2);


		// Odom subscriber
		//odom_sub=n.subscribe<nav_msgs::Odometry> ("odom", 1, &CylinderSegmentationROS::odomCallback, this);	
	}

};

