#include "cylinder_segmentation_hough.h"
#include "cylinder_segmentation_ransac.h"
#include "visualization_msgs/MarkerArray.h"
#include "multiple_tracker_manager.h"
#include "sensor_msgs/CameraInfo.h"
#include "active_semantic_mapping/Clusters.h"
#include "active_semantic_mapping/Cylinders.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/filters/voxel_grid.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include "cylinder_classifier.h"

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


	ros::Time odom_last_stamp;
	std::string odom_link;

	const std::string marker_detections_namespace_ ="detections";
	const std::string marker_trackers_namespace_ ="trackers";

	 Eigen::Matrix4f cam_intrinsic;

	std::map<int, Color> id_colors_map;
	MultipleTrackerManager tracker_manager;
	
	
	ros::NodeHandle n;
	ros::NodeHandle n_priv;
	boost::shared_ptr<tf::TransformListener> listener;
	ros::Subscriber point_cloud_sub;

	float classification_threshold;

	ros::Publisher cylinders_pub;
	ros::Publisher image_pub;

	// Aux pub
	ros::Publisher cluster_pub;
	ros::Subscriber cluster_sub;
	ros::Publisher vis_pub;

	boost::shared_ptr<detector_type> cylinder_segmentation;
	boost::shared_ptr<CylinderClassifier> cylinder_classifier;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, active_semantic_mapping::Clusters> MySyncPolicy;
	boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > image_sub;
	boost::shared_ptr<message_filters::Subscriber<active_semantic_mapping::Clusters> > clusters_sub;
    	boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> >sync;



	void callback (const sensor_msgs::Image::ConstPtr& input_image, const active_semantic_mapping::Clusters::ConstPtr & input_clusters)
	{
		static int image_number=0;
		cv::Mat image_cv;
		image_cv =cv_bridge::toCvCopy(input_image, "bgr8")->image;

		// Get cluster pcl point clouds and opencv bounding boxes
		std::vector<PointCloudT::Ptr> clusters_point_clouds;
		clusters_point_clouds.reserve(input_clusters->markers.markers.size());
		std::vector<cv::Rect> clusters_bboxes;
		clusters_bboxes.reserve(input_clusters->markers.markers.size());
		std::vector<int> cylinder_indices;
		cylinder_indices.reserve(input_clusters->markers.markers.size());


		const clock_t classification_begin_time = clock();

		for(unsigned int i=0; i<input_clusters->markers.markers.size();++i)
		{
			//if(i>0) break;
			//////////////////////
			// Get pcl clusters //
			//////////////////////

			PointCloudT::Ptr cloud_(new PointCloudT);
			cloud_->header.frame_id = input_clusters->header.frame_id;
			//cloud_->height = input_clusters->markers[i].height;

			for (unsigned int p=0; p<input_clusters->markers.markers[i].points.size();++p)
			{

				double x_=input_clusters->markers.markers[i].points[p].x;
				double y_=input_clusters->markers.markers[i].points[p].y;
				double z_=input_clusters->markers.markers[i].points[p].z;
				cloud_->points.push_back (pcl::PointXYZ(x_, y_, z_));
				//pcl::PointCloudinput->markers[i].points
			}

			/*tf::TransformListener tf_listener;
			tf_listener.waitForTransform( "/table",cloud_->header.frame_id,ros::Time(0), ros::Duration(5.0));
			pcl_ros::transformPointCloud("/table", *cloud_, *cloud_, tf_listener);*/

			PointCloudT::Ptr cloud_filtered (new PointCloudT);

			// Create the filtering object
			pcl::VoxelGrid<PointT> sor;
			sor.setInputCloud(cloud_);
			sor.setLeafSize(0.005f, 0.005f, 0.005f);
			sor.filter(*cloud_filtered);
			//cloud_filtered->header.frame_id="table";
			clusters_point_clouds.push_back(cloud_filtered);

			///////////////////////////
			// Get 2d bounding boxes //
			///////////////////////////

			// Get XY max and min and construct image
			PointCloudT::Ptr cloud_projected(new PointCloudT);
	  		pcl::transformPointCloud (*cloud_filtered, *cloud_projected, cam_intrinsic);

			for (unsigned int p=0; p<cloud_projected->points.size();++p)
			{	
				cloud_projected->points[p].x/=cloud_projected->points[p].z;
				cloud_projected->points[p].y/=cloud_projected->points[p].z;
				cloud_projected->points[p].z=1.0;
			}

			// Get minmax
			Eigen::Vector4f min_pt,max_pt;
			pcl::getMinMax3D(*cloud_projected,min_pt,max_pt);

			float padding =0.5;
			float width=max_pt[0]-min_pt[0];
			float height=max_pt[1]-min_pt[1];
	
			// PAD
			float width_padding=0.5*padding*width;
			float height_padding=0.5*padding*height;

			(min_pt[0]-width_padding)  <0 ? min_pt[0]=0 : min_pt[0]-=width_padding;
			(min_pt[1]-height_padding) <0 ? min_pt[1]=0 : min_pt[1]-=height_padding;

			(max_pt[0]+width_padding)  >(image_cv.cols-1) ? max_pt[0]=(image_cv.cols-1)  : max_pt[0]+=width_padding;
			(max_pt[1]+height_padding) >(image_cv.rows-1) ? max_pt[1]=(image_cv.rows-1) : max_pt[1]+=height_padding;
		
			width=max_pt[0]-min_pt[0];
			height=max_pt[1]-min_pt[1];
			// end pad

			cv::Rect rect(min_pt[0], min_pt[1], width, height);
			clusters_bboxes.push_back(rect);

			// Classify
			cv::Mat roi = image_cv(rect);
			float confidence=cylinder_classifier->classify(roi);
			ROS_ERROR_STREAM("CONFIDENCE:"<<confidence);
			if(confidence>classification_threshold)
			{
				cylinder_indices.push_back(i);

				// Visualization
				cv::rectangle(image_cv, cv::Point(min_pt[0],min_pt[1]), cv::Point(max_pt[0],max_pt[1]), cv::Scalar(0,255,0), 4);
			}
			else
			{
				// Visualization
				cv::rectangle(image_cv, cv::Point(min_pt[0],min_pt[1]), cv::Point(max_pt[0],max_pt[1]), cv::Scalar(255,0,0), 4);
			}
				

			// dataset creation
			/*cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
			cv::imshow( "Display window", roi );                   // Show our image inside it.
			cv::waitKey(0);*/ 

			//if((image_number%5)==0)
			//imwrite("/home/rui/other/other."+std::to_string(image_number)+".jpg", image_cv(rect) );

			//image_number++;
		}

		ROS_INFO_STREAM("Cylinders classification time: "<<float( clock () - classification_begin_time ) /  CLOCKS_PER_SEC<< " seconds");

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
		cylinders_msg.header.frame_id=clusters_point_clouds[0]->header.frame_id;
		cylinders_msg.cylinders.layout.dim.resize(2);
		cylinders_msg.cylinders.layout.dim[0].label  = "cylinders";
		cylinders_msg.cylinders.layout.dim[0].size   = 0;
		cylinders_msg.cylinders.layout.dim[0].stride = 0;
		cylinders_msg.cylinders.layout.dim[1].label  = "parameters";
		cylinders_msg.cylinders.layout.dim[1].size   = 8;
		cylinders_msg.cylinders.layout.dim[1].stride = 8;

		const clock_t begin_time = clock();

		std::string detections_frame_id=clusters_point_clouds[0]->header.frame_id;

		for(unsigned int ind=0; ind<cylinder_indices.size();++ind)
		{
			unsigned int i=cylinder_indices[ind];
			Eigen::VectorXf model_params=cylinder_segmentation->segment(clusters_point_clouds[i]);
			model_params.cast <double> ();

			Color color_=id_colors_map.find(0)->second;
			visualization_msgs::Marker marker=createMarker(model_params,visualization_msgs::Marker::CYLINDER,detections_frame_id, color_, i, marker_detections_namespace_);
			markers_.markers.push_back(marker);

			for(unsigned int p=0; p<8;++p)
			{
				cylinders_msg.cylinders.data.push_back((float)model_params[p]);
			}

			cylinders_msg.cylinders.layout.dim[0].size+=1;
			cylinders_msg.cylinders.layout.dim[0].stride+= 8;
		}

		ROS_INFO_STREAM("Cylinders fitting time: "<<float( clock () - begin_time ) /  CLOCKS_PER_SEC<< " seconds");

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
	/*		marker.pose.orientation.x = Q.x();
		marker.pose.orientation.y = Q.y();
		marker.pose.orientation.z = Q.z();
		marker.pose.orientation.w = Q.w();*/
		marker.scale.x = 2*model_params[6];
		marker.scale.y = 2*model_params[6];
		marker.scale.z = height;
			marker.color.a = 1.0;
			marker.color.r = 0;
			marker.color.g = 1;
			marker.color.b = 0;
		//marker.lifetime = ros::Duration(0.05);
		return marker;
	}


public:

	CylinderSegmentationROS(ros::NodeHandle & n_, ros::NodeHandle & n_priv_, boost::shared_ptr<detector_type> & cylinder_segmentation_) : 
		n(n_), 
		n_priv(n_priv_),
	    	listener(new tf::TransformListener(ros::Duration(3.0))),
		cylinder_segmentation(cylinder_segmentation_)
	{

		// INITIALIZE VISUALIZATION COLORS
		id_colors_map.insert(std::pair<int,Color>(0,Color(0,   0.9  , 0.9, 0.2) ) );
		id_colors_map.insert(std::pair<int,Color>(1,Color(0,   0.5, 0.7) ) );
		id_colors_map.insert(std::pair<int,Color>(2,Color(0.4, 0.1, 0.4) ) );
		id_colors_map.insert(std::pair<int,Color>(3,Color(0.3, 0.1, 0.9) ) );
		id_colors_map.insert(std::pair<int,Color>(4,Color(0.1, 0.4, 0.8) ) );
		id_colors_map.insert(std::pair<int,Color>(5,Color(0.5, 0.5, 0.9) ) );
		id_colors_map.insert(std::pair<int,Color>(6,Color(0.3, 0.7, 0.3) ) );
		id_colors_map.insert(std::pair<int,Color>(7,Color(0.9, 0.4, 0.5) ) );
		id_colors_map.insert(std::pair<int,Color>(8,Color(0.9, 0.6, 0.3) ) );
		id_colors_map.insert(std::pair<int,Color>(9,Color(0.8, 0.0, 0.9) ) );
		//odom_link="/odom";

		ROS_INFO("Getting cameras' parameterssss");
		std::string camera_info_topic;
		n_priv.param<std::string>("camera_info_topic", camera_info_topic, "camera_info_topic");
		ROS_INFO_STREAM("camera_info_topic:"<< camera_info_topic);
		sensor_msgs::CameraInfoConstPtr camera_info=ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic, ros::Duration(3.0));

		//set the cameras intrinsic parameters
		cam_intrinsic = Eigen::Matrix4f::Identity();
		cam_intrinsic(0,0) = (float)camera_info->K.at(0);
		cam_intrinsic(0,2) = (float)camera_info->K.at(2);
		cam_intrinsic(1,1) = (float)camera_info->K.at(4);
		cam_intrinsic(1,2) = (float)camera_info->K.at(5);
		cam_intrinsic(3,3) = 0.0;

		std::string absolute_path_folder;
		std::string model_file;
		std::string weight_file;
		std::string mean_file;
		std::string device;
		int device_id;

		ROS_INFO("Getting classifier parameters");
		n_priv.param<std::string>("absolute_path_folder", absolute_path_folder, "absolute_path_folder");
		n_priv.param<std::string>("model_file", model_file, "model_file");
		n_priv.param<std::string>("weight_file", weight_file, "weight_file");
		n_priv.param<std::string>("mean_file", mean_file, "mean_file");
		n_priv.param<std::string>("device", device, "device");
		n_priv.param<int>("device_id", device_id, 0);

		n_priv.param<float>("classification_threshold", classification_threshold, 0.9);

		ROS_INFO_STREAM("absolute_path_folder:"<< absolute_path_folder);
		ROS_INFO_STREAM("model_file:"<< model_file);
		ROS_INFO_STREAM("weight_file:"<< weight_file);
		ROS_INFO_STREAM("mean_file:"<< mean_file);
		ROS_INFO_STREAM("device:"<< device);
		ROS_INFO_STREAM("device_id:"<< device_id);
		ROS_INFO_STREAM("classification_threshold:"<< classification_threshold);
		cylinder_classifier=boost::shared_ptr<CylinderClassifier>(new CylinderClassifier(absolute_path_folder,model_file,weight_file,mean_file,device,(unsigned int)device_id));
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

