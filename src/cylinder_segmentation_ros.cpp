#include "cylinder_segmentation_ros.h"
#include <pcl/filters/voxel_grid.h>

template <class detector_type>
CylinderSegmentationROS<detector_type>::CylinderSegmentationROS(ros::NodeHandle & n_, boost::shared_ptr<detector_type> & cylinder_segmentation_) : 
	n(n_), 
	n_priv("~"),
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
	odom_link="/odom";

        ROS_INFO("Getting cameras' parameters");
        std::string camera_info_topic;
        n_priv.param<std::string>("camera_info_topic", camera_info_topic, "camera_info_topic");
        sensor_msgs::CameraInfoConstPtr camera_info=ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic, ros::Duration(10.0));

        //set the cameras intrinsic parameters
        cam_intrinsic = Eigen::Matrix4f::Identity();
        cam_intrinsic(0,0) = (float)camera_info->K.at(0);
        cam_intrinsic(0,2) = (float)camera_info->K.at(2);
        cam_intrinsic(1,1) = (float)camera_info->K.at(4);
        cam_intrinsic(1,2) = (float)camera_info->K.at(5);
        cam_intrinsic(3,3) = 0.0;


	// Advertise cylinders
	vis_pub = n.advertise<visualization_msgs::MarkerArray>( "cylinders_markers", 1);
	image_pub=n.advertise<sensor_msgs::Image >("cylinders_image", 1);

	// Subscribe to point cloud and planar segmentation
        image_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > (new message_filters::Subscriber<sensor_msgs::Image>(n, "image_in", 10));
        clusters_sub=boost::shared_ptr<message_filters::Subscriber<active_semantic_mapping::Clusters> > (new message_filters::Subscriber<active_semantic_mapping::Clusters>(n, "clusters_out_aux", 10));

	sync=boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > (new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub, *clusters_sub));
        sync->registerCallback(boost::bind(&CylinderSegmentationROS<detector_type>::callback, this, _1, _2));

	// Aux subscriber
	cluster_sub=n.subscribe<visualization_msgs::MarkerArray> ("clusters_in", 1, &CylinderSegmentationROS::clusters_cb, this);
	cluster_pub=n.advertise<active_semantic_mapping::Clusters>( "clusters_out_aux", 2);


	// Odom subscriber
	odom_sub=n.subscribe<nav_msgs::Odometry> ("odom", 1, &CylinderSegmentationROS::odomCallback, this);	
}


template <class detector_type>
void CylinderSegmentationROS<detector_type>::callback (const sensor_msgs::Image::ConstPtr& input_image, const active_semantic_mapping::Clusters::ConstPtr & input_clusters)
{
<<<<<<< HEAD
	static int image_number=0;
=======
	ros::Time odom_time=ros::Time::now();

	tf::StampedTransform deltaTf;


	// Get odom delta motion in cartesian coordinates with TF
	try
	{
		listener->waitForTransform(input_clusters->header.frame_id, odom_last_stamp, input_clusters->header.frame_id, odom_time , odom_link, ros::Duration(0.5) );
		listener->lookupTransform(input_clusters->header.frame_id, odom_last_stamp, input_clusters->header.frame_id, odom_time, odom_link, deltaTf); // delta position
	}
	catch (tf::TransformException &ex)
	{
		//odom_initialized_=false;
		ROS_WARN("%s",ex.what());
		return;
	}


	// Get relative transformation
	Eigen::Matrix3f rotation;
	rotation=Eigen::AngleAxisf(deltaTf.getRotation().getAngle(),
				   Eigen::Vector3f(deltaTf.getRotation().getAxis()[0],
				    		   deltaTf.getRotation().getAxis()[1],
				   		   deltaTf.getRotation().getAxis()[2])); 

	Eigen::Vector4f translation(deltaTf.getOrigin().getX(),deltaTf.getOrigin().getY(), deltaTf.getOrigin().getZ(),1.0);
    	double dx=deltaTf.getOrigin().getX();
    	double dy=deltaTf.getOrigin().getY();
    	double dz=deltaTf.getOrigin().getZ();
    	double d_theta=deltaTf.getRotation().getAxis()[2]*deltaTf.getRotation().getAngle();

	Eigen::Matrix4f transform;
	transform.block(0,0,3,3)=rotation;
	transform.block(0,3,4,1)=translation;

	ROS_ERROR_STREAM("dx:"<<dx<<" dy:"<<dy<<" dz:"<<dz);
	ROS_ERROR_STREAM("transform:"<<transform);
    	// See if we should update the filter
    	/*if(!(fabs(dx) > d_thresh_ || fabs(dy) > d_thresh_ || fabs(d_theta) > a_thresh_))
    	{
        	return;
    	}*/


	odom_last_stamp=odom_time;

>>>>>>> c79be99eb768273d386fdd5f57685ed7f6c4f4f0
	cv::Mat image_cv;
	image_cv =cv_bridge::toCvCopy(input_image, "bgr8")->image;

	// Get cluster pcl point clouds and opencv bounding boxes
	std::vector<PointCloudT::Ptr> clusters_point_clouds;
	clusters_point_clouds.reserve(input_clusters->markers.markers.size());
	std::vector<cv::Rect> clusters_bboxes;
	clusters_bboxes.reserve(input_clusters->markers.markers.size());
	for(unsigned int i=0; i<input_clusters->markers.markers.size();++i)
	{
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

		PointCloudT::Ptr cloud_filtered (new PointCloudT);
		// Create the filtering object
		pcl::VoxelGrid<PointT> sor;
		sor.setInputCloud(cloud_);
		sor.setLeafSize(0.005f, 0.005f, 0.005f);
		sor.filter(*cloud_filtered);
		clusters_point_clouds.push_back(cloud_filtered);
		
		///////////////////////////
		// Get 2d bounding boxes //
		///////////////////////////

		// Get XY max and min and construct image
		for (unsigned int p=0; p<cloud_filtered->points.size();++p)
		{	
			cloud_filtered->points[p].x/=cloud_filtered->points[p].z;
			cloud_filtered->points[p].y/=cloud_filtered->points[p].z;
			cloud_filtered->points[p].z=1.0;
		}

		PointCloudT::Ptr cloud_projected(new PointCloudT);
  		pcl::transformPointCloud (*cloud_filtered, *cloud_projected, cam_intrinsic);



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

		// Visualize
		cv::rectangle(image_cv, cv::Point(min_pt[0],min_pt[1]), cv::Point(max_pt[0],max_pt[1]), cv::Scalar(0), 4);

		//cv::Mat roi = image_cv(rect);

		/*cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
		cv::imshow( "Display window", roi );                   // Show our image inside it.
		cv::waitKey(0);*/ 

 		imwrite("/home/rui/cylinders/image_"+std::to_string(image_number++)+".jpg", image_cv(rect) );


	}


	sensor_msgs::ImagePtr image_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_cv).toImageMsg();
	image_pub.publish(image_out);

	visualization_msgs::MarkerArray markers_;

	// First delete all markers
	visualization_msgs::Marker marker_;
	marker_.action = 3;
	markers_.markers.push_back(marker_);

	// DETECT
	const clock_t begin_time = clock();
	std::vector<Eigen::VectorXd> detections_;
	std::string detections_frame_id=input_clusters->markers.markers[0].header.frame_id;
	for(unsigned int i=0; i<clusters_point_clouds.size();++i)
	{
		Eigen::VectorXf model_params=cylinder_segmentation->segment(clusters_point_clouds[i]);
		detections_.push_back(model_params.cast <double> ());
		Color color_=id_colors_map.find(0)->second;
		visualization_msgs::Marker marker=createMarker(model_params,visualization_msgs::Marker::CYLINDER,detections_frame_id, color_, i, marker_detections_namespace_);
		markers_.markers.push_back(marker);
	}

	ROS_INFO_STREAM("Cylinders detection time: "<<float( clock () - begin_time ) /  CLOCKS_PER_SEC<< " seconds");


	// TRACK
	const clock_t begin_time_ = clock();
	const std::vector<std::shared_ptr<Tracker<Cylinder, KalmanFilter> > > trackers=tracker_manager.process(detections_);

	for(unsigned int i=0; i<trackers.size();++i)
	{
		Eigen::VectorXf tracker_=trackers[i]->getObjPTR()->getObservableStates().cast<float>();
		Color color_=id_colors_map.find(trackers[i]->getTrackerId() + 1)->second;
		visualization_msgs::Marker marker=createMarker(tracker_,visualization_msgs::Marker::CYLINDER,detections_frame_id, color_, i, marker_trackers_namespace_);
		markers_.markers.push_back(marker);
	}

	ROS_INFO_STREAM("Cylinders tracking time: "<<float( clock () - begin_time_ ) /  CLOCKS_PER_SEC<< " seconds");

	// Publish the data.
	vis_pub.publish( markers_ );
}

template <class detector_type>
void CylinderSegmentationROS<detector_type>::cloud_cb (const PointCloudT::ConstPtr& input_cloud)
{
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);

	// Build a passthrough filter to remove spurious NaNs
	/*pcl::PassThrough<PointT> pass;
	pass.setInputCloud (input_cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-1.5, 1.5);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-1.5, 1.5);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (-1.5, 1.5);
	pass.filter (*cloud_filtered);
	std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

	// Estimate point normals
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud_filtered);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 

	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight (0.1);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.03);
	seg.setInputCloud (cloud_filtered);
	seg.setInputNormals (cloud_normals);
	// Obtain the plane inliers and coefficients
	seg.segment (*inliers_plane, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud_filtered);
	extract.setIndices (inliers_plane);
	extract.setNegative (true);
	extract.filter (*cloud_filtered);*/

	visualization_msgs::MarkerArray markers_;
	// Create a container for the data.	
	// Do data processing here...
	const clock_t begin_time = clock();
	Eigen::VectorXf model_params=cylinder_segmentation->segment(input_cloud);
	ROS_INFO_STREAM("Cylinders detection time: "<<float( clock () - begin_time ) /  CLOCKS_PER_SEC<< " seconds");

	std::string marker_namespace_= "detections";
	// Publish the data.

	Color color_=id_colors_map.find(0)->second;
	visualization_msgs::Marker marker=createMarker(model_params,visualization_msgs::Marker::CYLINDER,input_cloud->header.frame_id, color_,0, marker_namespace_);
	markers_.markers.push_back(marker);
	vis_pub.publish( markers_ );

}


// AUX METHOD
template <class detector_type>
void CylinderSegmentationROS<detector_type>::clusters_cb (const visualization_msgs::MarkerArray::ConstPtr& input_clusters)
{	
	if(input_clusters->markers.size()==0) return;

	active_semantic_mapping::Clusters clusters;
	clusters.header=input_clusters->markers[0].header;
	clusters.markers=*input_clusters;
	cluster_pub.publish(clusters);
	return;
}

template <class detector_type>
visualization_msgs::Marker CylinderSegmentationROS<detector_type>::createMarker(const Eigen::VectorXf & model_params, int model_type, const std::string & frame, Color & color_, int id, const std::string & marker_namespace_)
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
	marker.pose.position.x = model_params[0]+0.5*height*axis_vector[0];
	marker.pose.position.y = model_params[1]+0.5*height*axis_vector[1];
	marker.pose.position.z = model_params[2]+0.5*height*axis_vector[2];
	marker.pose.orientation = cylinder_orientation;
/*		marker.pose.orientation.x = Q.x();
	marker.pose.orientation.y = Q.y();
	marker.pose.orientation.z = Q.z();
	marker.pose.orientation.w = Q.w();*/
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

int main (int argc, char** argv)
{
	ros::init(argc, argv, "talker");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/

	ros::NodeHandle n;
	ros::NodeHandle n_priv("~");
	ros::Rate loop_rate(30);

	bool use_ransac;

	// Common params
 	double min_radius;
 	double max_radius;

    	n_priv.param("use_ransac",use_ransac,true);
	ROS_INFO_STREAM("use_ransac: "<< use_ransac);
    	
	n_priv.param("min_radius", min_radius, 0.1);
    	n_priv.param("max_radius", max_radius, 0.1);

	ROS_INFO_STREAM("min_radius: "<< min_radius);
	ROS_INFO_STREAM("max_radius: "<< max_radius);

	if (use_ransac)
	{
		// Ransac params
		double normal_distance_weight;
		int max_iterations;
		double distance_threshold;

	    	n_priv.param("normal_distance_weight",normal_distance_weight,0.1);
	    	n_priv.param("max_iterations",max_iterations,100);
	    	n_priv.param("distance_threshold",distance_threshold,0.1);

		ROS_INFO_STREAM("normal_distance_weight: "<< normal_distance_weight);
		ROS_INFO_STREAM("max_iterations: "<< max_iterations);
		ROS_INFO_STREAM("distance_threshold: "<< distance_threshold);

		boost::shared_ptr<CylinderSegmentationRansac> cylinder_segmentation(new CylinderSegmentationRansac((float)normal_distance_weight,(unsigned int)max_iterations,(unsigned int)distance_threshold,(float)min_radius, (float)max_radius));
		CylinderSegmentationROS<CylinderSegmentationRansac> cylinder_segmentation_ros(n, cylinder_segmentation);

		while (ros::ok())
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	else
	{
		// Hough params
		int angle_bins;
		int radius_bins;
	 	int position_bins;
		int gaussian_sphere_points_num;

	    	n_priv.param("angle_bins",angle_bins,50);
	    	n_priv.param("radius_bins",radius_bins,50);
	    	n_priv.param("position_bins",position_bins,50);
	    	n_priv.param("gaussian_sphere_points_num", gaussian_sphere_points_num, 1000);

		ROS_INFO_STREAM("angle_bins: "<< angle_bins);
		ROS_INFO_STREAM("radius_bins: "<< radius_bins);
		ROS_INFO_STREAM("position_bins: "<< position_bins);
		ROS_INFO_STREAM("gaussian_sphere_points_num: "<< gaussian_sphere_points_num);

		boost::shared_ptr<CylinderSegmentationHough> cylinder_segmentation(new CylinderSegmentationHough((unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(unsigned int)gaussian_sphere_points_num));
		CylinderSegmentationROS<CylinderSegmentationHough> cylinder_segmentation_ros(n, cylinder_segmentation);


		while (ros::ok())
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
	}



  
	return (0);
}

