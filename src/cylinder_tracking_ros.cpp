#include "cylinder_tracking_ros.h"

CylinderTrackingROS::CylinderTrackingROS(ros::NodeHandle & n_) : 
	n(n_), 
	n_priv("~"),
    	listener(new tf::TransformListener(ros::Duration(3.0)))
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
        /*cam_intrinsic = Eigen::Matrix4f::Identity();
        cam_intrinsic(0,0) = (float)camera_info->K.at(0);
        cam_intrinsic(0,2) = (float)camera_info->K.at(2);
        cam_intrinsic(1,1) = (float)camera_info->K.at(4);
        cam_intrinsic(1,2) = (float)camera_info->K.at(5);
        cam_intrinsic(3,3) = 0.0;*/


	// Advertise cylinders
	vis_pub = n.advertise<visualization_msgs::MarkerArray>( "cylinders_tracking_vis", 10);


	cylinders_sub=n.subscribe<active_semantic_mapping::Cylinders> ("cylinders_detections", 10, &CylinderTrackingROS::callback, this);
	// Subscribe to point cloud and planar segmentation
        /*cylinders_sub=boost::shared_ptr<message_filters::Subscriber<std_msgs::Int32MultiArray> > (new message_filters::Subscriber<std_msgs::Int32MultiArray>(n, "clusters_out_aux", 10));

	sync=boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > (new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),*cylinders_sub));
        sync->registerCallback(boost::bind(&CylinderTrackingROS::callback, this, _1));*/



	// Odom subscriber
	//odom_sub=n.subscribe<nav_msgs::Odometry> ("odom", 1, &CylinderTrackingROS::odomCallback, this);	
}



void CylinderTrackingROS::callback (const active_semantic_mapping::Cylinders::ConstPtr & input_clusters)
{

	/*static int image_number=0;

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
	ROS_ERROR_STREAM("transform:"<<transform);*/
    	// See if we should update the filter
    	/*if(!(fabs(dx) > d_thresh_ || fabs(dy) > d_thresh_ || fabs(d_theta) > a_thresh_))
    	{
        	return;
    	}*/


	//odom_last_stamp=odom_time;




	unsigned int detections_num=input_clusters->cylinders.layout.dim[0].size;
	std::vector<Eigen::VectorXd> detections_;
	for(unsigned int i=0; i<detections_num;++i)
	{
		Eigen::VectorXd detection(8,1);
		for(unsigned int p=0; p<8 ; ++p)
		{
			unsigned int index=p+i*8;

			detection[p]=(double)input_clusters->cylinders.data[index];
		}

		detections_.push_back(detection);
	}

	visualization_msgs::MarkerArray markers_;
	//markers_.header=input_clusters->header;

	// First delete all markers
	visualization_msgs::Marker marker_;
	marker_.action = 3;
	markers_.markers.push_back(marker_);

	// Compute object level odometry
	clock_t begin_time_ = clock();
	odom(detections_);
	ROS_INFO_STREAM("Cylinders odom time: "<<float( clock () - begin_time_ ) /  CLOCKS_PER_SEC<< " seconds");

	// TRACK
	begin_time_ = clock();
	const std::vector<std::shared_ptr<Tracker<Cylinder, KalmanFilter> > > trackers=tracker_manager.process(detections_);
	std::string trackers_frame_id=input_clusters->header.frame_id;
	for(unsigned int i=0; i<trackers.size();++i)
	{
		Eigen::VectorXf tracker_=trackers[i]->getObjPTR()->getObservableStates().cast<float>();
		Color color_=id_colors_map.find(trackers[i]->getTrackerId() + 1)->second;
		visualization_msgs::Marker marker=createMarker(tracker_,visualization_msgs::Marker::CYLINDER,trackers_frame_id, color_, i, marker_trackers_namespace_);
		markers_.markers.push_back(marker);
	}

	ROS_INFO_STREAM("Cylinders tracking time: "<<float( clock () - begin_time_ ) /  CLOCKS_PER_SEC<< " seconds");

	// Publish the data.
	vis_pub.publish( markers_ );
}


Eigen::Matrix4f CylinderTrackingROS::odom(const std::vector<Eigen::VectorXd> & detections_)
{
	if(tracker_manager.trackers.size()<1) return Eigen::Matrix4f();

	// Generate cylinders detections point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr detections_cloud (new pcl::PointCloud<pcl::PointXYZ>());
	for(unsigned int d=0; d< detections_.size();++d)
	{

		int height_samples=3;
		int angle_samples=5;

		// Generate random radius


		// Generate random height


		// Generate cylinder according to parameters
		float angle_step=2.0*M_PI/angle_samples;
		float height_step=fabs(detections_[d][7])/height_samples;

		for(int a=0; a < angle_samples; ++a)
		{
			float x,y,z;
			x=cos(angle_step*a)*fabs(detections_[d][6]);
			y=sin(angle_step*a)*fabs(detections_[d][6]);
			for(int h=0; h < height_samples; ++h)
			{
				z=(float)height_step*h;
				pcl::PointXYZ point(x,y,z);
				detections_cloud->push_back(point);
			}
		}
	}

	// Generate cylinders trackers point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr trackers_cloud (new pcl::PointCloud<pcl::PointXYZ>());
	for(unsigned int d=0; d< tracker_manager.trackers.size();++d)
	{

		int height_samples=3;
		int angle_samples=5;

		// Generate random radius


		// Generate random height


		// Generate cylinder according to parameters
		float angle_step=2.0*M_PI/angle_samples;
		float height_step=fabs(tracker_manager.trackers[d]->getState()[7])/height_samples;

		for(int a=0; a < angle_samples; ++a)
		{
			float x,y,z;
			x=cos(angle_step*a)*fabs(tracker_manager.trackers[d]->getState()[6]);
			y=sin(angle_step*a)*fabs(tracker_manager.trackers[d]->getState()[6]);
			for(int h=0; h < height_samples; ++h)
			{
				z=(float)height_step*h;
				pcl::PointXYZ point(x,y,z);
				trackers_cloud->push_back(point);
			}
		}
	}

	// COMPUTE ICP
 	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(detections_cloud);
	icp.setInputTarget(trackers_cloud);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	//std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  	//std::cout << icp.getFinalTransformation() << std::endl;
	// Return ICP transform as relative odometry
	return icp.getFinalTransformation();
}





visualization_msgs::Marker CylinderTrackingROS::createMarker(const Eigen::VectorXf & model_params, int model_type, const std::string & frame, Color & color_, int id, const std::string & marker_namespace_)
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


