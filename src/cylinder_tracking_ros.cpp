#include "cylinder_tracking_ros.h"
#include <pcl/visualization/cloud_viewer.h>
#include "tf_conversions/tf_eigen.h"
#include "helpers.h"
CylinderTrackingROS::CylinderTrackingROS(ros::NodeHandle & n_) : 
	n(n_), 
	n_priv("~"),
	vo_initialized_(false),
    	listener(new tf::TransformListener(ros::Duration(3.0)))
{
	odom_last_stamp=ros::Time::now();
	// INITIALIZE VISUALIZATION COLORS
	id_colors_map.insert(std::pair<int,Color>(0,Color(0,   0.9  , 0.9, 1.0) ) );
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
	tracking_frame="odom_filtered";


	// Advertise cylinders
	vis_pub = n.advertise<visualization_msgs::MarkerArray>( "cylinders_tracking_vis", 10);


	// Detector subscriber
	cylinders_sub=n.subscribe<active_semantic_mapping::Cylinders> ("cylinders_detections", 10, &CylinderTrackingROS::detectionsCallback, this);

	// Odom subscriber
	odom_sub=n.subscribe<nav_msgs::Odometry> ("odom", 1, &CylinderTrackingROS::odomCallback, this);	
}

  // initialize prior density of filter 
/*void CylinderTrackingROS::initialize(const tf::Transform& prior, const ros::Time& time)
{
	// set prior of filter
	Eigen::Matrix<double,6,1> prior_Mu; 
	decomposeTransform(prior, prior_Mu(1), prior_Mu(2), prior_Mu(3), prior_Mu(4), prior_Mu(5), prior_Mu(6));
	Eigen::Matrix<double,6,6> prior_Cov; 
	for (unsigned int i=0; i<6; i++) {
		for (unsigned int j=0; j<6; j++){
			if (i==j)  
				prior_Cov(i,j) = pow(0.001,2);
			else 
				prior_Cov(i,j) = 0;
		}
	}
	/*prior_  = new Gaussian(prior_Mu,prior_Cov);
	filter_ = new ExtendedKalmanFilter(prior_);

	// remember prior
	addMeasurement(StampedTransform(prior, time, output_frame_, base_footprint_frame_));
	filter_estimate_old_vec_ = prior_Mu;
	filter_estimate_old_ = prior;
	filter_time_old_     = time;

	// filter initialized
	filter_initialized_ = true;
	filter_time=time;
}*/

void CylinderTrackingROS::detectionsCallback(const active_semantic_mapping::Cylinders::ConstPtr & input_clusters)
{
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

	// TRACK
	clock_t begin_time_ = clock();
	Eigen::Matrix4d relative_motion;
	const std::vector<std::shared_ptr<Tracker<Cylinder, KalmanFilter> > > trackers=tracker_manager.update(detections_,relative_motion);
	std::string trackers_frame_id=input_clusters->header.frame_id;
	for(unsigned int i=0; i<trackers.size();++i)
	{
		Eigen::VectorXd tracker_=trackers[i]->getObjPTR()->getObservableStates().cast<double>();
		Color color_=id_colors_map.find(trackers[i]->getTrackerId() + 1)->second;
		visualization_msgs::Marker marker=createMarker(tracker_,visualization_msgs::Marker::CYLINDER,trackers_frame_id, color_, i, marker_trackers_namespace_);
		markers_.markers.push_back(marker);
	}

	ROS_INFO_STREAM("Cylinders tracking time: "<<( clock () - begin_time_ ) /  CLOCKS_PER_SEC<< " seconds");

	// Publish the data.
	vis_pub.publish( markers_ );
}

// decompose Transform into x,y,z,dx,dy,dz
void CylinderTrackingROS::decomposeTransform(const tf::Transform & trans_tf, Eigen::Matrix<double,6,1> & state_){

	Eigen::Affine3d trans_eigen;
	tf::transformTFToEigen(trans_tf, trans_eigen);
	Eigen::Vector3d init(1,1,1);
	//trans_eigen.matrix().block(0,3,3,1)=init;

	// Position part
	state_.head(3)=trans_eigen.matrix().block(0,3,3,1);

	// Orientation part
        state_.segment(3,3)=rotationToDirection(trans_eigen.matrix().block(0,0,3,3));
	std::cout << state_ << std::endl;
	//trans.getBasis().getEulerYPR(Rz,Ry,Rz);
};

// correct for angle overflow
void CylinderTrackingROS::angleOverflowCorrect(double& a, double ref)
{
	while ((a-ref) >  M_PI) a -= 2*M_PI;
	while ((a-ref) < -M_PI) a += 2*M_PI;
};


// Convert direction vector to transformation matrix
Eigen::Affine3d CylinderTrackingROS::stateToMatrix(const Eigen::Matrix<double,6,1> & state)
{
	Eigen::Vector3d direction_vector=state.segment(3,3);
	Eigen::Matrix3d rotationMatrix=directionToRotation(direction_vector);
	Eigen::Translation<double, 3> translation(state(0),state(1),state(2));
	return Eigen::Affine3d(rotationMatrix*translation);
}

bool CylinderTrackingROS::addMeasurement(const tf::StampedTransform& meas, const Eigen::Matrix<double, 6, 6> & covar)
{

	ROS_ERROR("AddMeasurement from %s to %s:  (%f, %f, %f)  (%f, %f, %f, %f)",
	      meas.frame_id_.c_str(),  meas.child_frame_id_.c_str(),
	      meas.getOrigin().x(),    meas.getOrigin().y(), meas.getOrigin().z(),
	      meas.getRotation().x(),  meas.getRotation().y(), 
	      meas.getRotation().z(),  meas.getRotation().w());
	
	listener->setTransform( meas );

	// process vo measurement
	// ----------------------
	filter_time=meas.stamp_;
	tf::StampedTransform vo_meas_=meas;
	if (!listener->canTransform(tracking_frame,"vo", filter_time)){
		ROS_ERROR("filter time older than vo message buffer");
		return false;
	}

	//listener->lookupTransform("vo", tracking_frame, filter_time, vo_meas_);

	if (vo_initialized_)
	{
		// convert absolute vo measurements to relative vo measurements
		tf::Transform vo_rel_frame =  meas_old_.inverse() * meas;

		// x y z direction_x direction_y direction_z
		Eigen::Matrix<double,6,1> vo_rel;

		Eigen::Affine3d trans_eigen;
		tf::transformTFToEigen(vo_rel_frame, trans_eigen);

		//Eigen::MatrixXd state=tracker_manager.pose->getState();
		//angleOverflowCorrect(vo_rel(5), state(5));

		// update filter
		//tracker_manager.updateFilter(vo_rel,  covar);
		tracker_manager.predict(trans_eigen.matrix(),  covar);
		//tf::transformEigenToTF(stateToMatrix(tracker_manager.pose->getState()),filter_estimate_old_);
	}
	else 
	{
		// Initialize
		filter_time_old_     = filter_time;

		Eigen::Matrix<double,6,1> vo_rel;
		decomposeTransform(meas,vo_rel);

		tracker_manager.initFilter(vo_rel, covar);
		//tf::transformEigenToTF(stateToMatrix(tracker_manager.pose->getState()),filter_estimate_old_);
		// filter initialized
		vo_initialized_ = true;
	}

	meas_old_ = meas;

	return true;
};

void CylinderTrackingROS::odomCallback (const nav_msgs::Odometry::ConstPtr & msg)
{
	//ROS_INFO("Seq: [%d]", msg->header.seq);
	//ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
	//ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	//ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

 	tf::Transform vo_meas_;
    	// get data
        ros::Time vo_stamp_ = msg->header.stamp;
	Eigen::Matrix<double, 6, 6> vo_covariance_;
	poseMsgToTF(msg->pose.pose, vo_meas_);
	for (unsigned int i=0; i<6; i++)
		for (unsigned int j=0; j<6; j++)
			vo_covariance_(i, j) = msg->pose.covariance[6*i+j];
	addMeasurement(tf::StampedTransform(vo_meas_.inverse(), vo_stamp_, tracking_frame, "vo"), vo_covariance_);
	
}

/*Eigen::Matrix4f CylinderTrackingROS::odom(const std::vector<Eigen::VectorXd> & detections_)
{
	
	Eigen::Matrix4f final_transform=Eigen::Matrix4f::Identity();

	std::string camera_link="camera_rgb_optical_frame";


        tf::StampedTransform baseDeltaTf;


        // Get delta motion in cartesian coordinates with TF
        ros::Time current_time;
	
        try
        {
            current_time = ros::Time::now();
            listener->waitForTransform(camera_link,odom_last_stamp, camera_link, current_time, odom_link, ros::Duration(0.1) );
            listener->lookupTransform(camera_link,odom_last_stamp, camera_link, current_time, odom_link, baseDeltaTf); // Velocity

        }
        catch (tf::TransformException &ex)
        {
            //ROS_WARN("%s",ex.what());
            return final_transform;
        }
	odom_last_stamp=current_time;
	Eigen::Affine3d transform_eigen;
	tf::transformTFToEigen (baseDeltaTf,transform_eigen);
	final_transform=transform_eigen.matrix().cast<double>();
	ROS_ERROR_STREAM("YA:"<<final_transform);
	if(tracker_manager.trackers.size()<2&&detections_.size()<2) return final_transform;

return final_transform;
	int height_samples=5;
	int angle_samples=4;

	// Generate cylinders detections point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr detections_cloud (new pcl::PointCloud<pcl::PointXYZ>());
	for(unsigned int d=0; d< detections_.size();++d)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>());

		Eigen::VectorXf state_=detections_[d].cast<double>();
		//Get rotation matrix
		Eigen::Matrix4f transf;
		transf=Eigen::Matrix4f::Identity();

	   	Eigen::Vector3f up = Eigen::Vector3f::UnitZ();
		Eigen::Vector3f cylinder_direction=state_.segment(3,3);
		if(up.dot(cylinder_direction)<0)
		{
			cylinder_direction=-cylinder_direction;
		}
	       	//Eigen::Vector3f rot_axis = up.cross(cylinder_direction);
		Eigen::Vector3f rot_axis = cylinder_direction.cross(up);

		rot_axis.normalize();
		if(!std::isnan(rot_axis[0])&&!std::isnan(rot_axis[1])&&!std::isnan(rot_axis[2]))
		{
			Eigen::Matrix3f aux;
			aux=Eigen::AngleAxisf(acos(cylinder_direction.dot(up)),rot_axis);
			transf.block(0,0,3,3)=aux;
		}

		// Get translation
		transf.block(0,3,3,1)=state_.segment(0,3);
		

		// Generate cylinder according to parameters
		double angle_step=2.0*M_PI/angle_samples;
		double height_step=fabs(detections_[d][7])/height_samples;

		for(int a=0; a < angle_samples; ++a)
		{
			double x,y,z;
			x=cos(angle_step*a)*fabs(detections_[d][6]);
			y=sin(angle_step*a)*fabs(detections_[d][6]);
			for(int h=0; h < height_samples; ++h)
			{
				z=(double)height_step*h;
				pcl::PointXYZ point(x,y,z);
				temp_cloud->push_back(point);
			}
		}

  		pcl::transformPointCloud (*temp_cloud, *temp_cloud, transf);

    		*detections_cloud += *temp_cloud;
	}

	// Generate cylinders trackers point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr trackers_cloud (new pcl::PointCloud<pcl::PointXYZ>());
	for(unsigned int d=0; d< tracker_manager.trackers.size();++d)
	{

		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>());


		Eigen::VectorXf state_=tracker_manager.trackers[d]->getState().cast<double>();

		//Get rotation matrix
		Eigen::Matrix4f transf;
		transf=Eigen::Matrix4f::Identity();

	   	Eigen::Vector3f up = Eigen::Vector3f::UnitZ();
		Eigen::Vector3f cylinder_direction=state_.segment(3,3);
		if(up.dot(cylinder_direction)<0)
		{
			cylinder_direction=-cylinder_direction;
		}
	       	//Eigen::Vector3f rot_axis = up.cross(cylinder_direction);
		Eigen::Vector3f rot_axis = cylinder_direction.cross(up);

		rot_axis.normalize();
		if(!std::isnan(rot_axis[0])&&!std::isnan(rot_axis[1])&&!std::isnan(rot_axis[2]))
		{
			Eigen::Matrix3f aux;
			aux=Eigen::AngleAxisf(acos(cylinder_direction.dot(up)),rot_axis);
			transf.block(0,0,3,3)=aux;
		}

		// Get translation
		transf.block(0,3,3,1)=state_.segment(0,3);





		// Generate cylinder according to parameters
		double angle_step=2.0*M_PI/angle_samples;
		double height_step=fabs(tracker_manager.trackers[d]->getState()[7])/height_samples;




		for(int a=0; a < angle_samples; ++a)
		{
			double x,y,z;
			x=cos(angle_step*a)*fabs(tracker_manager.trackers[d]->getState()[6]);
			y=sin(angle_step*a)*fabs(tracker_manager.trackers[d]->getState()[6]);
			for(int h=0; h < height_samples; ++h)
			{
				z=(double)height_step*h;
				pcl::PointXYZ point(x,y,z);
				temp_cloud->push_back(point);
			}
		}

  		pcl::transformPointCloud (*temp_cloud, *temp_cloud, transf);

    		*trackers_cloud += *temp_cloud;

		// Transform according to parameters
	}

	// COMPUTE ICP
 	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(trackers_cloud);
	icp.setInputTarget(detections_cloud);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);



	if(icp.hasConverged()&&icp.getFitnessScore()<0.1)
	{
		final_transform=icp.getFinalTransformation();

		std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  		//std::cout << icp.getFinalTransformation() << std::endl;
	}




	// Return ICP transform as relative odometry
	return final_transform;
}

*/



visualization_msgs::Marker CylinderTrackingROS::createMarker(const Eigen::VectorXd & model_params, int model_type, const std::string & frame, Color & color_, int id, const std::string & marker_namespace_)
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
	double height=model_params[7];

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
	marker.color.a = color_.a;
	marker.color.r = color_.r;
	marker.color.g = color_.g;
	marker.color.b = color_.b;
	//marker.lifetime = ros::Duration(0.05);
	return marker;
}


