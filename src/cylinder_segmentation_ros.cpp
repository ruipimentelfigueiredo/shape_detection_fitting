#include "cylinder_segmentation_ros.h"
#include <pcl/filters/voxel_grid.h>

template <class detector_type>
CylinderSegmentationROS<detector_type>::CylinderSegmentationROS(ros::NodeHandle & n_, boost::shared_ptr<detector_type> & cylinder_segmentation_) : 
	n(n_), 
	n_priv("~"),
	cylinder_segmentation(cylinder_segmentation_)
	
{
	cluster_sub=n.subscribe<visualization_msgs::MarkerArray> ("clusters_in", 1, &CylinderSegmentationROS::clusters_cb, this);
	point_cloud_sub=n.subscribe<pcl::PointCloud<PointT> > ("cloud_in", 1, &CylinderSegmentationROS::cloud_cb, this);
	
	vis_pub = n.advertise<visualization_msgs::MarkerArray>( "cylinders_markers", 0 );
	cloud_pub=n.advertise<pcl::PointCloud<PointT> >("input_cloud", 0 );
}

template <class detector_type>
void CylinderSegmentationROS<detector_type>::cloud_cb (const PointCloudT::ConstPtr& input)
{
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);

	// Build a passthrough filter to remove spurious NaNs
	/*pcl::PassThrough<PointT> pass;
	pass.setInputCloud (input);
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
	pcl::ModelCoefficients::Ptr model_params=cylinder_segmentation->segment(input);
	std::cout << float( clock () - begin_time ) /  CLOCKS_PER_SEC<< " seconds"<<std::endl;

	// Publish the data.
	visualization_msgs::Marker marker=createMarker(model_params,visualization_msgs::Marker::CYLINDER,input->header.frame_id, 0);
	markers_.markers.push_back(marker);
	vis_pub.publish( markers_ );

	cloud_pub.publish(input);
}

template <class detector_type>
void CylinderSegmentationROS<detector_type>::clusters_cb (const visualization_msgs::MarkerArray::ConstPtr& input)
{
	// Create a container for the data.
	sensor_msgs::PointCloud2 output;

	visualization_msgs::MarkerArray markers_;
	//pcl::ModelCoefficients::Ptr model_params=cylinder_segmentation.segment(input);
	for(int i=0; i<input->markers.size();++i)
	{

		PointCloudT::Ptr cloud_(new PointCloudT);

		cloud_->header.frame_id = input->markers[i].header.frame_id;
		//cloud_->height = input->markers[i].height;

		for (int p=0; p<input->markers[i].points.size();++p)
		{

			double x_=input->markers[i].points[p].x;
			double y_=input->markers[i].points[p].y;
			double z_=input->markers[i].points[p].z;
			cloud_->points.push_back (pcl::PointXYZ(x_, y_, z_));
					//pcl::PointCloudinput->markers[i].points
		}

		pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);


		// Create the filtering object
		pcl::VoxelGrid<PointT> sor;
		sor.setInputCloud(cloud_);
		sor.setLeafSize(0.005f, 0.005f, 0.005f);
		sor.filter(*cloud_filtered);

		pcl::ModelCoefficients::Ptr model_params=cylinder_segmentation->segment(cloud_filtered);
		visualization_msgs::Marker marker=createMarker(model_params,visualization_msgs::Marker::CYLINDER,input->markers[i].header.frame_id, i);

		markers_.markers.push_back(marker);
	}
	// Publish the data.
	vis_pub.publish( markers_ );

}

template <class detector_type>
visualization_msgs::Marker CylinderSegmentationROS<detector_type>::createMarker(const pcl::ModelCoefficients::Ptr & model_params, int model_type, const std::string & frame, int id)
{

	tf::Vector3 axis_vector(model_params->values[3], model_params->values[4], model_params->values[5]);
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
	float height=model_params->values[7];
	ROS_INFO_STREAM(frame);
	visualization_msgs::Marker marker;
	marker.header.frame_id =frame;
	marker.header.stamp = ros::Time();
	marker.ns = "model";
	marker.id = id;
	marker.type = model_type;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = model_params->values[0]+0.5*height*axis_vector[0];
	marker.pose.position.y = model_params->values[1]+0.5*height*axis_vector[1];
	marker.pose.position.z = model_params->values[2]+0.5*height*axis_vector[2];
	marker.pose.orientation = cylinder_orientation;
/*		marker.pose.orientation.x = Q.x();
	marker.pose.orientation.y = Q.y();
	marker.pose.orientation.z = Q.z();
	marker.pose.orientation.w = Q.w();*/
	marker.scale.x = 2*model_params->values[6];
	marker.scale.y = 2*model_params->values[6];
	marker.scale.z = height;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.lifetime = ros::Duration(0.05);
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

