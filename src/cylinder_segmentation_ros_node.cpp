#include "cylinder_segmentation_ros.h"

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
	ros::Rate loop_rate(300);

	ROS_INFO("Getting cameras' parameterssss");
	std::string camera_info_topic;
	n_priv.param<std::string>("camera_info_topic", camera_info_topic, "camera_info_topic");
	ROS_INFO_STREAM("camera_info_topic:"<< camera_info_topic);
	sensor_msgs::CameraInfoConstPtr camera_info=ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic, ros::Duration(3.0));

	//set the cameras intrinsic parameters
	Eigen::Matrix4f cam_intrinsic = Eigen::Matrix4f::Identity();
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
	double classification_threshold;

	ROS_INFO("Getting classifier parameters");
	n_priv.param<std::string>("absolute_path_folder", absolute_path_folder, "absolute_path_folder");
	n_priv.param<std::string>("model_file", model_file, "model_file");
	n_priv.param<std::string>("weight_file", weight_file, "weight_file");
	n_priv.param<std::string>("mean_file", mean_file, "mean_file");
	n_priv.param<std::string>("device", device, "device");
	n_priv.param<int>("device_id", device_id, 0);

	n_priv.param<double>("classification_threshold", classification_threshold, 0.9);

	ROS_INFO_STREAM("absolute_path_folder:"<< absolute_path_folder);
	ROS_INFO_STREAM("model_file:"<< model_file);
	ROS_INFO_STREAM("weight_file:"<< weight_file);
	ROS_INFO_STREAM("mean_file:"<< mean_file);
	ROS_INFO_STREAM("device:"<< device);
	ROS_INFO_STREAM("device_id:"<< device_id);
	ROS_INFO_STREAM("classification_threshold:"<< classification_threshold);
	boost::shared_ptr<CylinderClassifier> cylinder_classifier(new CylinderClassifier(absolute_path_folder,model_file,weight_file,mean_file,device,(unsigned int)device_id));

	/////////////////////////
	// Load fitting params //
	/////////////////////////

	bool use_ransac;

	// Common params
 	double min_radius;
 	double max_radius;


    	n_priv.param("use_ransac",use_ransac,true);
	ROS_INFO_STREAM("use_ransac: "<< use_ransac);
    	
	n_priv.param("min_radius", min_radius, 0.1);
    	n_priv.param("max_radius", max_radius, 0.5);

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

	        boost::shared_ptr<ShapeDetectionManager<CylinderSegmentationRansac> > shape_detection_manager(new ShapeDetectionManager<CylinderSegmentationRansac>(cylinder_classifier,cylinder_segmentation,cam_intrinsic,classification_threshold));

		CylinderSegmentationROS<CylinderSegmentationRansac> cylinder_segmentation_ros(n, n_priv,shape_detection_manager);

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
		double accumulator_peak_threshold;
		int orientation_accumulators_num;
		int mode;

	    	n_priv.param("angle_bins",angle_bins,50);
	    	n_priv.param("radius_bins",radius_bins,50);
	    	n_priv.param("position_bins",position_bins,50);
	    	n_priv.param("gaussian_sphere_points_num", gaussian_sphere_points_num, 1000);
	    	n_priv.param("accumulator_peak_threshold", accumulator_peak_threshold, 0.2);
	    	n_priv.param("orientation_accumulators_num", orientation_accumulators_num, 10);
	    	n_priv.param("mode", mode, 2);

		ROS_INFO_STREAM("angle_bins: "<< angle_bins);
		ROS_INFO_STREAM("radius_bins: "<< radius_bins);
		ROS_INFO_STREAM("position_bins: "<< position_bins);
		ROS_INFO_STREAM("gaussian_sphere_points_num: "<< gaussian_sphere_points_num);
		ROS_INFO_STREAM("accumulator_peak_threshold: "<< accumulator_peak_threshold);
		ROS_INFO_STREAM("orientation_accumulators_num: "<< orientation_accumulators_num);

		XmlRpc::XmlRpcValue orientation_hough_gmm;
		n_priv.getParam("orientation_hough_gmm", orientation_hough_gmm);

		ROS_INFO_STREAM("Loading gaussian mixtures: " << orientation_hough_gmm.size());

		std::vector<double> weights;
		std::vector<Eigen::Matrix<double, 3 ,1> > means;
		std::vector<Eigen::Matrix<double, 3 ,1> > std_devs;
		// Number o components
		for(int32_t i=0; i < orientation_hough_gmm.size(); ++i)
		{
			if(orientation_hough_gmm[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
			{
				double weight=orientation_hough_gmm[i]["weight"];
				weights.push_back(weight);

				XmlRpc::XmlRpcValue mean_param=orientation_hough_gmm[i]["mean"];
				Eigen::Matrix<double, 3 ,1> mean_eigen(mean_param[0],mean_param[1],mean_param[2]);
				means.push_back(mean_eigen);

			  	XmlRpc::XmlRpcValue std_dev_param=orientation_hough_gmm[i]["standard_deviation"];
				Eigen::Matrix<double, 3 ,1> std_dev_eigen(std_dev_param[0],std_dev_param[1],std_dev_param[2]);
				std_devs.push_back(std_dev_eigen);

				std::cout << "  weight:  " << weight << std::endl;
				std::cout << "  mean:    " << mean_eigen.transpose() << std::endl;
				std::cout << "  std_dev: " << std_dev_eigen.transpose() << std::endl;
			}
		}
		GaussianMixtureModel gmm(weights, means, std_devs);
		GaussianSphere gaussian_sphere(gmm,gaussian_sphere_points_num,orientation_accumulators_num);

		boost::shared_ptr<CylinderSegmentationHough> cylinder_segmentation(new CylinderSegmentationHough(gaussian_sphere,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,(unsigned int)mode));

	        boost::shared_ptr<ShapeDetectionManager<CylinderSegmentationHough> > shape_detection_manager(new ShapeDetectionManager<CylinderSegmentationHough>(cylinder_classifier,cylinder_segmentation,cam_intrinsic,classification_threshold));

		CylinderSegmentationROS<CylinderSegmentationHough> cylinder_segmentation_ros(n, n_priv,shape_detection_manager);

		while (ros::ok())
		{
			ros::spinOnce();
			loop_rate.sleep();
		}

	}

	return (0);
}

