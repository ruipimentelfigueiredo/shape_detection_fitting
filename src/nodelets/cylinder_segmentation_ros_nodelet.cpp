#include "nodelets/cylinder_segmentation_ros_nodelet.h"

namespace active_semantic_mapping
{
    void CylinderSegmentationRosNodelet::onInit()
    {

	ros::NodeHandle n_priv=getPrivateNodeHandle();
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


		NODELET_INFO("Initializing nodelet");
		inst_ransac_.reset(new CylinderSegmentationROS<CylinderSegmentationRansac>(getNodeHandle(),getPrivateNodeHandle(),cylinder_segmentation));
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


		NODELET_INFO("Initializing nodelet");
		inst_hough_.reset(new CylinderSegmentationROS<CylinderSegmentationHough>(getNodeHandle(),getPrivateNodeHandle(),cylinder_segmentation));

	}





    }
}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(active_semantic_mapping::CylinderSegmentationRosNodelet,nodelet::Nodelet)
