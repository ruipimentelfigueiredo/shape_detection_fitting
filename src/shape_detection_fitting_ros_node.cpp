/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include "shape_fitting_ros.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "shape_detection_fitting");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/

	ros::NodeHandle n;
	ros::NodeHandle n_priv("~");
	ros::Rate loop_rate(30);
	std::string config_file;
	n_priv.param<std::string>("config_file",config_file,"ola");

	Config config;
	config.parseConfig(config_file);

	boost::shared_ptr<CylinderClassifier> cylinder_classifier;
	if(config.with_classifier)
		cylinder_classifier=boost::shared_ptr<CylinderClassifier> (new CylinderClassifier(config.classifier_absolute_path_folder,config.model_file,config.weight_file,config.mean_file,config.device,(unsigned int)config.device_id));

	GaussianSphere gaussian_sphere(config.gmm,config.gaussian_sphere_points_num,config.orientation_accumulators_num);

	boost::shared_ptr<SphereFittingHough> sphere_fitting(new SphereFittingHough(gaussian_sphere,config.position_bins,config.radius_bins,config.min_radius, config.max_radius,config.accumulator_peak_threshold));

	boost::shared_ptr<CylinderFittingHough> cylinder_fitting(new CylinderFittingHough(gaussian_sphere,config.angle_bins,config.radius_bins,config.position_bins,config.min_radius, config.max_radius,config.accumulator_peak_threshold,config.mode));

	boost::shared_ptr<PlaneFittingRansac> plane_fitting(new PlaneFittingRansac(config.distance_threshold,config.cluster_tolerance,config.min_cluster_size,config.max_cluster_size,config.do_refine,config.table_z_filter_min,config.table_z_filter_max,config.z_filter_min, config.z_filter_max,config.plane_detection_voxel_size, config.cluster_voxel_size,config.inlier_threshold));

	boost::shared_ptr<PlanarTopDetector<CylinderFittingHough,SphereFittingHough,PlaneFittingRansac> > planar_top_detector(new PlanarTopDetector<CylinderFittingHough, SphereFittingHough, PlaneFittingRansac>(plane_fitting, cylinder_classifier, cylinder_fitting, sphere_fitting, config.cam_projection, config.classification_threshold, config.padding, config.cluster_voxel_size, config.z_filter_min, config.z_filter_max, config.with_classifier, config.visualize));

	ShapeFittingROS<CylinderFittingHough,SphereFittingHough,PlaneFittingRansac> shape_fitting_ros(n, n_priv, planar_top_detector);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return (0);
}

