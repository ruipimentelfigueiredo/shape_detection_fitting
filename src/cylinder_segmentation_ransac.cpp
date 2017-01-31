#include "cylinder_segmentation_ransac.h"
CylinderSegmentationRansac::CylinderSegmentationRansac(float normal_distance_weight_, unsigned int max_iterations_, float distance_threshold_, float min_radius_,float max_radius_, bool do_refine_) :
	CylinderSegmentation(min_radius_,max_radius_,do_refine_),
	normal_distance_weight(normal_distance_weight_),
	max_iterations(max_iterations_),
	distance_threshold(distance_threshold_)
{};

pcl::ModelCoefficients::Ptr CylinderSegmentationRansac::segment(const PointCloudT::ConstPtr & point_cloud_in_)
{
	// Estimate point normals
	ne.setSearchMethod (tree);
	ne.setInputCloud (point_cloud_in_);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_CYLINDER);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight (normal_distance_weight);
	seg.setMaxIterations (max_iterations);
	seg.setDistanceThreshold (distance_threshold);
	seg.setRadiusLimits (min_radius, max_radius);
	seg.setInputCloud (point_cloud_in_);
	seg.setInputNormals (cloud_normals);

	// Obtain the cylinder inliers and coefficients
	pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
	seg.segment (*inliers_cylinder, *coefficients_cylinder);

	std::cout << "inliers:" << inliers_cylinder->indices.size() << std::endl;

	// Extract the cylinder inliers
	pcl::ExtractIndices<PointT> extract; 
	extract.setInputCloud (point_cloud_in_);
	extract.setIndices (inliers_cylinder);
	extract.setNegative (true);
	extract.filter (*transformed_cloud);
	if (transformed_cloud->points.empty ()) 
		std::cerr << "Can't find the cylindrical component." << std::endl;

	std::cout << "in number:" << point_cloud_in_->points.size() << std::endl;
	std::cout << "out number:" << transformed_cloud->points.size() << std::endl;
	Eigen::Vector3f cylinder_direction(coefficients_cylinder->values[3],coefficients_cylinder->values[4],coefficients_cylinder->values[5]);

	//Get rotation matrix that aligns the cylinder point cloud with rotation axis
	Eigen::Matrix4f R2;
	R2=Eigen::Matrix4f::Identity();

   	Eigen::Vector3f up = Eigen::Vector3f::UnitZ();

	/*if(up.dot(cylinder_direction)<0)
	{
		ROS_INFO("YAHHHHH");
		//cylinder_direction=-cylinder_direction;
	}*/

	Eigen::Vector3f rot_axis = cylinder_direction.cross(up);

	rot_axis.normalize();

	if(std::isnan(rot_axis[0])||std::isnan(rot_axis[1])||std::isnan(rot_axis[2]))
	{
		R2=Eigen::Matrix4f::Identity();
	}
	else
	{
		Eigen::Matrix3f aux;
		aux=Eigen::AngleAxisf(acos(cylinder_direction.dot(up)),rot_axis);
		R2.block(0,0,3,3)=aux;
	}


	// Executing the transformation that aligns the cylinder rotation axis with z_up)
	pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, R2);

	Eigen::Vector4f min_pt,max_pt;
	pcl::getMinMax3D(*transformed_cloud,min_pt,max_pt);

	// First get cylinder posibilion in transformed coordinates
	Eigen::Vector3f cylinder_position=R2.block(0,0,3,3)*Eigen::Vector3f(coefficients_cylinder->values[0],coefficients_cylinder->values[1],coefficients_cylinder->values[2]);
	// Then change the height for the correct value and convert back to original coordinates;
	cylinder_position[2]=min_pt[2];
	Eigen::Vector3f cylinder_position_final=R2.block(0,0,3,3).transpose()*cylinder_position;


    	coefficients_cylinder->values[0]=cylinder_position_final[0];
    	coefficients_cylinder->values[1]=cylinder_position_final[1];
    	coefficients_cylinder->values[2]=cylinder_position_final[2];
	coefficients_cylinder->values[7]=fabs(max_pt[2]-min_pt[2]);

	
	/*viewer =simpleVis(point_cloud_in_,cloud_normals,coefficients_cylinder);


	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}//*/

	return coefficients_cylinder;
}




