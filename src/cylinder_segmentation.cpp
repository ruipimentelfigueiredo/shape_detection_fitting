#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/ransac.h>
#include <Eigen/Geometry>
#include <tf/tf.h>
typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
//pcl::visualization::CloudViewer viewer("Cloud Viewer");





    pcl::ModelCoefficients::Ptr CylinderSegmentationRansac::segment(const PointCloudT::ConstPtr & point_cloud_in_)
    {
    	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
        // Build a passthrough filter to remove spurious NaNs
        pass.setInputCloud (point_cloud_in_);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0, 1.5);
        pass.filter (*cloud_filtered);
        std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

        // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setInputCloud (point_cloud_in_);
        ne.setKSearch (50);
        ne.compute (*cloud_normals);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_CYLINDER);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight (0.1);
	seg.setMaxIterations (10000);
	seg.setDistanceThreshold (0.01);
	seg.setRadiusLimits (0.01, 0.1);
	seg.setInputCloud (point_cloud_in_);
	seg.setInputNormals (cloud_normals);

  	// Obtain the cylinder inliers and coefficients
  	seg.segment (*inliers_cylinder, *coefficients_cylinder);
  	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

        /*ne.setSearchMethod (tree);
        ne.setInputCloud (point_cloud_in_);
        ne.setKSearch (50);
        ne.compute (*cloud_normals);


	// RANSAC objects: model and algorithm.
	pcl::SampleConsensusModelCylinder<PointT, pcl::Normal>::Ptr model(new pcl::SampleConsensusModelCylinder<PointT, pcl::Normal>(point_cloud_in_,cloud_normals));
	model->setInputCloud(point_cloud_in_); 
  	model->setInputNormals(cloud_normals); 
    	model->setRadiusLimits (0.01, 0.1);
    	pcl::RandomSampleConsensus<PointT> ransac (model); 

  	ransac.setDistanceThreshold (0.01); 
  	ransac.setMaxIterations(10000); 

    	ransac.computeModel();

 	Eigen::VectorXf coef; 
  	ransac.getModelCoefficients(coef); 
	std::vector< int > inliers;
	ransac.getInliers (inliers);
	ROS_INFO_STREAM("INLIers size:"<<inliers.size());
	ROS_INFO_STREAM(coef);

  	// Write the cylinder inliers to disk
	//extract.setInputCloud (cloud_filtered2);
	//extract.setIndices (inliers_cylinder);
	//extract.setNegative (false);
	//pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());*/

	return coefficients_cylinder;
    }
};



