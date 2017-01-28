#include "cylinder_segmentation.h"
boost::shared_ptr<pcl::visualization::PCLVisualizer> CylinderSegmentation::simpleVis (PointCloudT::ConstPtr cloud,  pcl::PointCloud<pcl::Normal>::ConstPtr normals, pcl::ModelCoefficients::Ptr coefficients_cylinder)
{
	
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);

  viewer->addPointCloud<PointT> (cloud,  "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  //viewer->addPointCloudNormals<PointT, NormalT> (cloud, normals, 10, 0.01, "normals");
  viewer->addCylinder (*coefficients_cylinder);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();


  return (viewer);
}


Eigen::Matrix4f CylinderSegmentation::refine(const PointCloudT::ConstPtr & point_cloud_source_, const PointCloudT::ConstPtr & point_cloud_target_)
{
	pcl::IterativeClosestPoint<PointT, PointT> icp;

	icp.setInputSource(point_cloud_source_);
	icp.setInputTarget(point_cloud_target_);
	PointCloudT::Ptr Final(new PointCloudT);
	icp.align(*Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	return icp.getFinalTransformation();
}
