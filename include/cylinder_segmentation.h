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


class CylinderSegmentationHough
{

	// private attributes
	unsigned int angle_bins;
	float angle_step;

	std::vector<std::vector<unsigned int> > cyl_direction_accum;

	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::PointCloud<PointT>::Ptr cloud_filtered;// (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;// (new pcl::PointCloud<pcl::Normal>);
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree; 
	pcl::PointIndices::Ptr inliers_cylinder;// (new pcl::PointIndices);

	void houghDirection(const PointCloudT::ConstPtr & point_cloud_in_)
	{
		pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
		// Build a passthrough filter to remove spurious NaNs
		ROS_INFO_STREAM(" 1. Filter");
		pass.setInputCloud (point_cloud_in_);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0, 1.5);
		pass.filter (*cloud_filtered);
		std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

		//1.  Estimate point normals
		ROS_INFO_STREAM(" 2. Estimate normals");
		ne.setSearchMethod (tree);
		ne.setInputCloud (cloud_filtered);
		ne.setKSearch (50);
		ne.compute (*cloud_normals);


		//3. for each point normal
		ROS_INFO_STREAM(" 3. Hough Transform");
				
		ROS_INFO_STREAM("  3.1. Reset accumulator");
        	for (unsigned int theta_index=0; theta_index < cyl_direction_accum.size(); ++theta_index) {
			std::fill(cyl_direction_accum[theta_index].begin(),cyl_direction_accum[theta_index].end(), 0);
		}
	
		ROS_INFO_STREAM("  3.2. Vote");
		for(NormalCloudT::iterator it = cloud_normals->begin(); it != cloud_normals->end(); ++it)
		{
 			// 3.2.1 compute b
			Eigen::Vector3f b=(Eigen::Vector3f::UnitZ()-it->getNormalVector3fMap ());
			b.normalize();

 			// 3.2.2 derive matrix R
			Eigen::Matrix3f R=Eigen::Matrix3f::Identity() - 2*b*b.transpose();
			float theta=acos(10.0);
			float phi=acos(10.0);

			unsigned int theta_bin=floor(theta/angle_step);
			unsigned int phi_bin=floor(phi/angle_step);
 			
			//3.2 Get corresponding parameters and vote (+1)
			++cyl_direction_accum[theta_bin][phi_bin];
		}

		ROS_INFO_STREAM("  3.3. Get max peak");
		unsigned int best_theta;
		unsigned int best_phi;
		unsigned int most_votes=0;
		for (unsigned theta_index=0; theta_index < cyl_direction_accum.size(); ++theta_index) {
			for (unsigned phi_index=0; phi_index < cyl_direction_accum[theta_index].size(); ++phi_index) {
				if(cyl_direction_accum[theta_index][phi_index]>most_votes)
				{
					best_theta=theta_index;
					best_phi=phi_index;
				}
			}
		}
	}



	public:
	    CylinderSegmentationHough(float angle_bins_=30) : 
		angle_bins(angle_bins_),
		angle_step(M_PI/angle_bins),
		//coefficients_cylinder(new pcl::ModelCoefficients),
		cloud_filtered(new pcl::PointCloud<PointT>),
		cloud_normals(new pcl::PointCloud<pcl::Normal>),
		tree(new pcl::search::KdTree<PointT> ()),
		inliers_cylinder(new pcl::PointIndices)
	    {};

};

class CylinderSegmentation
{
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;


    pcl::PointCloud<PointT>::Ptr cloud_filtered;// (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;// (new pcl::PointCloud<pcl::Normal>);
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree; 
    pcl::PointIndices::Ptr inliers_cylinder;// (new pcl::PointIndices);
public:
    CylinderSegmentation() : 
	//coefficients_cylinder(new pcl::ModelCoefficients),
	cloud_filtered(new pcl::PointCloud<PointT>),
	cloud_normals(new pcl::PointCloud<pcl::Normal>),
	tree(new pcl::search::KdTree<PointT> ()),
	inliers_cylinder(new pcl::PointIndices)
    {};

    pcl::ModelCoefficients::Ptr segment(const PointCloudT::ConstPtr & point_cloud_in_);
};



