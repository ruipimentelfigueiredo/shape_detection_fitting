#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
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


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

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

    pcl::ModelCoefficients::Ptr segment(const PointCloudT::ConstPtr & point_cloud_in_)
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
        ne.setInputCloud (cloud_filtered);
        ne.setKSearch (50);
        ne.compute (*cloud_normals);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_CYLINDER);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight (0.1);
	seg.setMaxIterations (10000);
	seg.setDistanceThreshold (0.05);
	seg.setRadiusLimits (0, 0.1);
	seg.setInputCloud (cloud_filtered);
	seg.setInputNormals (cloud_normals);

  	// Obtain the cylinder inliers and coefficients
  	seg.segment (*inliers_cylinder, *coefficients_cylinder);
  	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  	// Write the cylinder inliers to disk
	//extract.setInputCloud (cloud_filtered2);
	//extract.setIndices (inliers_cylinder);
	//extract.setNegative (false);
	//pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());

	return coefficients_cylinder;
    }
};

class CylinderSegmentationROS {

	ros::Subscriber point_cloud_sub;
	ros::NodeHandle n;
	ros::Publisher vis_pub;

	void cloud_cb (const PointCloudT::ConstPtr& input)
	{
		// Create a container for the data.
		sensor_msgs::PointCloud2 output;

		// Do data processing here...
		//output = *input;

		pcl::ModelCoefficients::Ptr model_params=cylinder_segmentation.segment(input);
		// Publish the data.
		publishMarker(model_params,visualization_msgs::Marker::CYLINDER, input->header.frame_id);
	}
public:
	CylinderSegmentation cylinder_segmentation;

	CylinderSegmentationROS(ros::NodeHandle & n_) : n(n_)
	{
		point_cloud_sub=n.subscribe<pcl::PointCloud<PointT> > ("cloud_in", 1, &CylinderSegmentationROS::cloud_cb, this);
		vis_pub = n.advertise<visualization_msgs::Marker>( "cylinder_marker", 0 );
	};

 	void publishMarker(pcl::ModelCoefficients::Ptr model_params, int model_type, const std::string & frame)
	{
		ROS_INFO_STREAM(frame);
		visualization_msgs::Marker marker;
		marker.header.frame_id =frame;
		marker.header.stamp = ros::Time();
		marker.ns = "model";
		marker.id = 0;
		marker.type = model_type;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = model_params->values[0];
		marker.pose.position.y = model_params->values[1];
		marker.pose.position.z = model_params->values[2];
		marker.pose.orientation.x = model_params->values[3];
		marker.pose.orientation.y = model_params->values[4];
		marker.pose.orientation.z = model_params->values[5];
		marker.pose.orientation.w = 1.0;
		marker.scale.x = model_params->values[6];
		marker.scale.y = model_params->values[6];
		marker.scale.z = model_params->values[6];
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		//only if using a MESH_RESOURCE marker type:
		marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		vis_pub.publish( marker );
	}
};

int main (int argc, char** argv)
{
	ros::init(argc, argv, "talker");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/

	ros::NodeHandle n;
	ros::Rate loop_rate(30);

	CylinderSegmentationROS cylinder_segmentation_ros(n);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
  
	return (0);
}
