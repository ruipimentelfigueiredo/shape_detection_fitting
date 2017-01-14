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
typedef pcl::PointCloud<PointT> PointCloudT;
//pcl::visualization::CloudViewer viewer("Cloud Viewer");


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

class CylinderSegmentationROS {

	ros::Subscriber point_cloud_sub;
	ros::Subscriber cluster_sub;
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
		visualization_msgs::Marker marker=createMarker(model_params,visualization_msgs::Marker::CYLINDER, input->header.frame_id);

	}

	void clusters_cb (const visualization_msgs::MarkerArray::ConstPtr& input)
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

			pcl::ModelCoefficients::Ptr model_params=cylinder_segmentation.segment(cloud_);
			visualization_msgs::Marker marker=createMarker(model_params,visualization_msgs::Marker::CYLINDER,input->markers[i].header.frame_id, i);


			markers_.markers.push_back(marker);
break;
		}
		// Publish the data.
		vis_pub.publish( markers_ );

	}
public:
	CylinderSegmentation cylinder_segmentation;

	CylinderSegmentationROS(ros::NodeHandle & n_) : n(n_)
	{
		cluster_sub=n.subscribe<visualization_msgs::MarkerArray> ("clusters_in", 1, &CylinderSegmentationROS::clusters_cb, this);
		//point_cloud_sub=n.subscribe<pcl::PointCloud<PointT> > ("cloud_in", 1, &CylinderSegmentationROS::cloud_cb, this);
		vis_pub = n.advertise<visualization_msgs::MarkerArray>( "cylinder_marker", 0 );
	};


	visualization_msgs::Marker createMarker(const pcl::ModelCoefficients::Ptr & model_params, int model_type, const std::string & frame, int id=0)
	{

tf::Vector3 axis_vector(model_params->values[3], model_params->values[4], model_params->values[5]);

tf::Vector3 up_vector(0.0, 0.0, 1.0);
tf::Vector3 right_vector = axis_vector.cross(up_vector);
right_vector.normalized();
tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
q.normalize();
geometry_msgs::Quaternion cylinder_orientation;
tf::quaternionTFToMsg(q, cylinder_orientation);

//(...)




		ROS_INFO_STREAM(frame);
		visualization_msgs::Marker marker;
		marker.header.frame_id =frame;
		marker.header.stamp = ros::Time();
		marker.ns = "model";
		marker.id = id;
		marker.type = model_type;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = model_params->values[0];
		marker.pose.position.y = model_params->values[1];
		marker.pose.position.z = model_params->values[2];
marker.pose.orientation = cylinder_orientation;
/*		marker.pose.orientation.x = Q.x();
		marker.pose.orientation.y = Q.y();
		marker.pose.orientation.z = Q.z();
		marker.pose.orientation.w = Q.w();*/
		marker.scale.x = model_params->values[6];
		marker.scale.y = model_params->values[6];
		marker.scale.z = 0.1;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker.lifetime = ros::Duration(0.1);
		return marker;
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
