#include "cylinder_segmentation_hough.h"
#include <ctime>
#include <tf/transform_broadcaster.h>

#include <rosbag/bag.h>
#include <active_semantic_mapping/Cylinders.h>
#include "pcl_ros/point_cloud.h"

int main (int argc, char** argv)
{
	/* TODO - Process options */
	if (argc < 6)
	{
		std::cout << "invalid number of arguments: rosbag_dir iterations heights radii noise clutter"<< std::endl;
		exit(-1);
	}

    	std::string rosbag_file = std::string(argv[1]);

	unsigned int iterations=atoi(argv[2]);

	static std::string heights_ = std::string(argv[3]);
	std::cout << "heights: " << heights_<< std::endl;

	static std::string radii_ = std::string(argv[4]); 
	std::cout << "radii: " << radii_<< std::endl;

	static std::string noise_levels_ = std::string(argv[5]);
	std::cout << "noise: " << noise_levels_<< std::endl;

	static std::string clutter_levels_ = std::string(argv[6]);
	std::cout << "clutter: " << clutter_levels_<< std::endl;

	static int height_samples = atoi(argv[7]);
	std::cout << "height_samples: " << height_samples<< std::endl;

	static int angle_samples = atoi(argv[8]);
	std::cout << "angle_samples: " << angle_samples<< std::endl;

	std::vector<float> heights;
	std::vector<float> radii;
	std::vector<float> noise_levels; // percentage of object radius (std_dev)
	std::vector<float> clutter_levels; // percentage of object size (std_dev)

	float j;
	std::stringstream ss_(heights_);
	while (ss_ >> j)
	{
		heights.push_back(j);

		if (ss_.peek() == ',')
	    		ss_.ignore();
	}

	ss_=std::stringstream(radii_);
	while (ss_ >> j)
	{
		radii.push_back(j);

		if (ss_.peek() == ',')
	    		ss_.ignore();
	}

	ss_=std::stringstream(noise_levels_);
	while (ss_ >> j)
	{
		noise_levels.push_back(j);

		if (ss_.peek() == ',')
	    		ss_.ignore();
	}

	ss_=std::stringstream(clutter_levels_);
	while (ss_ >> j)
	{
		clutter_levels.push_back(j);

		if (ss_.peek() == ',')
	    		ss_.ignore();
	}

	ros::init(argc, argv, "cylinder_publisher");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/

	ros::NodeHandle n;

	rosbag::Bag bag;

	bag.open(rosbag_file, rosbag::bagmode::Write);

	
	std::vector<pcl::PointCloud<pcl::PointXYZ> > point_clouds;
	std::vector<active_semantic_mapping::Cylinders> ground_truths;
	std::vector<Eigen::Matrix4f> transfs;

	// First, generate 200 point clouds with different radius, heights at random poses

	for(unsigned int c=0; c<clutter_levels.size();++c)
	{
		for(unsigned int i=0; i<iterations; ++i)
		{
			for(unsigned int h_=0; h_<heights.size();++h_)
			{
				for(unsigned int r_=0; r_<radii.size();++r_)
				{
					float height=heights[h_];
					float radius=radii[r_];
					// Generate cylinder according to parameters
					pcl::PointCloud<PointT> cloud;
					cloud.header.frame_id="world";

					float angle_step=2.0*M_PI/angle_samples;
					float height_step=fabs(height/height_samples);

					//std::cout << "height:" << height.at<float>(0,0) << " radius:" << radius.at<float>(0,0) << std::endl;

	
					for(int a=0; a < angle_samples; ++a)
					{

						float x,y,z;
						x=cos(angle_step*a)*fabs(radius);
						y=sin(angle_step*a)*fabs(radius);
						for(int h=0; h < height_samples; ++h)
						{
							z=(float)height_step*h;
							pcl::PointXYZ point(x,y,z);
							cloud.push_back(point);
						}
					}

					// TAMPAS
					unsigned int plane_samples=clutter_levels[c]*sqrt(height_samples*angle_samples);
				
					float plane_size=2*radius;
					float plane_min=0.0-radius;
					float plane_step=(float)plane_size/plane_samples;
					float z=0.0;
					for(int x_=0; x_<plane_samples;++x_)
					{
						float x=plane_min+x_*plane_step;
						for(int y_=0; y_<plane_samples;++y_)
						{
						
							float y=plane_min+y_*plane_step;
							if(sqrt(x*x+y*y)>radius) continue;

							pcl::PointXYZ point(x,y,z);
							//cloud.push_back(point);

							cloud.push_back(pcl::PointXYZ(x,y,height));
							//ROS_ERROR_STREAM("YAH:"<<point);
						}
					}
					point_clouds.push_back(cloud);

					// Generate random orientation
					Eigen::Vector3f rot_dir;
					cv::Mat aux(1, 1, CV_32F);


					cv::randn(aux, 0.0, 1.0);
					rot_dir(0,0)=aux.at<float>(0,0);

					cv::randn(aux, 0.0, 1.0);
					rot_dir(1,0)=aux.at<float>(0,0);

					cv::randn(aux, 0.0, 1.0);
					rot_dir(2,0)=aux.at<float>(0,0);

					rot_dir.normalize();

					cv::Mat angle(1, 1, CV_32F);
					cv::randn(angle, 0.0, 1.0);
					//Eigen::Vector3f rot_dir(1.0,0.0,0.0);
					rot_dir.normalize();
					rot_dir=Eigen::Vector3f::UnitZ();
					Eigen::Matrix3f rot;
					rot=Eigen::AngleAxisf(M_PI*angle.at<float>(0,0),rot_dir);

					//Eigen::Vector3f cyl_dir=rot*Eigen::Vector3f::UnitZ();
					Eigen::Vector3f cyl_dir=Eigen::Vector3f::UnitZ();
					Eigen::Matrix4f transf;
					transf.block(0,0,3,3)=rot;
					transf.block(0,3,3,1)=Eigen::Vector3f(0,0,0);
					transfs.push_back(transf);

					active_semantic_mapping::Cylinders ground_truth;

					ground_truth.cylinders.layout.dim.resize(2);
					ground_truth.cylinders.layout.dim[0].label  = "cylinders";
					ground_truth.cylinders.layout.dim[0].size   = 1;
					ground_truth.cylinders.layout.dim[0].stride = 8;
					ground_truth.cylinders.layout.dim[1].label  = "parameters";
					ground_truth.cylinders.layout.dim[1].size   = 8;
					ground_truth.cylinders.layout.dim[1].stride = 8;

					ground_truth.cylinders.data.push_back(0.5*cyl_dir[0]*height);
					ground_truth.cylinders.data.push_back(0.5*cyl_dir[1]*height);
					ground_truth.cylinders.data.push_back(0.5*cyl_dir[2]*height);
		
					ground_truth.cylinders.data.push_back(cyl_dir[0]);
					ground_truth.cylinders.data.push_back(cyl_dir[1]);
					ground_truth.cylinders.data.push_back(cyl_dir[2]);

					ground_truth.cylinders.data.push_back(radius);
					ground_truth.cylinders.data.push_back(height);


					for(unsigned int n=0; n<noise_levels.size(); ++n)
					{
						bag.write("ground_truth",ros::Time::now(), ground_truth);
					}
				}
			}
		}
	}
	// Then corrupt with diferent levels of noise
	for(unsigned int n=0; n<noise_levels.size(); ++n)
	{
		for(unsigned int c=0; c<clutter_levels.size();++c)
		{
			for(unsigned int i=0; i<iterations; ++i)
			{
				for(unsigned int h_=0; h_<heights.size();++h_)
				{
					for(unsigned int r_=0; r_<radii.size();++r_)
					{
						int index=r_+h_*radii.size()+i*radii.size()*heights.size();
						pcl::PointCloud<PointT> cloud=point_clouds[index];
		

						pcl::PointCloud<PointT> noisy_cloud;
						noisy_cloud=cloud;
						for(unsigned int p=0; p<cloud.size();++p)
						{
							cv::Mat aux(1, 1, CV_32F);
							cv::randn(aux, 0.0, noise_levels[n]*radii[r_]);
							noisy_cloud.points[p].x+=aux.at<float>(0,0);

							cv::randn(aux, 0.0, noise_levels[n]*radii[r_]);
							noisy_cloud.points[p].y+=aux.at<float>(0,0);

							cv::randn(aux, 0.0, noise_levels[n]*radii[r_]);
							noisy_cloud.points[p].z+=aux.at<float>(0,0);
						}

						//Transform point cloud
						pcl::PointCloud<PointT> cloud_transf;
						// transfs[index]
						Eigen::Matrix4f transf=Eigen::Matrix4f::Identity();
						pcl::transformPointCloud (noisy_cloud, cloud_transf,transf);

			    			bag.write("point_cloud",ros::Time::now(), cloud_transf);


						ROS_INFO_STREAM(" iteration:" << i << " clutter_level: "<< clutter_levels[c] << " noise_level: "<< noise_levels[n] << " height: " << heights[h_] << " radius: " << radii[r_] << " total iteration: "<< r_+h_*radii.size()+i*radii.size()*heights.size()+c*radii.size()*heights.size()*iterations+n*radii.size()*heights.size()*clutter_levels.size()*iterations); 
					}
				}
			}
		}
	}

	bag.close();

	// Segment cylinder





	return (0);
}
