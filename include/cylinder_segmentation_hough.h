#include "cylinder_segmentation.h"

class CylinderSegmentationHough : public CylinderSegmentation
{

	// private attributes
	unsigned int gaussian_sphere_points_num;
	unsigned int angle_bins;
	unsigned int radius_bins;
	unsigned int position_bins;
	float angle_step;
	float r_step;

	std::vector<float> cyl_direction_accum;
	std::vector<Eigen::Vector3f> gaussian_sphere_points;
	std::vector<std::vector<std::vector<unsigned int> > > cyl_circ_accum;


	// private methods
	Eigen::Vector3f findCylinderDirection(const NormalCloudT::ConstPtr & cloud_normals);
	Eigen::Matrix<float,5,1> findCylinderPositionRadius(const PointCloudT::ConstPtr & point_cloud_in_);

	public:
		CylinderSegmentationHough(unsigned int angle_bins_=30,unsigned int radius_bins_=10,unsigned int position_bins_=10,float min_radius_=0.01,float max_radius_=0.1, unsigned int gaussian_sphere_points_num_=900, bool do_refine_=false);

		pcl::ModelCoefficients::Ptr segment(const PointCloudT::ConstPtr & point_cloud_in_);


};

