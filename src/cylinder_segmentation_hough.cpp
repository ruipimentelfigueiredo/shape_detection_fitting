#include "cylinder_segmentation_hough.h"

CylinderSegmentationHough::CylinderSegmentationHough(unsigned int angle_bins_,unsigned int radius_bins_, unsigned int position_bins_, float min_radius_, float max_radius_) : 
	angle_bins(angle_bins_),
	angle_step(2*M_PI/angle_bins),
	position_bins(position_bins_),
	radius_bins(radius_bins_),
	min_radius(min_radius_),
	max_radius(max_radius_),
	r_step((max_radius-min_radius)/radius_bins),
	//coefficients_cylinder(new pcl::ModelCoefficients),
	cloud_filtered(new pcl::PointCloud<PointT>),
	cloud_normals(new pcl::PointCloud<pcl::Normal>),
	tree(new pcl::search::KdTree<PointT> ()),
	inliers_cylinder(new pcl::PointIndices)
{

	// Create randomized structure
	// By sampling a unitary sphere

    	cv::Mat mean_mat(3, 1, CV_32F, cv::Scalar(0));
    	cv::Mat std_dev_mat(3, 1, CV_32F, cv::Scalar(1.0));
	int gaussian_sphere_points_num=10000;
	int p=0;
	std::vector<Eigen::Vector3f> gaussian_sphere_points;
        for(int i=0;i<gaussian_sphere_points_num;++i)
        {
            Eigen::Vector3f random_point;
            cv::Mat aux(1, 1, CV_64F);

            // Generate random patch on the sphere surface
            cv::randn(aux, mean_mat.at<float>(0,0), std_dev_mat.at<float>(0,0));
            random_point(0,0)=aux.at<float>(0,0);

            cv::randn(aux, mean_mat.at<float>(1,0), std_dev_mat.at<float>(1,0));
            random_point(1,0)=aux.at<float>(0,0);

            cv::randn(aux, mean_mat.at<float>(2,0), std_dev_mat.at<float>(2,0));
            random_point(2,0)=aux.at<float>(0,0);

	    gaussian_sphere_points.push_back(random_point);
	}

	cyl_direction_accum.resize(angle_bins);
	for(int i=0;i<angle_bins;++i)
	{
		cyl_direction_accum[i].resize(angle_bins);
	}

	cyl_circ_accum.resize(position_bins);
	for(int i=0;i<position_bins;++i)
	{
		cyl_circ_accum[i].resize(position_bins);
		for(int j=0;j<position_bins;++j)
		{
			cyl_circ_accum[i][j].resize(radius_bins);
		}
	}

	for(unsigned int s=0; s<angle_bins;++s)
	{
		float angle=(float)s*angle_step;
		xy_circle_points.push_back(Eigen::Vector3f(cos(angle),sin(angle),0.0));
	}	
};

pcl::ModelCoefficients::Ptr CylinderSegmentationHough::segment(const PointCloudT::ConstPtr & point_cloud_in_)
{
	//1.  Estimate point normals
	ROS_INFO_STREAM(" 2. Estimate normals");
	ne.setSearchMethod (tree);
	ne.setInputCloud (point_cloud_in_);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);

	//3. for each point normal
	ROS_INFO_STREAM(" 3. Step 1");
			
	ROS_INFO_STREAM("  3.1. Reset accumulator");
	for (unsigned int theta_index=0; theta_index < cyl_direction_accum.size(); ++theta_index) {
		std::fill(cyl_direction_accum[theta_index].begin(),cyl_direction_accum[theta_index].end(), 0);
	}

	ROS_INFO_STREAM("  3.2. Vote");
	for(NormalCloudT::iterator it = cloud_normals->begin(); it != cloud_normals->end(); ++it)
	{
		// 3.2.1 compute b
		Eigen::Vector3f b=(-Eigen::Vector3f::UnitZ()+it->getNormalVector3fMap ());
		b.normalize();

		// 3.2.2 derive matrix R
		Eigen::Matrix3f R=Eigen::Matrix3f::Identity() - 2.0*b*b.transpose();

		//std::cout << R << std::endl;
		for(unsigned int s=0; s<angle_bins;++s)
		{
			// rotate xy points to hough gaussian sphere space (IN THE FUTURE CHANGE TO BLOCK [SPARSE])
			Eigen::Vector3f rotated_direction_hypothesis=R*xy_circle_points[s];

			// Convert to spherical coordinates
			float theta_=atan2(rotated_direction_hypothesis[1],rotated_direction_hypothesis[0])+M_PI;
			float phi_=   acos(rotated_direction_hypothesis[2]);

			// Discretize and vote
			unsigned int theta_bin=floor(theta_/angle_step);
			unsigned int phi_bin=floor(phi_/angle_step);
			
			// std::cout << "theta_bin:"<< theta_bin << " phi_bin:" << phi_bin <<  " theta_:" << theta_<< " phi_:" << phi_ << std::endl;
			if (theta_bin>=angle_bins) theta_bin=angle_bins-1;
			
			// 3.2 Get corresponding parameters and vote (+1)
			++cyl_direction_accum[theta_bin][phi_bin];
		}	
	}

	ROS_INFO_STREAM("  3.3. Get max peak");
	unsigned int best_theta_bin;
	unsigned int best_phi_bin;
	unsigned int most_votes=0;
	for (unsigned theta_index=0; theta_index < cyl_direction_accum.size(); ++theta_index) {
		for (unsigned phi_index=0; phi_index < cyl_direction_accum[theta_index].size(); ++phi_index) {
			if(cyl_direction_accum[theta_index][phi_index]>most_votes)
			{
				best_theta_bin=theta_index;
				best_phi_bin=phi_index;
				most_votes=cyl_direction_accum[theta_index][phi_index];
			}
		}
	}

	ROS_INFO_STREAM("  3.4. Convert back to continuous");
	float best_theta=best_theta_bin*angle_step-M_PI;
	float best_phi=best_phi_bin*angle_step;


	ROS_INFO_STREAM("    best votes="<< most_votes<<" theta="<<best_theta<<" phi="<<best_phi);

	ROS_INFO_STREAM("  3.5. Convert to direction vector");
	Eigen::Vector3f cylinder_direction(cos(best_theta)*sin(best_phi),sin(best_theta)*sin(best_phi),cos(best_phi));
    

	std::cout << "dir_vector:" << cylinder_direction << std::endl;
	ROS_INFO_STREAM(" 4. Step 2");

	//Get rotation matrix
	Eigen::Matrix4f R2;
	R2=Eigen::Matrix4f::Identity();

   	Eigen::Vector3f up = Eigen::Vector3f::UnitZ();
       	//Eigen::Vector3f xaxis = cylinder_direction.cross(up);
       	Eigen::Vector3f rot_axis = up.cross(cylinder_direction);
	rot_axis.normalize();
	if( isnan(rot_axis[0])||isnan(rot_axis[1])||isnan(rot_axis[2]))
	{

		R2=Eigen::Matrix4f::Identity();
	}
	else
	{
	    	
		Eigen::Matrix3f aux;
		aux=Eigen::AngleAxisf(acos(up.dot(cylinder_direction)),rot_axis);
		R2.block(0,0,3,3)=aux;
	}
	/*if(isnan(xaxis[0])||isnan(xaxis[1])||isnan(xaxis[2])||isnan(yaxis[0])||isnan(yaxis[1])||isnan(yaxis[2]))
	{
		R2<<    	     1, 		     0,				0,  0,
		        	     0,  	     	     1, 			0,  0,
		  cylinder_direction[0],  cylinder_direction[1],     cylinder_direction[2], 0,
				     0, 	 	     0, 	                0,  1;

	}
	else
	{
		R2<<    	     xaxis[0], 		     xaxis[1],			xaxis[2], 0,
		        	     yaxis[0],  	     yaxis[1], 			yaxis[2], 0,
		        cylinder_direction[0],  cylinder_direction[1],     cylinder_direction[2], 0,
					   0, 			   0, 			      0,  1;
	}*/
    	std::cout << R2 << std::endl;
  	// HERE FILTER POINTS THAT HAVE NORMAL NOT PERPENDICULAR TO CILINDER DIRECTION (CHECK CROSS PRODUCT)

	// Extract the cylinder inliers from the input cloud
	float thresh_=cos(angle_step);
	pcl::PointIndices::Ptr  inliers_cylinder (new pcl::PointIndices);
	for (unsigned i=0; i < cloud_normals->points.size(); ++i) 
	{
		float dot_product=cloud_normals->points[i].getNormalVector3fMap ().dot(cylinder_direction);

		if(fabs(dot_product)>thresh_)
		{
			inliers_cylinder->indices.push_back(i);
		}
	}

	PointCloudT::Ptr transformed_cloud (new pcl::PointCloud<PointT> ());
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (point_cloud_in_);
	extract.setIndices (inliers_cylinder);
	extract.setNegative (true);
	extract.filter (*transformed_cloud);

	// Executing the transformation

	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, R2);

	// Get position voting boundaries
	Eigen::Vector4f min_pt,max_pt;
	pcl::getMinMax3D(*transformed_cloud,min_pt,max_pt);
		ROS_INFO_STREAM(" min="<< min_pt <<" max="<<max_pt);
	float u_position_step=(max_pt-min_pt)[0]/position_bins;
	float v_position_step=(max_pt-min_pt)[1]/position_bins;

	ROS_INFO_STREAM("  4.1. Reset accumulator");
	for (unsigned int u_index=0; u_index < position_bins; ++u_index) {
		for (unsigned int v_index=0; v_index < position_bins; ++v_index) {
			std::fill(cyl_circ_accum[u_index][v_index].begin(),cyl_circ_accum[u_index][v_index].end(), 0);
		}
	}
	

	ROS_INFO_STREAM("  4.2. Vote");
	for(unsigned int r=0; r<radius_bins;++r)
	{	
		float current_radius=r_step*r+min_radius;

		for(unsigned int w=0; w<angle_bins;++w)
		{
			float current_angle=w*angle_step;
			for(PointCloudT::iterator it = transformed_cloud->begin(); it != transformed_cloud->end(); ++it)
			{
				float u=it->x - min_pt[0];
				float v=it->y - min_pt[1] ;
		
				// Get discretized coordinates
				unsigned int u_hough=floor( (current_radius*cos(current_angle)+u)/u_position_step);
				unsigned int v_hough=floor( (current_radius*sin(current_angle)+v)/v_position_step);
				//std::cout << cos(current_angle) <<" " <<u << " u_houg:" << u_hough << " v_hough:" << v_hough<< std::endl;
				//if(u_hough<0||v_hough<0||u_hough>=position_bins||v_hough>=position_bins)
				if(u_hough>=position_bins)
					continue;//u_hough=position_bins-1;
				if(v_hough>=position_bins)
					continue;//v_hough=position_bins-1;
				//	continue;

				//std::cout << "nunca cheguei aqui" << std::endl;
				++cyl_circ_accum[u_hough][v_hough][r];
			}
		}
	}

	// Get best
	unsigned int best_u_bin=0, best_v_bin=0, best_r_bin=0;
	most_votes=0;
	for (unsigned int u_index=0; u_index < position_bins; ++u_index) {
		for (unsigned int v_index=0; v_index < position_bins; ++v_index) {
			for (unsigned int r_index=0; r_index < radius_bins; ++r_index) {

				if(cyl_circ_accum[u_index][v_index][r_index]>most_votes) {
					best_u_bin=u_index;
					best_v_bin=v_index;
					best_r_bin=r_index;
					most_votes=cyl_circ_accum[u_index][v_index][r_index];
				}
			}
		}
	}

	// Recover
	float best_u=best_u_bin*u_position_step+min_pt[0];
	float best_v=best_v_bin*v_position_step+min_pt[1];
	float best_r=best_r_bin*r_step+min_radius;
	ROS_INFO_STREAM("    best votes="<< most_votes<<" best_u="<< best_u_bin <<" best_v="<<best_v_bin<<" best_r="<<best_r);
	// Get u v in original frame
	Eigen::Vector4f cylinder_position=R2.transpose()*Eigen::Vector4f(best_u,best_v,min_pt[1],1.0);



	pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    	coefficients_cylinder->values.resize(7);
    	coefficients_cylinder->values[0]=cylinder_position[0];
    	coefficients_cylinder->values[1]=cylinder_position[1];
    	coefficients_cylinder->values[2]=cylinder_position[2];
    	coefficients_cylinder->values[3]=cylinder_direction[0];
    	coefficients_cylinder->values[4]=cylinder_direction[1];
    	coefficients_cylinder->values[5]=cylinder_direction[2];
    	coefficients_cylinder->values[6]=best_r;

	// EXTRACT HEIGHT AND MID POINT FOR INLIERS ONLY!!!
	// Create the filtering object
	PointCloudT::Ptr cloud_projected(new PointCloudT);
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType (pcl::SACMODEL_CYLINDER);
	proj.setInputCloud (transformed_cloud);
	proj.setModelCoefficients (coefficients_cylinder);
	proj.filter (*cloud_projected);

	// Get MinMax
	pcl::getMinMax3D(*cloud_projected,min_pt,max_pt);
	float height=max_pt[2]-min_pt[2];
	coefficients_cylinder->values[7]=height;


	// Redefine cylinder position (base);
	Eigen::Vector4f refined_cylinder_position=R2.transpose()*Eigen::Vector4f(best_u,best_v,min_pt[2],1.0);
    	coefficients_cylinder->values[0]=refined_cylinder_position[0];
    	coefficients_cylinder->values[1]=refined_cylinder_position[1];
    	coefficients_cylinder->values[2]=refined_cylinder_position[2];
	
	// VISUALIZE

    	viewer =simpleVis(transformed_cloud,cloud_normals,coefficients_cylinder);

   
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return coefficients_cylinder;

}


