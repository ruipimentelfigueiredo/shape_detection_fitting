//Get rotation matrix
Eigen::Matrix3d directionToRotation(Eigen::Vector3d & direction_vector)
{
	Eigen::Matrix3d R2;
	R2=Eigen::Matrix3d::Identity();

   	Eigen::Vector3d up = Eigen::Vector3d::UnitZ();

	if(up.dot(direction_vector)<0)
	{
		direction_vector=-direction_vector;
	}
       	//Eigen::Vector3f rot_axis = up.cross(cylinder_direction);
	Eigen::Vector3d rot_axis = direction_vector.cross(up);

	rot_axis.normalize();
	if(std::isnan(rot_axis[0])||std::isnan(rot_axis[1])||std::isnan(rot_axis[2]))
	{
		R2=Eigen::Matrix3d::Identity();
	}
	else
	{
		R2=Eigen::AngleAxisd(acos(direction_vector.dot(up)),rot_axis);
	}
}

Eigen::Vector3d rotationToDirection(const Eigen::Matrix3d & rotation)
{

}

