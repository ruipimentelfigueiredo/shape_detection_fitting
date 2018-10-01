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

	return R2;
}

/*Eigen::Vector3d rotationToDirection(const Eigen::Matrix3d & rotation)
{

	return
}*/





visualization_msgs::Marker createMarker(const Eigen::VectorXf & model_params, int model_type, const std::string & frame, int id, const std::string & marker_namespace_)
{
	// Convert direction vector to quaternion
	tf::Vector3 axis_vector(model_params[3], model_params[4], model_params[5]);

	tf::Vector3 up_vector(0.0, 0.0, 1.0);
	tf::Quaternion q;
	if(axis_vector.dot(up_vector)>0.99)
	{
		q=tf::createIdentityQuaternion();
	}
	else
	{
		tf::Vector3 right_vector = axis_vector.cross(up_vector);
		right_vector.normalized();

		q=tf::Quaternion(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
	}

	q.normalize();
	geometry_msgs::Quaternion cylinder_orientation;
	tf::quaternionTFToMsg(q, cylinder_orientation);
	float height=model_params[7];

	visualization_msgs::Marker marker;
	marker.header.frame_id =frame;
	marker.header.stamp = ros::Time();
	marker.ns = marker_namespace_;
	marker.id = id;
	marker.type = model_type;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = model_params[0];
	marker.pose.position.y = model_params[1];
	marker.pose.position.z = model_params[2];
	marker.pose.orientation = cylinder_orientation;
/*	marker.pose.orientation.x = Q.x();
	marker.pose.orientation.y = Q.y();
	marker.pose.orientation.z = Q.z();
	marker.pose.orientation.w = Q.w();*/
	marker.scale.x = 2*model_params[6];
	marker.scale.y = 2*model_params[6];
	marker.scale.z = height;
	marker.color.a = 0.5;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	//marker.lifetime = ros::Duration(0.05);
	return marker;
}
