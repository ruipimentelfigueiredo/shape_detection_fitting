#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

template <class detector_type>
class ShapeDetectionManager
{
	boost::shared_ptr<detector_type> cylinder_fitting;
	boost::shared_ptr<CylinderClassifier> cylinder_classifier;
	float classification_threshold;
 	const Eigen::Matrix4f cam_intrinsic;

	public:
		ShapeDetectionManager(boost::shared_ptr<CylinderClassifier> & cylinder_classifier_, boost::shared_ptr<detector_type> & cylinder_fitting_, const Eigen::Matrix4f & cam_intrinsic_) : 
			cylinder_classifier(cylinder_classifier_), 
			cylinder_fitting(cylinder_fitting_),
 			cam_intrinsic(cam_intrinsic_)
		{};
	
		std::vector<CylinderFitting> detect(cv::Mat & image_cv, std::vector<PointCloudT::Ptr> & pcl_clusters, const float & classification_threshold)
		{
			std::vector<cv::Rect> clusters_bboxes;
			clusters_bboxes.reserve(pcl_clusters.size());
			std::vector<int> cylinder_indices;
			cylinder_indices.reserve(pcl_clusters.size());
			std::vector<PointCloudT::Ptr> clusters_point_clouds;
			clusters_point_clouds.reserve(pcl_clusters.size());
			for(unsigned int i=0; i<pcl_clusters.size();++i)
			{
				PointCloudT::Ptr cloud_filtered (new PointCloudT);

				// Create the filtering object
				pcl::VoxelGrid<PointT> sor;
				sor.setInputCloud(pcl_clusters[i]);
				sor.setLeafSize(0.005f, 0.005f, 0.005f);
				sor.filter(*cloud_filtered);
				//cloud_filtered->header.frame_id="table";
				clusters_point_clouds.push_back(cloud_filtered);

				///////////////////////////
				// Get 2d bounding boxes //
				///////////////////////////

				// Get XY max and min and construct image
				PointCloudT::Ptr cloud_projected(new PointCloudT);
		  		pcl::transformPointCloud (*cloud_filtered, *cloud_projected, cam_intrinsic);

				for (unsigned int p=0; p<cloud_projected->points.size();++p)
				{	
					cloud_projected->points[p].x/=cloud_projected->points[p].z;
					cloud_projected->points[p].y/=cloud_projected->points[p].z;
					cloud_projected->points[p].z=1.0;
				}

				// Get minmax
				Eigen::Vector4f min_pt,max_pt;
				pcl::getMinMax3D(*cloud_projected,min_pt,max_pt);

				float padding =0.1;
				float width=max_pt[0]-min_pt[0];
				float height=max_pt[1]-min_pt[1];
	
				// PAD
				float width_padding=0.5*padding*width;
				float height_padding=0.5*padding*height;

				(min_pt[0]-width_padding)  <0 ? min_pt[0]=0 : min_pt[0]-=width_padding;
				(min_pt[1]-height_padding) <0 ? min_pt[1]=0 : min_pt[1]-=height_padding;

				(max_pt[0]+width_padding)  >(image_cv.cols-1) ? max_pt[0]=(image_cv.cols-1)  : max_pt[0]+=width_padding;
				(max_pt[1]+height_padding) >(image_cv.rows-1) ? max_pt[1]=(image_cv.rows-1) : max_pt[1]+=height_padding;
		
				width=max_pt[0]-min_pt[0];
				height=max_pt[1]-min_pt[1];
				// end pad

				cv::Rect rect(min_pt[0], min_pt[1], width, height);
				clusters_bboxes.push_back(rect);

				// Classify
				cv::Mat roi = image_cv(rect);
				float confidence=cylinder_classifier->classify(roi);
				//ROS_ERROR_STREAM("CONFIDENCE:"<<confidence);
				if(confidence>classification_threshold)
				{
					cylinder_indices.push_back(i);

					// Visualization
					cv::rectangle(image_cv, cv::Point(min_pt[0],min_pt[1]), cv::Point(max_pt[0],max_pt[1]), cv::Scalar(0,255,0), 4);
				}
				else
				{
					// Visualization
					cv::rectangle(image_cv, cv::Point(min_pt[0],min_pt[1]), cv::Point(max_pt[0],max_pt[1]), cv::Scalar(0,0,255), 4);
				}
				

				// dataset creation
				/*cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
				cv::imshow( "Display window", roi );                   // Show our image inside it.
				cv::waitKey(0);*/ 

				//if((image_number%5)==0)
				//imwrite("/home/rui/sphere/sphere.scene."+std::to_string(scene_number)+".cluster."+std::to_string(i)+".jpg", image_cv(rect) );

			}

			ROS_ERROR_STREAM("CHEGUEI AQUI");
			std::vector<CylinderFitting> cylinders;
			for(unsigned int ind=0; ind<cylinder_indices.size();++ind)
			{
				cylinders.push_back(cylinder_fitting->segment(clusters_point_clouds[cylinder_indices[ind]]));
			}
			ROS_ERROR_STREAM("CHEGUEI AQUI2");
			return cylinders;
		}
};