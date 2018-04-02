#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <shape_detection.h>

class ObjectDetectionRos {
	image_transport::ImageTransport it;
	image_transport::Subscriber sub;
	ros::NodeHandle nh;
	std::shared_ptr<ObjectDetection> object_detector;
	public:
		void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			cv::Mat frame=cv_bridge::toCvShare(msg, "bgr8")->image;
    			float scale = 1.0;
    			cv::Scalar mean;// = parser.get<Scalar>("mean");
			object_detector->detect(frame, mean, scale);
		}


		ObjectDetectionRos(const ros::NodeHandle & nh_, std::string & model, int & backend, int & target, bool & swapRB, int & inpWidth, int & inpHeight) : it(nh_)
		{
		  object_detector=std::shared_ptr<ObjectDetection>(new ObjectDetection(model, backend, target, swapRB, inpWidth, inpHeight));
		  image_transport::ImageTransport it(nh);
		  sub = it.subscribe("camera/image", 1, &ObjectDetectionRos::imageCallback,this);
		};
};

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  //cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
  ros::spin();
  //cv::destroyWindow("view");
}
