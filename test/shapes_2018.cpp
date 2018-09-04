/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/
#include "shape_fitting_ros.h"
#include <boost/filesystem.hpp>
#include <ctime>
#include <random>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tinyxml.h>

struct path_leaf_string
{
    std::string operator()(const boost::filesystem::directory_entry& entry) const
    {
        return entry.path().leaf().string();
    }
};
 
void readDirectory(const std::string& name, std::vector<std::string>& v)
{
    boost::filesystem::path p(name);
    boost::filesystem::directory_iterator start(p);
    boost::filesystem::directory_iterator end;
    std::transform(start, end, std::back_inserter(v), path_leaf_string());
    std::sort(v.begin(), v.end());
}


TiXmlElement * getElementByName(TiXmlDocument & doc, std::string const & elemt_value) {

   TiXmlElement * elem = doc.RootElement(); //Tree root
   while (elem) {
      if (!std::string(elem -> Value()).compare(elemt_value)) return (elem);
      /*elem = elem->NextSiblingElement();*/
      if (elem -> FirstChildElement()) {
         elem = elem -> FirstChildElement();
      } else if (elem -> NextSiblingElement()) {
         elem = elem -> NextSiblingElement();
      } else {
         while (!elem -> Parent() -> NextSiblingElement()) {
            if (elem -> Parent() -> ToElement() == doc.RootElement()) {
               return NULL;
            }
            elem = elem -> Parent() -> NextSiblingElement();
         }
      }
   }
   return (NULL);
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "shape_detection_fitting");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/

	ros::NodeHandle n;
	ros::NodeHandle n_priv("~");
	ros::Rate loop_rate(300);

	ROS_INFO("Getting cameras' parameterssss");

	//set the cameras intrinsic parameters
	Eigen::Matrix4f cam_intrinsic = Eigen::Matrix4f::Identity();

	std::string dataset_path;
	n_priv.param<string>("dataset_path", dataset_path, "");
	std::vector<std::string> image_files,annotations_files;
	std::cout << dataset_path << std::endl;

	std::string images_dataset_path=dataset_path+"/train/images/";
	std::string annotations_dataset_path=dataset_path+"/train/annotations/";
	readDirectory(images_dataset_path,image_files);
	readDirectory(annotations_dataset_path,annotations_files);
	boost::shared_ptr<TiXmlDocument> doc(new TiXmlDocument());

	// For each dataset sample
	for(unsigned int i=0;image_files.size();++i)
	{
		std::cout << image_files[i] << std::endl;
    		//cv::Mat image = cv::imread(images_dataset_path+image_files[i], CV_LOAD_IMAGE_COLOR);   // Read the file

		std::cout << annotations_files[i] << std::endl;

		try 
		{
			if(doc->LoadFile(annotations_dataset_path+annotations_files[i]))
			{
				//doc->Print();
	 			//TiXmlElement* xml_frame = doc->FirstChild( "annotation" )->FirstChild( "object" )->FirstChild( "name" )->ToElement();

	 			TiXmlNode* xml_object_node = doc->FirstChild( "annotation" )->FirstChild( "object" );
	 			if (xml_object_node) {
	 				//TiXmlElement* xml_object = xml_object_node->ToElement();
	 				for( xml_object_node; xml_object_node; xml_object_node=xml_object_node->NextSibling() )
					{
	 					const char* object_category=xml_object_node->FirstChildElement()->GetText( );
						
	 					const char* object_test=xml_object_node->FirstChildElement()->NextSibling()->ToElement()->GetText( );
						if(object_test)
							std::cout << " " << object_test << std::endl;
						
					}
				}


				//std::cout << doc->RootElement()->Attribute("annotation") << std::endl;
			}
			else
			{
				std::cout << "failed to load file " << annotations_dataset_path << annotations_files[i] << std::endl;
			}
		}
  		catch (std::bad_alloc& ba) {
			continue;
		}
		//std::cout << doc->RootElement()->Attribute("object") <<std::endl;
		//std::string str = doc->RootElement()->Attribute("object");
		//const clock_t begin_time = clock();
		//std::vector<FittingData> detections=shape_detection_manager->detect(image_cv, pcl_clusters);
		//ROS_INFO_STREAM("Cylinders fitting time: "<<float( clock () - begin_time ) /  CLOCKS_PER_SEC<< " seconds");

	}
	std::cout << "end" << std::endl;
	/*cam_intrinsic(0,0) = (float)camera_info->K.at(0);
	cam_intrinsic(0,2) = (float)camera_info->K.at(2);
	cam_intrinsic(1,1) = (float)camera_info->K.at(4);
	cam_intrinsic(1,2) = (float)camera_info->K.at(5);
	cam_intrinsic(3,3) = 0.0;*/
 	return 0;
}
