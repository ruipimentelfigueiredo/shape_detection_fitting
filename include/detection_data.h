#ifndef DETECTIONDATA_H
#define DETECTIONDATA_H

class Color
{
	public:
		Color(double r_=0.0,double g_=0.0, double b_=0.0, double a_=1.0) : r(r_),g(g_),b(b_),a(a_)
		{}
		double r,g,b,a;
};


class DetectionData
{
	std::map<int, cv::Scalar> id_colors_map_bb;
	public:
	FittingData plane_fitting_data;
	std::vector<FittingData> clusters_fitting_data;
	std::vector<cv::Rect> bounding_boxes;
	std::vector<ClassificationData> clusters_classification_data;

	DetectionData(FittingData & plane_fitting_data_, 
		      std::vector<FittingData> & clusters_fitting_data_,
	              std::vector<cv::Rect> & bounding_boxes_,
		      std::vector<ClassificationData> & clusters_classification_data_) : 
		plane_fitting_data(plane_fitting_data_), 
		clusters_fitting_data(clusters_fitting_data_), 
		bounding_boxes(bounding_boxes_),
		clusters_classification_data(clusters_classification_data_)
	{
		id_colors_map_bb.insert(std::pair<int,cv::Scalar>(0,cv::Scalar(0,  255, 0) ) );
		id_colors_map_bb.insert(std::pair<int,cv::Scalar>(1,cv::Scalar(0,  0, 255) ) );
	};

	void draw_bounding_boxes(cv::Mat & image, std::vector<cv::Scalar> colors_=std::vector<cv::Scalar>(), std::vector<std::string> classes_=std::vector<std::string>())
	{
		for(size_t i=0; i<bounding_boxes.size();++i)
		{
			if(colors_.size()>0)
				cv::rectangle(image, bounding_boxes[i], colors_[i], 4);	
			else
				cv::rectangle(image, bounding_boxes[i], cv::Scalar(255,0,0), 4);

			if(classes_.size()>0)
				cv::putText(image, classes_[i], cv::Point2f(bounding_boxes[i].x,bounding_boxes[i].y), cv::FONT_HERSHEY_PLAIN, 4,  colors_[i]);	
		}
	}


	void visualize(cv::Mat & image)
	{
		std::vector<cv::Scalar> colors_cv;
		std::vector<std::string> classes;
		for(unsigned int ind=0; ind<clusters_fitting_data.size(); ++ind)
		{	
			// Cylinder shape
			if(clusters_fitting_data[ind].type==FittingData::CYLINDER)
			{
				classes.push_back("cylinder");
				cv::Scalar color_cv=id_colors_map_bb.find(0)->second;
				colors_cv.push_back(color_cv);
			}
			else
			{
				classes.push_back("other");
				cv::Scalar color_cv=id_colors_map_bb.find(1)->second;
				colors_cv.push_back(color_cv);
			}
		}		

		draw_bounding_boxes(image,colors_cv,classes);
		cv::namedWindow("Display window", cv::WINDOW_NORMAL);   	   // Create a window for display.
		cv::imshow("Display window", image);                  	 	   // Show our image inside it.
		cv::resizeWindow("Display window", 0.5*image.cols,0.5*image.rows);
		cv::waitKey(100);  
	}
};

#endif // DETECTIONDATA_H
