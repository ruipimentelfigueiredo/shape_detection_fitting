#ifndef DETECTIONDATA_H
#define DETECTIONDATA_H
class DetectionData
{
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
	{};

	void draw_bounding_boxes(cv::Mat & image)
	{
		for(size_t i=0; i<bounding_boxes.size();++i)
		{
			cv::rectangle(image, bounding_boxes[i], cv::Scalar(255,0,0), 4);
		}
	}


	void visualize(cv::Mat & image)
	{
		draw_bounding_boxes(image);
		cv::namedWindow("Display window", cv::WINDOW_NORMAL);   	   // Create a window for display.
		cv::imshow("Display window", image );                  	 	   // Show our image inside it.
		cv::resizeWindow("Display window", 0.5*image.cols,0.5*image.rows);
		cv::waitKey(1000);  
	}
};

#endif // DETECTIONDATA_H
