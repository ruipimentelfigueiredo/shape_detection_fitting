#ifndef ANNOTATIONS_H
#define ANNOTATIONS_H

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include <fstream>
#include <tinyxml.h>
#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include "fitting_data.h"
namespace annotations {
    void storeTrainingData(
        const std::string & images_path, 
        const std::string & pcl_path, 
        const std::string & pcl_clusters_path, 
        const std::string & images_clusters_path,
        const std::string & annotations_path,
        const int & iteration, 
        const cv::Mat & image_cv,
        const PointCloudRGB::Ptr & point_cloud, 
        std::vector<cv::Rect> & bounding_boxes, 
        std::vector<PointCloudT::Ptr> & pcl_clusters, 
        std::string & type) // All assumed the same
    {
        std::string ext_pcl = ".pcd";
        std::string ext_img = ".png";
        std::string ext_data = ".xml";

        std::stringstream ss;
        ss << std::fixed << std::setfill('0') << std::setw(5) << iteration;
        std::string iteration_str=ss.str();
        std::string image_name=iteration_str+ext_img;
        std::string pcl_name=iteration_str+ext_pcl;
        std::string data_name=iteration_str+ext_data;

        // Store whole image
        cv::imwrite(images_path+image_name, image_cv); //+std::string("scene.")

        // Store whole PCL
        pcl::PCDWriter pcd_writer;

        pcd_writer.writeBinary(pcl_path+pcl_name,*point_cloud);

        // Store individual clusters PCL
        for(int i=0; i<pcl_clusters.size(); ++i)
        {
            std::string cluster_name=std::to_string(i)+ext_pcl;
            pcd_writer.writeBinary(pcl_clusters_path+iteration_str+std::string(".cluster.")+cluster_name,*pcl_clusters[i]);
        }

        // Store individual image BB crops
        for(int i=0; i<bounding_boxes.size();++i)
        {
            std::string cluster_name=std::to_string(i)+ext_img;
            cv::imwrite(images_clusters_path+iteration_str+std::string(".cluster.")+cluster_name, image_cv(bounding_boxes[i])); //+std::string("scene.")
        } 

        // Store RGB annotations info
        std::ofstream out(annotations_path+data_name);

        out << "<annotation>\n"
            << "  <folder>images</folder>\n"
            << "  <filename>" << image_name << "</filename>\n"
            << "  <source>\n"
            << "    <image>" << image_name <<"</image>\n"
            << "  </source>\n"
            << "  <size>\n"
            << "    <width>"  << image_cv.cols  << "</width>\n"
            << "    <height>" << image_cv.rows << "</height>\n"
            << "    <depth>"  << image_cv.channels()  << "</depth>\n"
            << "  </size>\n"
            << "  <segmented>1</segmented>\n";

        for (int i = 0; i < bounding_boxes.size(); i++)
        {
            out << "  <object>\n"
                << "    <name>" << type << "</name>\n"
                << "    <bndbox>\n"
                << "      <xmin>"<< bounding_boxes[i].x <<"</xmin>\n"
                << "      <ymin>"<< bounding_boxes[i].y <<"</ymin>\n"
                << "      <xmax>"<< bounding_boxes[i].x+bounding_boxes[i].width <<"</xmax>\n"
                << "      <ymax>"<< bounding_boxes[i].y+bounding_boxes[i].height <<"</ymax>\n"
                << "    </bndbox>\n"
                << "  </object>\n";
        }

        out << "</annotation>";
        out.close();
    }

    void storeTestData(
        const std::string & images_clusters_path,
        const std::string & annotations_path,
        const std::string & annotations_file,
        const int iteration, 
        DetectionData & detections,
        bool & with_classifier
        ) // All assumed the same
    {
        std::ofstream out(annotations_path+annotations_file);

        out << "<annotation>\n"
            << "  <folder>images</folder>\n"
            << "  <size>\n"
            //<< "    <width>"  << image_cv.cols  << "</width>\n"
            //<< "    <height>" << image_cv.rows << "</height>\n"
            //<< "    <depth>"  << image_cv.channels()  << "</depth>\n"
            << "  </size>\n"
            << "  <segmented>1</segmented>\n"
            << "  <with_classifier>" << with_classifier << "</with_classifier>\n";

        for (unsigned int i = 0; i < detections.bounding_boxes.size(); ++i)
        {
            std::string object_type;
            if(with_classifier)
            {
                if(detections.clusters_classification_data[i].id==0)
                    object_type="cylinder";
                else if (detections.clusters_classification_data[i].id==3)
                    object_type="other";
            }
            else
            {
                if(detections.clusters_fitting_data[i].type==FittingData::CYLINDER)
                    object_type="cylinder";
                else if (detections.clusters_fitting_data[i].type==FittingData::OTHER)
                    object_type="other";
            }

            out << "  <object>\n"
                << "    <name>" << object_type << "</name>\n"
                << "    <bndbox>\n"
                << "      <xmin>"<< detections.bounding_boxes[i].x <<"</xmin>\n"
                << "      <ymin>"<< detections.bounding_boxes[i].y <<"</ymin>\n"
                << "      <xmax>"<< detections.bounding_boxes[i].x+detections.bounding_boxes[i].width <<"</xmax>\n"
                << "      <ymax>"<< detections.bounding_boxes[i].y+detections.bounding_boxes[i].height <<"</ymax>\n"
                << "    </bndbox>\n"
                << "  </object>\n";
        }

        out << "</annotation>";
        out.close();
    }
}
#endif //ANNOTATIONS_H