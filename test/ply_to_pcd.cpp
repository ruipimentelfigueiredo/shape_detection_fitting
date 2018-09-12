/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include "helpers.h"
#include <string>
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s [-format 0|1] input.ply output.pcd\n", argv[0]);
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  pcl::PLYReader reader;
  tt.tic ();
  if (reader.read (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool format)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  
  pcl::PCDWriter writer;
  writer.write (filename, cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), format);
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
}
void replaceExt(std::string& s, const std::string& newExt) {

   std::string::size_type i = s.rfind('.', s.length());

   if (i != std::string::npos) {
      s.replace(i+1, newExt.length(), newExt);
   }
}
/* ---[ */
int
main (int argc, char** argv)
{
	print_info ("Convert a PLY file to PCD format. For more information, use: %s -h\n", argv[0]);

	if (argc < 2)
	{
	printHelp (argc, argv);
	return (-1);
	}
	std::string dataset_path=std::string(argv[1]);
	std::cout << "dataset_path: " << dataset_path << std::endl;
	std::vector<std::string> point_cloud_files;
	bool format = 1;
	// Parse the command line arguments for .pcd and .ply files
	//std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
	//std::vector<int> ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");
	/*if (pcd_file_indices.size () != 1 || ply_file_indices.size () != 1)
	{
	print_error ("Need one input PLY file and one output PCD file.\n");
	return (-1);
	}

	// Command line parsing

	parse_argument (argc, argv, "-format", format);
	print_info ("PCD output format: "); print_value ("%s\n", (format ? "binary" : "ascii"));*/

	readDirectory(dataset_path,point_cloud_files);
	// Load the first file
	std::vector<std::string> wrong_pcds;
	for(unsigned int i=0;point_cloud_files.size();++i)
	{
		pcl::PCLPointCloud2 cloud;
		std::string absolute_path_ply=dataset_path+point_cloud_files[i];
		if (!loadCloud (dataset_path+point_cloud_files[i], cloud)) 
		{
			wrong_pcds.push_back(point_cloud_files[i]);
			//continue;
		}
		replaceExt(absolute_path_ply,"pcd");
		// Convert to PLY and save
		saveCloud (absolute_path_ply, cloud, format);
		std::cout << "converted " << i+1 << " of " << point_cloud_files.size() << std::endl;
	}

  return (0);
}

