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

#include <iostream>
#include <fstream>
#include <cstdio>
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

/* ---[ */
int main (int argc, char** argv)
{
	if (argc < 2)
	{
		printHelp (argc, argv);
		return (-1);
	}
	std::string path=std::string(argv[1]);
	std::cout << "path: " << path << std::endl;
	std::vector<std::string> filenames;


	std::string images_path=path+"eval/images/";
	std::string point_clouds_path=path+"eval/point_clouds/";
	std::string annotations_path=path+"eval/annotations/";
	std::cout << images_path << std::endl;
	unsigned int offset=0;
	//images
	readDirectory(images_path,filenames);
	for(unsigned int i=0;i<filenames.size();++i)
	{
  		std::stringstream ss;
		ss << images_path << std::setfill('0') << std::setw(6) << (i+offset) << ".jpg";

		std::string old_name=images_path+filenames[i];
		std::string new_name=ss.str();
		
		int result=std::rename(old_name.c_str(), new_name.c_str());
		if ( result == 0 )
			puts ( "File successfully renamed" );
		else
		{
			perror( "Error renaming image file" );
			exit(-1);
		}
	}

	//point_clouds
	filenames.clear();
	readDirectory(point_clouds_path,filenames);
	for(unsigned int i=0;i<filenames.size();++i)
	{	
  		std::stringstream ss;
		ss << point_clouds_path << std::setfill('0') << std::setw(6) << (i+offset) << ".pcd";

		std::string old_name=point_clouds_path+filenames[i];
		std::string new_name=ss.str();
		std::cout << old_name << " " << new_name  << std::endl;
		int result=std::rename(old_name.c_str(), new_name.c_str());
		if ( result == 0 )
			puts ( "File successfully renamed" );
		else
		{
			perror( "Error renaming point cloud file" );
			exit(-1);
		}
	}

	//annotations
	filenames.clear();
	readDirectory(annotations_path,filenames);
	for(unsigned int i=0;i<filenames.size();++i)
	{	
  		std::stringstream ss;
		ss << annotations_path << std::setfill('0') << std::setw(6) << (i+offset) << ".xml";

		std::string old_name=annotations_path+filenames[i];
		std::string new_name=ss.str();
		
		int result=std::rename(old_name.c_str(), new_name.c_str());
		if ( result == 0 )
			puts ( "File successfully renamed" );
		else
		{
			perror( "Error renaming annotation file" );
			exit(-1);
		}
	}

	return (0);
}

