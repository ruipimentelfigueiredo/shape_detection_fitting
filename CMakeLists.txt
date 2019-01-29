cmake_minimum_required(VERSION 2.8.3)
project(shape_detection_fitting)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -std=c++11 -Wno-comment -Wno-maybe-uninitialized -Wno-deprecated-declarations -Wno-sign-compare -Wunused-function")  #-faligned-new
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")
set(ROS TRUE)

find_package(TinyXML REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV)

set(SHAPE_DETECTION_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/shape-classification)
add_subdirectory(${SHAPE_DETECTION_DIR})

set(SHAPE_FITTING_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/shape-fitting)
add_subdirectory(${SHAPE_FITTING_DIR})

include_directories(
  include
  ${SHAPE_DETECTION_DIR}/include
  ${SHAPE_FITTING_DIR}/include
  ${SHAPE_FITTING_DIR}/test
  ${PCL_INCLUDE_DIRS}
  ${Caffe_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)


if(${ROS})
	find_package(catkin REQUIRED COMPONENTS
	  cmake_modules
	  roscpp
	  message_filters
	  visualization_msgs
	  tf_conversions
	  rosbag
	  pcl_ros
	  nav_msgs
	  std_msgs
	  cv_bridge
	  image_transport
	  message_generation
	)

	#find_package(OpenCV 3.4 REQUIRED)
	find_package(Boost COMPONENTS system filesystem regex REQUIRED)

	add_message_files(
	   FILES
	   BoundingBox.msg
	   BoundingBoxArray.msg
	   TestParameters.msg
	   Clusters.msg
	   Shape.msg
	   Shapes.msg
	)

	generate_messages(
	   DEPENDENCIES
	   std_msgs  # Or other packages containing msgs
	   visualization_msgs

	)
	catkin_package(
	    DEPENDS
	      #TinyXML
	)

	include_directories(
		${catkin_INCLUDE_DIRS}
	)

	# ROS NODES
	add_executable(shape_detection_fitting_ros_node src/shape_detection_fitting_ros_node.cpp)
	target_link_libraries(shape_detection_fitting_ros_node ${catkin_LIBRARIES} ${Caffe_LIBS} ${TinyXML_LIBRARY} ${YAML_CPP_LIBRARIES} ${OpenCV_LIBS} -lglog fitting_library shape_classifier)
	add_dependencies(shape_detection_fitting_ros_node shape_detection_fitting_ros_node_generate_messages_cpp)

	#ROS NODELETS
	#add_library(shape_detection_fitting_ros_nodelet src/nodelets/shape_detection_fitting_ros_nodelet.cpp)
	#target_link_libraries(shape_detection_fitting_ros_nodelet ${catkin_LIBRARIES} ${Caffe_LIBS} -lglog fitting_library shape_classifier)
	#add_dependencies(shape_detection_fitting_ros_nodelet shape_detection_fitting_ros_nodelet_gencpp)

	add_executable(planar_top_test test/planar_top_test.cpp)
	target_link_libraries(planar_top_test ${OpenCV_LIBS} ${TinyXML_LIBRARY} ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES} fitting_library shape_classifier)
endif(${ROS})


#set(SHAPE_TRACKING_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/shape-tracking)
#add_subdirectory(${SHAPE_TRACKING_DIR})

# ROS WRAPPERS
#add_library (cylinder_tracking_ros src/cylinder_tracking_ros.cpp)
#target_link_libraries (cylinder_tracking_ros ${catkin_LIBRARIES} HybridEstimator)
#add_dependencies(cylinder_tracking_ros ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

#add_executable (tracking_ros_node src/particle_filter_node.cpp)
#target_link_libraries (tracking_ros_node ${catkin_LIBRARIES} ${BFL_LIBRARIES} particle_filter)

#install(TARGETS cylinder_segmentation_ros_nodelet
#        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#add_library(cylinder_tracking_ros_nodelet src/nodelets/cylinder_tracking_ros_nodelet.cpp)
#target_link_libraries (cylinder_tracking_ros_nodelet ${PCL_LIBRARIES} ${catkin_LIBRARIES}  cylinder_tracking_ros)

#add_dependencies(cylinder_tracking_ros_nodelet cylinder_segmentation_ros_gencpp)

#install(TARGETS cylinder_tracking_ros_nodelet
#        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

# TESTS
add_executable(ply_to_pcd test/ply_to_pcd.cpp)
target_link_libraries(ply_to_pcd ${PCL_LIBRARIES})	

add_executable(rename test/rename.cpp)
target_link_libraries(rename ${PCL_LIBRARIES})



