cmake_minimum_required(VERSION 2.8.3)
project(shape_detection_fitting)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -Wno-comment -std=c++11 -faligned-new")
find_package(catkin REQUIRED COMPONENTS
  #find_object_2d
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
find_package(TinyXML REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Caffe REQUIRED)
# bfl (Bayesian Filtering Library) is a third party package that uses pkg-config
#find_package(PkgConfig)

#pkg_check_modules(BFL REQUIRED orocos-bfl)
#link_directories(${BFL_LIBRARY_DIRS})

#find_package(OpenCV 3.4 REQUIRED)
find_package(Boost COMPONENTS system filesystem regex REQUIRED)

## Generate messages in the 'msg' folder
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
#  INCLUDE_DIRS include
#  LIBRARIES shape_detection_fitting
#  CATKIN_DEPENDS find_object_2d roscpp
#  DEPENDS system_lib
    DEPENDS
      TinyXML
)


###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -std=c++11 -Wno-comment -Wno-deprecated-declarations -Wno-sign-compare -faligned-new")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}")

## Declare a C++ library
# add_library(shape_detection_fitting
#   src/${PROJECT_NAME}/shape_detection_fitting.cpp
# )

set(SHAPE_DETECTION_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/shape-classification)
add_subdirectory(${SHAPE_DETECTION_DIR})

set(SHAPE_FITTING_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/shape-fitting)
add_subdirectory(${SHAPE_FITTING_DIR})

#set(SHAPE_TRACKING_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/shape-tracking)
#add_subdirectory(${SHAPE_TRACKING_DIR})


include_directories(
  include
  ${SHAPE_DETECTION_DIR}/include
  ${SHAPE_FITTING_DIR}/include
  ${SHAPE_FITTING_DIR}/test
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Caffe_INCLUDE_DIRS}
  #${BFL_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ROS WRAPPERS
#add_library (cylinder_tracking_ros src/cylinder_tracking_ros.cpp)
#target_link_libraries (cylinder_tracking_ros ${catkin_LIBRARIES} HybridEstimator)
#add_dependencies(cylinder_tracking_ros ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# ROS NODES
add_executable (shape_detection_fitting_ros_node src/shape_detection_fitting_ros_node.cpp)
target_link_libraries (shape_detection_fitting_ros_node ${catkin_LIBRARIES} ${Caffe_LIBS} -lglog fitting_library shape_classifier)
add_dependencies(shape_detection_fitting_ros_node shape_detection_fitting_ros_node_generate_messages_cpp)

#add_executable (tracking_ros_node src/particle_filter_node.cpp)
#target_link_libraries (tracking_ros_node ${catkin_LIBRARIES} ${BFL_LIBRARIES} particle_filter)

#ROS NODELETS
# Nodelet library
add_library(shape_detection_fitting_ros_nodelet src/nodelets/shape_detection_fitting_ros_nodelet.cpp)
target_link_libraries (shape_detection_fitting_ros_nodelet ${catkin_LIBRARIES} ${Caffe_LIBS} -lglog fitting_library shape_classifier)
add_dependencies(shape_detection_fitting_ros_nodelet shape_detection_fitting_ros_nodelet_gencpp)

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

add_executable(tabletop_test test/shapes_2018.cpp)
target_link_libraries(tabletop_test ${catkin_LIBRARIES} ${TinyXML_LIBRARIES} ${Caffe_LIBS} -lglog fitting_library shape_classifier)

#add_executable (cylinder_segmentation_test_clutter src/cylinder_segmentation_test_clutter.cpp)
#target_link_libraries (cylinder_segmentation_test_clutter ${PCL_LIBRARIES} ${catkin_LIBRARIES} cylinder_segmentation_hough)

#add_executable (cylinder_segmentation_test_clutter2 src/cylinder_segmentation_test_clutter2.cpp)
#target_link_libraries (cylinder_segmentation_test_clutter2 ${PCL_LIBRARIES} ${catkin_LIBRARIES} cylinder_segmentation_hough)


#add_executable (hough_lines_cuda src/hough_lines_cuda.cpp)
#target_link_libraries (hough_lines_cuda -lopencv_cudaimgproc ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable (pcd_visualizer src/pcd_visualizer.cpp)
#target_link_libraries (pcd_visualizer ${PCL_LIBRARIES} ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(shape_detection_fitting ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(shape_detection_fitting_node src/shape_detection_fitting_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(shape_detection_fitting_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(shape_detection_fitting_node
#   ${catkin_LIBRARIES}
# )


