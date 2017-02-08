cmake_minimum_required(VERSION 2.8.3)
project(active_semantic_mapping)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  find_object_2d
  roscpp
  message_filters
  visualization_msgs
  tf_conversions
)

find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Clusters.msg
   Cylinders.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
  visualization_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES active_semantic_mapping
#  CATKIN_DEPENDS find_object_2d roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)




## Declare a C++ library
# add_library(active_semantic_mapping
#   src/${PROJECT_NAME}/active_semantic_mapping.cpp
# )

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED
NO_MODULE # should be optional, tells CMake to use config mode
PATHS /usr/local # look here
NO_DEFAULT_PATH) # and don't look anywhere else

set(TRACKER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/tracking/cpp)
include_directories(${TRACKER_DIR}/HybridEstimator)
#include(${TRACKER_DIR}/HybridEstimator/CMakeLists.txt)
## foveated stereo lib
add_subdirectory(${TRACKER_DIR}/HybridEstimator)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# CYLINDER LIBS
add_library (cylinder_segmentation_hough src/cylinder_segmentation.cpp src/cylinder_segmentation_hough.cpp)
target_link_libraries (cylinder_segmentation_hough ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_library (cylinder_segmentation_ransac src/cylinder_segmentation.cpp src/cylinder_segmentation_ransac.cpp)
target_link_libraries (cylinder_segmentation_ransac ${PCL_LIBRARIES} ${catkin_LIBRARIES})


# ROS WRAPPERS
add_library (cylinder_tracking_ros src/cylinder_tracking_ros.cpp)
target_link_libraries (cylinder_tracking_ros ${catkin_LIBRARIES} HybridEstimator)

# ROS NODES
add_executable (cylinder_segmentation_ros_node src/cylinder_segmentation_ros_node.cpp)
target_link_libraries (cylinder_segmentation_ros_node ${PCL_LIBRARIES} ${catkin_LIBRARIES} cylinder_segmentation_ransac cylinder_segmentation_hough)

#add_executable (cylinder_tracking_ros_node src/cylinder_tracking_ros_node.cpp)
#target_link_libraries (cylinder_tracking_ros_node ${PCL_LIBRARIES} ${catkin_LIBRARIES} cylinder_tracking_ros)


# TESTS
add_executable (cylinder_segmentation_test src/cylinder_segmentation_test.cpp)
target_link_libraries (cylinder_segmentation_test ${PCL_LIBRARIES} ${catkin_LIBRARIES} cylinder_segmentation_hough)

add_executable (cylinder_segmentation_test2 src/cylinder_segmentation_test2.cpp)
target_link_libraries (cylinder_segmentation_test2 ${PCL_LIBRARIES} ${catkin_LIBRARIES} cylinder_segmentation_hough)



#add_executable (hough_lines_cuda src/hough_lines_cuda.cpp)
#target_link_libraries (hough_lines_cuda -lopencv_cudaimgproc ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable (pcd_visualizer src/pcd_visualizer.cpp)
#target_link_libraries (pcd_visualizer ${PCL_LIBRARIES} ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(active_semantic_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(active_semantic_mapping_node src/active_semantic_mapping_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(active_semantic_mapping_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(active_semantic_mapping_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS active_semantic_mapping active_semantic_mapping_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_active_semantic_mapping.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
