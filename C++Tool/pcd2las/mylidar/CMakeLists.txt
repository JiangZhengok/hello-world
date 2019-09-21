cmake_minimum_required(VERSION 2.8.3)
project(mylidar)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_msgs sensor_msgs 
)
#find_package(OpenCV REQUIRED)
#find_package(OpenNI REQUIRED)
find_package(catkin REQUIRED COMPONENTS)
find_package(PCL REQUIRED COMPONENTS common io visualization)
find_package(Eigen3 REQUIRED)
find_package(libLAS REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs pcl_msgs sensor_msgs
  DEPENDS EIGEN3 PCL 
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${LIBLAS_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/mylidar_node.cpp)

message(STATUS "FOUND ...${libLAS_LIBRARIES}")

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES}
   ${libLAS_LIBRARIES}
 )

