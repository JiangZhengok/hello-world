cmake_minimum_required(VERSION 2.8.3)
project(tflidar2pcd)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Debug")

find_package(PCL REQUIRED COMPONENTS common io visualization)

include_directories(
    include
	"/usr/include/eigen3"
    ${PCL_INCLUDE_DIRS}
    #${LIBLAS_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME} tflidar2pcd.cpp)

#message(STATUS "FOUND PCL_LIBRARIES ...${PCL_LIBRARIES}")
#message(STATUS "FOUND PCL_IO_LIBRARIES ...${PCL_IO_LIBRARIES}")
#message(STATUS "FOUND PCL_COMMON_LIBRARIES ...${PCL_COMMON_LIBRARIES}")
#message(STATUS "FOUND LIBLAS_INCLUDE_DIR ...${libLAS_LIBRARIES}")

target_link_libraries(${PROJECT_NAME}
   ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES}
   #${libLAS_LIBRARIES}
 )

