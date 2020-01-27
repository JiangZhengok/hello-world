# 1、在cmakelists中使用find_package链接自定义安装的opencv
set(THIRDLIB "/home/jiang/6_lib/install")
set(OpenCV_DIR  ${THIRDLIB}/opencv3.2.0/share/OpenCV)
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV_FOUND: ${OpenCV_FOUND}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
MESSAGE(STATUS "OpenCV_LIBRARY_DIRS: ${OpenCV_LIBRARY_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_executable( xxx xx.cpp )
target_link_libraries(xxx ${OpenCV_LIBS})

# 1、在cmakelists中使用find_package链接自定义安装的Ceres
set(Ceres_DIR ${THIRDLIB}/ceres/lib/cmake/Ceres) 
find_package(Ceres REQUIRED )

include_directories(${CERES_INCLUDE_DIRS})
link_directories(${CERES_LIBRARY_DIRS})
add_executable( xxx xx.cpp )
target_link_libraries( xxx ${CERES_LIBRARIES})