# 1、在cmakelists中使用find_package链接自定义安装的opencv
set(THIRDLIB "/home/jiang/6_lib/install")
set(OpenCV_DIR  ${THIRDLIB}/opencv3.2.0/share/OpenCV)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_FOUND: ${OpenCV_FOUND}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
MESSAGE(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")