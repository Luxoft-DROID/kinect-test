cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3")
project(MY_GRAND_PROJECT)
find_package(PCL 1.7 REQUIRED COMPONENTS io common visualization segmentation)
include_directories(${PCL_INCLUDE_DIRS} /usr/local/include/libfreenect /usr/include/libusb-1.0 /usr/include/pcl-1.7)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcd_write_test main.cpp KinectDevice.cpp KinectDevice.h PCLDetector.cpp PCLDetector.h Application.h Application.cpp)
target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} -lfreenect -lpcl_visualization)