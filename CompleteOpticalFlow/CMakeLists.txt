cmake_minimum_required(VERSION 2.8)
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/)

project("peopleTracking")

find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

add_executable("peopleTracking" 
main.cpp 
markersDetector.cpp 
trackingFilter.cpp
outputControl.cpp 
opticalFlow.cpp
ticToc.cpp)

target_link_libraries("peopleTracking" ${OpenCV_LIBS})
target_link_libraries("peopleTracking" ${aruco_LIBS})
