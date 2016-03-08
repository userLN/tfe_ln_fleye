cmake_minimum_required(VERSION 2.8)
project("markerDetectors")
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/)

find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

add_executable("markerDetectors" markersDetector.cpp)

target_link_libraries("markerDetectors" ${OpenCV_LIBS})
target_link_libraries("markerDetectors" ${aruco_LIBS})