cmake_minimum_required(VERSION 2.8)
project("MarkersGroundTruth")
find_package(OpenCV REQUIRED)
add_executable("MarkersGroundTruth" MarkersGroundTruth.cpp OutputControl.cpp)
target_link_libraries("MarkersGroundTruth" ${OpenCV_LIBS})

