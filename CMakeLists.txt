cmake_minimum_required(VERSION 3.0)

project(asn1_conversions VERSION 0.1)

include_directories( include )

add_library(asn1_conversions src/asn1_conversions.cpp)
add_library(asn1_eigen_conversions src/asn1_eigen_conversions.cpp)
add_library(asn1_pcl_conversions src/asn1_pcl_conversions.cpp)
add_library(asn1_ros_conversions src/asn1_ros_conversions.cpp)
