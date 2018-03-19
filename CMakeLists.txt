cmake_minimum_required(VERSION 3.0)

project(asn1_conversions VERSION 0.1)

include_directories( include )

add_library(asn1_conversions src/asn1_conversions.cpp)
target_link_libraries(asn1_conversions cdff_types)

add_library(asn1_eigen_conversions src/asn1_eigen_conversions.cpp)
target_link_libraries(asn1_eigen_conversions cdff_types)

add_library(asn1_pcl_conversions src/asn1_pcl_conversions.cpp)
target_link_libraries(asn1_pcl_conversions cdff_types)

add_library(asn1_ros_conversions src/asn1_ros_conversions.cpp)
target_link_libraries(asn1_ros_conversions cdff_types)
