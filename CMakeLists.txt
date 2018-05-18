cmake_minimum_required(VERSION 3.0)

project(asn1_conversions VERSION 0.1)

include_directories( include )

SET(DEPENDENCIES "cdff_types")
SET(HEADERS_DIR "include")

SET(LIBRARY_NAME "asn1_conversions")

SET(DEPENDENCIES "cdff_types")
add_library(asn1_conversions src/asn1_conversions.cpp)
target_link_libraries(asn1_conversions cdff_types)
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 
# Install script
set(HEADERS include/conversions/asn1_conversions.hpp)
set(HEADERS_INSTALL_DIR include/conversions)
set(LIB_INSTALL_DIR lib)
include(${CMAKE_SOURCE_DIR}/CMakeModules/CDFF_at_LAAS-install.cmake)

SET(DEPENDENCIES "cdff_types")
SET(LIBRARY_NAME "asn1_eigen_conversions")
add_library(asn1_eigen_conversions src/asn1_eigen_conversions.cpp)
target_link_libraries(asn1_eigen_conversions cdff_types)
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 
# Install script
set(HEADERS include/conversions/asn1_eigen_conversions.hpp)
include(${CMAKE_SOURCE_DIR}/CMakeModules/CDFF_at_LAAS-install.cmake)

SET(DEPENDENCIES "cdff_types asn1_conversions")
SET(LIBRARY_NAME "asn1_pcl_conversions")
add_library(asn1_pcl_conversions src/asn1_pcl_conversions.cpp)
target_link_libraries(asn1_pcl_conversions asn1_conversions cdff_types)
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 
# Install script
set(HEADERS include/conversions/asn1_pcl_conversions.hpp)
include(${CMAKE_SOURCE_DIR}/CMakeModules/CDFF_at_LAAS-install.cmake)

SET(DEPENDENCIES "cdff_types")
SET(LIBRARY_NAME "asn1_ros_conversions")
add_library(asn1_ros_conversions src/asn1_ros_conversions.cpp)
target_link_libraries(asn1_ros_conversions cdff_types)
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 
# Install script
set(HEADERS include/conversions/asn1_ros_conversions.hpp)
include(${CMAKE_SOURCE_DIR}/CMakeModules/CDFF_at_LAAS-install.cmake)

SET(DEPENDENCIES "cdff_types")
set(LIBRARY_NAME asn1_opencv_conversions)
add_library(asn1_opencv_conversions src/asn1_opencv_conversions.cpp)
target_link_libraries(asn1_opencv_conversions cdff_types)
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 
# Install script
set(HEADERS include/conversions/asn1_opencv_conversions.hpp)
include(${CMAKE_SOURCE_DIR}/CMakeModules/CDFF_at_LAAS-install.cmake)

SET(DEPENDENCIES "cdff_types asn1_conversions")
set(LIBRARY_NAME asn1_pom_conversions)
add_library(asn1_pom_conversions src/asn1_pom_conversions.cpp)
target_link_libraries(asn1_pom_conversions cdff_types asn1_conversions)
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 
# Install script
set(HEADERS include/conversions/asn1_pom_conversions.hpp)
include(${CMAKE_SOURCE_DIR}/CMakeModules/CDFF_at_LAAS-install.cmake)


