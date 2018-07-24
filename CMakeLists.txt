cmake_minimum_required (VERSION 3.0)

####
# Set variables:
#   * PROJECT_NAME
#   * PROJECT_VERSION
project (infuse_asn1_conversions VERSION 0.1)


####
# Imports infuse_cdff_types targets
find_package (infuse_cdff_types 0.1 CONFIG REQUIRED)
include_directories (${infuse_cdff_types_INCLUDE_DIR})


include_directories (include)

####
# Define all options
option (WITH_EIGEN "Compile conversions to/from Eigen structures" TRUE)
option (WITH_PCL   "Compile conversions to/from PCL structures"   TRUE)
option (WITH_ROS   "Compile conversions to/from ROS structures"   TRUE)


####
# Base types conversions
set (component_name base)
add_library (${component_name}_conversions SHARED src/asn1_${component_name}_conversions.cpp)
target_link_libraries (${component_name}_conversions infuse_cdff_types::infuse_cdff_types)
set_target_properties (${component_name}_conversions 
    PROPERTIES
        OUTPUT_NAME ${PROJECT_NAME}_${component_name}
        PUBLIC_HEADER include/${PROJECT_NAME}/asn1_${component_name}_conversions.hpp
)
set (TARGETS_TO_INSTALL ${component_name}_conversions)

####
# Eigen conversions
if (WITH_EIGEN)
    find_package (Eigen3 REQUIRED)
    include_directories (${EIGEN3_INCLUDE_DIRS})

    set (component_name eigen)
    add_library (${component_name}_conversions SHARED src/asn1_${component_name}_conversions.cpp)
    target_link_libraries (${component_name}_conversions infuse_cdff_types::infuse_cdff_types)
    set_target_properties (${component_name}_conversions 
        PROPERTIES
            OUTPUT_NAME ${PROJECT_NAME}_${component_name}
            PUBLIC_HEADER include/${PROJECT_NAME}/asn1_${component_name}_conversions.hpp
    )

    list (APPEND TARGETS_TO_INSTALL ${component_name}_conversions)
endif (WITH_EIGEN)


####
# PCL conversions
if (WITH_PCL)
    find_package( PCL 1.7.1 REQUIRED )
    include_directories (${PCL_INCLUDE_DIRS})

    set (component_name pcl)
    add_library (${component_name}_conversions SHARED src/asn1_${component_name}_conversions.cpp)
    target_link_libraries (${component_name}_conversions 
        base_conversions
        infuse_cdff_types::infuse_cdff_types)
    set_target_properties (${component_name}_conversions 
        PROPERTIES
            OUTPUT_NAME ${PROJECT_NAME}_${component_name}
            PUBLIC_HEADER include/${PROJECT_NAME}/asn1_${component_name}_conversions.hpp
    )

    list (APPEND TARGETS_TO_INSTALL ${component_name}_conversions)
endif (WITH_PCL)


####
# ROS
if (WITH_ROS)
    # Adds function 'pkg_check_modules'
    find_package( PkgConfig REQUIRED)

    pkg_check_modules( ROS REQUIRED rostime cpp_common)
    include_directories (${ROS_INCLUDE_DIRS})
    link_directories (${ROS_LIBRARY_DIRS})

    set (component_name ros)
    add_library (${component_name}_conversions SHARED src/asn1_${component_name}_conversions.cpp)
    target_link_libraries (${component_name}_conversions 
        ${ROS_LIBRARIES}
        infuse_cdff_types::infuse_cdff_types)
    set_target_properties (${component_name}_conversions 
        PROPERTIES
            OUTPUT_NAME ${PROJECT_NAME}_${component_name}
            PUBLIC_HEADER include/${PROJECT_NAME}/asn1_${component_name}_conversions.hpp
    )

    list (APPEND TARGETS_TO_INSTALL ${component_name}_conversions)
endif (WITH_ROS)


####
# Generation of configuration files


# Introduce variables:
# * CMAKE_INSTALL_LIBDIR
# * CMAKE_INSTALL_BINDIR
# * CMAKE_INSTALL_INCLUDEDIR
include(GNUInstallDirs)


# Layout. This works for all platforms:
#   * <prefix>/lib*/cmake/<PROJECT-NAME>
#   * <prefix>/lib*/
#   * <prefix>/include/
set(config_install_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")


set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")


# Configuration
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
set(namespace "${PROJECT_NAME}::")


# Include module with fuctions:
#   * 'write_basic_package_version_file'
#   * 'configure_package_config_file'
include(CMakePackageConfigHelpers)


# Configure '<PROJECT-NAME>ConfigVersion.cmake'
# Uses:
#   * PROJECT_VERSION
write_basic_package_version_file(
  "${version_config}" COMPATIBILITY SameMajorVersion
)


# Configure '<PROJECT-NAME>Config.cmake'
# Uses variables:
#   * TARGETS_EXPORT_NAME
#   * PROJECT_NAME
#   * CMAKE_INSTALL_INCLUDEDIR
configure_package_config_file(
    "cmake/Config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${config_install_dir}"
    PATH_VARS
      CMAKE_INSTALL_INCLUDEDIR
)


####
# Installation

install(
    TARGETS ${TARGETS_TO_INSTALL}
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
)


# Config
#   * <prefix>/lib/cmake/infuse_cdff_types/infuse_cdff_typesConfig.cmake
#   * <prefix>/lib/cmake/infuse_cdff_types/infuse_cdff_typesConfigVersion.cmake
install(
    FILES "${project_config}" "${version_config}"
    DESTINATION "${config_install_dir}"
)


# Config
#   * <prefix>/lib/cmake/infuse_cdff_types/infuse_cdff_typesTargets.cmake
install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    NAMESPACE "${namespace}"
    DESTINATION "${config_install_dir}"
)

###
# Add an uninstall target
configure_file(
    "cmake/cmake_uninstall.cmake.in"
    "${generated_dir}/cmake_uninstall.cmake" @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${generated_dir}/cmake_uninstall.cmake")