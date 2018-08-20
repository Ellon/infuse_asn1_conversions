cmake_minimum_required (VERSION 3.0)

####
# Set variables:
#   * PROJECT_NAME
#   * PROJECT_VERSION
project (infuse_asn1_conversions VERSION 0.2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")


####
# Imports infuse_asn1_types targets
find_package (infuse_asn1_types 0.2 CONFIG REQUIRED)
include_directories (${infuse_asn1_types_INCLUDE_DIR})


include_directories (include)

####
# Define all options
option (WITH_EIGEN "Compile conversions to/from Eigen structures" TRUE)
option (WITH_PCL   "Compile conversions to/from PCL structures"   TRUE)
option (WITH_ROS   "Compile conversions to/from ROS structures"   TRUE)
option (WITH_POM   "Compile conversions to/from PoM structures"   TRUE)


####
# Base types conversions
set (component_name base)
add_library (${component_name}_conversions SHARED src/asn1_${component_name}_conversions.cpp)
target_link_libraries (${component_name}_conversions infuse_asn1_types::infuse_asn1_types)
set_target_properties (${component_name}_conversions 
    PROPERTIES
        VERSION ${PROJECT_VERSION}
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
    target_link_libraries (${component_name}_conversions infuse_asn1_types::infuse_asn1_types)
    set_target_properties (${component_name}_conversions 
        PROPERTIES
            VERSION ${PROJECT_VERSION}
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
        infuse_asn1_types::infuse_asn1_types)
    set_target_properties (${component_name}_conversions 
        PROPERTIES
            VERSION ${PROJECT_VERSION}
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
        infuse_asn1_types::infuse_asn1_types)
    set_target_properties (${component_name}_conversions 
        PROPERTIES
            VERSION ${PROJECT_VERSION}
            OUTPUT_NAME ${PROJECT_NAME}_${component_name}
            PUBLIC_HEADER include/${PROJECT_NAME}/asn1_${component_name}_conversions.hpp
    )

    list (APPEND TARGETS_TO_INSTALL ${component_name}_conversions)
endif (WITH_ROS)

####
# POM 
if (WITH_POM)
    # Adds function 'pkg_check_modules'
    find_package( infuse_pom_base REQUIRED)
    include_directories (${infuse_pom_base_INCLUDE_DIRS})
    link_directories (${infuse_pom_base_LIBRARY_DIRS})

    set (component_name pom)
    add_library (${component_name}_conversions SHARED src/asn1_${component_name}_conversions.cpp)
    target_link_libraries (${component_name}_conversions 
        ${pom-base_LIBRARIES}
        infuse_asn1_types::infuse_asn1_types)
    set_target_properties (${component_name}_conversions 
        PROPERTIES
            VERSION ${PROJECT_VERSION}
            OUTPUT_NAME ${PROJECT_NAME}_${component_name}
            PUBLIC_HEADER include/${PROJECT_NAME}/asn1_${component_name}_conversions.hpp
    )

    list (APPEND TARGETS_TO_INSTALL ${component_name}_conversions)
endif (WITH_POM)

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

# This is exclusive for pkg-config
set(pkgconfig_install_dir "${CMAKE_INSTALL_LIBDIR}/pkgconfig")


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

# Configure '<PROJECT-NAME>_<COMPONENT_NAME>.pc'
# Uses variables:
#   * PROJECT_NAME
#   * PROJECT_VERSION
#   * CMAKE_INSTALL_PREFIX
#   * CMAKE_INSTALL_LIBDIR
#   * CMAKE_INSTALL_INCLUDEDIR
#   * COMPONENT_NAME
#   * REQUIRED_COMPONENTS
set (COMPONENT_NAME base)
set (REQUIRED_COMPONENTS "")
configure_file(
    "cmake/pkgconfig_component.pc.cmakein"
    "${generated_dir}/${PROJECT_NAME}_${COMPONENT_NAME}.pc" @ONLY
)


if (WITH_EIGEN)
    set (COMPONENT_NAME eigen)
    set (REQUIRED_COMPONENTS ${PROJECT_NAME}_base)
    configure_file(
        "cmake/pkgconfig_component.pc.cmakein"
        "${generated_dir}/${PROJECT_NAME}_${COMPONENT_NAME}.pc" @ONLY
    )
endif()


if (WITH_PCL)
    set (COMPONENT_NAME pcl)
    set (REQUIRED_COMPONENTS ${PROJECT_NAME}_base)
    configure_file(
        "cmake/pkgconfig_component.pc.cmakein"
        "${generated_dir}/${PROJECT_NAME}_${COMPONENT_NAME}.pc" @ONLY
    )
endif()


if (WITH_ROS)
    set (COMPONENT_NAME ros)
    set (REQUIRED_COMPONENTS ${PROJECT_NAME}_base)
    configure_file(
        "cmake/pkgconfig_component.pc.cmakein"
        "${generated_dir}/${PROJECT_NAME}_${COMPONENT_NAME}.pc" @ONLY
    )
endif()


####
# Installation

# Targets
install(
    TARGETS ${TARGETS_TO_INSTALL}
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
)


# CMake Config files
#   * <prefix>/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake
#   * <prefix>/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake
install(
    FILES "${project_config}" "${version_config}"
    DESTINATION "${config_install_dir}"
)

# pkg-config file 
#   * <prefix>/lib/pkgconfig/${PROJECT_NAME}.pc
set (COMPONENT_NAME base)
install(
    FILES "${generated_dir}/${PROJECT_NAME}_${COMPONENT_NAME}.pc"
    DESTINATION "${pkgconfig_install_dir}"
)


if (WITH_EIGEN)
    set (COMPONENT_NAME eigen)
    install(
        FILES "${generated_dir}/${PROJECT_NAME}_${COMPONENT_NAME}.pc"
        DESTINATION "${pkgconfig_install_dir}"
    )
endif()


if (WITH_PCL)
    set (COMPONENT_NAME pcl)
    install(
        FILES "${generated_dir}/${PROJECT_NAME}_${COMPONENT_NAME}.pc"
        DESTINATION "${pkgconfig_install_dir}"
    )
endif()


if (WITH_ROS)
    set (COMPONENT_NAME ros)
    install(
        FILES "${generated_dir}/${PROJECT_NAME}_${COMPONENT_NAME}.pc"
        DESTINATION "${pkgconfig_install_dir}"
    )
endif()


# Exported targets config file
#   * <prefix>/lib/cmake/infuse_cdff_types/infuse_cdff_typesTargets.cmake
install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    NAMESPACE "${namespace}"
    DESTINATION "${config_install_dir}"
)


###
# Add an uninstall target, useful when we want to clean files installed by hand
configure_file(
    "cmake/cmake_uninstall.cmake.in"
    "${generated_dir}/cmake_uninstall.cmake" @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${generated_dir}/cmake_uninstall.cmake")
