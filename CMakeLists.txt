# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
#  minimum required cmake version: 3.10.0
cmake_minimum_required(VERSION 3.10.0)

project(cwipc_kinect)

# Save the command line compile commands in the build output
# set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# View the makefile commands during build
# set(CMAKE_VERBOSE_MAKEFILE on)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/CMakeFiles)

# option(BUILD_GRAPHICAL_EXAMPLES "Build graphical examples and tools." ON)
option(BUILD_SHARED_LIBS "Build shared library" ON)

include(CTest)
include(GNUInstallDirs)

# Ensure all executable, dll/so and link-library outputs end up in the same directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})

#
# Find cwipc_util
#
find_package(cwipc_util REQUIRED)
include_directories(${cwipc_util_INCLUDE_DIRS})
link_directories(${cwipc_util_LIBRARY_DIRS})

#
# Find PCL. 1.11 is expected
#
find_package(PCL REQUIRED COMPONENTS common io visualization filters) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#
# Find Azure Kinect SDK (either with pkgconfig or with cmake)
#
find_package(KINECT REQUIRED)
include_directories(${KINECT_INCLUDE_DIRS})
link_directories(${KINECT_LIBRARY_DIRS})

# Set policy to adhere to JPEG_Turbo_ROOT (default is ignore, which will not work on win10)
cmake_policy(SET CMP0074 NEW)
find_package(JPEG_Turbo REQUIRED)
include_directories(${JPEG_Turbo_INCLUDE_DIR})
link_directories(${JPEG_Turbo_LIBRARY_DIR})


# we need the thirdparty tinyXML for the test programs.
include(third-party/tinyXML/CMakeLists.txt)
include_directories(third-party/tinyXML)

# If we're on Linux os MacOS we need libusb

if(NOT WIN32)
	list(APPEND DEPENDENCIES m ${LIBUSB1_LIBRARIES})
endif()


include_directories(include)

add_subdirectory(src)
add_subdirectory(apps/cwipc_kgrab)
add_subdirectory(apps/cwipc_kmultigrab)
add_subdirectory(apps/cwipc_k4aoffline)
add_subdirectory(apps/cwipc_k4aoffline_check)
# needs glfw3 add_subdirectory(apps/cwipc_krenderer)
add_subdirectory(python)
