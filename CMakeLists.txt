# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
cmake_minimum_required(VERSION 3.16.0)

project(cwipc_kinect)

# Save the command line compile commands in the build output
# set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# View the makefile commands during build
# set(CMAKE_VERBOSE_MAKEFILE on)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/CMakeFiles)

option(BUILD_SHARED_LIBS "Build shared library" ON)

include(CTest)
include(GNUInstallDirs)

#
# Find cwipc_util
#
if( CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR )
	message(WARNING Building cwipc_kinect outside cwipc umbrella is not tested)
	find_package(cwipc_util REQUIRED)
	include_directories(${cwipc_util_INCLUDE_DIRS})
	link_directories(${cwipc_util_LIBRARY_DIRS})
endif()

#
# Find PCL. 1.10-1.12 is expected
#
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#
# Find Azure Kinect SDK (either with pkgconfig or with cmake)
#
find_package(k4a REQUIRED)
find_package(k4abt REQUIRED)

find_package(libjpeg-turbo REQUIRED)

# We need OpenCV for some image filtering

find_package( OpenCV REQUIRED
	PATHS "C:/opencv" 
	PATH_SUFFIXES build 
	)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# we need the thirdparty tinyXML for the test programs.
include(third-party/tinyXML/CMakeLists.txt)
include_directories(third-party/tinyXML)

# If we are on Linux we should skip tests if we have no X11 server
if(NOT WIN32 AND NOT APPLE)
	if(NOT DEFINED ENV{DISPLAY})
		set(BUILDING_ON_HEADLESS_UNIX on)
	endif()
endif()

include_directories(include)

add_subdirectory(src)
add_subdirectory(apps/cwipc_kgrab)
add_subdirectory(apps/cwipc_kmultigrab)
add_subdirectory(apps/cwipc_k4aoffline)
add_subdirectory(apps/cwipc_k4aoffline_check)
add_subdirectory(python)
add_subdirectory(tests)