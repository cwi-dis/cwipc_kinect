# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# This code serves to find Intel RealSense SDK software
# Running it defines:
#	REALSENSE2_FOUND
#	REALSENSE2_INC
#	REALSENSE2_LIB
#	REALSENSE2_DLL
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================

if(WIN32)

  set(CMAKE_FIND_LIBRARY_SUFFIXES .dll ${CMAKE_FIND_LIBRARY_SUFFIXES})
  if(NOT K4A_ROOT)
    set(K4A_ROOT "C:/Program Files/Azure Kinect SDK v1.4.1")
  endif()
  if(NOT K4ABT_ROOT)
    set(K4ABT_ROOT "C:/Program Files/Azure Kinect Body Tracking SDK")
  endif()
endif()

#
# Find Kinect for Azure SDK
# xxxjack manual hardcoded pathname for now.
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(KINECT_LIBRARY_DIRS "${K4A_ROOT}/sdk/windows-desktop/amd64/release/lib" "${K4ABT_ROOT}/sdk/windows-desktop/amd64/release/lib")
	set(KINECT_INCLUDE_DIRS "${K4A_ROOT}/sdk/include" "${K4ABT_ROOT}/sdk/include")
	set(KINECT_LIBRARIES "k4a" "k4arecord" "k4abt")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	set(KINECT_LIBRARY_DIRS "/usr/lib/x86_64-linux-gnu")
	set(KINECT_INCLUDE_DIRS "/usr/include")
	set(KINECT_LIBRARIES "k4a" "k4arecord" "k4abt")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	MESSAGE(WARNING "Kinect grabber not supported on OSX, SDK not available")
else()
	MESSAGE(WARNING "Kinect grabber not supported on this unknown system ${CMAKE_SYSTEM_NAME}")
endif()


find_path(KINECT_INC k4a/k4a.h k4arecord/playback.h ka4bt.h PATHS ${KINECT_INCLUDE_DIRS})
find_library(KINECT_LIB NAMES k4a k4arecord k4abt PATHS ${KINECT_LIBRARY_DIRS})

if(KINECT_LIB AND KINECT_INC)
  set(KINECT_FOUND TRUE)
else()
  set(KINECT_FOUND FALSE)
endif()
