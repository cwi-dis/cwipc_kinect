# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# This code serves to find OPENCV software
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

endif()

#
# Find OPENCV
# xxxjack manual hardcoded pathname for now.
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(OpenCV_LIBRARY_DIRS "C:/OpenCV-4.5.5/build/x64/vc15/lib")
	set(OpenCV_INCLUDE_DIRS "C:/OpenCV-4.5.5/build/include")
	set(OpenCV_LIBS "C:/OpenCV-4.5.5/build/x64/vc15/lib/opencv_world455.lib" "C:/OpenCV-4.5.5/build/x64/vc15/lib/opencv_world455d.lib")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	set(OpenCV_LIBS "/usr/lib/x86_64-linux-gnu")
	set(OpenCV_INCLUDE_DIRS "/usr/include")
	set(OpenCV_LIBS "opencv_world455" "opencv_world455d")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	MESSAGE(WARNING "OpenCV supported on OSX, SDK not available")
else()
	MESSAGE(WARNING "OpenCV not supported on this unknown system ${CMAKE_SYSTEM_NAME}")
endif()


#find_path(OpenCV_INC opencv2/opencv.hpp PATHS ${OpenCV_INCLUDE_DIRS})
#find_library(OpenCV_LIB NAMES opencv_world455 opencv_world455d PATHS ${OpenCV_LIBRARY_DIRS})

#if(OpenCV_LIB AND OpenCV_INC)
#  set(OpenCV_FOUND TRUE)
#else()
#  set(OpenCV_FOUND FALSE)
#endif()
