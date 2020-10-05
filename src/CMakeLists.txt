# ======================================================
# Compiled by Fons @ CWI, Amsterdam for VRTogether
#
# Copyright (C) 2018 by CWI. All rights reserved.
# ======================================================
#
#  minimum required cmake version: 3.10.0
cmake_minimum_required(VERSION 3.10.0)


add_library(cwipc_kinect 
	SHARED 
	cwipc_kinect.cpp 
	K4ACapture.cpp 
	K4ACamera.cpp 
	utils.cpp 
	cwipc_pcl_additions.cpp 
	${TINY_SRC}
)

target_sources(cwipc_kinect
	PRIVATE
	"../include/cwipc_kinect/defs.h"
	"../include/cwipc_kinect/utils.h"
	"../include/cwipc_kinect/K4ACapture.hpp"
	"../include/cwipc_kinect/K4ACamera.hpp"
	"../include/cwipc_kinect/window_util.hpp"
	"../include/cwipc_kinect/stb_image_write.h"
	"../include/cwipc_kinect/api.h"
)

target_link_libraries(cwipc_kinect PUBLIC cwipc_util)
target_link_libraries(cwipc_kinect PRIVATE ${PCL_LIBRARIES} ${KINECT_LIBRARIES})

install(TARGETS cwipc_kinect
    EXPORT cwipc_kinect
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include/cwipc_kinect"
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/cwipc_kinect
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${PROJECT_SOURCE_DIR}/CMakeFiles/cwipc_kinect-config.cmake DESTINATION lib/cmake/cwipc_kinect)

install(EXPORT cwipc_kinect DESTINATION lib/cmake/cwipc_kinect)

# Set library pkgconfig file for facilitating 3rd party integration
# install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config/cwipc_realsense.pc"
#         DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
# )
