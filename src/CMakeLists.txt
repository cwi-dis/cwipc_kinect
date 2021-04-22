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
	K4AOfflineCapture.cpp 
	K4AOfflineCamera.cpp 
	K4AConfig.cpp 
	cwipc_pcl_additions.cpp 
	${TINY_SRC}
)

target_sources(cwipc_kinect
	PRIVATE
	"../include/cwipc_kinect/private/K4AConfig.hpp"
	"../include/cwipc_kinect/private/K4ACapture.hpp"
	"../include/cwipc_kinect/private/K4ACamera.hpp"
	"../include/cwipc_kinect/private/readerwriterqueue.h"
	"../include/cwipc_kinect/private/atomicops.h"
	"../include/cwipc_kinect/api.h"
)

target_link_libraries(cwipc_kinect PUBLIC cwipc_util)
target_link_libraries(cwipc_kinect PRIVATE ${PCL_LIBRARIES} ${KINECT_LIBRARIES} ${JPEG_Turbo_LIBRARY})

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC" AND CMAKE_BUILD_TYPE MATCHES "Release")
   target_compile_options(cwipc_kinect PRIVATE /Zi)

   # Tell linker to include symbol data
    set_target_properties(cwipc_kinect PROPERTIES 
        LINK_FLAGS "/INCREMENTAL:NO /DEBUG /OPT:REF /OPT:ICF"
    )

    # Set file name & location
    set_target_properties(cwipc_kinect PROPERTIES 
        COMPILE_PDB_NAME cwipc_kinect 
        COMPILE_PDB_OUTPUT_DIR ${CMAKE_BINARY_DIR}
    )
	
	# Install pdb file
	install(DIRECTORY ${PROJECT_BINARY_DIR}/Release
		DESTINATION ${CMAKE_INSTALL_BINDIR}
		FILES_MATCHING PATTERN *.pdb
	)
endif()

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
