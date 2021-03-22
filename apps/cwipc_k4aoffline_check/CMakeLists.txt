
cmake_minimum_required(VERSION 3.10.0)

add_executable(cwipc_k4aoffline_check cwipc_k4aoffline_check.cpp)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_k4aoffline_check cwipc_kinect cwipc_util ${PCL_LIBRARIES} ${KINECT_LIBRARIES})

install(TARGETS cwipc_k4aoffline_check RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# add_test(NAME cwipc_rs2offline COMMAND cwipc_rs2offline input/offlineconfig.xml input/color.png input/depth.png output/rs2offline.ply WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/tests/fixtures )
