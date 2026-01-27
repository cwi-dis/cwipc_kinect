
cmake_minimum_required(VERSION 3.16.0)

add_executable(cwipc_kinect_playback cwipc_kinect_playback.cpp stb_image.h)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_kinect_playback cwipc_kinect cwipc_util)

#install(TARGETS cwipc_kinect_playback RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(NOT BUILDING_ON_HEADLESS_UNIX)
	# This test depends on the test input recording containing exactly 6
	# frames (although it will succeed if there are more, but the intention is
	# that we test that we read all frames)
	add_test(NAME cwipc_kinect_playback COMMAND cwipc_kinect_playback ${CMAKE_CURRENT_SOURCE_DIR}/../../tests/fixtures/input/kinect_recording . 6)
endif()
