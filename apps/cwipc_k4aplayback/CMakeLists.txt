
cmake_minimum_required(VERSION 3.16.0)

add_executable(cwipc_k4aplayback cwipc_k4aplayback.cpp stb_image.h)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_k4aplayback cwipc_kinect cwipc_util)

#install(TARGETS cwipc_k4aplayback RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(NOT BUILDING_ON_HEADLESS_UNIX)
	# This test depends on the test input recording containing exactly 6
	# frames (although it will succeed if there are more, but the intention is
	# that we test that we read all frames)
	add_test(NAME cwipc_k4aplayback COMMAND cwipc_k4aplayback ${CMAKE_CURRENT_SOURCE_DIR}/../../tests/fixtures/input/recording . 6)
endif()
