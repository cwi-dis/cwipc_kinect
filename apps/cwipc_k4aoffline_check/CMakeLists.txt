
cmake_minimum_required(VERSION 3.16.0)

add_executable(cwipc_k4aoffline_check cwipc_k4aoffline_check.cpp)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_k4aoffline_check cwipc_kinect cwipc_util k4a::k4a k4a::k4arecord)

#install(TARGETS cwipc_k4aoffline_check RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_test(NAME cwipc_k4aoffline_check COMMAND cwipc_k4aoffline_check ${PROJECT_SOURCE_DIR}/tests/fixtures/input/recording/output_master.mkv ${PROJECT_SOURCE_DIR}/tests/fixtures/input/recording/output_sub_1.mkv ) 