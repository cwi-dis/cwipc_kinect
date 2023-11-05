
cmake_minimum_required(VERSION 3.16.0)

add_executable(cwipc_kinect_install_check cwipc_kinect_install_check.cpp)
include_directories(
	"../../include"
)
target_link_libraries(cwipc_kinect_install_check cwipc_kinect)

install(TARGETS cwipc_kinect_install_check RUNTIME DESTINATION ${CMAKE_INSTALL_LIBEXECDIR})

add_test(NAME cwipc_kinect_install_check COMMAND cwipc_kinect_install_check )
