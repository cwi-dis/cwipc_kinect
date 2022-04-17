
cmake_minimum_required(VERSION 3.16.0)

add_executable(cwipc_kmultigrab cwipc_kmultigrab.cpp)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_kmultigrab cwipc_kinect cwipc_util)

# install(TARGETS cwipc_kmultigrab RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_test(NAME cwipc_kmultigrab COMMAND cwipc_kmultigrab 10 - )
set_tests_properties(cwipc_kmultigrab PROPERTIES ENVIRONMENT CWIPC_KINECT_TESTING=yes)
