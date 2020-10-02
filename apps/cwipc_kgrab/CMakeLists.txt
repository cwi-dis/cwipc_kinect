
cmake_minimum_required(VERSION 3.10.0)

add_executable(cwipc_kgrab cwipc_kgrab.cpp)
include_directories(
	"../../include"
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(cwipc_kgrab cwipc_kinect cwipc_util ${PCL_LIBRARIES})

install(TARGETS cwipc_kgrab RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_test(NAME cwipc_kgrab COMMAND cwipc_kgrab 2 . )
set_tests_properties(cwipc_grab PROPERTIES ENVIRONMENT CWIPC_KINECT_TESTING=yes)
