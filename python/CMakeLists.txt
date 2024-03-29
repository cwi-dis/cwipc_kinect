cmake_minimum_required(VERSION 3.16.0)
cmake_policy(SET CMP0074 NEW)

# Setup venv-based Python

include(CwipcPythonSupport)

#
# It is expected that CMAKE_PYWHEELS_INSTALL_DIRECTORY and CMAKE_PYWHEELS_OUTPUT_DIRECTORY
# have been set already.
#

#
# Building.
#

# Create Python wheel in the staging area

cwipc_build_wheel(NAME "cwipc_kinect" SOURCEDIR ${CMAKE_CURRENT_SOURCE_DIR} WHEELDIR ${CMAKE_PYWHEELS_OUTPUT_DIRECTORY})
file(COPY test_cwipc_kinect.py DESTINATION ${CMAKE_PYWHEELS_OUTPUT_DIRECTORY})

#
# Installation.
#

# Copy the whole Python staging subdirectory to the installation (so all scripts are available for inspection and testing)
# Also install the wheel into the current system Python.

install(FILES test_cwipc_kinect.py DESTINATION ${CMAKE_PYWHEELS_INSTALL_DIRECTORY})
cwipc_install_wheel(NAME "cwipc_kinect" WHEELDIR ${CMAKE_PYWHEELS_OUTPUT_DIRECTORY})

#
# Testing.
#

# setup test environment (installs module into venv)

add_test(NAME _python_setup_cwipc_kinect
	COMMAND ${Python3_EXECUTABLE} -m pip install --no-cache-dir --upgrade --find-links=${CMAKE_PYWHEELS_OUTPUT_DIRECTORY} cwipc_kinect
	)
set_tests_properties(_python_setup_cwipc_kinect PROPERTIES FIXTURES_SETUP venv RUN_SERIAL TRUE)


add_test(NAME cwipc_kinect_python_tests
	COMMAND ${Python3_EXECUTABLE} ${CMAKE_PYWHEELS_OUTPUT_DIRECTORY}/test_cwipc_kinect.py
	WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
cwipc_python_test(TEST cwipc_kinect_python_tests FIXTURES_REQUIRED venv)
