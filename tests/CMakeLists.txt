cmake_minimum_required(VERSION 3.16.0)


file(COPY fixtures/input/recording DESTINATION ${CMAKE_TESTDATA_OUTPUT_DIRECTORY}/fixtures/input/)
install(DIRECTORY fixtures/input/recording DESTINATION ${CMAKE_TESTDATA_INSTALL_DIRECTORY}/fixtures/input/)
