#include <iostream>
#include <fstream>
#include "string.h"
#include <stdlib.h>
#include <inttypes.h>
#include "cwipc_util/api.h"
#include "cwipc_kinect/api.h"

int main(int argc, char** argv) {
    char *error = NULL;
    cwipc_source *generator = cwipc_kinect("auto", &error, CWIPC_API_VERSION);
    if (generator == NULL) {
        char* expectedError = strstr(error, "no kinect cameras found");
        if (expectedError == NULL) {
            expectedError = strstr(error, "k4a_device_open failed");
        }
        if (expectedError == NULL) {
            // Any other error is unexpected.
            std::cerr << argv[0] << ": Error: " << error << std::endl;
            return 1;
        }
    }
    else {
        generator->free();
    }
    return 0;
}

