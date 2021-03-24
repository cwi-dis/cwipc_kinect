#include <iostream>
#include <istream>
#include <fstream>
#include "string.h"

#include <stdio.h>
#include <malloc.h>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>

#include "cwipc_util/api.h"
#include "cwipc_kinect/api.h"

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " inputdirectory [outputdirectory]" << std::endl;
        std::cerr << "Generates pointclouds from kinect4a camera recordings using the cameraconfig.xml" << std::endl;
        std::cerr << "If no outputdirectory a subfolder is created in the current folder" << std::endl;
        return 2;
    }
    //int count = atoi(argv[1]);
    char filename[500];
    char* error = NULL;
    cwipc_tiledsource* generator;
    char* inputdir = NULL;
    inputdir = argv[1];
    std::string configFile(inputdir);
    configFile += "/cameraconfig.xml";

    generator = cwipc_k4aoffline(configFile.c_str(), &error, CWIPC_API_VERSION);
    if (generator == NULL) {
        std::cerr << argv[0] << ": creating offlinekinect grabber failed: " << error << std::endl;
        if (getenv("CWIPC_KINECT_TESTING") != NULL) return 0; // No failure while running tests, so we can at least test linking, etc.
        return 1;
    }
    if (error) {
        std::cerr << argv[0] << ": warning while creating kinect grabber: " << error << std::endl;
    }

    int ok = 0;
    while (!generator->eof()) {
        cwipc* pc = NULL;
        pc = generator->get();
        if (pc == NULL) {
            error = (char*)"grabber returned NULL pointcloud";
            ok = -1;
            break;
        }
        if (pc->count() <= 0) {
            std::cerr << argv[0] << ": warning: empty pointcloud, grabbing again" << std::endl;
            break;
        }
        //Good, write the pointcloud
        snprintf(filename, sizeof(filename), "D:/dump/pointcloud-%lld.cwipcdump", pc->timestamp());
        std::cout << "-> Writing " << filename << std::endl;
        ok = cwipc_write_debugdump(filename, pc, &error);
        pc->free();
    }
    generator->free();
    if (ok < 0) {
        std::cerr << "Error: " << error << std::endl;
        return 1;
    }
    return 0;
}

