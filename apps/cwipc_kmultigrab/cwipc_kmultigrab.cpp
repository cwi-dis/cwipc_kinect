#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <inttypes.h>

#include "cwipc_util/api_pcl.h"
#include "cwipc_kinect/api.h"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " count directory [configfile]" << std::endl;
        std::cerr << "Creates COUNT pointclouds from a kinect camera and stores the PLY files in the given DIRECTORY" << std::endl;
        return 2;
    }

    int count = atoi(argv[1]);
    char filename[500];
    char *error = NULL;
    std::vector<uint64_t> timestamps;
    std::vector<cwipc_pcl_pointcloud> capturepointclouds;
    cwipc_tiledsource *generator;
    char *configFile = NULL;

    if (argc == 3) {
        configFile = argv[2];
    }

    generator = cwipc_kinect(argv[3], &error, CWIPC_API_VERSION);

    if (generator == NULL) {
        std::cerr << argv[0] << ": creating kinect grabber failed: " << error << std::endl;

        if (getenv("CWIPC_KINECT_TESTING") != NULL) {
            return 0; // No failure while running tests, so we can at least test linking, etc.
        }

        return 1;
    }

    if (error) {
        std::cerr << argv[0] << ": warning while creating kinect grabber: " << error << std::endl;
    }

    int ok = 0;
    while (count-- > 0 && ok == 0) {
        cwipc *pc = generator->get();
        cwipc_pcl_pointcloud pcl_pc = pc->access_pcl_pointcloud();
        capturepointclouds.push_back(pcl_pc->makeShared());
        timestamps.push_back(pc->timestamp());
        pc->free();
    }

    count = atoi(argv[1]);

    while (count-- > 0 && ok == 0) {
        if (strcmp(argv[2], "-") != 0) {
            snprintf(filename, sizeof(filename), "%s/pointcloud-%" PRIu64 ".ply", argv[2], timestamps.back());
            ok = cwipc_write(filename, cwipc_from_pcl(capturepointclouds.back(), timestamps.back(), &error, CWIPC_API_VERSION), &error);
        }

        capturepointclouds.pop_back();
        timestamps.pop_back();
    }

    generator->free();

    if (ok < 0) {
        std::cerr << "Error: " << error << std::endl;
        return 1;
    }

    return 0;
}

