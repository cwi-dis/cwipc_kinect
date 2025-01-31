#include <iostream>
#include <fstream>
#include "string.h"
#include <stdlib.h>
#include <inttypes.h>

#include "cwipc_util/api.h"
#include "cwipc_kinect/api.h"

#undef DEBUG_AUXDATA

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "ERROR. Usage: " << argv[0] << " inputdirectory outputdirectory [count]" << std::endl;
        std::cerr << "Generates pointclouds from kinect4a camera recordings using the cameraconfig.json" << std::endl;
        std::cerr << "If no outputdirectory a subfolder is created in the current folder" << std::endl;
        return 2;
    }

    int countWanted = 0;

    if (argc > 3) {
        countWanted = atoi(argv[3]);
    }

    char filename[500];
    char* error = NULL;
    cwipc_tiledsource* generator;
    char* inputdir = NULL;
    inputdir = argv[1];
    char *outputdir = argv[2];

    std::string configFile(inputdir);
    configFile += "/cameraconfig.json";

    generator = cwipc_k4aoffline(configFile.c_str(), &error, CWIPC_API_VERSION);

    if (generator == NULL) {
        std::cerr << argv[0] << ": creating offlinekinect grabber failed: " << error << std::endl;
        //if (getenv("CWIPC_KINECT_TESTING") != NULL) return 0; // No failure while running tests, so we can at least test linking, etc.

        return 1;
    }

    if (error) {
        std::cerr << argv[0] << ": warning while creating kinect grabber: " << error << std::endl;
    }

#ifdef DEBUG_AUXDATA
    generator->request_auxiliary_data("rgb");
    generator->request_auxiliary_data("depth");
    generator->request_auxiliary_data("skeletons");
#endif

    int ok = 0;
    int framenum = 0;
    int nGrabbedSuccessfully = 0;

    while (!generator->eof()) {
        if (countWanted != 0 && framenum >= countWanted) {
            break;
        }

        cwipc* pc = NULL;
        pc = generator->get();

        if (pc == NULL) {
            error = (char*)"grabber returned NULL pointcloud";
            ok = -1;

            break;
        }

        if (pc->count() <= 0) {
            std::cerr << argv[0] << ": warning: empty pointcloud, grabbing again" << std::endl;
            pc->free();

            continue;
        }

#ifdef DEBUG_AUXDATA
        cwipc_auxiliary_data* ap = pc->access_auxiliary_data();

        if (ap == nullptr) {
            std::cerr << argv[0] << ": access_auxiliary_data: returned null pointer" << std::endl;
        } else {
            std::cerr << argv[0] << ": auxdata: " << ap->count() << " items:" << std::endl;

            for (int i = 0; i < ap->count(); i++) {
                void* ptr = ap->pointer(i);
                std::cerr << argv[0] << "auxdata: item " << i << " name=" << ap->name(i) << ", size=" << (int)ap->size(i) << ", descr=" << ap->description(i) << ", pointer=0x" << std::hex << (uint64_t)ptr << std::dec << std::endl;
            }
        }
#endif

        framenum++;

        if (strcmp(outputdir, "-") != 0) {
            snprintf(filename, sizeof(filename), "%s/pointcloud-%08.8" PRIu64 ".ply", outputdir, pc->timestamp());
            std::cout << "-> Writing frame " << framenum << " with " << pc->count() << " points to "<< filename << std::endl;
            ok = cwipc_write_ext(filename, pc, 1, &error);
        } else {
            std::cout << "-> Dropping frame " << framenum << " with " << pc->count() << " points" << std::endl;
        }

        pc->free();
        nGrabbedSuccessfully++;
    }

    generator->free();

    if (ok < 0) {
        std::cerr << "cwipc_k4aoffline: Error: " << error << std::endl;
        return 1;
    }

    if (countWanted != 0 && nGrabbedSuccessfully != countWanted) {
        std::cerr << "cwipc_k4aoffline: Wanted " << countWanted << " pointclouds but got only " << nGrabbedSuccessfully << std::endl;
        return 1;
    }

    std::cerr << "cwipc_k4aoffline: Saved " << nGrabbedSuccessfully << " pointclouds" << std::endl;
    return 0;
}

