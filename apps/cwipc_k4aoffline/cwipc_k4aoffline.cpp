#include <iostream>
#include <istream>
#include <fstream>
#include "string.h"

#include <stdio.h>
#include <malloc.h>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>

#include "cwipc_kinect/private/K4AConfig.hpp"
#include "cwipc_util/api.h"
#include "cwipc_kinect/api.h"

#define DEBUG_AUXDATA

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "ERROR. Usage: " << argv[0] << " inputdirectory outputdirectory [count]" << std::endl;
        std::cerr << "Generates pointclouds from kinect4a camera recordings using the cameraconfig.xml" << std::endl;
        std::cerr << "If no outputdirectory a subfolder is created in the current folder" << std::endl;
        return 2;
    }
    int countWanted = 0;
    if (argc > 3) countWanted = atoi(argv[1]);
    char filename[500];
    char* error = NULL;
    cwipc_tiledsource* generator;
    char* inputdir = NULL;
    inputdir = argv[1];
    std::string outputdir(argv[2]);

    std::string configFile(inputdir);
    configFile += "/cameraconfig.xml";

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
        if (countWanted != 0 && framenum >= countWanted) break;
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
        }
        else {
            std::cerr << argv[0] << ": auxdata: " << ap->count() << " items:" << std::endl;
            for (int i = 0; i < ap->count(); i++) {
                void* ptr = ap->pointer(i);
                std::cerr << argv[0] << "auxdata: item " << i << " name=" << ap->name(i) << ", size=" << (int)ap->size(i) << ", descr=" << ap->description(i) << ", pointer=0x" << std::hex << (uint64_t)ptr << std::dec << std::endl;
            }
        }
#endif
        std::cerr << "xxxjack pointcloud has " << pc->count() << " points" << std::endl;
        framenum++;
        //Good, write the pointcloud
        std::ostringstream ts;
        ts << pc->timestamp();
        std::string outputfile = outputdir + "/pointcloud-" + ts.str() + ".cwipcdump";
        std::cout << "-> Writing frame " << framenum << " => "<< outputfile << std::endl;
        ok = cwipc_write_debugdump(outputfile.c_str(), pc, &error); //FASTER WRITE
        //ok = cwipc_write(filename, pc, &error);
        if (ok == -1) {
            std::cerr << "cwipc_k4aoffline: Error writing: " << error << std::endl;
            break;
        }
        pc->free();
        std::cout << "--------------------------------------------------------" << std::endl;
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

