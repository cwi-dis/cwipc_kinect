#include <iostream>
#include <fstream>
#include "string.h"
#include <stdlib.h>
#include <inttypes.h>

#include "cwipc_util/api.h"
#include "cwipc_kinect/api.h"

#undef DEBUG_AUXDATA

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " count directory [configfile]" << std::endl;
		std::cerr << "Creates COUNT pointclouds from a kinect4a camera and stores the PLY files in the given DIRECTORY" << std::endl;
		std::cerr << "If directory is - then drop the pointclouds on the floor" << std::endl;
		return 2;
    }
    int count = atoi(argv[1]);
    char filename[500];
    char *error = NULL;
    
	cwipc_tiledsource *generator;
	char *configFile = NULL;
	if (argc == 4) {
		configFile = argv[3];
	}
	generator = cwipc_kinect(configFile, &error, CWIPC_API_VERSION);
    if (generator == NULL) {
        std::cerr << argv[0] << ": creating kinect grabber failed: " << error << std::endl;
        if (getenv("CWIPC_KINECT_TESTING") != NULL) return 0; // No failure while running tests, so we can at least test linking, etc.
        return 1;
    }
    if (error) {
        std::cerr << argv[0] << ": warning while creating kinect grabber: " << error << std::endl;
    }
#ifdef DEBUG_AUXDATA
    generator->request_auxiliary_data("rgb");
    generator->request_auxiliary_data("depth");
#endif
	cwipc_tileinfo tif;
	generator->get_tileinfo(0, &tif);
	generator->get_tileinfo(1, &tif);
	generator->get_tileinfo(2, &tif);
	generator->get_tileinfo(3, &tif);
	generator->get_tileinfo(4, &tif);
	int ok = 0;
    while (count-- > 0 && ok == 0) {
        cwipc *pc = NULL;
        while(1) {
            pc = generator->get();
            if (pc == NULL) {
                error = (char *)"grabber returned NULL";
                ok = -1;
                break;
            }
            int count = pc->count();
            if (count > 0) break;
            std::cerr << argv[0] << ": warning: empty pointcloud, grabbing again" << std::endl;
        }
		if (strcmp(argv[2], "-") != 0) {
			snprintf(filename, sizeof(filename), "%s/pointcloud-%" PRIu64 ".ply", argv[2], pc->timestamp());
            std::cout << "-> Writing " << filename << std::endl;
			ok = cwipc_write(filename, pc, &error);
		}
#ifdef DEBUG_AUXDATA
        cwipc_auxiliary_data* ap = pc->access_auxiliary_data();
        if (ap == nullptr) {
            std::cerr << argv[0] << ": access_auxiliary_data: returned null pointer" << std::endl;
        }
        else {
            std::cerr << argv[0] << ": auxdata: " << ap->count() << " items:" << std::endl;
            for (int i = 0; i < ap->count(); i++) {
                std::cerr << argv[0] << "auxdata: item " << i << " name=" << ap->name(i) << ", size=" << (int)ap->size(i) << ", descr=" << ap->description(i) << std::endl;
            }
        }
#endif
        pc->free();
    }
    generator->free();
    if (ok < 0) {
    	std::cerr << "Error: " << error << std::endl;
    	return 1;
    }
    return 0;
}

