#include <iostream>
#include <fstream>
#include "string.h"

#include <stdio.h>
#include <malloc.h>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>

#include "cwipc_util/api.h"
#include "cwipc_kinect/api.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#include "stb_image.h"

typedef struct
{
	char* filename;
	k4a_playback_t handle;
	k4a_record_configuration_t record_config;
	k4a_capture_t capture;
} recording_t;

int main(int argc, char** argv)
{
    bool ok;

	if (argc < 3)
	{
        std::cerr << "Usage: cwipc_k4aoffline <master.mkv> <sub1.mkv> ..." << std::endl;
		std::cerr << "Convert multiple mkvfiles into pointclouds" << std::endl;
		return 1;
	}

	size_t file_count = (size_t)(argc - 1);
	bool master_found = false;
	k4a_result_t result = K4A_RESULT_SUCCEEDED;

	// Allocate memory to store the state of N recordings.

	recording_t* files = (recording_t*) malloc(sizeof(recording_t) * file_count);
	if (files == NULL)
	{
		printf("Failed to allocate memory for playback (%zu bytes)\n", sizeof(recording_t) * file_count);
		return 1;
	}
	memset(files, 0, sizeof(recording_t) * file_count);

    // Open each recording file and validate they were recorded in master/subordinate mode.
    for (size_t i = 0; i < file_count; i++)
    {
        files[i].filename = argv[i + 1];

        result = k4a_playback_open(files[i].filename, &files[i].handle);
        if (result != K4A_RESULT_SUCCEEDED)
        {
            printf("Failed to open file: %s\n", files[i].filename);
            break;
        }

        result = k4a_playback_get_record_configuration(files[i].handle, &files[i].record_config);
        if (result != K4A_RESULT_SUCCEEDED)
        {
            printf("Failed to get record configuration for file: %s\n", files[i].filename);
            break;
        }

        if (files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER)
        {
            printf("Opened master recording file: %s\n", files[i].filename);
            if (master_found)
            {
                printf("ERROR: Multiple master recordings listed!\n");
                result = K4A_RESULT_FAILED;
                break;
            }
            else
            {
                master_found = true;
            }
        }
        else if (files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE)
        {
            printf("Opened subordinate recording file: %s\n", files[i].filename);
        }
        else
        {
            printf("ERROR: Recording file was not recorded in master/sub mode: %s\n", files[i].filename);
            result = K4A_RESULT_FAILED;
            break;
        }

        // Read the first capture of each recording into memory.
        k4a_stream_result_t stream_result = k4a_playback_get_next_capture(files[i].handle, &files[i].capture);
        if (stream_result == K4A_STREAM_RESULT_EOF)
        {
            printf("ERROR: Recording file is empty: %s\n", files[i].filename);
            result = K4A_RESULT_FAILED;
            break;
        }
        else if (stream_result == K4A_STREAM_RESULT_FAILED)
        {
            printf("ERROR: Failed to read first capture from file: %s\n", files[i].filename);
            result = K4A_RESULT_FAILED;
            break;
        }

        k4a_image_t depth = k4a_capture_get_depth_image(files[i].capture);
        if (depth == NULL) {
            std::cerr << "Depth is missing in capture 0 from " << files[i].filename << std::endl;
        }
        k4a_image_t color = k4a_capture_get_color_image(files[i].capture);
        if (color == NULL) {
            std::cerr << "Color is missing in capture 0 from " << files[i].filename << std::endl;
        }
    }

    if (result == K4A_RESULT_SUCCEEDED)
    {
        printf("Loading succeeded");
    }
    else 
    {
        std::cerr << "Loading failed" << std::endl;
    }





    /*if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " configfile colorimage depthimage outputfile" << std::endl;
		std::cerr << "Convert a depth image and color image to a pointcloud" << std::endl;
		return 2;
    }
	char *configFile = argv[1];
	char *colorFile = argv[2];
	char *depthFile = argv[3];
	char *outputFile = argv[4];
	if (strcmp(configFile, "-") == 0) configFile = NULL;

    char *error = NULL;
    cwipc_offline *converter;
	cwipc_tiledsource *generator;
	if (argc == 4) {
		configFile = argv[3];
	}
	converter = cwipc_rs2offline(settings, configFile, &error, CWIPC_API_VERSION);
    if (error) {
    	std::cerr << argv[0] << ": creating realsense2 offline converter failed: " << error << std::endl;
    	return 1;
    }
	generator = converter->get_source();

	int depthWidth, depthHeight, depthComponents;
    unsigned short *depthData = stbi_load_16(depthFile, &depthWidth, &depthHeight, &depthComponents, 1);
	assert(depthWidth == 640);
	assert(depthHeight == 480);
	assert(depthComponents == 1);
    size_t depthDataSize = depthWidth*depthHeight*2;
    int colorWidth, colorHeight, colorComponents;
    unsigned char *colorData = stbi_load(colorFile, &colorWidth, &colorHeight, &colorComponents, 3);
	assert(colorWidth == 640);
	assert(colorHeight == 480);
	assert(colorComponents == 3 || colorComponents == 4);
    size_t colorDataSize = colorWidth*colorHeight*3;
	int frameNum = 0;
	ok = converter->feed(0, frameNum, colorData, colorDataSize, depthData, depthDataSize);
	if (!ok) {
		std::cerr << argv[0] << ": Error feeding color and depth data" << std::endl;
		exit(1);
	}
	if(!generator->available(true)) {
		std::cerr << argv[0] << ": No pointcloud produced" << std::endl;
		exit(1);
	}
	cwipc *pc = generator->get();
	if (pc == NULL) {
		std::cerr << argv[0] << ": NULL pointcloud?" << std::endl;
		exit(1);
	}
	if (pc->get_uncompressed_size() == 0) {
		std::cerr << argv[0] << ": Empty pointcloud" << std::endl;
	} else {
		if (strcmp(outputFile, "-") != 0) {
			int sts = cwipc_write(outputFile, pc, &error);
			if (sts < 0) {
				if (error == NULL) error = (char *)"Unknown error";
				std::cerr << argv[0] << ": Error writing output file: " << error << std::endl;
			}
		}
	}
	pc->free();
    generator->free();*/
    return 0;
}

