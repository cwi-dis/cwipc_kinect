#include <iostream>
#include <istream>
#include <fstream>
#include "string.h"

#include <stdio.h>
#include <malloc.h>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>

typedef struct
{
	char* filename;
	k4a_playback_t handle;
	k4a_record_configuration_t record_config;
	k4a_capture_t capture;
    uint64_t current_capture_timestamp;
    int capture_id = 0;
} recording_t;

int frame_num = 0;
int master_id;
bool first;
bool eof = false;
uint64_t max_delay_cond = 1600; // 160x10 will work for at least 10 cameras

bool check_color_and_depth(recording_t* file) {
    bool both = true;
    k4a_image_t color = k4a_capture_get_color_image(file->capture);
    k4a_image_t depth = k4a_capture_get_depth_image(file->capture);

    if((color == NULL) || (depth == NULL)){
        return false;
    }
    else {
        file->current_capture_timestamp = k4a_image_get_device_timestamp_usec(color);
        return true;
    }
}

bool prepare_next_valid_frame(recording_t* file) {
    k4a_result_t result = K4A_RESULT_SUCCEEDED;
    // Read the next capture into memory
    bool succeeded = false;
    while (!succeeded){
        if (file->capture != NULL)
            k4a_capture_release(file->capture);
        k4a_stream_result_t stream_result = k4a_playback_get_next_capture(file->handle, &file->capture);
        if (stream_result == K4A_STREAM_RESULT_EOF)
        {
            if (file->current_capture_timestamp == 0) {
                std::cerr << "ERROR: Recording file is empty: "<< file->filename << std::endl;
                result = K4A_RESULT_FAILED;
            }
            else {
                std::cout << "Recording file " << file->filename << " reached EOF" << std::endl;
                eof = true;
            }
            break;
        }
        else if (stream_result == K4A_STREAM_RESULT_FAILED)
        {
            std::cerr << "ERROR: Failed to read first capture from file: " <<  file->filename << std::endl;
            result = K4A_RESULT_FAILED;
            break;
        }
        file->capture_id++;

        k4a_image_t color = k4a_capture_get_color_image(file->capture);
        if (color == NULL) {
            if(!first)
                std::cerr << "Color is missing in capture " << file->capture_id << " from " << file->filename << std::endl;
            continue;
        }

        k4a_image_t depth = k4a_capture_get_depth_image(file->capture);
        if (depth == NULL) {
            if (!first)
                std::cerr << "Depth is missing in capture " << file->capture_id << " from " << file->filename << std::endl;
            k4a_image_release(color);
            continue;
        }
        file->current_capture_timestamp = k4a_image_get_device_timestamp_usec(color);
        succeeded = true;
        k4a_image_release(color);
        k4a_image_release(depth);
    }
    return succeeded;
}

int prepare_cond_next_valid_frame(recording_t* file, uint64_t master_timestamp, uint64_t max_delay) {
    /// <summary>
    /// returns -1 if there was a problem, 1 if satisfied condition, 2 if we need to update master frame
    /// </summary>
    /// <param name="file"></param>
    /// <param name="master_timestamp"></param>
    /// <param name="max_delay"></param>
    /// <returns></returns>
    bool satisfies_condition = false;
    //check if there is a current frame that already satisfies the condition
    if (file->capture != NULL && (file->current_capture_timestamp > master_timestamp)) {
        if (file->current_capture_timestamp < (master_timestamp + max_delay)) //satisfies
        {
            return 1;
        }
        else { //update master
            return 2;
        }
        
    }
    //otherwise start process to find a frame that satisfies the condition.
    while (!satisfies_condition)
    {
        bool ok = prepare_next_valid_frame(file);
        if (!ok) break;
        if (file->current_capture_timestamp > master_timestamp) {
            if (file->current_capture_timestamp < (master_timestamp + max_delay)) {
                satisfies_condition = true;
                return 1;
            }
            else {  //it is a future frame, we need to update master frame
                return 2;
            }
        }
    }
    return -1;
}

int main(int argc, char* argv[])
{
    /*
    * //for external debugging
    char c;
    std::cin >> c;*/


    bool ok;

	if (argc < 3)
	{
        std::cerr << "Usage: cwipc_k4aoffline_check <master.mkv> <sub1.mkv> ..." << std::endl;
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
		std::cerr << "Failed to allocate memory for playback (" << sizeof(recording_t) * file_count << " bytes)" << std::endl;
		return 1;
	}
	memset(files, 0, sizeof(recording_t) * file_count);

    // Open each recording file and validate they were recorded in master/subordinate mode.
    for (size_t i = 0; i < file_count; i++)
    {
        files[i].filename = argv[i + 1];
        
        /*char rr;
        size_t vs*/;
        //k4a_buffer_result_t = k4a_playback_get_tag(files[i].handle, "K4A_COLOR_FIRMWARE_VERSION", &rr, &vs);

        result = k4a_playback_open(files[i].filename, &files[i].handle);

        if (result != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to open file: " << files[i].filename << std::endl;
            break;
        }

        result = k4a_playback_get_record_configuration(files[i].handle, &files[i].record_config);
        if (result != K4A_RESULT_SUCCEEDED)
        {
            std::cerr << "Failed to get record configuration for file:" << files[i].filename << std::endl;
            break;
        }

        if (files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER)
        {
            std::cout << "Opened master recording file: " << files[i].filename << std::endl;
            if (master_found)
            {
                std::cerr << "ERROR: Multiple master recordings listed!" << std::endl;
                result = K4A_RESULT_FAILED;
                break;
            }
            else
            {
                master_found = true;
                master_id = i;
            }
        }
        else if (files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE)
        {
            std::cout << "Opened subordinate recording file: " << files[i].filename << std::endl;
        }
        else
        {
            std::cerr << "ERROR: Recording file was not recorded in master/sub mode: " << files[i].filename << std::endl;
            result = K4A_RESULT_FAILED;
            break;
        }
    }

    if ((result == K4A_RESULT_SUCCEEDED) && master_found)
    {
        std::cout << "Loading succeeded" << std::endl;
        int out_frames = 0;
        int skipped_frames = 0;
        first = true;
        while (!eof) {
            //get next valid frame for all cameras
            ok = prepare_next_valid_frame(&files[master_id]);
            if (eof) {
                std::cout << "Last timestamp from " << files[master_id].filename << " = " << files[master_id].current_capture_timestamp << " | capture " << files[master_id].capture_id << std::endl;
                break;
            }
            if (!ok) return -1;
            bool update_master = false;
            for (size_t i = 0; i < file_count; i++)
            {
                if (i == master_id) continue; //we already read master frame
                int res = prepare_cond_next_valid_frame(&files[i], files[master_id].current_capture_timestamp, max_delay_cond);
                if (res == -1) { //errors
                    if (eof) {
                        std::cout << "Last timestamp from " << files[i].filename << " = " << files[i].current_capture_timestamp << " | capture " << files[i].capture_id << std::endl;
                        break;
                    }
                    else {
                        std::cerr << "errors " << i << std::endl;
                        return -1;
                    }
                }else if (res == 2){ //we need to update master
                    std::cerr << "Updating master | stats: " << files[i].filename << "\tcapture " << files[i].capture_id <<"\tt=" << files[i].current_capture_timestamp << "\t| master_t=" << files[master_id].current_capture_timestamp << std::endl;
                    update_master = true;
                    break;
                }
            }
            if (eof)
                break;
            if (update_master) {
                skipped_frames++;
                continue;
            }
            if (first) { //to start counting output/skipped frames from the first synced frame.
                out_frames = 0;
                skipped_frames = 0;
                first = false;
            }
            out_frames++;
            /*std::cout << "succeeded" << std::endl; 
            for (size_t i = 0; i < file_count; i++)
            {
                std::cout << "\t" << files[i].filename << "\tt=" << files[i].current_capture_timestamp << std::endl;
            }*/


            // At this point all the current captures correspond to the same timestamp
            // Now we can generate the pointclouds





            ////REMEMBER to release captures
            //k4a_capture_release(files[0].capture);
            //files[0].capture = NULL;
        }

        std::cout << "________________________________________________________________" << std::endl;
        std::cout << "RESULT : Succeded frames = " << out_frames << "\t| Skipped frames = " << skipped_frames << std::endl;
    }
    else 
    {
        if (master_found) {
            std::cerr << "Loading failed" << std::endl;
        }
        else {
            std::cerr << "Loading failed -> Master not found" << std::endl;
        }
    }

    for (size_t i = 0; i < file_count; i++)
    {
        if (files[i].handle != NULL)
        {
            k4a_playback_close(files[i].handle);
            files[i].handle = NULL;
        }
    }
    free(files);
    return result == K4A_RESULT_SUCCEEDED ? 0 : 1;
}

