#ifndef cwipc_realsense_K4AOfflineCapture_hpp
#define cwipc_realsense_K4AOfflineCapture_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <k4a/k4a.h>

#include "K4ABaseCapture.hpp"
#include "K4AOfflineCamera.hpp"

class K4AOfflineCapture : public K4ABaseCapture<k4a_playback_t, K4AOfflineCamera> {
	typedef k4a_playback_t Type_api_camera;
	typedef K4AOfflineCamera Type_our_camera;
public:
	// methods
	K4AOfflineCapture(const char* configFilename = NULL);
private:
	bool _init_config_from_configfile(const char *configFilename); // Get configuration from configfile.
	void _create_cameras(std::vector<Type_api_camera>& camera_handles);
	bool _open_recording_files(std::vector<Type_api_camera>& camera_handles); // Open the recordings
	void _init_camera_positions(); // Compute camera positions
	void _start_cameras(); // Start camera hardware and per-camera threads
	bool _capture_all_cameras();
	uint64_t _get_best_timestamp();
public:
	virtual ~K4AOfflineCapture();
	cwipc* get_pointcloud(); // API function that returns the merged pointcloud and timestamp
	bool pointcloud_available(bool wait);					  // Returns true if a pointcloud is available
	cwipc* get_mostRecentPointCloud();          // return the merged cloud most recently captured/merged (don't grab a new one)
	K4ACameraData& get_camera_data(std::string serial);
	Type_our_camera* get_camera(std::string serial);
	float get_pointSize();

	// variables
	bool sync_inuse = false;
	int master_id = -1;
protected:
	void _control_thread_main();              // Internal: main thread that controls per-camera grabbing and processing and combines pointclouds.
	std::thread* control_thread;

private:
	void merge_views();                       // Internal: merge all camera's pointclouds into one
	void _request_new_pointcloud();           // Internal: request a new pointcloud to be grabbed and processed
};
#endif // cwipc_realsense_RS2Offline_hpp
