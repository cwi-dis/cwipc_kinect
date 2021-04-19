#ifndef cwipc_realsense_MFCapture_hpp
#define cwipc_realsense_MFCapture_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <k4a/k4a.h>

#include "cwipc_kinect/private/defs.h"


class K4ACamera;

class K4ACapture {
protected:
	K4ACapture(int dummy);
public:
	// methods
	K4ACapture(const char *configFilename=NULL);
	virtual ~K4ACapture();
	cwipc_pcl_pointcloud get_pointcloud(uint64_t *timestamp); // API function that returns the merged pointcloud and timestamp
	bool pointcloud_available(bool wait);					  // Returns true if a pointcloud is available
	cwipc_pcl_pointcloud get_mostRecentPointCloud();                     // return the merged cloud most recently captured/merged (don't grab a new one)
	K4ACameraData& get_camera_data(std::string serial);
	K4ACamera* get_camera(std::string serial);
	float get_pointSize();

	// variables
    K4ACaptureConfig configuration;
	uint64_t starttime;
	int numberOfPCsProduced;
    bool no_cameras;                        // True of no cameras attached
protected:
	virtual void _create_cameras(k4a_device_t* cameras, std::vector<std::string> serials, uint32_t camera_count);
	std::vector<K4ACamera*> cameras;                // Storage of camera specifics
	void _control_thread_main();              // Internal: main thread that controls per-camera grabbing and processing and combines pointclouds.
	bool stopped;
	std::thread *control_thread;

private:
	void merge_views();                       // Internal: merge all camera's pointclouds into one
	void _request_new_pointcloud();           // Internal: request a new pointcloud to be grabbed and processed
	cwipc_pcl_pointcloud mergedPC;                            // Merged pointcloud
	std::mutex mergedPC_mutex;                                // Lock for all mergedPC-related dta structures
	bool mergedPC_is_fresh;                                   // True if mergedPC contains a freshly-created pointcloud
	std::condition_variable mergedPC_is_fresh_cv;             // Condition variable for signalling freshly-created pointcloud
	bool mergedPC_want_new;                                   // Set to true to request a new pointcloud
	std::condition_variable mergedPC_want_new_cv;             // Condition variable for signalling we want a new pointcloud
};
#endif // cwipc_realsense_MFCapture_hpp
