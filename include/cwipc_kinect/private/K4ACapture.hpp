#ifndef cwipc_realsense_MFCapture_hpp
#define cwipc_realsense_MFCapture_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <k4a/k4a.h>

#include "cwipc_kinect/private/K4AConfig.hpp"
#include "cwipc_kinect/private/K4ACamera.hpp"


class K4ACapture {
	typedef k4a_device_t Type_api_camera;
	typedef K4ACamera Type_our_camera;
	const std::string CLASSNAME = "cwipc_kinect: K4ACapture";
protected:
	K4ACapture(int dummy);
public:
	// methods
	K4ACapture(const char *configFilename=NULL);
private:
	bool _init_config_from_devices(std::vector<Type_api_camera>& camera_handles, std::vector<std::string>& serials); // Get initial configuration from attached hardware devices.
	bool _init_config_from_configfile(const char *configFilename); // Get configuration from configfile.
	void _update_config_from_devices(); // update config to match attached hardware
	void _init_hardware_settings(std::vector<Type_api_camera>& camera_handles); // initialize hardware parameters from configuration
	void _create_cameras(std::vector<Type_api_camera>& camera_handles, std::vector<std::string>& serials);
	void _init_camera_positions(); // Compute camera positions
	void _start_cameras(); // Start camera hardware and per-camera threads
	bool _capture_all_cameras();
	uint64_t _get_best_timestamp();
public:
	virtual ~K4ACapture();
	cwipc* get_pointcloud(); // API function that returns the merged pointcloud and timestamp
	bool pointcloud_available(bool wait);					  // Returns true if a pointcloud is available
	cwipc* get_mostRecentPointCloud();                     // return the merged cloud most recently captured/merged (don't grab a new one)
	K4ACameraData& get_camera_data(std::string serial);
	Type_our_camera* get_camera(std::string serial);
	float get_pointSize();

	// variables
    K4ACaptureConfig configuration;
	uint64_t starttime;
	int numberOfPCsProduced;
    bool no_cameras;                        // True of no cameras attached
	bool eof = false;                 
	void request_image_auxdata(bool _rgb, bool _depth) {
		want_auxdata_rgb = _rgb;
		want_auxdata_depth = _depth;
	}
protected:
	bool want_auxdata_rgb;
	bool want_auxdata_depth;
	std::vector<Type_our_camera*> cameras;                // Storage of camera specifics
	void _control_thread_main();              // Internal: main thread that controls per-camera grabbing and processing and combines pointclouds.
	bool stopped;
	std::thread *control_thread;

private:
	void merge_views();                       // Internal: merge all camera's pointclouds into one
	void _request_new_pointcloud();           // Internal: request a new pointcloud to be grabbed and processed
	cwipc* mergedPC;                            // Merged pointcloud
	std::mutex mergedPC_mutex;                                // Lock for all mergedPC-related dta structures
	bool mergedPC_is_fresh;                                   // True if mergedPC contains a freshly-created pointcloud
	std::condition_variable mergedPC_is_fresh_cv;             // Condition variable for signalling freshly-created pointcloud
	bool mergedPC_want_new;                                   // Set to true to request a new pointcloud
	std::condition_variable mergedPC_want_new_cv;             // Condition variable for signalling we want a new pointcloud
};
#endif // cwipc_realsense_MFCapture_hpp
