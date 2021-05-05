#ifndef cwipc_realsense_K4ABaseCamera_hpp
#define cwipc_realsense_K4ABaseCamera_hpp
#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <k4abt.h>

#include "K4AConfig.hpp"
#include "readerwriterqueue.h"


template<typename Type_api_camera>
class K4ABaseCamera {
public:
	float pointSize = 0;	//<! (Approximate) 3D cellsize of pointclouds captured by this camera
	std::string serial; //<! Serial number for this camera
	bool eof = false;	//<! True when end of file reached on this camera stream
protected:
	std::string CLASSNAME;
	Type_api_camera camera_handle;
//	double minx = 0;
//	double minz = 0;
//	double maxz = 0;
	int camera_index;	//<! For messages only
	bool stopped = true;	//<! True when stopping
	bool camera_started = false;	//<! True when camera hardware is grabbing
	std::thread* processing_thread = nullptr;	//<! Handle for thread that runs processing loop
	std::thread* grabber_thread = nullptr;	//<! Handle for thread that rungs grabber (if applicable)
	K4ACameraData& camData;	//<! Per-camera data for this camera
	bool want_auxdata_skeleton = false;	//<! True if caller wants skeleton auxdata
	std::vector<k4abt_skeleton_t> skeletons; //<! Skeletons extracted using the body tracking sdk
	K4ACameraConfig& camSettings;	//<! Settings for all cameras
	cwipc_pcl_pointcloud current_pointcloud = nullptr;	//<! Most recent grabbed pointcloud
	k4a_transformation_t transformation_handle = nullptr;	//<! k4a structure describing relationship between RGB and D cameras
	moodycamel::BlockingReaderWriterQueue<k4a_capture_t> captured_frame_queue;	//<! Frames from capture-thread, waiting to be inter-camera synchronized
	moodycamel::BlockingReaderWriterQueue<k4a_capture_t> processing_frame_queue;	//<! Synchronized frames, waiting for processing thread
	k4a_capture_t current_frameset = nullptr;	//<! Current frame being moved from captured_frame_queue to processing_frame_queue
	int color_height;	//<! Parameter from camData
	int depth_height;	//<! Parameter from camData
	int camera_fps;	//<! Parameter from camData
	bool camera_sync_ismaster;	//<! Parameter from camData
	bool camera_sync_inuse;	//<! Parameter from camData
	bool do_greenscreen_removal;	//<! Parameter from camData
	bool do_height_filtering;	//<! Parameter from camData
	double height_min;	//<! Parameter from camData
	double height_max;	//<! Parameter from camData
	std::mutex processing_mutex;	//<! Exclusive lock for frame to pointcloud processing.
	std::condition_variable processing_done_cv;	//<! Condition variable signalling pointcloud ready
	bool processing_done = false;	//<! Boolean for processing_done_cv

	k4abt_tracker_t tracker_handle = nullptr;	//<! Handle to k4abt skeleton tracker
	k4a_calibration_t sensor_calibration;	//<! k4a calibration data read from hardware camera or recording
	k4a_calibration_extrinsics_t depth_to_color_extrinsics;	//<! k4a calibration data read from hardware camera or recording
public:
	K4ABaseCamera(const std::string& _Classname, Type_api_camera _handle, K4ACaptureConfig& configuration, int _camera_index, K4ACameraData& _camData)
	:	CLASSNAME(_Classname),
		camera_handle(_handle),
		camData(_camData),
		camSettings(configuration.camera_config),
		camera_index(_camera_index),
		serial(_camData.serial),
		captured_frame_queue(1),
		processing_frame_queue(1),
		color_height(configuration.color_height),
		depth_height(configuration.depth_height),
		camera_fps(configuration.fps),
		camera_sync_ismaster(serial == configuration.sync_master_serial),
		camera_sync_inuse(configuration.sync_master_serial != ""),
		do_greenscreen_removal(configuration.greenscreen_removal),
		do_height_filtering(configuration.height_min != configuration.height_max),
		height_min(configuration.height_min),
		height_max(configuration.height_max)
	{

	}

};
#endif // cwipc_realsense_K4ABaseCamera_hpp