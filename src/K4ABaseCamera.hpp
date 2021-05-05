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
	K4ACameraData& camData;
	bool want_auxdata_skeleton = false;	//<! True if caller wants skeleton auxdata
	std::vector<k4abt_skeleton_t> skeletons; //<! Skeletons extracted using the body tracking sdk
	K4ACameraConfig& camSettings;
	cwipc_pcl_pointcloud current_pointcloud = nullptr;
	k4a_transformation_t transformation_handle = nullptr;
	moodycamel::BlockingReaderWriterQueue<k4a_capture_t> captured_frame_queue;
	moodycamel::BlockingReaderWriterQueue<k4a_capture_t> processing_frame_queue;
	k4a_capture_t current_frameset = nullptr;
	int color_height;
	int depth_height;
	int camera_fps;
	bool camera_sync_ismaster;
	bool camera_sync_inuse;
	bool do_greenscreen_removal;
	bool do_height_filtering;
	double height_min;
	double height_max;
	std::mutex processing_mutex;
	std::condition_variable processing_done_cv;
	bool processing_done = false;

	k4abt_tracker_t tracker_handle;
	k4a_calibration_t sensor_calibration;
k4a_calibration_extrinsics_t depth_to_color_extrinsics; public:
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