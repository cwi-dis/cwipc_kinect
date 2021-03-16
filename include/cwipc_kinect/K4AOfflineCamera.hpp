#ifndef cwipc_realsense_K4AOfflineCamera_hpp
#define cwipc_realsense_K4AOfflineCamera_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <k4a/k4a.h>
#include <k4arecord/playback.h>

#include "defs.h"
#include "cwipc_kinect/K4ACamera.hpp"

class K4AOfflineCamera : public K4ACamera {
private:
	K4AOfflineCamera(const K4AOfflineCamera&);	// Disable copy constructor
	K4AOfflineCamera& operator=(const K4AOfflineCamera&);	// Disable assignment
public:
	K4AOfflineCamera(k4a_playback_t _handle, K4ACaptureConfig& configuration, int _camera_index, K4ACameraData& _camData);
	~K4AOfflineCamera();

	void _start_capture_thread();
	void _capture_thread_main();
	bool feed_image_data(int frameNum, void *colorBuffer, size_t colorSize, void *depthBuffer, size_t depthSize);
private:
	int depth_width, depth_height, depth_bpp, depth_fps;
	rs2_format depth_format;
	int color_width, color_height, color_bpp, color_fps;
	rs2_format color_format;
	rs2_extrinsics depth_to_color_extrinsics;
	rs2::frameset current_frameset;
	rs2::software_device dev;
	rs2::software_sensor depth_sensor;
	rs2::software_sensor color_sensor;
	rs2::stream_profile color_stream;
	rs2::stream_profile depth_stream;
	rs2::syncer sync;
	int feedFrameNum;
};
#endif // cwipc_realsense_RS2OfflineCamera_hpp
