#ifndef cwipc_realsense_K4AOfflineCamera_hpp
#define cwipc_realsense_K4AOfflineCamera_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <k4a/k4a.h>
#include <k4arecord/playback.h>

#include "cwipc_kinect/private/K4AConfig.hpp"
#include "cwipc_kinect/private/readerwriterqueue.h"

class K4AOfflineCamera {
	typedef k4a_playback_t Type_api_camera;
	const std::string CLASSNAME = "cwipc_kinect: K4AOfflineCamera";
private:
	K4AOfflineCamera(const K4AOfflineCamera&);	// Disable copy constructor
	K4AOfflineCamera& operator=(const K4AOfflineCamera&);	// Disable assignment
public:
	K4AOfflineCamera(Type_api_camera _handle, K4ACaptureConfig& configuration, int _camera_index, K4ACameraData& _camData);
	virtual ~K4AOfflineCamera();

	bool start();
	virtual void start_capturer();
	void stop();
	bool capture_frameset(uint64_t master_timestamp);
	void create_pc_from_frames();
	void wait_for_pc();
	void save_auxdata(cwipc* pc, bool rgb, bool depth);
	uint64_t get_capture_timestamp();
	cwipc_pcl_pointcloud get_current_pointcloud() { return current_pointcloud; }
	bool is_sync_master() { return camera_sync_ismaster; }
public:
	float pointSize;
public:
	// These are public because pcl_align wants to access them
	double minx;
	double minz;
	double maxz;
	Type_api_camera camera_handle;
	int capture_id = 0;
	uint64_t current_frameset_timestamp;
	int camera_index;
	std::string serial;
	// xxxjack unused? std::string filename;
	bool eof = false;

protected:
	bool stopped;
	bool camera_started;
	// xxxjack unused? bool capture_started;
	std::thread* processing_thread;
	void _filter_depth_data(int16_t* depth_values, int width, int height); // Internal: depth data processing
	void _computePointSize();
	void _processing_thread_main();
	cwipc_pcl_pointcloud generate_point_cloud_color_to_depth(const k4a_image_t depth_image, const k4a_image_t color_image);
	cwipc_pcl_pointcloud generate_point_cloud_depth_to_color(const k4a_image_t depth_image, const k4a_image_t color_image);
	cwipc_pcl_pointcloud generate_point_cloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image);
	virtual void _start_capture_thread();
	virtual void _capture_thread_main();
	void transformPoint(cwipc_pcl_point& pt);
	bool _prepare_next_valid_frame();
	bool _prepare_cond_next_valid_frame(uint64_t master_timestamp);
	k4a_image_t _uncompress_color_image(k4a_image_t color_image);
private:
	K4ACameraData& camData;
	K4ACameraConfig& camSettings;
	cwipc_pcl_pointcloud current_pointcloud;
	k4a_transformation_t transformation_handle;
	moodycamel::BlockingReaderWriterQueue<k4a_capture_t> captured_frame_queue;
	moodycamel::BlockingReaderWriterQueue<k4a_capture_t> processing_frame_queue;
	k4a_capture_t current_frameset;
	int color_height;
	int depth_height;
	int camera_fps;
	bool camera_sync_ismaster;
	bool camera_sync_inuse;
	bool do_greenscreen_removal;
	bool do_height_filtering;
	double height_min;
	double height_max;
	uint64_t max_delay;

	std::mutex processing_mutex;
	std::condition_variable processing_done_cv;
	bool processing_done;

	void _init_filters();

};
#endif // cwipc_realsense_K4AOfflineCamera_hpp
