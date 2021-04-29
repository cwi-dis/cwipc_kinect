#ifndef cwipc_realsense_K4AOfflineCamera_hpp
#define cwipc_realsense_K4AOfflineCamera_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <k4a/k4a.h>
#include <k4arecord/playback.h>

#include "cwipc_kinect/private/K4AConfig.hpp"
#include "readerwriterqueue.h"

typedef struct
{
	char* filename;
	k4a_playback_t handle;
	k4a_record_configuration_t record_config;
} recording_t;

class K4AOfflineCamera {
private:
	K4AOfflineCamera(const K4AOfflineCamera&);	// Disable copy constructor
	K4AOfflineCamera& operator=(const K4AOfflineCamera&);	// Disable assignment
public:
	K4AOfflineCamera(recording_t _recording, K4ACaptureConfig& configuration, int _camera_index);
	virtual ~K4AOfflineCamera();

	//bool start(); playbacks do not need a start function
	virtual void start_capturer();
	void stop();
	bool capture_frameset(uint64_t master_timestamp);
	void create_pc_from_frames();
	void wait_for_pc();
	void save_auxdata(cwipc* pc, bool rgb, bool depth);
	void dump_color_frame(const std::string& filename);
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
	k4a_playback_t playback_handle;
	// xxxjack unused? k4a_record_configuration_t record_config;
	int capture_id = 0;
	uint64_t current_frameset_timestamp;
	int camera_index;
	std::string serial;
	std::string filename;
	bool eof = false;

protected:
	bool stopped;
	bool camera_started;
	bool capture_started;
	std::thread* processing_thread;
	void _filter_depth_data(int16_t* depth_values, int width, int height); // Internal: depth data processing
	void _computePointSize();
	void _processing_thread_main();
	cwipc_pcl_pointcloud generate_point_cloud_color_to_depth(const k4a_image_t depth_image, const k4a_image_t color_image);
	cwipc_pcl_pointcloud generate_point_cloud_depth_to_color(const k4a_image_t depth_image, const k4a_image_t color_image);
	cwipc_pcl_pointcloud generate_point_cloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image);
	void transformPoint(cwipc_pcl_point& pt);
	bool prepare_next_valid_frame();
	bool prepare_cond_next_valid_frame(uint64_t master_timestamp);
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
	// xxxjack unused? bool do_depth_filtering;
	// xxxjack unused? bool do_background_removal;
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
