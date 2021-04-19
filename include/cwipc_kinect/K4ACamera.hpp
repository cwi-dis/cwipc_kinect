#ifndef cwipc_realsense_K4ACamera_hpp
#define cwipc_realsense_K4ACamera_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <k4a/k4a.h>

#include "defs.h"
#include "readerwriterqueue.h"

class K4ACamera {
private:
	K4ACamera(const K4ACamera&);	// Disable copy constructor
	K4ACamera& operator=(const K4ACamera&);	// Disable assignment
public:
	K4ACamera(k4a_device_t _handle, K4ACaptureConfig& configuration, int _camera_index, K4ACameraData& _camData);
	virtual ~K4ACamera();

	bool start();
	virtual void start_capturer();
	void stop();
	bool capture_frameset();
	void create_pc_from_frames();
	void wait_for_pc();
	void dump_color_frame(const std::string& filename);
	uint64_t get_capture_timestamp();
	bool is_sync_master() { return camera_sync_ismaster;  }
public:
	float pointSize;
public:
	// These are public because pcl_align wants to access them
	double minx;
	double minz;
	double maxz;
	k4a_device_t device_handle;
	int camera_index;
	std::string serial;

protected:
	bool stopped;
	bool camera_started;
	bool capture_started;
	std::thread *processing_thread;
	void _filter_depth_data(int16_t* depth_values, int width, int height); // Internal: depth data processing
	void _computePointSize(/*rs2::pipeline_profile profile*/);
	void _processing_thread_main();
	bool generate_point_cloud_color_to_depth(k4a_transformation_t transformation_handle, const k4a_image_t depth_image, const k4a_image_t color_image);
	bool generate_point_cloud_depth_to_color(k4a_transformation_t transformation_handle, const k4a_image_t depth_image, const k4a_image_t color_image);
	void generate_point_cloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image);
	virtual void _start_capture_thread();
	virtual void _capture_thread_main();
	void transformPoint(cwipc_pcl_point& pt);
private:
	K4ACameraData& camData;
	K4ACameraSettings& camSettings;
	k4a_transformation_t transformation_handle;
	moodycamel::BlockingReaderWriterQueue<k4a_capture_t> captured_frame_queue;
	moodycamel::BlockingReaderWriterQueue<k4a_capture_t> processing_frame_queue;
	k4a_capture_t current_frameset;
	int color_height;
	int depth_height;
	int camera_fps;
	bool camera_sync_ismaster;
	bool camera_sync_inuse;
	bool do_depth_filtering;
	bool do_background_removal;
	bool do_greenscreen_removal;
	bool do_height_filtering;
	double height_min;
	double height_max;

	std::thread *grabber_thread;
	std::mutex processing_mutex;
	std::condition_variable processing_done_cv;
	bool processing_done;

	void _init_filters();

};
#endif // cwipc_realsense_K4ACamera_hpp
