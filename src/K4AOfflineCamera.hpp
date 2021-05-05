#ifndef cwipc_realsense_K4AOfflineCamera_hpp
#define cwipc_realsense_K4AOfflineCamera_hpp
#pragma once


#include "K4ABaseCamera.hpp"

class K4AOfflineCamera : public K4ABaseCamera<k4a_playback_t> {
	typedef k4a_playback_t Type_api_camera;
	const std::string CLASSNAME = "cwipc_kinect: K4AOfflineCamera";
public:
	uint64_t current_frameset_timestamp = 0; //!< Unsure? How is this different from get_capture_timestamp()?

private:
	K4AOfflineCamera(const K4AOfflineCamera&);	// Disable copy constructor
	K4AOfflineCamera& operator=(const K4AOfflineCamera&);	// Disable assignment
public:
	K4AOfflineCamera(Type_api_camera _handle, K4ACaptureConfig& configuration, int _camera_index, K4ACameraData& _camData);
	virtual ~K4AOfflineCamera();

	void request_skeleton_auxdata(bool _skl) {
		want_auxdata_skeleton = _skl;
	}
	bool start();
	virtual void start_capturer();
	void stop();

	bool is_sync_master() { return camera_sync_ismaster; }

	void create_pc_from_frames();
	void wait_for_pc();

	uint64_t get_capture_timestamp();
	cwipc_pcl_pointcloud get_current_pointcloud() { return current_pointcloud; }
	void save_auxdata_images(cwipc* pc, bool rgb, bool depth);
	void save_auxdata_skeleton(cwipc* pc);
public:
	bool capture_frameset(uint64_t master_timestamp);

protected:
	void _init_filters();
	void _init_tracker();

	virtual void _start_capture_thread();
	virtual void _capture_thread_main();
	void _processing_thread_main();

	void _filter_depth_data(int16_t* depth_values, int width, int height); // Internal: depth data processing
	void _computePointSize();
	cwipc_pcl_pointcloud generate_point_cloud_color_to_depth(const k4a_image_t depth_image, const k4a_image_t color_image);
	cwipc_pcl_pointcloud generate_point_cloud_depth_to_color(const k4a_image_t depth_image, const k4a_image_t color_image);
	cwipc_pcl_pointcloud generate_point_cloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image);
	void transformPoint(cwipc_pcl_point& pt);
	void transformDepthToColorPoint(cwipc_pcl_point& pt);

private:
	bool _prepare_next_valid_frame();
	bool _prepare_cond_next_valid_frame(uint64_t master_timestamp);
	k4a_image_t _uncompress_color_image(k4a_image_t color_image);

private:
	uint64_t max_delay = 0;
	int capture_id = 0;


};
#endif // cwipc_realsense_K4AOfflineCamera_hpp
