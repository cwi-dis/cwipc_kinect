#ifndef cwipc_realsense_K4ACamera_hpp
#define cwipc_realsense_K4ACamera_hpp
#pragma once

#include "K4ABaseCamera.hpp"

class K4ACamera : public K4ABaseCamera<k4a_device_t> {
	typedef k4a_device_t Type_api_camera;
	//const std::string CLASSNAME = "cwipc_kinect: K4ACamera";
private:
	K4ACamera(const K4ACamera&);	// Disable copy constructor
	K4ACamera& operator=(const K4ACamera&);	// Disable assignment
public:
	K4ACamera(Type_api_camera _handle, K4ACaptureConfig& configuration, int _camera_index, K4ACameraData& _camData);
	virtual ~K4ACamera();

	bool start();
	virtual void start_capturer();
	void stop();
	bool capture_frameset();
	void create_pc_from_frames();
	void wait_for_pc();
	void save_auxdata_images(cwipc* pc, bool rgb, bool depth);
	void save_auxdata_skeleton(cwipc* pc);
	uint64_t get_capture_timestamp();
	cwipc_pcl_pointcloud get_current_pointcloud() { return current_pointcloud; }
	bool is_sync_master() { return camera_sync_ismaster; }
public:
	// These are public because pcl_align wants to access them
	void request_skeleton_auxdata(bool _skl) {
		want_auxdata_skeleton = _skl;
	}

protected:
	void _filter_depth_data(int16_t* depth_values, int width, int height); // Internal: depth data processing
	void _computePointSize();
	void _processing_thread_main();
	cwipc_pcl_pointcloud generate_point_cloud_color_to_depth(const k4a_image_t depth_image, const k4a_image_t color_image);
	cwipc_pcl_pointcloud generate_point_cloud_depth_to_color(const k4a_image_t depth_image, const k4a_image_t color_image);
	cwipc_pcl_pointcloud generate_point_cloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image);
	virtual void _start_capture_thread();
	virtual void _capture_thread_main();
	void transformPoint(cwipc_pcl_point& pt);
	void transformDepthToColorPoint(cwipc_pcl_point& pt);
	bool _setup_device(k4a_device_configuration_t& device_config);
private:


	void _init_filters();
	void _init_tracker();
};
#endif // cwipc_realsense_K4ACamera_hpp
