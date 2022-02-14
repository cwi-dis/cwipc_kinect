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

#include "cwipc_util/api_pcl.h"
#include "cwipc_kinect/api.h"
#include "K4AConfig.hpp"
#include "readerwriterqueue.h"

// Check which K4ABT version we have
# if K4ABT_VERSION_MAJOR >= 1 && K4ABT_VERSION_MINOR >= 1
#define K4ABT_SUPPORTS_MODEL_PATH
#endif

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD
#undef CWIPC_MEMORY_DEBUG

#ifdef CWIPC_MEMORY_DEBUG
#include <vld.h>
#endif

typedef struct HsvColor
{
	unsigned char h;
	unsigned char s;
	unsigned char v;
} HsvColor;


inline HsvColor rgbToHsv(cwipc_pcl_point* pnt)
{
	HsvColor hsv;
	unsigned char rgbMin, rgbMax;

	rgbMin = pnt->r < pnt->g ? (pnt->r < pnt->b ? pnt->r : pnt->b) : (pnt->g < pnt->b ? pnt->g : pnt->b);
	rgbMax = pnt->r > pnt->g ? (pnt->r > pnt->b ? pnt->r : pnt->b) : (pnt->g > pnt->b ? pnt->g : pnt->b);

	hsv.v = rgbMax;
	if (hsv.v == 0)
	{
		hsv.h = 0;
		hsv.s = 0;
		return hsv;
	}

	hsv.s = 255 * ((long)(rgbMax - rgbMin)) / hsv.v;
	if (hsv.s == 0)
	{
		hsv.h = 0;
		return hsv;
	}

	if (rgbMax == pnt->r)
		hsv.h = 0 + 43 * (pnt->g - pnt->b) / (rgbMax - rgbMin);
	else if (rgbMax == pnt->g)
		hsv.h = 85 + 43 * (pnt->b - pnt->r) / (rgbMax - rgbMin);
	else
		hsv.h = 171 + 43 * (pnt->r - pnt->g) / (rgbMax - rgbMin);

	return hsv;
}

inline bool isNotGreen(cwipc_pcl_point* p)
{
	HsvColor hsv = rgbToHsv(p);

	if (hsv.h >= 60 && hsv.h <= 130) {
		if (hsv.s >= 0.15 && hsv.v >= 0.15) {
			// reducegreen
			if ((p->r * p->b) != 0 && (p->g * p->g) / (p->r * p->b) > 1.5) {
				p->r *= 1.4;
				p->b *= 1.4;
			}
			else {
				p->r *= 1.2;
				p->b *= 1.2;
			}
		}
		return !(hsv.s >= 0.4 && hsv.v >= 0.3);
	}
	return true;
}

inline bool isPointInRadius(cwipc_pcl_point& pt, float radius_filter) {
	float distance_2 = pow(pt.x, 2) + pow(pt.z, 2);
	return distance_2 < radius_filter * radius_filter; // radius^2 to avoid sqrt
}

template<typename Type_api_camera>
class K4ABaseCamera {
public:
	float pointSize = 0;	//<! (Approximate) 3D cellsize of pointclouds captured by this camera
	std::string serial; //<! Serial number for this camera
	bool eof = false;	//<! True when end of file reached on this camera stream
protected:
	std::string CLASSNAME;
	K4ACaptureConfig& configuration;
	Type_api_camera camera_handle;
	int camera_index;	//<! For messages only
	bool stopped = true;	//<! True when stopping
	bool camera_started = false;	//<! True when camera hardware is grabbing
	std::thread* processing_thread = nullptr;	//<! Handle for thread that runs processing loop
	std::thread* grabber_thread = nullptr;	//<! Handle for thread that rungs grabber (if applicable)
	K4ACameraData& camData;	//<! Per-camera data for this camera
	bool want_auxdata_skeleton = false;	//<! True if caller wants skeleton auxdata
	std::vector<k4abt_skeleton_t> skeletons; //<! Skeletons extracted using the body tracking sdk
	cwipc_pcl_pointcloud current_pointcloud = nullptr;	//<! Most recent grabbed pointcloud
	k4a_transformation_t transformation_handle = nullptr;	//<! k4a structure describing relationship between RGB and D cameras
	moodycamel::BlockingReaderWriterQueue<k4a_capture_t> captured_frame_queue;	//<! Frames from capture-thread, waiting to be inter-camera synchronized
	moodycamel::BlockingReaderWriterQueue<k4a_capture_t> processing_frame_queue;	//<! Synchronized frames, waiting for processing thread
	k4a_capture_t current_frameset = nullptr;	//<! Current frame being moved from captured_frame_queue to processing_frame_queue
	bool camera_sync_ismaster;	//<! Parameter from camData
	bool camera_sync_inuse;	//<! Parameter from camData
	bool do_height_filtering;	//<! Parameter from camData
	std::mutex processing_mutex;	//<! Exclusive lock for frame to pointcloud processing.
	std::condition_variable processing_done_cv;	//<! Condition variable signalling pointcloud ready
	bool processing_done = false;	//<! Boolean for processing_done_cv

	k4abt_tracker_t tracker_handle = nullptr;	//<! Handle to k4abt skeleton tracker
	k4a_calibration_t sensor_calibration;	//<! k4a calibration data read from hardware camera or recording
	k4a_calibration_extrinsics_t depth_to_color_extrinsics;	//<! k4a calibration data read from hardware camera or recording
public:
	K4ABaseCamera(const std::string& _Classname, Type_api_camera _handle, K4ACaptureConfig& _configuration, int _camera_index, K4ACameraData& _camData)
	:	CLASSNAME(_Classname),
		configuration(_configuration),
		camera_handle(_handle),
		camData(_camData),
		camera_index(_camera_index),
		serial(_camData.serial),
		captured_frame_queue(1),
		processing_frame_queue(1),
		camera_sync_ismaster(serial == configuration.sync_master_serial),
		camera_sync_inuse(configuration.sync_master_serial != ""),
		do_height_filtering(configuration.height_min != configuration.height_max)
	{

	}
	
	~K4ABaseCamera()
	{
#ifdef CWIPC_DEBUG
		std::cout << CLASSNAME << ": destroying " << serial << std::endl;
#endif
		assert(stopped);
		if (tracker_handle) {
			k4abt_tracker_shutdown(tracker_handle);
			k4abt_tracker_destroy(tracker_handle);
			tracker_handle = nullptr;
		}
	}


public:
public:
	virtual bool request_skeleton_auxdata(bool _skl) final {
		want_auxdata_skeleton = _skl;
		if (want_auxdata_skeleton) {
			return _init_tracker();
		}
		else {
			return false;
		}
	}

	virtual bool start() = 0;
	virtual void start_capturer() = 0;
	virtual void stop() = 0;

	virtual bool is_sync_master() final { 
		return camera_sync_ismaster;
	}

	virtual void create_pc_from_frames() final {
		assert(current_frameset);
		if (!processing_frame_queue.try_enqueue(current_frameset)) {
			std::cerr << CLASSNAME << ":  camera " << serial << ": drop frame before processing" << std::endl;
			k4a_capture_release(current_frameset);
		}
		current_frameset = NULL;
	}
	
	virtual void wait_for_pc() final {
		std::unique_lock<std::mutex> lock(processing_mutex);
		processing_done_cv.wait(lock, [this]{ return processing_done; });
		processing_done = false;
	}

	virtual uint64_t get_capture_timestamp() final {
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	}
	
	virtual cwipc_pcl_pointcloud get_current_pointcloud() final {
		return current_pointcloud;
	}
	
	virtual void save_auxdata_images(cwipc* pc, bool rgb, bool depth) final {
		if (rgb) {
			std::string name = "rgb." + serial;
			k4a_image_t image = k4a_capture_get_color_image(current_frameset);
			image = _uncompress_color_image(current_frameset, image);
			if (image != NULL) {
				uint8_t* data_pointer = k4a_image_get_buffer(image);
				const size_t size = k4a_image_get_size(image);
				int width = k4a_image_get_width_pixels(image);
				int height = k4a_image_get_height_pixels(image);
				int stride = k4a_image_get_stride_bytes(image);
				int format = k4a_image_get_format(image);
				std::string description =
					"width=" + std::to_string(width) +
					",height=" + std::to_string(height) +
					",stride=" + std::to_string(stride) +
					",format=" + std::to_string(format);
				void* pointer = malloc(size);
				if (pointer) {
					memcpy(pointer, data_pointer, size);
					cwipc_auxiliary_data* ap = pc->access_auxiliary_data();
					ap->_add(name, description, pointer, size, ::free);
				}
			}
			k4a_image_release(image);
		}
		if (depth) {
			std::string name = "depth." + serial;
			k4a_image_t image = k4a_capture_get_depth_image(current_frameset);
			if (image != NULL) {
				uint8_t* data_pointer = k4a_image_get_buffer(image);
				const size_t size = k4a_image_get_size(image);
				int width = k4a_image_get_width_pixels(image);
				int height = k4a_image_get_height_pixels(image);
				int stride = k4a_image_get_stride_bytes(image);
				int format = k4a_image_get_format(image);
				std::string description =
					"width=" + std::to_string(width) +
					",height=" + std::to_string(height) +
					",stride=" + std::to_string(stride) +
					",format=" + std::to_string(format);
				void* pointer = malloc(size);
				if (pointer) {
					memcpy(pointer, data_pointer, size);
					cwipc_auxiliary_data* ap = pc->access_auxiliary_data();
					ap->_add(name, description, pointer, size, ::free);
				}
			}
		}
	}
	
	virtual void save_auxdata_skeleton(cwipc* pc) final {
		int n_skeletons = skeletons.size();
		size_t size_str = sizeof(cwipc_skeleton_collection) + n_skeletons * (int)K4ABT_JOINT_COUNT * sizeof(cwipc_skeleton_joint);
		cwipc_skeleton_collection* skl = (cwipc_skeleton_collection*)malloc(size_str);
		if (skl != NULL) {
			skl->n_skeletons = n_skeletons;
			skl->n_joints = (int)K4ABT_JOINT_COUNT;
			cwipc_skeleton_joint* p = skl->joints;
			for (auto s : skeletons) {
				for (auto j : s.joints) {
					p->confidence = (int)j.confidence_level;
					p->x = j.position.xyz.x;
					p->y = j.position.xyz.y;
					p->z = j.position.xyz.z;
					p->q_w = j.orientation.wxyz.w;
					p->q_x = j.orientation.wxyz.x;
					p->q_y = j.orientation.wxyz.y;
					p->q_z = j.orientation.wxyz.z;
					p++;
				}
			}
			std::string name = "skeleton." + serial;
			cwipc_auxiliary_data* ap = pc->access_auxiliary_data();
			ap->_add(name, "", (void*)skl, size_str, ::free);
		}
	}

protected:
	virtual void _init_filters() final {
	}
	
	virtual bool _init_tracker() final {
		if (tracker_handle != NULL) return true;
		k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
		tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
		if (configuration.bt_processing_mode >= 0) {
			tracker_config.processing_mode = (k4abt_tracker_processing_mode_t)configuration.bt_processing_mode;
		}
		if (configuration.bt_sensor_orientation >= 0) {
			tracker_config.sensor_orientation = (k4abt_sensor_orientation_t)configuration.bt_sensor_orientation;
		}
#ifdef K4ABT_SUPPORTS_MODEL_PATH
		if (configuration.bt_model_path != "") {
			tracker_config.model_path = configuration.bt_model_path.c_str();
		}
#endif
		auto sts = k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker_handle);
		char* processingModesStr[] = { "GPU", "CPU", "GPU_CUDA", "GPU_TENSORRT", "GPU_DIRECTML"	};
		char* sensorOrientationsStr[] = { "Default", "Clockwise_90", "CounterClockwise_90", "Flip_180" };
		if (sts != K4A_RESULT_SUCCEEDED) {
			std::cerr << CLASSNAME << " Body tracker initialization failed on camera " << serial << " processing_mode=" << processingModesStr[tracker_config.processing_mode] << " sensor_orientation=" << sensorOrientationsStr[tracker_config.sensor_orientation] << std::endl;
			return false;
		}
		else {
			std::cout << CLASSNAME << " Body tracker initialization succeeded on camera " << serial << " processing_mode=" << processingModesStr[tracker_config.processing_mode] << " sensor_orientation=" << sensorOrientationsStr[tracker_config.sensor_orientation] << std::endl;
		}
		return true;
	}

#ifdef notrs2
	virtual void _computePointSize(rs2::pipeline_profile profile) final {

		// Get the 3D distance between camera and (0,0,0) or use 1m if unreasonable
		float tx = (*camData.trafo)(0, 3);
		float ty = (*camData.trafo)(1, 3);
		float tz = (*camData.trafo)(2, 3);
		float dist = sqrt(tx * tx + ty * ty + tz * tz);
		if (dist == 0) dist = 1;

		// Now get the intrinsics for the depth stream
		auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
		auto intrinsics = stream.get_intrinsics(); // Calibration data

		// Compute 2D coordinates of adjacent pixels in the middle of the field of view
		float pixel0[2], pixel1[2];
		pixel0[0] = camera_width / 2;
		pixel0[1] = camera_height / 2;
		pixel1[0] = pixel0[0] + 1;
		pixel1[1] = pixel0[1] + 1;

		// Deproject to get 3D distance
		float point0[3], point1[3];
		rs2_deproject_pixel_to_point(point0, &intrinsics, pixel0, dist);
		rs2_deproject_pixel_to_point(point1, &intrinsics, pixel1, dist);
		float rv = sqrt(pow(point1[0] - point0[0], 2) + pow(point1[1] - point0[1], 2) + pow(point1[2] - point0[2], 2));
		pointSize = rv;
	}
#endif

	virtual void _start_capture_thread() = 0;
	virtual void _capture_thread_main() = 0;


	virtual void _processing_thread_main() final {
#ifdef CWIPC_DEBUG_THREAD
		std::cerr << CLASSNAME << ": processing: cam=" << serial << " thread started" << std::endl;
#endif
		while (!stopped) {
			k4a_capture_t processing_frameset = NULL;
			k4a_image_t depth_image = NULL;
			k4a_image_t color_image = NULL;

			bool ok = processing_frame_queue.wait_dequeue_timed(processing_frameset, std::chrono::milliseconds(10000));
			if (processing_frameset == NULL) {
#ifdef CWIPC_DEBUG_THREAD
				std::cerr << CLASSNAME << ": processing thread: null frameset" << std::endl;
#endif
				continue;
			}
			if (!ok) {
				std::cerr << CLASSNAME << ": no frameset for 10 seconds, camera " << serial << std::endl;
				continue;
			}
#ifdef CWIPC_DEBUG_THREAD
			std::cerr << CLASSNAME << ": processing: got frame for camera " << serial << std::endl;
#endif
			assert(processing_frameset);
			std::lock_guard<std::mutex> lock(processing_mutex);
			depth_image = k4a_capture_get_depth_image(processing_frameset);
			color_image = k4a_capture_get_color_image(processing_frameset);
			color_image = _uncompress_color_image(processing_frameset, color_image);

			if (want_auxdata_skeleton && tracker_handle) {
				//
				// Push frameset into the tracker. Wait indefinitely for the result.
				//
				k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker_handle, processing_frameset, K4A_WAIT_INFINITE);
				if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
				{
					// It should never hit timeout when K4A_WAIT_INFINITE is set.
					cwipc_k4a_log_warning("k4abt_tracker_enqueue_capture: timeout");
				}
				else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
				{
					cwipc_k4a_log_warning("k4abt_tracker_enqueue_capture: failed");
				}
				//
				// Now pop the result. Again wait indefinitely.
				//
				k4abt_frame_t body_frame = NULL;
				skeletons.clear();
				k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker_handle, &body_frame, K4A_WAIT_INFINITE);
				if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
				{
					uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
#ifdef CWIPC_DEBUG_THREAD
					std::cout << CLASSNAME << " : " << serial << " - Numbodies = " << num_bodies << std::endl;
#endif
					if (num_bodies > 0) {
						// Transform each 3d joints from 3d depth space to 2d color image space
						for (uint32_t i = 0; i < num_bodies; i++)
						{
							//printf("- Person[%u]:\n", i);
							k4abt_skeleton_t skeleton;
							auto sts = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
							if (sts != K4A_RESULT_SUCCEEDED) {
								cwipc_k4a_log_warning("Get body from body frame failed");
								break;
							}
							for (int joint_id = 0; joint_id < (int)K4ABT_JOINT_COUNT; joint_id++)
							{
								k4a_float3_t::_xyz pos = skeleton.joints[joint_id].position.xyz; //millimiters
								cwipc_pcl_point point;
								point.x = pos.x;
								point.y = pos.y;
								point.z = pos.z;
								if (!configuration.camera_config.map_color_to_depth)
									transformDepthToColorPoint(point);
								transformPoint(point);
								pos.x = point.x;
								pos.y = point.y;
								pos.z = point.z;
								skeleton.joints[joint_id].position.xyz = pos;
								//k4abt_joint_confidence_level_t confidence = skeleton.joints[joint_id].confidence_level;
								//k4a_quaternion_t orientation = skeleton.joints[joint_id].orientation;
								//std::cout << "\tJoint " << joint_id << " : \t(" << pos.x << "," << pos.y << "," << pos.z << ")\t\tconfidence_level = " << confidence << "\t orientation_wxyz: (" << orientation.wxyz.w << "," << orientation.wxyz.x << "," << orientation.wxyz.y << "," << orientation.wxyz.z << ")" << std::endl;
							}
							skeletons.push_back(skeleton);
						}
					}
				}
				else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
				{
					cwipc_k4a_log_warning("k4abt_tracker_pop_result: timeout");
				}
				else
				{
					cwipc_k4a_log_warning("k4abt_tracker_pop_result: failed");
				}
				if (body_frame != nullptr) k4abt_frame_release(body_frame);
			}


			cwipc_pcl_pointcloud new_pointcloud = nullptr;
			if (configuration.camera_config.map_color_to_depth) {
				new_pointcloud = generate_point_cloud_color_to_depth(depth_image, color_image);
			}
			else {
				new_pointcloud = generate_point_cloud_depth_to_color(depth_image, color_image);
			}
			if (new_pointcloud != nullptr) {
				current_pointcloud = new_pointcloud;
#ifdef CWIPC_DEBUG_THREAD
				std::cerr << CLASSNAME << ": camera " << serial << " produced " << current_pointcloud->size() << " point" << std::endl;
#endif
				if (current_pointcloud->size() == 0) {
					std::cerr << CLASSNAME << ": warning: captured empty pointcloud from camera " << camData.serial << std::endl;
					//continue;
				}
				// Notify wait_for_pc that we're done.
				processing_done = true;
				processing_done_cv.notify_one();
			}
			if (depth_image) k4a_image_release(depth_image);
			depth_image = nullptr;
			if (color_image) k4a_image_release(color_image);
			color_image = nullptr;
			if (processing_frameset) k4a_capture_release(processing_frameset);
			processing_frameset = nullptr;
		}
#ifdef CWIPC_DEBUG_THREAD
		std::cerr << CLASSNAME << ": processing: cam=" << serial << " thread stopped" << std::endl;
#endif
	}

	virtual void _filter_depth_data(int16_t* depth_values, int width, int height) final {
		int16_t min_depth = (int16_t)(configuration.camera_config.threshold_near * 1000);
		int16_t max_depth = (int16_t)(configuration.camera_config.threshold_far * 1000);
		int16_t *z_values = (int16_t *)calloc(width * height, sizeof(int16_t));
		// Pass one: Copy Z values to temporary buffer, but leave out-of-range values at zero.
		for (int i = 0; i < width * height; i++)
		{
			int i_pc = i * 3;
			int16_t z = depth_values[i_pc + 2];
			if (configuration.camera_config.do_threshold && (z <= min_depth || z >= max_depth)) continue;
			z_values[i] = z;
		}
		// Pass two: loop for zero pixels in temp buffer, and clear out x/y pixels adjacent in depth buffer
		int x_delta = configuration.camera_config.depth_x_erosion;
		int y_delta = configuration.camera_config.depth_y_erosion;
		if (x_delta || y_delta) {
			for (int x = 0; x < width; x++) {
				for (int y = 0; y < height; y++) {
					if (z_values[x + y * width] != 0) continue;
					// Zero depth at (x, y). Clear out pixels 
					for (int ix = x - x_delta; ix <= x + x_delta; ix++) {
						if (ix < 0 || ix >= width ) continue;
						int i_pc = (ix + y * width) * 3;
						depth_values[i_pc + 2] = 0;
					}
					for (int iy = y - y_delta; iy <= y + y_delta; iy++) {
						if (iy < 0 || iy >= height) continue;
						int i_pc = (x + iy * width) * 3;
						depth_values[i_pc + 2] = 0;
					}
				}
			}
		} else {
			// Pass three: clear out zero pixels from temporary buffer.
			for (int i = 0; i < width * height; i++)
			{
				if (z_values[i] != 0) continue;
				int i_pc = i * 3;
				depth_values[i_pc + 2] = 0;
			}
		}
		free(z_values);
	}
	
	//virtual void _computePointSize() = 0;
	virtual cwipc_pcl_pointcloud generate_point_cloud_color_to_depth(const k4a_image_t depth_image, const k4a_image_t color_image) final {
		int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
		int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
		k4a_image_t transformed_color_image = NULL;
		k4a_result_t status;
		status = k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
			depth_image_width_pixels,
			depth_image_height_pixels,
			depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
			&transformed_color_image);
		if (status != K4A_RESULT_SUCCEEDED)
		{
			std::cerr << "cwipc_kinect: Failed to create transformed color image: " << status << std::endl;
			return nullptr;
		}

		k4a_image_t point_cloud_image = NULL;
		status = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
			depth_image_width_pixels,
			depth_image_height_pixels,
			depth_image_width_pixels * 3 * (int)sizeof(int16_t),
			&point_cloud_image);
		if (status != K4A_RESULT_SUCCEEDED)
		{
			std::cerr << "cwipc_kinect: Failed to create point cloud image: " << status << std::endl;
			return nullptr;
		}

		status = k4a_transformation_color_image_to_depth_camera(transformation_handle,
			depth_image,
			color_image,
			transformed_color_image);
		if (status != K4A_RESULT_SUCCEEDED)
		{
			std::cerr << "cwipc_kinect: Failed to compute transformed color image: " << status  << std::endl;
			return nullptr;
		}

		status = k4a_transformation_depth_image_to_point_cloud(transformation_handle,
			depth_image,
			K4A_CALIBRATION_TYPE_DEPTH,
			point_cloud_image);
		if (status != K4A_RESULT_SUCCEEDED)
		{
			std::cerr << "cwipc_kinect: Failed to compute point cloud: " << status << std::endl;
			return nullptr;
		}

		cwipc_pcl_pointcloud rv = generate_point_cloud(point_cloud_image, transformed_color_image);

		k4a_image_release(transformed_color_image);
		k4a_image_release(point_cloud_image);

		return rv;
	}
	
	virtual cwipc_pcl_pointcloud generate_point_cloud_depth_to_color(const k4a_image_t depth_image, const k4a_image_t color_image) final {
		// transform color image into depth camera geometry
		int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
		int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
		k4a_image_t transformed_depth_image = NULL;
		k4a_result_t status;
		status = k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
			color_image_width_pixels,
			color_image_height_pixels,
			color_image_width_pixels * (int)sizeof(uint16_t),
			&transformed_depth_image);
		if (status != K4A_RESULT_SUCCEEDED)
		{
			std::cerr << "cwipc_kinect: Failed to create transformed depth image: " << status << std::endl;
			return nullptr;
		}

		k4a_image_t point_cloud_image = NULL;
		status = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
			color_image_width_pixels,
			color_image_height_pixels,
			color_image_width_pixels * 3 * (int)sizeof(int16_t),
			&point_cloud_image);
		if (status != K4A_RESULT_SUCCEEDED)
		{
			std::cerr << "cwipc_kinect: Failed to create point cloud image: " << status << std::endl;
			return nullptr;
		}

		status = k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image);
		if (status != K4A_RESULT_SUCCEEDED)
		{
			std::cerr << "cwipc_kinect: Failed to compute transformed depth image: " << status << std::endl;
			return nullptr;
		}

		status = k4a_transformation_depth_image_to_point_cloud(transformation_handle,
			transformed_depth_image,
			K4A_CALIBRATION_TYPE_COLOR,
			point_cloud_image);
		if (status != K4A_RESULT_SUCCEEDED)
		{
			std::cerr << "cwipc_kinect: Failed to compute point cloud: " << status << std::endl;
			return nullptr;
		}

		cwipc_pcl_pointcloud rv = generate_point_cloud(point_cloud_image, color_image);

		k4a_image_release(transformed_depth_image);
		k4a_image_release(point_cloud_image);

		return rv;
	}
	
	virtual cwipc_pcl_pointcloud generate_point_cloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image) final {
		int width = k4a_image_get_width_pixels(point_cloud_image);
		int height = k4a_image_get_height_pixels(color_image);

		uint8_t * color_data = k4a_image_get_buffer(color_image);
		int16_t* point_cloud_image_data = (int16_t*)k4a_image_get_buffer(point_cloud_image);
		if (configuration.camera_config.do_threshold || configuration.camera_config.depth_x_erosion || configuration.camera_config.depth_y_erosion) {
			_filter_depth_data(point_cloud_image_data, width, height);
		}
		// Setup depth filtering, if needed
		// now loop over images and create points.
		cwipc_pcl_pointcloud new_cloud = new_cwipc_pcl_pointcloud();
		new_cloud->clear();
		new_cloud->reserve(width * height);
		for (int i = 0; i < width * height; i++)
		{
			int i_pc = i * 3;
			int i_rgba = i * 4;
			cwipc_pcl_point point;
			int16_t x = point_cloud_image_data[i_pc + 0];
			int16_t y = point_cloud_image_data[i_pc + 1];
			int16_t z = point_cloud_image_data[i_pc + 2];
			if (z == 0) continue;

			// color_data is BGR
			point.r = color_data[i_rgba + 2];
			point.g = color_data[i_rgba + 1];
			point.b = color_data[i_rgba + 0];
			point.a = (uint8_t)1 << camera_index;
			uint8_t alpha = color_data[i_rgba + 3];

			if (point.r == 0 && point.g == 0 && point.b == 0 && alpha == 0) continue;
			point.x = x;
			point.y = y;
			point.z = z; 
			if (configuration.camera_config.map_color_to_depth)
				transformDepthToColorPoint(point);
			transformPoint(point);
			if (configuration.radius_filter > 0.0) { // apply radius filter
				if(!isPointInRadius(point, configuration.radius_filter)) continue;
			}
			if (do_height_filtering && (point.y < configuration.height_min || point.y > configuration.height_max)) continue;
			if (!configuration.greenscreen_removal || isNotGreen(&point)) // chromakey removal
				new_cloud->push_back(point);
		}
#ifdef CWIPC_DEBUG_THREAD
		std::cerr << "cwipc_kinect: camera " << serial << " produced " << camData.cloud->size() << " point" << std::endl;
#endif
		return new_cloud;
	}
	
	virtual void transformPoint(cwipc_pcl_point& pt) final {
		float x = pt.x / 1000.0;
		float y = pt.y / 1000.0;
		float z = pt.z / 1000.0;
		pt.x = (*camData.trafo)(0,0)*x + (*camData.trafo)(0,1)*y + (*camData.trafo)(0,2)*z + (*camData.trafo)(0,3);
		pt.y = (*camData.trafo)(1,0)*x + (*camData.trafo)(1,1)*y + (*camData.trafo)(1,2)*z + (*camData.trafo)(1,3);
		pt.z = (*camData.trafo)(2,0)*x + (*camData.trafo)(2,1)*y + (*camData.trafo)(2,2)*z + (*camData.trafo)(2,3);
	}
	
	virtual void transformDepthToColorPoint(cwipc_pcl_point& pt) final {
		float x = pt.x;
		float y = pt.y;
		float z = pt.z;
		float *rotation = depth_to_color_extrinsics.rotation;
		float *translation = depth_to_color_extrinsics.translation;
		pt.x = rotation[0] * x + rotation[1] * y + rotation[2] * z + translation[0];
		pt.y = rotation[3] * x + rotation[4] * y + rotation[5] * z + translation[1];
		pt.z = rotation[6] * x + rotation[7] * y + rotation[8] * z + translation[2];
	}
	
	virtual k4a_image_t _uncompress_color_image(k4a_capture_t capture, k4a_image_t color_image) = 0;


};
#endif // cwipc_realsense_K4ABaseCamera_hpp