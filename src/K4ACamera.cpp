//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//
#include <cstdlib>

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD
#undef CWIPC_MEMORY_DEBUG

#ifdef CWIPC_MEMORY_DEBUG
#include <vld.h>
#endif

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_KINECT_EXPORT __declspec(dllexport)
#endif

#include "cwipc_kinect/defs.h"
#include "cwipc_kinect/utils.h"
#include "cwipc_kinect/K4ACamera.hpp"

#ifdef WITH_DUMP_VIDEO_FRAMES
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "cwipc_kinect/stb_image_write.h"
#endif


K4ACamera::K4ACamera(k4a_device_t _handle, K4ACaptureConfig& configuration, int _camera_index, K4ACameraData& _camData)
:	pointSize(0), minx(0), minz(0), maxz(0),
	device_handle(_handle),
	camera_index(_camera_index),
	serial(_camData.serial),
	stopped(true),
	camera_started(false),
	capture_started(false),
	camData(_camData),
	camSettings(configuration.default_camera_settings),
	captured_frame_queue(1),
	processing_frame_queue(1),
	current_frameset(NULL),
	camera_width(configuration.width),
	camera_height(configuration.height),
	camera_fps(configuration.fps),
	do_greenscreen_removal(configuration.greenscreen_removal),
	do_height_filtering(configuration.height_min != configuration.height_max),
	height_min(configuration.height_min),
	height_max(configuration.height_max),
	grabber_thread(nullptr)
{
#ifdef CWIPC_DEBUG
		std::cout << "K4ACapture: creating camera " << serial << std::endl;
#endif
	_init_filters();
}

K4ACamera::~K4ACamera()
{
#ifdef CWIPC_DEBUG
	std::cout << "K4ACamera: destroying " << serial << std::endl;
#endif
	assert(stopped);
}

void K4ACamera::_init_filters()
{
	if (!do_depth_filtering) return;
}

bool K4ACamera::capture_frameset()
{
	if (stopped) return false;
	k4a_capture_t new_frameset = NULL;
	bool rv = captured_frame_queue.wait_dequeue_timed(new_frameset, 5000000);
	if (rv) {
		if (current_frameset) {
			k4a_capture_release(current_frameset);
			std::cerr << "xxxjack release frameset" << std::endl;
		}
		current_frameset = new_frameset;
//#ifdef CWIPC_DEBUG_THREAD
		if (current_frameset == NULL) {
			std::cerr << "cwipc_kinect: K4ACamera: " << camera_index <<" forward NULL frame"  << std::endl;
		} else {
			k4a_image_t color = k4a_capture_get_color_image(current_frameset);
			uint64_t tsRGB = k4a_image_get_device_timestamp_usec(color);
			k4a_image_release(color);
			k4a_image_t depth = k4a_capture_get_depth_image(current_frameset);
			uint64_t tsD = k4a_image_get_device_timestamp_usec(depth);
			k4a_image_release(depth);
			std::cerr << "cwipc_kinect: K4ACamera: forward frame: cam=" << serial << ", rgbseq=" << tsRGB << ", dseq=" << tsD << std::endl;
		}
//#endif
	}

	return rv;
}

// Configure and initialize caputuring of one camera
bool K4ACamera::start()
{
	assert(stopped);
	k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	device_config.color_resolution = K4A_COLOR_RESOLUTION_720P; // xxxjack
	device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	device_config.camera_fps = K4A_FRAMES_PER_SECOND_30; // xxxjack
	device_config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture

	//SYNC:
	bool useSync = true
		; //should be set from the configfile
	bool isMaster = false;
	if (useSync) {
		if (serial == "000241702712") { //TODO: //should be set from the configfile  IMPORTANT: master has to be the last camera to start to achieve sync.
			device_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
			isMaster = true;
		}
		else {
			device_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
			device_config.subordinate_delay_off_master_usec = 160 * camera_index;	//160 allows max 9 cameras
		}
	}

	k4a_calibration_t calibration;
	if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(device_handle, device_config.depth_mode, device_config.color_resolution, &calibration))
	{
		std::cerr << "cwipc_kinect: Failed to k4a_device_get_calibration" << std::endl;
		return false;
	}
	transformation_handle = k4a_transformation_create(&calibration);

	k4a_result_t res = k4a_device_start_cameras(device_handle, &device_config);
	if (res != K4A_RESULT_SUCCEEDED) {
		std::cerr << "cwipc_kinect: failed to start camera " << serial << std::endl;
		return false;
	}
	std::cerr << "cwipc_kinect: starting camera " << camera_index << " with serial="<< serial << ". Res=(" << camera_width << "x" << camera_height << ") @" << camera_fps << "fps as " << (useSync? (isMaster? "Master" : "Subordinate") : "Standalone") << std::endl;
	
	camera_started = true;
	return true;
}

#ifdef notrs2
void K4ACamera::_computePointSize(rs2::pipeline_profile profile)
{

	// Get the 3D distance between camera and (0,0,0) or use 1m if unreasonable
	float tx = (*camData.trafo)(0,3);
	float ty = (*camData.trafo)(1,3);
	float tz = (*camData.trafo)(2,3);
	float dist = sqrt(tx*tx + ty*ty + tz*tz);
	if (dist == 0) dist = 1;

	// Now get the intrinsics for the depth stream
	auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	auto intrinsics = stream.get_intrinsics(); // Calibration data

	// Compute 2D coordinates of adjacent pixels in the middle of the field of view
	float pixel0[2], pixel1[2];
	pixel0[0] = camera_width / 2;
	pixel0[1] = camera_height / 2;
	if (do_depth_filtering && camSettings.do_decimation) {
		pixel1[0] = pixel0[0] + camSettings.decimation_value;
		pixel1[1] = pixel0[1] + camSettings.decimation_value;
	} else {
		pixel1[0] = pixel0[0] + 1;
		pixel1[1] = pixel0[1] + 1;
	}

	// Deproject to get 3D distance
	float point0[3], point1[3];
	rs2_deproject_pixel_to_point(point0, &intrinsics, pixel0, dist);
	rs2_deproject_pixel_to_point(point1, &intrinsics, pixel1, dist);
	float rv = sqrt(pow(point1[0]-point0[0], 2)+pow(point1[1]-point0[1], 2)+pow(point1[2]-point0[2], 2));
	pointSize = rv;
}
#endif

void K4ACamera::stop()
{
	assert(!stopped);
	stopped = true;
	processing_frame_queue.try_enqueue(NULL);
	if (capture_started) {
		k4a_device_stop_cameras(device_handle);
		k4a_transformation_destroy(transformation_handle);
		camera_started = false;
	}
	capture_started = false;
	if (grabber_thread) grabber_thread->join();
	delete grabber_thread;
	if (processing_thread) processing_thread->join();
	delete processing_thread;
	processing_done = true;
	processing_done_cv.notify_one();
	k4a_device_close(device_handle);
}

void K4ACamera::start_capturer()
{
	if (!camera_started) return;
	assert(stopped);
	stopped = false;
	capture_started = true;
	_start_capture_thread();
	processing_thread = new std::thread(&K4ACamera::_processing_thread_main, this);
	_cwipc_setThreadName(processing_thread, L"cwipc_kinect::K4ACamera::processing_thread");
}

void K4ACamera::_start_capture_thread()
{
	grabber_thread = new std::thread(&K4ACamera::_capture_thread_main, this);
	_cwipc_setThreadName(grabber_thread, L"cwipc_kinect::K4ACamera::capture_thread");
}

void K4ACamera::_capture_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "cwipc_kinect: K4ACamera: cam=" << serial << " thread started" << std::endl;
#endif
	while(!stopped) {
		k4a_capture_t capture_handle;
		if (k4a_capture_create(&capture_handle) != K4A_RESULT_SUCCEEDED) {
			std::cerr << "cwipc_kinect: camera " << serial << ": k4a_capture_create failed" << std::endl;
			cwipc_k4a_log_warning("k4a_capture_create failed");
			break;
		}
		k4a_wait_result_t ok = k4a_device_get_capture(device_handle, &capture_handle, 5000);
		if (ok != K4A_RESULT_SUCCEEDED) {
			std::cerr << "cwipc_kinect: camera " << serial << ": error " << ok << std::endl;
			cwipc_k4a_log_warning("k4a_device_get_capture failed");
			k4a_capture_release(capture_handle);
			continue;
		}
		assert(capture_handle != NULL);
#ifdef CWIPC_DEBUG_THREAD
		k4a_image_t color = k4a_capture_get_color_image(capture_handle);
		uint64_t tsRGB = k4a_image_get_device_timestamp_usec(color);
		k4a_image_release(color);
		k4a_image_t depth = k4a_capture_get_depth_image(capture_handle);
		uint64_t tsD = k4a_image_get_device_timestamp_usec(depth);
		k4a_image_release(depth);
		std::cerr << "cwipc_kinect: K4ACamera: capture: cam=" << serial << ", rgbseq=" << tsRGB << ", dseq=" << tsD << std::endl;
#endif
		if (!captured_frame_queue.try_enqueue(capture_handle)) {
			// Queue is full. discard.
#ifdef CWIPC_DEBUG_THREAD
			k4a_image_t color = k4a_capture_get_color_image(capture_handle);
			uint64_t tsRGB = k4a_image_get_device_timestamp_usec(color);
			k4a_image_release(color);
			k4a_image_t depth = k4a_capture_get_depth_image(capture_handle);
			uint64_t tsD = k4a_image_get_device_timestamp_usec(depth);
			k4a_image_release(depth);
			std::cerr << "cwipc_kinect: K4ACamera: drop frame " << tsRGB << "/" << tsD <<" from camera "<< serial << std::endl;
#endif
			k4a_capture_release(capture_handle);
			std::this_thread::sleep_for(std::chrono::milliseconds(25));
		}
		else {
			// Frame deposited in queue
			std::this_thread::sleep_for(std::chrono::milliseconds(25));
		}
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "cwipc_kinect: K4ACamera: cam=" << serial << " thread stopped" << std::endl;
#endif
}

void K4ACamera::_processing_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "cwipc_kinect: K4ACamera: processing: cam=" << serial << " thread started" << std::endl;
#endif
	k4a_image_t point_cloud_image = NULL;
	k4a_image_t transformed_depth = NULL;
	while(!stopped) {
		k4a_capture_t processing_frameset = NULL;
		k4a_image_t depth = NULL;
		k4a_image_t color = NULL;

		bool ok = processing_frame_queue.wait_dequeue_timed(processing_frameset, std::chrono::milliseconds(10000));
		if (processing_frameset == NULL) {
#ifdef CWIPC_DEBUG_THREAD
			std::cerr << "cwipc_kinect: processing thread: null frameset" << std::endl;
#endif
			continue;
		}
		if (!ok) {
			std::cerr << "cwipc_kinect: no frameset for 10 seconds, camera " << serial << std::endl;
			continue;
		}
#ifdef CWIPC_DEBUG_THREAD
		std::cerr << "cwipc_kinect: processing: got frame for camera " << serial << std::endl;
#endif
		assert(processing_frameset);
		std::lock_guard<std::mutex> lock(processing_mutex);
		depth = k4a_capture_get_depth_image(processing_frameset);
		color = k4a_capture_get_color_image(processing_frameset);

		// Note: the following code uses color as the main source of resolution. To be decided.
		int color_image_width_pixels = k4a_image_get_width_pixels(color);
		int color_image_height_pixels = k4a_image_get_height_pixels(color);
#ifdef CWIPC_DEBUG_THREAD
		std::cerr << "cwipc_kinect: processing: got images for camera " << serial << std::endl;
#endif

		k4a_result_t sts;
		if (transformed_depth == NULL) {
			sts = k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, color_image_width_pixels, color_image_height_pixels, color_image_width_pixels * (int)sizeof(uint16_t), &transformed_depth);
			if (sts != K4A_RESULT_SUCCEEDED) {
				std::cerr << "cwipc_kinect: cannot create transformed depth image" << std::endl;
				goto endloop;
			}
		}
		if (point_cloud_image == NULL) {
			sts = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, color_image_width_pixels, color_image_height_pixels, color_image_width_pixels * 3 * (int)sizeof(int16_t), &point_cloud_image);
			if (sts != K4A_RESULT_SUCCEEDED) {
				std::cerr << "cwipc_kinect: cannot create pointcloud image" << std::endl;
				goto endloop;
			}
		}
#ifdef CWIPC_DEBUG_THREAD
		std::cerr << "cwipc_kinect: processing: created aux images for camera " << serial << std::endl;
#endif

		sts = k4a_transformation_depth_image_to_color_camera(transformation_handle, depth, transformed_depth);
		if (sts != K4A_RESULT_SUCCEEDED) {
			std::cerr << "cwipc_kinect: cannot transform depth image" << std::endl;
			goto endloop;
		}
#ifdef CWIPC_DEBUG_THREAD
		std::cerr << "cwipc_kinect: processing: transformed depth image for camera " << serial << std::endl;
#endif
		sts = k4a_transformation_depth_image_to_point_cloud(transformation_handle, transformed_depth, K4A_CALIBRATION_TYPE_COLOR, point_cloud_image);
		if (sts != K4A_RESULT_SUCCEEDED) {
			std::cerr << "cwipc_kinect: cannot create point cloud" << std::endl;
			goto endloop;
		}
#ifdef CWIPC_DEBUG_THREAD
		std::cerr << "cwipc_kinect: processing: created pointcloud image for camera " << serial << std::endl;
#endif
		{
			uint8_t* color_data = k4a_image_get_buffer(color);
			int16_t* point_cloud_image_data = (int16_t*)k4a_image_get_buffer(point_cloud_image);
			// Setup depth filtering, if needed
			int16_t min_depth = (int16_t)(camSettings.threshold_near * 1000);
			int16_t max_depth = (int16_t)(camSettings.threshold_far * 1000);
			// now loop over images and create points.
			camData.cloud->clear();
			camData.cloud->reserve(color_image_width_pixels * color_image_height_pixels);
			for (int i = 0; i < color_image_width_pixels * color_image_height_pixels; i++)
			{
				int i_pc = i * 3;
				int i_rgba = i * 4;
				cwipc_pcl_point point;
				int16_t x = point_cloud_image_data[i_pc + 0];
				int16_t y = point_cloud_image_data[i_pc + 1];
				int16_t z = point_cloud_image_data[i_pc + 2];
				if (z == 0) continue;
				if (camSettings.do_threshold && (z < min_depth || z > max_depth)) continue;

				point.r = color_data[i_rgba + 2];
				point.g = color_data[i_rgba + 1];
				point.b = color_data[i_rgba + 0];
				point.a = (uint8_t)1 << camera_index;
				uint8_t alpha = color_data[i_rgba + 3];

				if (point.r == 0 && point.g == 0 && point.b == 0 && alpha == 0) continue;
				point.x = x;
				point.y = y;
				point.z = z;
				transformPoint(point);
				if (do_height_filtering && (point.y < height_min || point.y > height_max)) continue;
				if (!do_greenscreen_removal || cwipc_k4a_noChromaRemoval(&point)) // chromakey removal
					camData.cloud->push_back(point);
			}
#ifdef CWIPC_DEBUG_THREAD
			std::cerr << "cwipc_kinect: camera " << serial << " produced " << camData.cloud->size() << " point" << std::endl;
#endif
		}
		if (camData.cloud->size() == 0) {
			std::cerr << "cwipc_kinect: warning: captured empty pointcloud from camera " << camData.serial << std::endl;
			//continue;
		}
		// Notify wait_for_pc that we're done.
		processing_done = true;
		processing_done_cv.notify_one(); 
	endloop:
		//  free all allocated images
		if (color) k4a_image_release(color);
		if (depth) k4a_image_release(depth);
		if (processing_frameset) k4a_capture_release(processing_frameset);
	}
	if (transformed_depth) k4a_image_release(transformed_depth);
	if (point_cloud_image) k4a_image_release(point_cloud_image);
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "cwipc_kinect: K4ACamera: processing: cam=" << serial << " thread stopped" << std::endl;
#endif
}

void K4ACamera::transformPoint(cwipc_pcl_point& pt)
{
	float x = pt.x / 1000.0;
	float y = pt.y / 1000.0;
	float z = pt.z / 1000.0;
	pt.x = (*camData.trafo)(0,0)*x + (*camData.trafo)(0,1)*y + (*camData.trafo)(0,2)*z + (*camData.trafo)(0,3);
	pt.y = (*camData.trafo)(1,0)*x + (*camData.trafo)(1,1)*y + (*camData.trafo)(1,2)*z + (*camData.trafo)(1,3);
	pt.z = (*camData.trafo)(2,0)*x + (*camData.trafo)(2,1)*y + (*camData.trafo)(2,2)*z + (*camData.trafo)(2,3);
}

void K4ACamera::create_pc_from_frames()
{
	assert(current_frameset);
	if (!processing_frame_queue.try_enqueue(current_frameset)) {
		std::cerr << "cwipc_kinect: camera " << serial << ": drop frame before processing" << std::endl;
		k4a_capture_release(current_frameset);
	}
	current_frameset = NULL;
}

void K4ACamera::wait_for_pc()
{
	std::unique_lock<std::mutex> lock(processing_mutex);
	processing_done_cv.wait(lock, [this]{ return processing_done; });
	processing_done = false;
}

uint64_t K4ACamera::get_capture_timestamp()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void
K4ACamera::dump_color_frame(const std::string& filename)
{
#ifdef WITH_DUMP_VIDEO_FRAMES
	k4a_image_t color = k4a_capture_get_color_image(current_frameset);
	if(color != NULL){
		int color_image_width_pixels = k4a_image_get_width_pixels(color);
		int color_image_height_pixels = k4a_image_get_height_pixels(color);
		int bytes_per_pixel = 4;
		uint8_t* color_data = k4a_image_get_buffer(color);
		int color_stride_bytes = k4a_image_get_stride_bytes(color);
		long timestamp = k4a_image_get_device_timestamp_usec(color);

		stbi_write_png(filename.c_str(), color_image_width_pixels, color_image_height_pixels,
			bytes_per_pixel, color_data, color_stride_bytes);

		std::cout << "cwipc_kinect: dumped image. Camera: " << camera_index << " t=" << timestamp << std::endl;
		k4a_image_release(color);
	}
	else {
		std::cerr << "cwipc_kinect: error: dumping image. serial: " << camData.serial << std::endl;
	}
#endif
}

