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

#include "K4ACamera.hpp"

K4ACamera::K4ACamera(Type_api_camera _handle, K4ACaptureConfig& configuration, int _camera_index, K4ACameraData& _camData)
:	K4ABaseCamera("cwipc_kinect: K4ACamera", _handle, configuration, _camera_index, _camData)
{
#ifdef CWIPC_DEBUG
	std::cout << CLASSNAME << "creating camera " << serial << std::endl;
#endif
	_init_filters();
}

bool K4ACamera::capture_frameset()
{
	if (stopped) return false;
	k4a_capture_t new_frameset = NULL;
	bool rv = captured_frame_queue.wait_dequeue_timed(new_frameset, 5000000);
	if (!rv) return rv;
	if (current_frameset) {
		k4a_capture_release(current_frameset);
	}
	current_frameset = new_frameset;
#ifdef CWIPC_DEBUG_THREAD
	if (current_frameset == NULL) {
		std::cerr << CLASSNAME << ": " << camera_index <<" forward NULL frame"  << std::endl;
	} else {
		k4a_image_t color = k4a_capture_get_color_image(current_frameset);
		uint64_t tsRGB = k4a_image_get_device_timestamp_usec(color);
		k4a_image_release(color);
		k4a_image_t depth = k4a_capture_get_depth_image(current_frameset);
		uint64_t tsD = k4a_image_get_device_timestamp_usec(depth);
		k4a_image_release(depth);
		std::cerr << CLASSNAME << ": forward frame: cam=" << serial << ", rgbseq=" << tsRGB << ", dseq=" << tsD << std::endl;
	}
#endif

	return rv;
}

// Configure and initialize caputuring of one camera
bool K4ACamera::start()
{
	assert(stopped);
	k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	if (!_setup_device(device_config)) return false;


	if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(camera_handle, device_config.depth_mode, device_config.color_resolution, &sensor_calibration))
	{
		std::cerr << CLASSNAME << ": Failed to k4a_device_get_calibration" << std::endl;
		camera_started = false;
		return false;
	}
	depth_to_color_extrinsics = sensor_calibration.extrinsics[0][1];
	transformation_handle = k4a_transformation_create(&sensor_calibration);

	k4a_result_t res = k4a_device_start_cameras(camera_handle, &device_config);
	if (res != K4A_RESULT_SUCCEEDED) {
		std::cerr << "cwipc_kinect: failed to start camera " << serial << std::endl;
		return false;
	}
	std::cerr << "cwipc_kinect: starting camera " << camera_index << " with serial="<< serial << ". color_height=" << color_height << ", depth_height=" << depth_height << " map_color_to_depth=" << camSettings.map_color_to_depth << " @" << camera_fps << "fps as " << (camera_sync_inuse ? (camera_sync_ismaster? "Master" : "Subordinate") : "Standalone") << std::endl;
	
	camera_started = true;
	return true;
}

bool K4ACamera::_setup_device(k4a_device_configuration_t& device_config) {
	device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	switch (color_height) {
	case 720:
		device_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
		break;
	case 1080:
		device_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
		break;
	case 1440:
		device_config.color_resolution = K4A_COLOR_RESOLUTION_1440P;
		break;
	case 1536:
		device_config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
		break;
	case 2160:
		device_config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
		break;
	case 3072:
		device_config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
		break;
	default:
		std::cerr << "cwipc_kinect: invalid color_height: " << color_height << std::endl;
		return false;
	}
	switch (depth_height) {
	case 288:
		device_config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
		break;
	case 576:
		device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		break;
	case 512:
		device_config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
		break;
	case 1024:
		device_config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
		break;
	default:
		std::cerr << "cwipc_kinect: invalid depth_height: " << depth_height << std::endl;
		return false;
	}
	switch (camera_fps) {
	case 5:
		device_config.camera_fps = K4A_FRAMES_PER_SECOND_5;
		break;
	case 15:
		device_config.camera_fps = K4A_FRAMES_PER_SECOND_15;
		break;
	case 30:
		device_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
		break;
	default:
		std::cerr << "cwipc_kinect: invalid camera_fps: " << camera_fps << std::endl;
		return false;
	}

	device_config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture

	//SYNC:
	if (camera_sync_ismaster) {
		device_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
	}
	else if (camera_sync_inuse) {
		device_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
		device_config.subordinate_delay_off_master_usec = 160 * (camera_index+1);	//160 allows max 9 cameras
	} else {
		// standalone mode, nothing to set
	}

	return true;
}


void K4ACamera::stop()
{
	if (stopped) return;
	stopped = true;
	processing_frame_queue.try_enqueue(NULL);
	// Stop threads
	if (grabber_thread) grabber_thread->join();
	delete grabber_thread;
	if (processing_thread) processing_thread->join();
	delete processing_thread;

	if (tracker_handle) {
		//Stop body tracker
		k4abt_tracker_shutdown(tracker_handle);
		k4abt_tracker_destroy(tracker_handle);
		tracker_handle = nullptr;
	}
	if (camera_started) {
		// Stop camera
		k4a_device_stop_cameras(camera_handle);
		camera_started = false;
	}
	if (camera_handle) {
		k4a_device_close(camera_handle);
		camera_handle = nullptr;
	}
	// Delete objects
	if (current_frameset != NULL) {
		k4a_capture_release(current_frameset);
		current_frameset = NULL;
	}
	if (transformation_handle) {
		k4a_transformation_destroy(transformation_handle);
		transformation_handle = NULL;
	}	
	processing_done = true;
	processing_done_cv.notify_one();
}

void K4ACamera::start_capturer()
{
	if (!camera_started) return;
	assert(stopped);
	stopped = false;
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
	std::cerr << CLASSNAME << ": cam=" << serial << " thread started" << std::endl;
#endif
	while(!stopped) {
		k4a_capture_t capture_handle;
		if (k4a_capture_create(&capture_handle) != K4A_RESULT_SUCCEEDED) {
			std::cerr << "cwipc_kinect: camera " << serial << ": k4a_capture_create failed" << std::endl;
			cwipc_k4a_log_warning("k4a_capture_create failed");
			break;
		}
		k4a_wait_result_t ok = k4a_device_get_capture(camera_handle, &capture_handle, 5000);
		if (ok != K4A_WAIT_RESULT_SUCCEEDED) {
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
		std::cerr << CLASSNAME << ": capture: cam=" << serial << ", rgbseq=" << tsRGB << ", dseq=" << tsD << std::endl;
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
			std::cerr << CLASSNAME << ": drop frame " << tsRGB << "/" << tsD <<" from camera "<< serial << std::endl;
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
	std::cerr << CLASSNAME << ": cam=" << serial << " thread stopped" << std::endl;
#endif
}

k4a_image_t K4ACamera::_uncompress_color_image(k4a_capture_t capture, k4a_image_t color_image) {
	return color_image;
}


