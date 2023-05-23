//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//
#include <cstdlib>

#include "K4AOfflineCamera.hpp"
#include "turbojpeg.h"

K4AOfflineCamera::K4AOfflineCamera(Type_api_camera _handle, K4ACaptureConfig& configuration, int _camera_index, K4ACameraConfig& _camData)
:	K4ABaseCamera("cwipc_kinect: K4AOfflineCamera", _handle, configuration, _camera_index, _camData),
	capture_id(-1),
	current_frameset_timestamp(0),
	max_delay(0)	
{
#ifdef CWIPC_DEBUG
	std::cout << CLASSNAME << "creating camera " << serial << std::endl;
#endif
	_init_filters();
}

bool K4AOfflineCamera::capture_frameset(uint64_t master_timestamp)
{
	if (stopped) return false;
	bool rv;
	bool use_timestamp = camera_sync_inuse && !camera_sync_ismaster;
	if (use_timestamp) {
		rv = _prepare_cond_next_valid_frame(master_timestamp);
	} else {
		rv = _prepare_next_valid_frame();
	}
	if (!rv) return rv;

#ifdef CWIPC_DEBUG_THREAD
	if (current_frameset == NULL) {
		std::cerr << CLASSNAME << ": " << camera_index << " forward NULL frame" << std::endl;
	}
	else {
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

bool K4AOfflineCamera::seek(uint64_t timestamp) {
	uint64_t first_t = file_config.start_timestamp_offset_usec;
	uint64_t last_t = k4a_playback_get_recording_length_usec(camera_handle);
	if (timestamp > first_t + last_t) {
		std::cerr << CLASSNAME << " ERROR: desired seek timestamp " << timestamp << " is > than the last timestamp " << first_t + last_t << " of the recording." << std::endl;
		return false;
	}
	if (k4a_playback_seek_timestamp(camera_handle, timestamp, K4A_PLAYBACK_SEEK_DEVICE_TIME) != K4A_RESULT_SUCCEEDED) {
		return false;
	}
	else {
		return true;
	}
}

bool K4AOfflineCamera::_prepare_next_valid_frame() {
	k4a_stream_result_t stream_result;
	// Read the next capture into memory
	bool succeeded = false;
	while (!succeeded) {
		if (stopped) return false;
		assert(camera_handle);
		if (current_frameset != NULL) {
			k4a_capture_release(current_frameset);
			current_frameset = nullptr;
		}
		stream_result = k4a_playback_get_next_capture(camera_handle, &current_frameset);
		if (stream_result == K4A_STREAM_RESULT_EOF)
		{
			eof = true;
#ifdef CWIPC_DEBUG
			if (current_frameset_timestamp == 0) {
				std::cerr << CLASSNAME << ": Recording file is empty: " << camData.filename << std::endl;
			}
			else {
				std::cout << CLASSNAME << ": Recording file " << camData.filename << " reached EOF" << std::endl;
			}
#endif
			return false;
		}
		if (stream_result == K4A_STREAM_RESULT_FAILED)
		{
			std::cerr << CLASSNAME << ": ERROR: Failed to read first capture from file: " << camData.filename << std::endl;
			return false;
		}
		capture_id++;
		k4a_image_t color = k4a_capture_get_color_image(current_frameset);
		k4a_image_t depth = k4a_capture_get_depth_image(current_frameset);
		if (color == NULL) {
			std::cerr << CLASSNAME << ": Color is missing in capture " << capture_id << " from " << camData.filename << std::endl;
		} else if (depth == NULL) {
			std::cerr << CLASSNAME << ": Depth is missing in capture " << capture_id << " from " << camData.filename << std::endl;
		} else {
			succeeded = true;
		}
		current_frameset_timestamp = k4a_image_get_device_timestamp_usec(color);
		k4a_image_release(color);
		k4a_image_release(depth);
	}
	return succeeded;
}

bool K4AOfflineCamera::_prepare_cond_next_valid_frame(uint64_t master_timestamp) {
	//check if current frame already satisfies the condition
	if (current_frameset != NULL && (current_frameset_timestamp > master_timestamp)) {
		// Even if the current frame is too far in the future we use it.
		// Jack is unsure why this is (unlike in the next test)
		return true;
	}
	//otherwise start process to find a frame that satisfies the condition.
	while (true)
	{
		if (!_prepare_next_valid_frame()) return false;
		if (current_frameset_timestamp > master_timestamp) {
			if (current_frameset_timestamp < (master_timestamp + max_delay)) {
				return true;
			}
			else {  //it is a future frame, we need to update master frame
				return false;
			}
		}
	}
}

bool K4AOfflineCamera::start() {
	// We don't have to start anything (opening the file did that) but
	// we do have to get the RGB<->D transformation.
	if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(camera_handle, &sensor_calibration))
	{
		std::cerr << CLASSNAME << ":  Failed to k4a_device_get_calibration" << std::endl;
		camera_started = false;
		return false;
	}
	depth_to_color_extrinsics = sensor_calibration.extrinsics[0][1];
	transformation_handle = k4a_transformation_create(&sensor_calibration);

	if (xy_table == NULL) {	// generate xy_table for the fast pc_gen v2
		create_xy_table(&sensor_calibration);
	}

	//Get file config for further use of the parameters.
	k4a_result_t result = k4a_playback_get_record_configuration(camera_handle, &file_config);
	if (result != K4A_RESULT_SUCCEEDED)
	{
		std::cerr << CLASSNAME << ": Failed to get record configuration for camera: " << serial << std::endl;
		return false;
	}

	camera_started = true;
	max_delay = 10 * 160; //we set a 160us delay between cameras to avoid laser interference. It is enough for 10x cameras
	return true;
}

void K4AOfflineCamera::stop()
{
	if (stopped) return;
	stopped = true;
	processing_frame_queue.try_enqueue(NULL);
	// Stop threads
	if (processing_thread) processing_thread->join();
	delete processing_thread;

	if (camera_started) {
		// Nothing to stop for reading from file
		camera_started = false;
	}
	// Delete objects
	if (current_frameset != NULL) {
		k4a_capture_release(current_frameset);
		current_frameset = NULL;
	}
	if (camera_handle) {
		k4a_playback_close(camera_handle);
		camera_handle = nullptr;
	}
	if (transformation_handle) {
		k4a_transformation_destroy(transformation_handle);
		transformation_handle = NULL;
	}	
	processing_done = true;
	processing_done_cv.notify_one();
	if (tracker_handle) {
		k4abt_tracker_destroy(tracker_handle);
		tracker_handle = nullptr;
	}
}

void K4AOfflineCamera::start_capturer()
{
	if (!camera_started) return;
	assert(stopped);
	stopped = false;
	processing_thread = new std::thread(&K4AOfflineCamera::_processing_thread_main, this);
	_cwipc_setThreadName(processing_thread, L"cwipc_kinect::K4AOfflineCamera::processing_thread");
}


void K4AOfflineCamera::_start_capture_thread()
{
	// Not needed for offline camera
}

void K4AOfflineCamera::_capture_thread_main()
{
	// Not needed for offline camera
}

k4a_image_t K4AOfflineCamera::_uncompress_color_image(k4a_capture_t capture, k4a_image_t color_image) {
	assert(capture);
	assert(color_image);
	k4a_image_t uncompressed_color_image = nullptr;
	int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
	int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
	if (k4a_image_get_format(color_image) != K4A_IMAGE_FORMAT_COLOR_MJPG) {
		// Not the format we expect. Return as-is.
		return color_image;
	}
	//COLOR image is JPEG compressed. we need to convert the image to BGRA format.

	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * 4 * (int)sizeof(uint8_t),
		&uncompressed_color_image))
	{
		cwipc_k4a_log_warning("Failed to create image buffer for image decompression");
		return color_image;
	}

	tjhandle tjHandle = tjInitDecompress();
	if (tjDecompress2(tjHandle,
		k4a_image_get_buffer(color_image),
		static_cast<unsigned long>(k4a_image_get_size(color_image)),
		k4a_image_get_buffer(uncompressed_color_image),
		color_image_width_pixels,
		0, // pitch
		color_image_height_pixels,
		TJPF_BGRA,
		TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0)
	{
		cwipc_k4a_log_warning("Failed to decompress color frame");
	}
	if (tjDestroy(tjHandle))
	{
		cwipc_k4a_log_warning("Failed to destroy turboJPEG handle");
	}
	assert(uncompressed_color_image);
	k4a_image_release(color_image);
	k4a_capture_set_color_image(capture, uncompressed_color_image);
	return uncompressed_color_image;
}


