//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//
#include <cstdlib>

#include "K4APlaybackCamera.hpp"
#include "turbojpeg.h"

#undef CWIPC_DEBUG

K4APlaybackCamera::K4APlaybackCamera(Type_api_camera _handle, K4ACaptureConfig& configuration, int _camera_index)
:   K4ABaseCamera("cwipc_kinect: K4APlaybackCamera", _handle, configuration, _camera_index),
    capture_id(-1),
    current_frameset_timestamp(0),
    max_delay(0)
{
    _log_debug("creating playback camera from file " + camera_config.filename);
    _init_filters();
}

uint64_t K4APlaybackCamera::wait_for_captured_frameset(uint64_t earliest_timestamp) {
    if (camera_stopped) {
        return false;
    }
    bool ok = true;
    if (earliest_timestamp > 0) {
        ok = _capture_frame_no_earlier_than_timestamp(earliest_timestamp);
    } else {
        ok = _capture_next_valid_frame();
    }

    if (!ok) {
        // Any error message will have been printed by the methods above.
        return 0;
    }

    return current_frameset_timestamp;
}

bool K4APlaybackCamera::seek(uint64_t timestamp) {
    uint64_t first_t = file_config.start_timestamp_offset_usec;
    uint64_t duration_t = k4a_playback_get_recording_length_usec(camera_handle);

    if (timestamp > first_t + duration_t) {
        _log_error("seek timestamp " + std::to_string(timestamp) + " out of range start=" + std::to_string(first_t) + ", stop=" + std::to_string(first_t + duration_t));
        return false;
    }

    if (k4a_playback_seek_timestamp(camera_handle, timestamp, K4A_PLAYBACK_SEEK_DEVICE_TIME) != K4A_RESULT_SUCCEEDED) {
        return false;
    } else {
        return true;
    }
}

bool K4APlaybackCamera::_capture_next_valid_frame() {
    k4a_stream_result_t stream_result;
    // Read the next capture into memory
    bool succeeded = false;

    while (!succeeded) {
        if (camera_stopped) {
            return false;
        }

        assert(camera_handle);

        if (current_captured_frameset != NULL) {
            k4a_capture_release(current_captured_frameset);
            current_captured_frameset = nullptr;
        }

        stream_result = k4a_playback_get_next_capture(camera_handle, &current_captured_frameset);
        if (stream_result == K4A_STREAM_RESULT_EOF) {
            end_of_stream_reached = true; // xxxjack note that this means eof is true *after all frames have been processed*.
            if (current_frameset_timestamp == 0) {
                _log_warning("Recording file is empty: " + camera_config.filename);
            } else {
                _log_trace("Recording file " + camera_config.filename + " reached EOF at frame " + std::to_string(capture_id));
            }
            return false;
        }

        if (stream_result == K4A_STREAM_RESULT_FAILED) {
            _log_error("First capture failed");
            return false;
        }

        capture_id++;
        k4a_image_t color = k4a_capture_get_color_image(current_captured_frameset);
        k4a_image_t depth = k4a_capture_get_depth_image(current_captured_frameset);

        if (color == NULL) {
            _log_warning("Color is missing in capture " + std::to_string(capture_id) + " serial " + camera_config.serial + " from " + camera_config.filename);
        } else if (depth == NULL) {
            _log_warning("Depth is missing in capture " + std::to_string(capture_id) + " serial " + camera_config.serial + " from " + camera_config.filename);
        } else {
            succeeded = true;
        }

        // xxxjack stop-gap: suddenly sometimes color is missing from the first frame.
        // Try to capture again.
        if (!succeeded) {
            continue;
        }
        // xxxjack note that this code uses *color* frame timestamp...
        current_frameset_timestamp = k4a_image_get_device_timestamp_usec(color);
        _log_debug("Prepared frame " + std::to_string(capture_id) + " with timestamp " + std::to_string(current_frameset_timestamp) + " from file " + camera_config.filename);
        k4a_image_release(color);
        k4a_image_release(depth);
    }

    return succeeded;
}

bool K4APlaybackCamera::_capture_frame_no_earlier_than_timestamp(uint64_t master_timestamp) {
    //check if current frame already satisfies the condition
    if (current_captured_frameset != NULL && (current_frameset_timestamp > master_timestamp)) {
        // Even if the current frame is too far in the future we use it.
        _log_warning("reusing frame " + std::to_string(current_frameset_timestamp) + " for master timestamp " +  std::to_string(master_timestamp));
        return true;
    }

    //otherwise start process to find a frame that satisfies the condition.
    while (true) {
        if (!_capture_next_valid_frame()) {
            return false;
        }

        if (current_frameset_timestamp > master_timestamp) {
            if (current_frameset_timestamp > (master_timestamp + max_delay)) {
                // it is a future frame, we need to update master frame
                _log_warning("return frame " + std::to_string(current_frameset_timestamp) + ", too early by " + std::to_string(current_frameset_timestamp - master_timestamp));
                return true;
            } else {  
                return true;
            }
        }
    }
}

bool K4APlaybackCamera::start_camera() {
    // We don't have to start anything (opening the file did that) but
    // we do have to get the RGB<->D transformation.
    if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(camera_handle, &sensor_calibration)) {
        _log_error("Failed to k4a_device_get_calibration");
        camera_started = false;

        return false;
    }

    depth_to_color_extrinsics = sensor_calibration.extrinsics[0][1];
    transformation_handle = k4a_transformation_create(&sensor_calibration);

    if (depth_uv_mapping == NULL) { // generate depth_uv_mapping for the fast pc_gen v2
        _create_depth_uv_mapping(&sensor_calibration);
    }

    //Get file config for further use of the parameters.
    k4a_result_t result = k4a_playback_get_record_configuration(camera_handle, &file_config);
    if (result != K4A_RESULT_SUCCEEDED) {
        _log_error("Failed to get record configuration");
        return false;
    }

    camera_started = true;
    max_delay = 10 * 160; //we set a 160us delay between cameras to avoid laser interference. It is enough for 10x cameras
    return true;
}

void K4APlaybackCamera::stop_camera() {
    if (camera_stopped) {
        return;
    }

    camera_stopped = true;
    processing_frame_queue.try_enqueue(NULL);

    // Stop threads
    if (camera_processing_thread) {
        camera_processing_thread->join();
    }

    delete camera_processing_thread;
    camera_processing_thread = nullptr;

    if (camera_started) {
        // Nothing to stop for reading from file
        camera_started = false;
    }

    // Delete objects
    if (current_captured_frameset != NULL) {
        k4a_capture_release(current_captured_frameset);
        current_captured_frameset = NULL;
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

void K4APlaybackCamera::start_camera_streaming() {
    if (!camera_started) {
        return;
    }

    assert(camera_stopped);
    camera_stopped = false;
    camera_processing_thread = new std::thread(&K4APlaybackCamera::_processing_thread_main, this);
    _cwipc_setThreadName(camera_processing_thread, L"cwipc_kinect::K4APlaybackCamera::camera_processing_thread");
}

void K4APlaybackCamera::_start_capture_thread() {
    // Not needed for playback camera
}

void K4APlaybackCamera::_capture_thread_main() {
    // Not needed for playback camera
}

k4a_image_t K4APlaybackCamera::_uncompress_color_image(k4a_capture_t capture, k4a_image_t color_image) {
    assert(capture);
    assert(color_image);

    if (k4a_image_get_format(color_image) != K4A_IMAGE_FORMAT_COLOR_MJPG) {
        // Not the format we expect. Return as-is.
        if (k4a_image_get_format(color_image) != K4A_IMAGE_FORMAT_COLOR_BGRA32) {
            _log_error("Color image format is not MJPG or BGRA32, cannot decompress");
        }
        return color_image;
    }

    k4a_image_t uncompressed_color_image = nullptr;

    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);

    //COLOR image is JPEG compressed. we need to convert the image to BGRA format.
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 4 * (int)sizeof(uint8_t),
        &uncompressed_color_image))
    {
      _log_error("Failed to create image buffer for image decompression");
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
        _log_error("Failed to decompress color frame");
    }

    if (tjDestroy(tjHandle)) {
        _log_error("Failed to destroy turboJPEG handle");
    }

    assert(uncompressed_color_image);
    k4a_image_release(color_image);
    k4a_capture_set_color_image(capture, uncompressed_color_image);

    return uncompressed_color_image;
}


