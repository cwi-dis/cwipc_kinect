#include <cstdlib>

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD

#include "K4ACamera.hpp"

K4ACamera::K4ACamera(Type_api_camera _handle, K4ACaptureConfig& configuration, int _camera_index, K4ACameraConfig& _camData)
:   K4ABaseCamera("cwipc_kinect: K4ACamera", _handle, configuration, _camera_index, _camData)
{
    _log_debug("K4ACamera constructor called");
    if (configuration.record_to_directory != "") {
        record_to_file = configuration.record_to_directory + "/" + serial + ".mkv";
    }
    _init_filters();
}

bool K4ACamera::capture_frameset() {
    if (stopped) {
        return false;
    }

    k4a_capture_t new_frameset = NULL;
    bool rv = captured_frame_queue.wait_dequeue_timed(new_frameset, 5000000);

    if (!rv) {
        return rv;
    }

    if (current_frameset) {
        k4a_capture_release(current_frameset);
    }

    current_frameset = new_frameset;
#ifdef CWIPC_DEBUG_THREAD
    if (current_frameset == NULL) {
        _log_debug_thread("forward frame: NULL frameset");
    } else {
        k4a_image_t color = k4a_capture_get_color_image(current_frameset);
        uint64_t tsRGB = k4a_image_get_device_timestamp_usec(color);
        k4a_image_release(color);
        k4a_image_t depth = k4a_capture_get_depth_image(current_frameset);
        uint64_t tsD = k4a_image_get_device_timestamp_usec(depth);
        k4a_image_release(depth);
        _log_debug_thread("forward frame: cam=" + serial + ", rgbseq=" + std::to_string(tsRGB) + ", dseq=" + std::to_string(tsD));
    }
#endif

    return rv;
}

// Configure and initialize caputuring of one camera
bool K4ACamera::start() {
    assert(stopped);
    k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    if (!_setup_device(device_config)) {
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(camera_handle, device_config.depth_mode, device_config.color_resolution, &sensor_calibration)) {
        _log_error("k4a_device_get_calibration failed");
        return false;
    }

    depth_to_color_extrinsics = sensor_calibration.extrinsics[0][1];
    transformation_handle = k4a_transformation_create(&sensor_calibration);

    if (xy_table == NULL) { // generate xy_table for the fast pc_gen v2
        create_xy_table(&sensor_calibration);
    }

    k4a_result_t res = k4a_device_start_cameras(camera_handle, &device_config);
    if (res != K4A_RESULT_SUCCEEDED) {
        _log_error("k4a_device_start_cameras failed");
        return false;
    }
     if (record_to_file != "") {
        k4a_record_t _recorder;
        _log_trace("starting recording to " + record_to_file);
        res = k4a_record_create(record_to_file.c_str(), camera_handle, device_config, &_recorder);
        if (res != K4A_RESULT_SUCCEEDED) {
            _log_error("k4a_record_create failed for file " + record_to_file);
            return false;
        }
        res = k4a_record_write_header(_recorder);
        if (res != K4A_RESULT_SUCCEEDED) {
            _log_error("k4a_record_write_header failed for file " + record_to_file);
            return false;
        }
        recorder = _recorder;
    }
    _log_debug("camera started");

    camera_started = true;
    return true;
}

bool K4ACamera::_setup_device(k4a_device_configuration_t& device_config) {
    device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;

    switch (configuration.color_height) {
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
        _log_error("invalid color_height: " + std::to_string(configuration.color_height));
        return false;
    }

    switch (configuration.depth_height) {
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
        _log_error("invalid depth_height: " + std::to_string(configuration.depth_height));
        return false;
    }

    switch (configuration.fps) {
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
        _log_error("invalid camera_fps: " + std::to_string(configuration.fps));
        return false;
    }

    device_config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture

    //SYNC:
    if (camera_sync_ismaster) {
        device_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
    } else if (camera_sync_inuse) {
        device_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
        device_config.subordinate_delay_off_master_usec = 160 * (camera_index+1); //160 allows max 9 cameras
    } else {
        // standalone mode, nothing to set
    }

    return true;
}

void K4ACamera::stop() {
    if (stopped) {
        return;
    }

    stopped = true;
    processing_frame_queue.try_enqueue(NULL);

    // Stop threads
    if (grabber_thread) {
        grabber_thread->join();
    }

    delete grabber_thread;
    grabber_thread = nullptr;

    if (processing_thread) {
        processing_thread->join();
    }

    delete processing_thread;
    processing_thread = nullptr;

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

    if (recorder) {
        auto res = k4a_record_flush(recorder);
        if (res != K4A_RESULT_SUCCEEDED) {
            _log_error("k4a_record_flush failed");
        }
        k4a_record_close(recorder);
        recorder = nullptr;
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

void K4ACamera::start_capturer() {
    if (!camera_started) {
        return;
    }

    assert(stopped);
    stopped = false;
    _start_capture_thread();
    processing_thread = new std::thread(&K4ACamera::_processing_thread_main, this);
    _cwipc_setThreadName(processing_thread, L"cwipc_kinect::K4ACamera::processing_thread");
}

void K4ACamera::_start_capture_thread() {
    grabber_thread = new std::thread(&K4ACamera::_capture_thread_main, this);
    _cwipc_setThreadName(grabber_thread, L"cwipc_kinect::K4ACamera::capture_thread");
}

void K4ACamera::_capture_thread_main() {
    _log_debug_thread("capture thread started");
    while(!stopped) {
        k4a_capture_t capture_handle;
        if (k4a_capture_create(&capture_handle) != K4A_RESULT_SUCCEEDED) {
            _log_error("k4a_capture_create failed");

            break;
        }

        k4a_wait_result_t ok = k4a_device_get_capture(camera_handle, &capture_handle, 5000);
        if (ok != K4A_WAIT_RESULT_SUCCEEDED) {
            _log_warning("k4a_device_get_capture failed");
            k4a_capture_release(capture_handle);

            continue;
        }

        assert(capture_handle != NULL);

        if (recorder != nullptr) {
            auto res = k4a_record_write_capture(recorder, capture_handle);
            if (res != K4A_RESULT_SUCCEEDED) {
                _log_warning("k4a_record_write_capture failed");
            }
        }
#ifdef CWIPC_DEBUG_THREAD
        k4a_image_t color = k4a_capture_get_color_image(capture_handle);
        uint64_t tsRGB = k4a_image_get_device_timestamp_usec(color);
        k4a_image_release(color);
        k4a_image_t depth = k4a_capture_get_depth_image(capture_handle);
        uint64_t tsD = k4a_image_get_device_timestamp_usec(depth);
        k4a_image_release(depth);
        _log_debug_thread("captured frame " + std::to_string(tsRGB) + "/" + std::to_string(tsD) + " from camera " + serial);
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
            _log_debug_thread("dropped frame " + std::to_string(tsRGB) + "/" + std::to_string(tsD) + " from camera " + serial);
#endif
            k4a_capture_release(capture_handle);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        } else {
            // Frame deposited in queue
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    }

    _log_debug_thread("capture thread stopping");
}

k4a_image_t K4ACamera::_uncompress_color_image(k4a_capture_t capture, k4a_image_t color_image) {
    return color_image;
}
