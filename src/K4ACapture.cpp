// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD

#include <chrono>

#include "K4ACapture.hpp"

K4ACapture::K4ACapture()
: K4ABaseCapture("K4ACapture", "kinect")
{}

int K4ACapture::count_devices() {
    return k4a_device_get_installed_count();
}

bool K4ACapture::config_reload_and_start_capturing(const char* configFilename) {
    _unload_cameras();

    //
    // Read the configuration.
    //
    if (!_apply_config(configFilename)) {
        return false;
    }

    auto camera_count = configuration.all_camera_configs.size();
    if (camera_count == 0) {
        return false;
    }

    // xxxjack rs2 does setup_camera_hardware_parameters() before _open_cameras()
    if (!_open_cameras()) {
        // xxxjack we should really close all cameras too...
        camera_count = 0; 
        return false;
    }

    //
    // Initialize hardware capture setting (for all cameras)
    //
    if (!_init_hardware_for_all_cameras()) {
        // xxxjack we should really close all cameras too...
        camera_count = 0;
        return false;
    }

    _setup_inter_camera_sync();

    // Now we have all the configuration information. Create our K4ACamera objects.
    if (!_create_cameras()) {
        _unload_cameras();
        return false;
    }

    // xxxjack RS2 does _check_cameras_connected() here.

    _init_camera_positions();
    _start_cameras();

    // xxxjack RS2 does per-camera start_camera_streaming() here

    // xxxjack RS2 does per-camera post_start_all_cameras() here.
    //
    // start our run thread (which will drive the capturers and merge the pointclouds)
    //
    stopped = false;
    control_thread = new std::thread(&K4ACapture::_control_thread_main, this);
    _cwipc_setThreadName(control_thread, L"cwipc_kinect::K4ACapture::control_thread");

    return true;
}

bool K4ACapture::_apply_auto_config() {
    bool any_failure = false;

    // Enumerate over all attached cameras and create a default configuration
    for (uint32_t i = 0; i < k4a_device_get_installed_count(); i++) {
        Type_api_camera handle;

        if (k4a_device_open(i, &handle) != K4A_RESULT_SUCCEEDED) {
            _log_error("k4a_device_open failed");
            any_failure = true;
            continue;
        }

        K4ACameraConfig cd;
        cd.type = "kinect";
        char serial_buf[64];
        size_t serial_buf_size = sizeof(serial_buf) / sizeof(serial_buf[0]);

        if (k4a_device_get_serialnum(handle, serial_buf, &serial_buf_size) != K4A_BUFFER_RESULT_SUCCEEDED) {
            _log_error("k4a_device_get_serialnum failed");
            any_failure = true;

            continue;
        }

        cd.serial = std::string(serial_buf);
        pcl::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
        default_trafo->setIdentity();
        cd.trafo = default_trafo;
        cd.cameraposition = { 0, 0, 0 };
        configuration.all_camera_configs.push_back(cd);
        k4a_device_close(handle);
    }

    if (any_failure) {
        _unload_cameras();
        return false;
    }

    return true;
}

bool K4ACapture::_open_cameras() {
    for (uint32_t i = 0; i < k4a_device_get_installed_count(); i++) {
        Type_api_camera handle;

        if (k4a_device_open(i, &handle) != K4A_RESULT_SUCCEEDED) {
            _log_error("k4a_device_open failed");
            return false;
        }

        char serial_buf[64];
        size_t serial_buf_size = sizeof(serial_buf) / sizeof(serial_buf[0]);

        if (k4a_device_get_serialnum(handle, serial_buf, &serial_buf_size) != K4A_BUFFER_RESULT_SUCCEEDED) {
            _log_error("k4a_device_get_serialnum failed");
            return false;
        }

        std::string serial(serial_buf);
        K4ACameraConfig* cd = get_camera_config(serial);

        if (cd == nullptr) {
            _log_error("No configuration found for camera with serial " + serial);
            return false;
        }

        cd->handle = handle;
        cd->connected = (handle != nullptr);
    }

    // Check that all configured cameras have been opened.
    for (auto& cd : configuration.all_camera_configs) {
        if (!cd.connected) {
            _log_error("Camera " + cd.serial + " is not connected");
            return false;
        }
    }

    return true;
}

bool K4ACapture::_init_hardware_for_all_cameras() {

    return true;
}

bool K4ACapture::_create_cameras() {
    for(auto& cd : configuration.all_camera_configs) {
        if (cd.handle == nullptr) {
            continue;
        }

        Type_api_camera handle = (Type_api_camera)cd.handle;

        _log_debug("creating camera with serial " + cd.serial);
        // We look up the config by its serial number.
        if (cd.type != "kinect") {
            _log_error("configuration: camera with serial " + cd.serial + " has wrong type " + cd.type);
            return false;
        }

        int camera_index = cameras.size();
        auto cam = new Type_our_camera(handle, configuration, camera_index, cd);
        cameras.push_back(cam);
        cd.handle = nullptr;
    }

    return true;
}

bool K4ACapture::_capture_all_cameras() {
    bool all_captures_ok = true;

    for(auto cam : cameras) {
        if (!cam->capture_frameset()) {
            all_captures_ok = false;
            continue;
        }
    }

    return all_captures_ok;
}

uint64_t K4ACapture::_get_best_timestamp() {
    uint64_t timestamp = 0;

    for(auto cam: cameras) {
        uint64_t camts = cam->get_capture_timestamp();
        if (camts > timestamp) {
            timestamp = camts;
        }
    }

    if (timestamp <= 0) {
        timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }

    return timestamp;
}

bool K4ACapture::seek(uint64_t timestamp) {
    return false;
}
