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

bool K4ACapture::config_reload(const char* configFilename) {
    _unload_cameras();

    //
    // Check for attached cameras and create dummy configuration entries (basically only serial number)
    //
    camera_count = count_devices();
    if (camera_count == 0) {
        // no camera connected, so we'll return nothing
        return false;
    }

    //
    // Read the configuration.
    //
    if (!_apply_config(configFilename)) {
        camera_count = 0;
        return false;
    }

    if (!_open_cameras()) {
        // xxxjack we should really close all cameras too...
        camera_count = 0; 
        return false;
    }

    //
    // Initialize hardware capture setting (for all cameras)
    //
    if (!_init_hardware_settings()) {
        // xxxjack we should really close all cameras too...
        camera_count = 0;
        return false;
    }

    // Now we have all the configuration information. Create our K4ACamera objects.
    if (!_create_cameras()) {
        _unload_cameras();
        return false;
    }

      _init_camera_positions();
      _start_cameras();

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

        if (cd->disabled) {
            k4a_device_close(handle);
            handle = nullptr;
        }

        cd->handle = handle;
        cd->connected = (handle != nullptr);
    }

    // Check that all configured cameras have been opened.
    for (auto& cd : configuration.all_camera_configs) {
        if (!cd.disabled && !cd.connected) {
            _log_error("Camera " + cd.serial + " is not connected");
            return false;
        }
    }

    return true;
}

bool K4ACapture::_init_hardware_settings() {
    // Set various camera hardware parameters (color)
    for(auto& cd : configuration.all_camera_configs) {
        if (cd.disabled || cd.handle == nullptr) {
            continue;
        }

        Type_api_camera handle = (Type_api_camera)cd.handle;

        //options for color sensor
        if (configuration.camera_processing.color_exposure_time >= 0) { //MANUAL
            k4a_result_t res = k4a_device_set_color_control(handle, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_processing.color_exposure_time); // Exposure_time (in microseconds)

            if (res != K4A_RESULT_SUCCEEDED) {
              _log_error("configuration: k4a_device_set_color_control: color_exposure_time should be microsecond and in range (500-133330)");
              return false;
            }
        } else {  //AUTO
            k4a_device_set_color_control(handle, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_AUTO, 0);
        }

        if (configuration.camera_processing.color_whitebalance >= 0) {  //MANUAL
            k4a_result_t res = k4a_device_set_color_control(handle, K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_processing.color_whitebalance); // White_balance (2500-12500)

            if (res != K4A_RESULT_SUCCEEDED) {
                _log_error("configuration: k4a_device_set_color_control: color_whitebalance should be in range (2500-12500)");
                return false;
            }
        } else {  //AUTO
            k4a_device_set_color_control(handle, K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_AUTO, 0);
        }

        if (configuration.camera_processing.color_backlight_compensation >= 0){
            k4a_result_t res = k4a_device_set_color_control(handle, K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_processing.color_backlight_compensation); // Backlight_compensation 0=disabled | 1=enabled. Default=0

            if (res != K4A_RESULT_SUCCEEDED) {
                _log_error("configuration: k4a_device_set_color_control: color_backlight_compensation should be 0=disabled | 1=enabled");
                return false;
            }
        }

        if (configuration.camera_processing.color_brightness >= 0){
          k4a_result_t res = k4a_device_set_color_control(handle, K4A_COLOR_CONTROL_BRIGHTNESS, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_processing.color_brightness); // Brightness. (0 to 255). Default=128.

          if (res != K4A_RESULT_SUCCEEDED) {
              _log_warning("configuration: k4a_device_set_color_control: color_brightness should be in range (0-255)");
              return false;
          }
        }

        if (configuration.camera_processing.color_contrast >= 0){
          k4a_result_t res = k4a_device_set_color_control(handle, K4A_COLOR_CONTROL_CONTRAST, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_processing.color_contrast); // Contrast (0-10). Default=5

          if (res != K4A_RESULT_SUCCEEDED) {
              _log_error("configuration: k4a_device_set_color_control: color_contrast should be in range (0-10)");
              return false;
          }
        }

        if (configuration.camera_processing.color_saturation >= 0){
          k4a_result_t res = k4a_device_set_color_control(handle, K4A_COLOR_CONTROL_SATURATION, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_processing.color_saturation); // saturation (0-63). Default=32

          if (res != K4A_RESULT_SUCCEEDED) {
              _log_error("configuration: k4a_device_set_color_control: color_saturation should be in range (0-63)");
              return false;
          }
        }

        if (configuration.camera_processing.color_sharpness >= 0){
            k4a_result_t res = k4a_device_set_color_control(handle, K4A_COLOR_CONTROL_SHARPNESS, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_processing.color_sharpness); // Sharpness (0-4). Default=2

            if (res != K4A_RESULT_SUCCEEDED) {
                _log_error("configuration: k4a_device_set_color_control: color_sharpness should be in range (0-4)");
                return false;
            }
        }

        if (configuration.camera_processing.color_gain >= 0){ //if autoexposure mode=AUTO gain does not affect
            k4a_result_t res = k4a_device_set_color_control(handle, K4A_COLOR_CONTROL_GAIN, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_processing.color_gain); // Gain (0-255). Default=0

            if (res != K4A_RESULT_SUCCEEDED) {
                _log_error("configuration: k4a_device_set_color_control: color_gain should be in range (0-255)");
                return false;
            }
        }

        if (configuration.camera_processing.color_powerline_frequency >= 0){
            k4a_result_t res = k4a_device_set_color_control(handle, K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_processing.color_powerline_frequency); // Powerline_Frequency (1=50Hz, 2=60Hz). Default=2

            if (res != K4A_RESULT_SUCCEEDED) {
                _log_error("configuration:k4a_device_set_color_control: color_powerline_frequency should be 1=50Hz or 2=60Hz");
                return false;
            }
        }
    }

    return true;
}

bool K4ACapture::_create_cameras() {
    for(auto& cd : configuration.all_camera_configs) {
        if (cd.disabled || cd.handle == nullptr) {
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

    camera_count = cameras.size();
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
