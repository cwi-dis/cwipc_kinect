// Define to get (a little) debug prints
#define CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD

#include <chrono>

#include "K4APlaybackCapture.hpp"

// Static variable used to print a warning message when we re-create an K4APlaybackCapture
// if there is another one open.
static int numberOfCapturersActive = 0;

K4APlaybackCapture::K4APlaybackCapture()
: K4ABaseCapture("K4APlaybackCapture", "kinect_playback")
{
}

K4APlaybackCapture::~K4APlaybackCapture() {
    if (camera_count > 0) {
        _unload_cameras();
    }

    numberOfCapturersActive--;
}

bool K4APlaybackCapture::config_reload(const char* configFilename) {
    _unload_cameras();

    //
    // Read the configuration. We do this only now because for historical reasons the configuration
    // reader is also the code that checks whether the configuration file contents match the actual
    // current hardware setup. To be fixed at some point.
    //
    if (!_apply_config(configFilename)) {
        return false;
    }

    for (std::vector<K4ACameraConfig>::iterator it = configuration.all_camera_configs.begin(); it != configuration.all_camera_configs.end();) {
        if (it->disabled) {
            std::cout << CLASSNAME << ": Camera " << it->serial << " is disabled in cameraconfig.json" << std::endl;
            it = configuration.all_camera_configs.erase(it); // we remove the cameradata from config as we don't plan to use it in this session.
        } else {
            camera_count++;
            ++it;
        }
    }

    std::vector<Type_api_camera> camera_handles(camera_count, nullptr);
    if (!_open_recording_files(camera_handles, configFilename)) {
        _unload_cameras();
        return false;
    }

    _create_cameras(camera_handles);
    _init_camera_positions();
    _start_cameras();

    //
    // start our run thread (which will drive the capturers and merge the pointclouds)
    //
    stopped = false;
    control_thread = new std::thread(&K4APlaybackCapture::_control_thread_main, this);
    _cwipc_setThreadName(control_thread, L"cwipc_kinect::K4APlaybackCapture::control_thread");

    return true;
}

bool K4APlaybackCapture::_open_recording_files(std::vector<Type_api_camera>& camera_handles, const char *configFilename) {
    if (camera_handles.size() == 0) {
        // no camera connected, so we'll return nothing
        return false;
    }

    bool master_found = false;
    k4a_result_t result;

    // Open each recording file and validate they were recorded in master/subordinate mode.
    for (size_t i = 0; i < camera_handles.size(); i++) {
        if (configuration.all_camera_configs[i].disabled == true) {
            continue;
        }

        std::string camerafile(configuration.all_camera_configs[i].filename);

        if (camerafile.substr(0, 1) != "/") {
            // Relative path (so don''t use windows drive numbers;-)
            std::string filename_cpp(configFilename);
            size_t lastSlashPos = filename_cpp.find_last_of("/\\");

            if (lastSlashPos != std::string::npos) {
                camerafile = filename_cpp.substr(0, lastSlashPos + 1) + camerafile;
            }
        }

        result = k4a_playback_open(camerafile.c_str(), &camera_handles[i]);

        if (result != K4A_RESULT_SUCCEEDED) {
            std::cerr << CLASSNAME << ": Failed to open file: " << camerafile << std::endl;
            return false;
        }

        k4a_record_configuration_t file_config;
        result = k4a_playback_get_record_configuration(camera_handles[i], &file_config);

        if (result != K4A_RESULT_SUCCEEDED) {
            std::cerr << CLASSNAME << ": Failed to get record configuration for file: " << camerafile << std::endl;
            return false;
        }
        if (!configuration.ignore_sync) {
            if (file_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER) {
                std::cerr << CLASSNAME << ": Opened master recording file: " << camerafile << std::endl;

                if (master_found) {
                    std::cerr << CLASSNAME << ": ERROR: Multiple master recordings listed!" << std::endl;
                    result = K4A_RESULT_FAILED;

                    return false;
                }
                else {
                    master_found = true;
                    master_id = i;
                }
            }
            else if (file_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE) {
                std::cout << CLASSNAME << ": Opened subordinate recording file: " << camerafile << std::endl;
            }
            else {
                std::cerr << CLASSNAME << ": ERROR: Recording file was not recorded in master/sub mode: " << camerafile << std::endl;
                result = K4A_RESULT_FAILED;

                return false;
            }
        }
        char serial_buf[100];
        size_t serial_buf_size = 100;
        k4a_buffer_result_t result2 = k4a_playback_get_tag(camera_handles[i], "K4A_DEVICE_SERIAL_NUMBER", serial_buf, &serial_buf_size);
        if (result2 == K4A_BUFFER_RESULT_SUCCEEDED) {
            configuration.all_camera_configs[i].serial = std::string(serial_buf);
        }
        else
        {
            std::cerr << CLASSNAME << ": Could not get camera serial from recording " << camerafile << std::endl;
        }

        //initialize cameradata attributes:
        configuration.all_camera_configs[i].cameraposition = { 0, 0, 0 };
    }
    if (configuration.sync_master_serial == "" && master_id >= 0) {
        configuration.sync_master_serial = configuration.all_camera_configs[master_id].serial;
    }
    // xxxjack we should chack that configuration.sync_master_serial matches master_id...

    if (master_id != -1 && configuration.sync_master_serial != "") {
        sync_inuse = true;
    }

    return true;
}

void K4APlaybackCapture::_create_cameras(std::vector<Type_api_camera>& camera_handles) {
    for (uint32_t i = 0; i < camera_handles.size(); i++) {
        assert (camera_handles[i] != nullptr);


        // Found a kinect camera. Create a default data entry for it.
        K4ACameraConfig& cd = configuration.all_camera_configs[i];
        _log_debug("opening camera " + cd.serial);
        if (cd.type == "kinect_offline") {
            _log_warning("configuration: camera with serial " + cd.serial + " has deprecated type 'kinect_offline', changing to 'kinect_playback'");
            cd.type = "kinect_playback";
        }
        if (cd.type != "kinect_playback") {
            _log_error("configuration: camera with serial " + cd.serial + " has wrong type " + cd.type);
        }

        int camera_index = cameras.size();

        if (!cd.disabled) {
            auto cam = new K4APlaybackCapture::Type_our_camera(camera_handles[i], configuration, camera_index, cd);
            cameras.push_back(cam);
        }
    }
}

bool K4APlaybackCapture::_capture_all_cameras() {
    bool all_captures_ok = true;

    //
    //f irst capture master frame (it is the referrence to sync).
    // For the master we simply get the next frame available (indicated by timestamp==0)
    //
   for (auto cam : cameras) { //MASTER
        if (!cam->is_sync_master()) {
            continue;
        }

        if (!cam->capture_frameset(0)) {
            _log_error("Master camera " + cam->serial + " failed to capture frameset");
            all_captures_ok = false;
        }
        break;
   }

   //
   // If we have a sync master we now know the timestamp we want from the other cameras.
   //
   uint64_t wanted_timestamp = 0;
   if (sync_inuse) {
        wanted_timestamp = cameras[master_id]->current_frameset_timestamp;
    }

    //
    // Now capture the rest of the cameras
    //
    for (auto cam : cameras) { //SUBORDINATE or STANDALONE
        if (cam->is_sync_master()) {
            continue;
        }

        if (!cam->capture_frameset(wanted_timestamp)) {
            _log_error("Camera " + cam->serial + " failed to capture frameset at timestamp " + std::to_string(wanted_timestamp));
            all_captures_ok = false;
        }
    }
    _log_debug("Captured all cameras for timestamp " + std::to_string(wanted_timestamp));

    return all_captures_ok;
}

uint64_t K4APlaybackCapture::_get_best_timestamp() {
    int timestamp = 0;

    if (sync_inuse) { //sync on
      timestamp = cameras[master_id]->current_frameset_timestamp;
    } else {
        for (auto cam : cameras) {
            uint64_t camts = cam->current_frameset_timestamp;

            if (camts > timestamp) {
                timestamp = camts;
            }
        }
    }

    if (timestamp <= 0) {
        timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }

    return timestamp;
}

bool K4APlaybackCapture::seek(uint64_t timestamp) {
    for (auto cam : cameras) { //SUBORDINATE or STANDALONE
        if (cam->seek(timestamp) != true) {
            _log_error("Camera " + cam->serial + " failed to seek to timestamp " + std::to_string(timestamp));
            return false;
        }
    }

    return true;
}
