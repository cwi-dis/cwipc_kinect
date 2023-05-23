//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

// Define to try and use hardware sync to synchronize multiple cameras
#define WITH_INTER_CAM_SYNC

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD


#include <chrono>

#include "K4AOfflineCapture.hpp"

// Static variable used to print a warning message when we re-create an K4AOfflineCapture
// if there is another one open.
static int numberOfCapturersActive = 0;

K4AOfflineCapture::K4AOfflineCapture(const char* configFilename)
:	K4ABaseCapture("cwipc_kinect: K4AOfflineCapture"),
	sync_inuse(false),
	master_id(-1)
	{
	// First check that no other K4AOfflineCapture is active within this process (trying to catch programmer errors)
	numberOfCapturersActive++;
	if (numberOfCapturersActive > 1) {
		cwipc_k4a_log_warning("Attempting to create capturer while one is already active.");
	}

	//
	// Read the configuration. We do this only now because for historical reasons the configuration
	// reader is also the code that checks whether the configuration file contents match the actual
	// current hardware setup. To be fixed at some point.
	//
	(void)_init_config_from_configfile(configFilename);
	int camera_count = 0;
	for (std::vector<K4ACameraConfig>::iterator it = configuration.all_camera_configs.begin(); it != configuration.all_camera_configs.end();) {
		if (it->disabled) {
			std::cout << CLASSNAME << ": Camera " << it->serial << " is disabled in cameraconfig.xml" << std::endl;
			it = configuration.all_camera_configs.erase(it); // we remove the cameradata from config as we don't plan to use it in this session.
		}
		else {
			camera_count++;
			++it;
		}
	}
	std::vector<Type_api_camera> camera_handles(camera_count, nullptr);
	if (!_open_recording_files(camera_handles)) {
		no_cameras = true;
		return;
	}

	_create_cameras(camera_handles);

	// delete camera_handles;
	no_cameras = false;

	_init_camera_positions();

	_start_cameras();
	//
	// start our run thread (which will drive the capturers and merge the pointclouds)
	//
	stopped = false;
	control_thread = new std::thread(&K4AOfflineCapture::_control_thread_main, this);
	_cwipc_setThreadName(control_thread, L"cwipc_kinect::K4AOfflineCapture::control_thread");
}

bool K4AOfflineCapture::_open_recording_files(std::vector<Type_api_camera>& camera_handles) {
	
	if (camera_handles.size() == 0) {
		// no camera connected, so we'll return nothing
		no_cameras = true;
		return false;
	}
	bool master_found = false;
	k4a_result_t result;

	// Open each recording file and validate they were recorded in master/subordinate mode.
	for (size_t i = 0; i < camera_handles.size(); i++)
	{
		if (configuration.all_camera_configs[i].disabled == true) continue;

		const char *filename = configuration.all_camera_configs[i].filename.c_str();

		result = k4a_playback_open(filename, &camera_handles[i]);

		if (result != K4A_RESULT_SUCCEEDED)
		{
			std::cerr << CLASSNAME << ": Failed to open file: " << filename << std::endl;
			return false;
		}
		k4a_record_configuration_t file_config;
		result = k4a_playback_get_record_configuration(camera_handles[i], &file_config);
		if (result != K4A_RESULT_SUCCEEDED)
		{
			std::cerr << CLASSNAME << ": Failed to get record configuration for file: " << filename << std::endl;
			return false;
		}

		if (file_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER)
		{
			std::cerr << CLASSNAME << ": Opened master recording file: " << filename << std::endl;
			if (master_found)
			{
				std::cerr << CLASSNAME << ": ERROR: Multiple master recordings listed!" << std::endl;
				result = K4A_RESULT_FAILED;
				return false;
			}
			else
			{
				master_found = true;
				master_id = i;
			}
		}
		else if (file_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE)
		{
			std::cout << CLASSNAME << ": Opened subordinate recording file: " << filename << std::endl;
		}
		else
		{
			std::cerr << CLASSNAME << ": ERROR: Recording file was not recorded in master/sub mode: " << filename << std::endl;
			result = K4A_RESULT_FAILED;
			return false;
		}

		//initialize cameradata attributes:
		configuration.all_camera_configs[i].cameraposition = { 0, 0, 0 };
	}
	// xxxjack we should chack that configuration.sync_master_serial matches master_id...

	if (master_id != -1 && configuration.sync_master_serial != "")
		sync_inuse = true;
	return true;
}

void K4AOfflineCapture::_create_cameras(std::vector<Type_api_camera>& camera_handles) {
	for (uint32_t i = 0; i < camera_handles.size(); i++) {
		assert (camera_handles[i] != nullptr);
#ifdef CWIPC_DEBUG
		std::cout << CLASSNAME << ": opening camera " << serials[i] << std::endl;
#endif
		// Found a kinect camera. Create a default data entry for it.
		K4ACameraConfig& cd = configuration.all_camera_configs[i];
		if (cd.type != "kinect") {
			cwipc_k4a_log_warning("Camera " + cd.serial + " is type " + cd.type + " in stead of kinect");
		}
		int camera_index = cameras.size();
		if (!cd.disabled) {
			auto cam = new K4AOfflineCapture::Type_our_camera(camera_handles[i], configuration, camera_index, cd);
			cameras.push_back(cam);
		}
	}
}

bool K4AOfflineCapture::_capture_all_cameras() {
	bool all_captures_ok = true;
	//
	//f irst capture master frame (it is the referrence to sync).
	// For the master we simply get the next frame available (indicated by timestamp==0)
	//
	uint64_t wanted_timestamp = 0;
	for (auto cam : cameras) { //MASTER
		if (!cam->is_sync_master()) continue;
		if (!cam->capture_frameset(wanted_timestamp)) {
			all_captures_ok = false;
			break;
		}
	}
	//
	// If we have a sync master we now know the timestamp we want from the other cameras.
	//
	if (sync_inuse) {
		wanted_timestamp = cameras[master_id]->current_frameset_timestamp;
	}
	//
	// Now capture the rest of the cameras
	//
	for (auto cam : cameras) { //SUBORDINATE or STANDALONE
		if (cam->is_sync_master()) continue;
		if (!cam->capture_frameset(wanted_timestamp)) {
			all_captures_ok = false;
			break;
		}
	}
	return all_captures_ok;
}

uint64_t K4AOfflineCapture::_get_best_timestamp() {
	int timestamp = 0;
	if (sync_inuse) { //sync on
		timestamp = cameras[master_id]->current_frameset_timestamp;
	}
	else {
		for (auto cam : cameras) {
			uint64_t camts = cam->current_frameset_timestamp;
			if (camts > timestamp) timestamp = camts;
		}
	}
	if (timestamp <= 0) {
		timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	}
	return timestamp;
}

bool K4AOfflineCapture::seek(uint64_t timestamp) {
	for (auto cam : cameras) { //SUBORDINATE or STANDALONE
		if (cam->seek(timestamp) != true) {
			std::cerr << "Camera " << cam->serial << " failed to seek to timestamp " << timestamp << std::endl;
			return false;
		}
	}
	return true;
}
