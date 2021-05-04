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

#include "cwipc_kinect/private/K4AOfflineCapture.hpp"

// Static variable used to print a warning message when we re-create an K4AOfflineCapture
// if there is another one open.
static int numberOfCapturersActive = 0;

K4AOfflineCapture::K4AOfflineCapture(const char* configFilename)
	: numberOfPCsProduced(0),
	sync_inuse(false),
	master_id(-1),
	eof(false),
	want_auxdata_rgb(false),
	want_auxdata_depth(false),
    no_cameras(true),
	mergedPC(nullptr),
	stopped(false),
	mergedPC_is_fresh(false),
	mergedPC_want_new(false),
	current_ts(0)
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
	int camera_count =  configuration.camera_data.size();
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


bool K4AOfflineCapture::_init_config_from_configfile(const char *configFilename) {
	if (configFilename == NULL) {
		configFilename = "cameraconfig.xml";
	}
	return cwipc_k4a_file2config(configFilename, &configuration);
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
		const char *filename = configuration.camera_data[i].filename.c_str();

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
		configuration.camera_data[i].cameraposition = { 0, 0, 0 };
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
		K4ACameraData& cd = configuration.camera_data[i];
		if (cd.type != "kinect") {
			cwipc_k4a_log_warning("Camera " + cd.serial + " is type " + cd.type + " in stead of kinect");
		}
		int camera_index = cameras.size();
		auto cam = new K4AOfflineCapture::Type_our_camera(camera_handles[i], configuration, camera_index, cd);
		cameras.push_back(cam);
	}
}

void K4AOfflineCapture::_init_camera_positions() {
	// find camerapositions
	for (int i = 0; i < configuration.camera_data.size(); i++) {
		cwipc_pcl_pointcloud pcptr(new_cwipc_pcl_pointcloud());
		cwipc_pcl_point pt;
		pt.x = 0;
		pt.y = 0;
		pt.z = 0;
		pcptr->push_back(pt);
		transformPointCloud(*pcptr, *pcptr, *configuration.camera_data[i].trafo);
		cwipc_pcl_point pnt = pcptr->points[0];
		configuration.camera_data[i].cameraposition.x = pnt.x;
		configuration.camera_data[i].cameraposition.y = pnt.y;
		configuration.camera_data[i].cameraposition.z = pnt.z;
	}
}

void K4AOfflineCapture::_start_cameras() {
	//
	// start the cameras. First start all non-sync-master cameras, then start the sync-master camera.
	//
	for (auto cam : cameras) {
		if (cam->is_sync_master()) continue;
		if (!cam->start()) {
			cwipc_k4a_log_warning("Not all cameras could be started");
			no_cameras = true;
			return;
		}
	}
	for (auto cam : cameras) {
		if (!cam->is_sync_master()) continue;
		if (!cam->start()) {
			cwipc_k4a_log_warning("Not all cameras could be started");
			no_cameras = true;
			return;
		}
	}

	starttime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	//
	// start the per-camera capture threads. Master camera has to be started latest
	//
	for (auto cam : cameras) {
		if (cam->is_sync_master()) continue;
		cam->start_capturer();
	}
	for (auto cam : cameras) {
		if (!cam->is_sync_master()) continue;
		cam->start_capturer();
	}
}
K4AOfflineCapture::~K4AOfflineCapture() {
	if (no_cameras) {
		numberOfCapturersActive--;
		return;
	}
	uint64_t stopTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	// Stop all cameras
	for (auto cam : cameras)
		cam->stop();
	stopped = true;
	mergedPC_is_fresh = true;
	mergedPC_want_new = false;
	mergedPC_is_fresh_cv.notify_all();
	mergedPC_want_new = true;
	mergedPC_want_new_cv.notify_all();
	control_thread->join();
	delete control_thread;
	std::cerr << CLASSNAME << ": stopped all cameras\n";

	// Delete all cameras (which will stop their threads as well)
	for (auto cam : cameras)
		delete cam;
	cameras.clear();
	std::cerr << CLASSNAME << ": deleted all cameras\n";

	// Print some minimal statistics of this run
	float deltaT = (stopTime - starttime) / 1000.0;
	std::cerr << CLASSNAME << ": ran for " << deltaT << " seconds, produced " << numberOfPCsProduced << " pointclouds at " << numberOfPCsProduced / deltaT << " fps." << std::endl;
	numberOfCapturersActive--;
}

// API function that triggers the capture and returns the merged pointcloud and timestamp
cwipc* K4AOfflineCapture::get_pointcloud()
{
	if (no_cameras) return nullptr;
	_request_new_pointcloud();
	// Wait for a fresh mergedPC to become available.
	// Note we keep the return value while holding the lock, so we can start the next grab/process/merge cycle before returning.
	cwipc* rv;
	{
		std::unique_lock<std::mutex> mylock(mergedPC_mutex);
		mergedPC_is_fresh_cv.wait(mylock, [this] {return mergedPC_is_fresh; });
		mergedPC_is_fresh = false;
		numberOfPCsProduced++;
		rv = mergedPC;
		mergedPC = nullptr;
	}
	_request_new_pointcloud();
	return rv;
}

float K4AOfflineCapture::get_pointSize()
{
	if (no_cameras) return 0;
	float rv = 99999;
	for (auto cam : cameras) {
		if (cam->pointSize < rv) rv = cam->pointSize;
	}
	if (rv > 9999) rv = 0;
	return rv;
}

bool K4AOfflineCapture::pointcloud_available(bool wait)
{
	if (no_cameras) return false;
	_request_new_pointcloud();
	std::this_thread::yield();
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	auto duration = std::chrono::seconds(wait ? 1 : 0);
	mergedPC_is_fresh_cv.wait_for(mylock, duration, [this] {return mergedPC_is_fresh; });
	return mergedPC_is_fresh;
}

void K4AOfflineCapture::_control_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << CLASSNAME << ": processing thread started" << std::endl;
#endif
	while (!stopped) {
		{
			std::unique_lock<std::mutex> mylock(mergedPC_mutex);
			mergedPC_want_new_cv.wait(mylock, [this] { return mergedPC_want_new; });
		}
		//check EOF:
		for (auto cam : cameras) {
			if (cam->eof) {
				eof = true;
				stopped = true;
				break;
			}
		}
		if (stopped) {
			break;
		}
		assert(cameras.size() > 0);
		// Step one: grab frames from all cameras. This should happen as close together in time as possible,
		// because that gives use he biggest chance we have the same frame (or at most off-by-one) for each
		// camera.
		bool all_captures_ok = _capture_all_cameras();
		if (!all_captures_ok) {
			//std::cerr << CLASSNAME << ": xxxjack not all captures succeeded. Retrying." << std::endl;
			std::this_thread::yield();
			continue;
		}
		// And get the best timestamp
		uint64_t timestamp = _get_best_timestamp();
		current_ts = timestamp;
		
		// Step 2 - Create pointcloud, and save rgb/depth images if wanted
		cwipc_pcl_pointcloud pcl_pointcloud = new_cwipc_pcl_pointcloud();
		char* error_str = NULL;
		cwipc* newPC = cwipc_from_pcl(pcl_pointcloud, timestamp, &error_str, CWIPC_API_VERSION);
		if (newPC == nullptr) {
			std::cerr << CLASSNAME << ": cwipc_from_pcl returned error: " << error_str << std::endl;
			break;
		}

		if (want_auxdata_rgb || want_auxdata_depth) {
			for (auto cam : cameras) {
				cam->save_auxdata_images(newPC, want_auxdata_rgb, want_auxdata_depth);
			}
		}

		// Step 3: start processing frames to pointclouds, for each camera
		for (auto cam : cameras) {
			cam->create_pc_from_frames();
		}
		// Lock mergedPC already while we are waiting for the per-camera
		// processing threads. This so the main thread doesn't go off and do
		// useless things if it is calling available(true).
		std::unique_lock<std::mutex> mylock(mergedPC_mutex);
		if (mergedPC && mergedPC_is_fresh) {
			mergedPC->free();
			mergedPC = nullptr;
		}
		mergedPC = newPC;

		// Step 4: wait for frame processing to complete.
		for (auto cam : cameras) {
			cam->wait_for_pc();
		}
		// Step 5: merge views
		merge_views();
		if (mergedPC->access_pcl_pointcloud()->size() > 0) {
#ifdef CWIPC_DEBUG
			std::cerr << CLASSNAME << ": capturer produced a merged cloud of " << mergedPC->size() << " points" << std::endl;
#endif
        } else {
#ifdef CWIPC_DEBUG
			std::cerr << CLASSNAME << ": Warning: capturer got an empty pointcloud\n";
#endif
#if 0
			// HACK to make sure the encoder does not get an empty pointcloud
			cwipc_pcl_point point;
			point.x = 1.0;
			point.y = 1.0;
			point.z = 1.0;
			point.rgb = 0.0;
			mergedPC->points.push_back(point);
#endif
		}
		// Signal that a new mergedPC is available. (Note that we acquired the mutex earlier)
		mergedPC_is_fresh = true;
		mergedPC_want_new = false;
		mergedPC_is_fresh_cv.notify_all();
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << CLASSNAME << ": processing thread stopped" << std::endl;
#endif
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

// return the merged cloud 
cwipc* K4AOfflineCapture::get_mostRecentPointCloud()
{
	if (no_cameras) return nullptr;
	// This call doesn't need a fresh pointcloud (Jack thinks), but it does need one that is
	// consistent. So we lock, but don't wait on the condition.
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	return mergedPC;
}

void K4AOfflineCapture::_request_new_pointcloud()
{
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	if (!mergedPC_want_new && !mergedPC_is_fresh) {
		mergedPC_want_new = true;
		mergedPC_want_new_cv.notify_all();
	}
}

void K4AOfflineCapture::merge_views()
{
	cwipc_pcl_pointcloud aligned_cld(mergedPC->access_pcl_pointcloud());
	aligned_cld->clear();
	// Pre-allocate space in the merged pointcloud
	size_t nPoints = 0;
	for (auto cam : cameras) {
		cwipc_pcl_pointcloud cam_cld = cam->get_current_pointcloud();
		if (cam_cld == nullptr) {
			cwipc_k4a_log_warning("Camera " + cam->serial + " returned NULL cloud, ignoring");
			continue;
		}
		nPoints += cam_cld->size();
	}
	aligned_cld->reserve(nPoints);
	// Now merge all pointclouds
	for (auto cam : cameras) {
		cwipc_pcl_pointcloud cam_cld = cam->get_current_pointcloud();
		if (cam_cld == nullptr) continue;
		*aligned_cld += *cam_cld;
	}
	if (aligned_cld->size() != nPoints) {
		cwipc_k4a_log_warning("Combined pointcloud has different number of points than expected");
	}
}

K4ACameraData& K4AOfflineCapture::get_camera_data(std::string serial) {
	for (int i = 0; i < configuration.camera_data.size(); i++)
		if (configuration.camera_data[i].serial == serial)
			return configuration.camera_data[i];
	cwipc_k4a_log_warning("Unknown camera " + serial);
	abort();
}

K4AOfflineCapture::Type_our_camera* K4AOfflineCapture::get_camera(std::string serial) {
	for (auto cam : cameras)
		if (cam->serial == serial)
			return cam;
	return NULL;
}
