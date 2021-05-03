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

#include "cwipc_kinect/private/K4ACapture.hpp"
#include "cwipc_kinect/private/K4ACamera.hpp"


// Static variable used to print a warning message when we re-create an K4ACapture
// if there is another one open.
static int numberOfCapturersActive = 0;

K4ACapture::K4ACapture(int dummy)
:	numberOfPCsProduced(0),
	want_auxdata_rgb(false),
	want_auxdata_depth(false),
    no_cameras(true),
	mergedPC(nullptr),
	mergedPC_is_fresh(false),
	mergedPC_want_new(false)
{
	numberOfCapturersActive++;
	starttime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

K4ACapture::K4ACapture(const char *configFilename)
:	K4ACapture(0)
{
	// First check that no other K4ACapture is active within this process (trying to catch programmer errors)
	if (numberOfCapturersActive > 1) {
		cwipc_k4a_log_warning("Attempting to create capturer while one is already active.");
	}

	int camera_count = k4a_device_get_installed_count();
	if (camera_count == 0) {
		// no camera connected, so we'll return nothing
		no_cameras = true;
		return;
	}
	no_cameras = false;
	
	//
	// Check for attached cameras and create dummy configuration entries (basically only serial number)
	//
	std::vector<std::string> serials;
	std::vector<k4a_device_t> camera_handles(camera_count, nullptr);
	if (!_init_config_from_devices(camera_handles, serials)) return;
	//
	// Read the configuration. We do this only now because for historical reasons the configuration
	// reader is also the code that checks whether the configuration file contents match the actual
	// current hardware setup. To be fixed at some point.
	//
	if (!_init_config_from_configfile(configFilename)) {
		//
		// If the attached devices don't match the config file we update our configuration to
		// match the hardware situation.
		//
		_update_config_from_devices();
	}

	//
	// Initialize hardware capture setting (for all cameras)
	//
	_init_hardware_settings(camera_handles);

	// Now we have all the configuration information. Open the cameras.
	_create_cameras(camera_handles, serials);
	// We can now free camera_handles
	//delete camera_handles;

	_init_camera_positions();
	
	_start_cameras();
	//
	// start our run thread (which will drive the capturers and merge the pointclouds)
	//
	stopped = false;
	control_thread = new std::thread(&K4ACapture::_control_thread_main, this);
	_cwipc_setThreadName(control_thread, L"cwipc_kinect::K4ACapture::control_thread");
}

bool K4ACapture::_init_config_from_devices(std::vector<k4a_device_t>& camera_handles, std::vector<std::string>& serials) {
	
	bool any_failure = false;
	for (uint32_t i = 0; i < camera_handles.size(); i++) {
		if (k4a_device_open(i, &camera_handles[i]) != K4A_RESULT_SUCCEEDED) {
			cwipc_k4a_log_warning("k4a_device_open failed");
			any_failure = true;
			continue;
		}
		K4ACameraData cd;
		char serial_buf[64];
		size_t serial_buf_size = sizeof(serial_buf) / sizeof(serial_buf[0]);
		if (k4a_device_get_serialnum(camera_handles[i], serial_buf, &serial_buf_size) != K4A_BUFFER_RESULT_SUCCEEDED) {
			cwipc_k4a_log_warning("get_serialnum failed");
			any_failure = true;
			continue;
		}
		cd.serial = std::string(serial_buf);
		serials.push_back(serial_buf);
		pcl::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
		default_trafo->setIdentity();
		cd.trafo = default_trafo;
		cd.intrinsicTrafo = default_trafo;
		cd.cameraposition = { 0, 0, 0 };
		configuration.camera_data.push_back(cd);
	}
	if (any_failure) {
		no_cameras = true;
		return false;
	}
	return true;
}

bool K4ACapture::_init_config_from_configfile(const char *configFilename) {
	if (configFilename == NULL) {
		configFilename = "cameraconfig.xml";
	}
	return cwipc_k4a_file2config(configFilename, &configuration);
}

void K4ACapture::_update_config_from_devices() {

	// the configuration file did not fully match the current situation so we have to update the admin
	std::vector<std::string> serials;
	std::vector<K4ACameraData> realcams;


	// collect all camera's in the config that are connected
	for (K4ACameraData cd : configuration.camera_data) {
#if 1 // xxxjack find() doesn't work??!?
		realcams.push_back(cd);
#else
		if ((find(serials.begin(), serials.end(), cd.serial) != serials.end()))
			realcams.push_back(cd);
		else
			cwipc_k4a_log_warning("Camera " + cd.serial + " is not connected");
#endif
	}
	// Reduce the active configuration to cameras that are connected
	configuration.camera_data = realcams;
}

void K4ACapture::_init_hardware_settings(std::vector<k4a_device_t>& camera_handles) {
	
	// Set various camera hardware parameters (color)
	for (int i = 0; i < camera_handles.size(); i++) {
		//options for color sensor
		if (configuration.camera_config.color_exposure_time >= 0) {	//MANUAL
			k4a_result_t res = k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_config.color_exposure_time); // Exposure_time (in microseconds)
			if (res != K4A_RESULT_SUCCEEDED) std::cerr << "cwipc_kinect: cameraconfig.xml: color_exposure_time should be microsecond and in range (500-133330)" << std::endl;
		}
		else {	//AUTO
			k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_AUTO, 0);
		}
		if (configuration.camera_config.color_whitebalance >= 0) {	//MANUAL
			k4a_result_t res = k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_config.color_whitebalance); // White_balance (2500-12500)
			if (res != K4A_RESULT_SUCCEEDED) std::cerr << "cwipc_kinect: cameraconfig.xml: color_whitebalance should be in range (2500-12500)" << std::endl;
		}
		else {	//AUTO
			k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_AUTO, 0);
		}

		if (configuration.camera_config.color_backlight_compensation >= 0){
			k4a_result_t res = k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_config.color_backlight_compensation); // Backlight_compensation 0=disabled | 1=enabled. Default=0
			if (res != K4A_RESULT_SUCCEEDED) std::cerr << "cwipc_kinect: cameraconfig.xml: color_backlight_compensation should be 0=Enabled, 1= Disabled" << std::endl;
		}
		if (configuration.camera_config.color_brightness >= 0){
			k4a_result_t res = k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_BRIGHTNESS, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_config.color_brightness); // Brightness. (0 to 255). Default=128.
			if (res != K4A_RESULT_SUCCEEDED) std::cerr << "cwipc_kinect: cameraconfig.xml: color_brightness should be in range (0-255)" << std::endl;
		}
		if (configuration.camera_config.color_contrast >= 0){
			k4a_result_t res = k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_CONTRAST, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_config.color_contrast); // Contrast (0-10). Default=5
			if (res != K4A_RESULT_SUCCEEDED) std::cerr << "cwipc_kinect: cameraconfig.xml: color_contrast should be in range (0-10)" << std::endl;
		}
		if (configuration.camera_config.color_saturation >= 0){
			k4a_result_t res = k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_SATURATION, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_config.color_saturation); // saturation (0-63). Default=32
			if (res != K4A_RESULT_SUCCEEDED) std::cerr << "cwipc_kinect: cameraconfig.xml: color_saturation should be in range (0-63)" << std::endl;
		}
		if (configuration.camera_config.color_sharpness >= 0){
			k4a_result_t res = k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_SHARPNESS, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_config.color_sharpness); // Sharpness (0-4). Default=2
			if (res != K4A_RESULT_SUCCEEDED) std::cerr << "cwipc_kinect: cameraconfig.xml: color_sharpness should be in range (0-4)" << std::endl;
		}
		if (configuration.camera_config.color_gain >= 0){	//if autoexposure mode=AUTO gain does not affect
			k4a_result_t res = k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_GAIN, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_config.color_gain); // Gain (0-255). Default=0
			if (res != K4A_RESULT_SUCCEEDED) std::cerr << "cwipc_kinect: cameraconfig.xml: color_gain should be in range (0-255)" << std::endl;
		}
		if (configuration.camera_config.color_powerline_frequency >= 0){
			k4a_result_t res = k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, K4A_COLOR_CONTROL_MODE_MANUAL, configuration.camera_config.color_powerline_frequency); // Powerline_Frequency (1=50Hz, 2=60Hz). Default=2
			if (res != K4A_RESULT_SUCCEEDED) std::cerr << "cwipc_kinect: cameraconfig.xml: color_powerline_frequency should be 1=50Hz or 2=60Hz" << std::endl;
		}
	}
#ifdef CWIPC_DEBUG
	// Hmm. Jack thinks this shouldn't be printed normally (too confusing to normal
	// users). But it would be nice to have something like an environment variable
	// CWIPC_LOGGING=trace to enable it.
	//PRINTING CURRENT COLOR CONFIG
	std::cout << "\n### Current color configuration: ########" << std::endl;
	int32_t value;
	k4a_color_control_mode_t mode;

	k4a_device_get_color_control(camera_handles[0], K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, &mode, &value);
	bool isManual = (mode == K4A_COLOR_CONTROL_MODE_MANUAL);
	std::cout << "#\tExposure time:\t" << (isManual==true ? "Manual,\t" : "Auto") << (isManual==true ? std::to_string(value) +"\t#" : "\t\t#") << std::endl;

	k4a_device_get_color_control(camera_handles[0], K4A_COLOR_CONTROL_WHITEBALANCE, &mode, &value);
	isManual = (mode == K4A_COLOR_CONTROL_MODE_MANUAL);
	std::cout << "#\tWhite balance:\t" << (isManual == true ? "Manual,\t" : "Auto") << (isManual == true ? std::to_string(value) + "\t#" : "\t\t#") << std::endl;

	k4a_device_get_color_control(camera_handles[0], K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, &mode, &value);
	isManual = (mode == K4A_COLOR_CONTROL_MODE_MANUAL);
	std::cout << "#\tBL comp.:\t" << (isManual == true ? "Manual,\t" : "Auto") << (isManual == true ? std::to_string(value) + "\t#" : "\t\t#") << std::endl;

	k4a_device_get_color_control(camera_handles[0], K4A_COLOR_CONTROL_BRIGHTNESS, &mode, &value);
	isManual = (mode == K4A_COLOR_CONTROL_MODE_MANUAL);
	std::cout << "#\tBrightness:\t" << (isManual == true ? "Manual,\t" : "Auto") << (isManual == true ? std::to_string(value) + "\t#" : "\t\t#") << std::endl;

	k4a_device_get_color_control(camera_handles[0], K4A_COLOR_CONTROL_CONTRAST, &mode, &value);
	isManual = (mode == K4A_COLOR_CONTROL_MODE_MANUAL);
	std::cout << "#\tContrast:\t" << (isManual == true ? "Manual,\t" : "Auto") << (isManual == true ? std::to_string(value) + "\t#" : "\t\t#") << std::endl;

	k4a_device_get_color_control(camera_handles[0], K4A_COLOR_CONTROL_SATURATION, &mode, &value);
	isManual = (mode == K4A_COLOR_CONTROL_MODE_MANUAL);
	std::cout << "#\tSaturation:\t" << (isManual == true ? "Manual,\t" : "Auto") << (isManual == true ? std::to_string(value) + "\t#" : "\t\t#") << std::endl;

	k4a_device_get_color_control(camera_handles[0], K4A_COLOR_CONTROL_SHARPNESS, &mode, &value);
	isManual = (mode == K4A_COLOR_CONTROL_MODE_MANUAL);
	std::cout << "#\tSharpness:\t" << (isManual == true ? "Manual,\t" : "Auto") << (isManual == true ? std::to_string(value) + "\t#" : "\t\t#") << std::endl;

	k4a_device_get_color_control(camera_handles[0], K4A_COLOR_CONTROL_GAIN, &mode, &value);
	isManual = (mode == K4A_COLOR_CONTROL_MODE_MANUAL);
	std::cout << "#\tGain:\t\t" << (isManual == true ? "Manual,\t" : "Auto") << (isManual == true ? std::to_string(value) + "\t#" : "\t\t#") << std::endl;

	k4a_device_get_color_control(camera_handles[0], K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, &mode, &value);
	isManual = (mode == K4A_COLOR_CONTROL_MODE_MANUAL);
	std::cout << "#\tPow freq.:\t" << (isManual == true ? "Manual,\t" : "Auto") << (isManual == true ? std::to_string(value) + "\t#" : "\t\t#") << std::endl;
	std::cout << "#########################################\n" << std::endl;
	//END PRINTING CURRENT COLOR CONFIG
#endif
}

void K4ACapture::_create_cameras(std::vector<k4a_device_t>& camera_handles, std::vector<std::string>& serials) {
	int serial_index = 0;
	for(uint32_t i=0; i<camera_handles.size(); i++) {
		if (camera_handles[i] == NULL) continue;
#ifdef CWIPC_DEBUG
		std::cout << "K4ACapture: opening camera " << serials[i] << std::endl;
#endif
		// Found a realsense camera. Create a default data entry for it.
		std::string serial(serials[serial_index++]);

		K4ACameraData& cd = get_camera_data(serial);
		if (cd.type != "kinect") {
			cwipc_k4a_log_warning("Camera " + serial + " is type " + cd.type + " in stead of kinect");
		}
		int camera_index = cameras.size();
		auto cam = new K4ACamera(camera_handles[i], configuration, camera_index, cd);
		cameras.push_back(cam);
	}
}

void K4ACapture::_init_camera_positions() {
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

void K4ACapture::_start_cameras() {
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

K4ACapture::~K4ACapture() {
    if (no_cameras) {
        numberOfCapturersActive--;
        return;
    }
	uint64_t stopTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	stopped = true;
	// Stop all cameras
	for (auto cam : cameras)
		cam->stop();
    mergedPC_is_fresh = true;
    mergedPC_want_new = false;
    mergedPC_is_fresh_cv.notify_all();
    mergedPC_want_new = true;
    mergedPC_want_new_cv.notify_all();
	control_thread->join();
	delete control_thread;
	std::cerr << "cwipc_kinect: stopped all cameras\n";
	// Delete all cameras (which will stop their threads as well)
	for (auto cam : cameras)
		delete cam;
	cameras.clear();
	std::cerr << "cwipc_kinect: deleted all cameras\n";
	// Print some minimal statistics of this run
	float deltaT = (stopTime - starttime) / 1000.0;
	std::cerr << "cwipc_kinect: ran for " << deltaT << " seconds, produced " << numberOfPCsProduced << " pointclouds at " << numberOfPCsProduced / deltaT << " fps." << std::endl;
	numberOfCapturersActive--;
}

// API function that triggers the capture and returns the merged pointcloud and timestamp
cwipc* K4ACapture::get_pointcloud()
{
    if (no_cameras) return nullptr;
	_request_new_pointcloud();
	// Wait for a fresh mergedPC to become available.
	// Note we keep the return value while holding the lock, so we can start the next grab/process/merge cycle before returning.
	cwipc* rv;
	{
		std::unique_lock<std::mutex> mylock(mergedPC_mutex);
		mergedPC_is_fresh_cv.wait(mylock, [this]{return mergedPC_is_fresh; });
		mergedPC_is_fresh = false;
		numberOfPCsProduced++;
		rv = mergedPC;
		mergedPC = nullptr;
	}
	_request_new_pointcloud();
	return rv;
}

float K4ACapture::get_pointSize()
{
    if (no_cameras) return 0;
	float rv = 99999;
	for (auto cam : cameras) {
		if (cam->pointSize < rv) rv = cam->pointSize;
	}
	if (rv > 9999) rv = 0;
	return rv;
}

bool K4ACapture::pointcloud_available(bool wait)
{
    if (no_cameras) return false;
	_request_new_pointcloud();
	std::this_thread::yield();
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	auto duration = std::chrono::seconds(wait?1:0);
	mergedPC_is_fresh_cv.wait_for(mylock, duration, [this]{return mergedPC_is_fresh; });
	return mergedPC_is_fresh;
}

void K4ACapture::_control_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "cwipc_kinect: K4ACapture: processing thread started" << std::endl;
#endif
	while(!stopped) {
		{
			std::unique_lock<std::mutex> mylock(mergedPC_mutex);
			mergedPC_want_new_cv.wait(mylock, [this]{ return mergedPC_want_new;});
		}
		if (stopped) {
			break;
		}
        assert (cameras.size() > 0);
        // Step one: grab frames from all cameras. This should happen as close together in time as possible,
        // because that gives use he biggest chance we have the same frame (or at most off-by-one) for each
        // camera.
		bool all_captures_ok = true;
        for(auto cam : cameras) {
			if (!cam->capture_frameset()) {
				all_captures_ok = false;
				continue;
			}
        }
		if (!all_captures_ok) {
			//std::cerr << "cwipc_kinect: K4ACapture: xxxjack not all captures succeeded. Retrying." << std::endl;
			std::this_thread::yield();
			continue;
		}
        // And get the best timestamp
        uint64_t timestamp = 0;
        for(auto cam: cameras) {
            uint64_t camts = cam->get_capture_timestamp();
            if (camts > timestamp) timestamp = camts;
        }
		if (timestamp <= 0) {
			timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		}
		// Step 2 - Create pointcloud, and save rgb/depth images if wanted
		cwipc_pcl_pointcloud pcl_pointcloud = new_cwipc_pcl_pointcloud();
		char* error_str = NULL;
		cwipc *newPC = cwipc_from_pcl(pcl_pointcloud, timestamp, &error_str, CWIPC_API_VERSION);
		if (newPC == nullptr) {
			std::cerr << "cwipc_kinect: K4ACapturer: cwipc_from_pcl returned error: " << error_str << std::endl;
			break;
		}

		if (want_auxdata_rgb || want_auxdata_depth) {
			for (auto cam : cameras) {
				cam->save_auxdata(newPC, want_auxdata_rgb, want_auxdata_depth);
			}
		}

        // Step 3: start processing frames to pointclouds, for each camera
        for(auto cam : cameras) {
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
        for(auto cam : cameras) {
            cam->wait_for_pc();
        }
        // Step 5: merge views
        merge_views();
        if (mergedPC->access_pcl_pointcloud()->size() > 0) {
#ifdef CWIPC_DEBUG
            std::cerr << "cwipc_kinect: capturer produced a merged cloud of " << mergedPC->size() << " points" << std::endl;
#endif
        } else {
#ifdef CWIPC_DEBUG
            std::cerr << "cwipc_kinect: Warning: capturer got an empty pointcloud\n";
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
	std::cerr << "cwipc_kinect: K4ACapture: processing thread stopped" << std::endl;
#endif
}

// return the merged cloud 
cwipc* K4ACapture::get_mostRecentPointCloud()
{
    if (no_cameras) return nullptr;
	// This call doesn't need a fresh pointcloud (Jack thinks), but it does need one that is
	// consistent. So we lock, but don't wait on the condition.
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	return mergedPC;
}

void K4ACapture::_request_new_pointcloud()
{
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	if (!mergedPC_want_new && !mergedPC_is_fresh) {
		mergedPC_want_new = true;
		mergedPC_want_new_cv.notify_all();
	}
}

void K4ACapture::merge_views()
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

K4ACameraData& K4ACapture::get_camera_data(std::string serial) {
	for (int i = 0; i < configuration.camera_data.size(); i++)
		if (configuration.camera_data[i].serial == serial)
			return configuration.camera_data[i];
	cwipc_k4a_log_warning("Unknown camera " + serial);
	abort();
}

K4ACamera* K4ACapture::get_camera(std::string serial) {
	for (auto cam : cameras)
		if (cam->serial == serial)
			return cam;
	return NULL;
}
