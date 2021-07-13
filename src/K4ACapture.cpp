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

#include "K4ACapture.hpp"


K4ACapture::K4ACapture(const char *configFilename)
:	K4ABaseCapture("cwipc_kinect: K4ACapture")
{

	//
	// Check for attached cameras and create dummy configuration entries (basically only serial number)
	//
	int camera_count = k4a_device_get_installed_count();
	if (camera_count == 0) {
		// no camera connected, so we'll return nothing
		no_cameras = true;
		return;
	}
	
	std::vector<std::string> serials;
	std::vector<Type_api_camera> camera_handles(camera_count, nullptr);
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

	// Now we have all the configuration information. Create our K4ACamera objects.
	_create_cameras(camera_handles, serials);

	// delete camera_handles;
	no_cameras = false;

	_init_camera_positions();

	_start_cameras();
	//
	// start our run thread (which will drive the capturers and merge the pointclouds)
	//
	stopped = false;
	control_thread = new std::thread(&K4ACapture::_control_thread_main, this);
	_cwipc_setThreadName(control_thread, L"cwipc_kinect::K4ACapture::control_thread");
}

bool K4ACapture::_init_config_from_devices(std::vector<Type_api_camera>& camera_handles, std::vector<std::string>& serials) {
	
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

void K4ACapture::_update_config_from_devices() {

	// the configuration file did not fully match the current situation so we have to update the admin
	std::vector<std::string> serials;
	std::vector<K4ACameraData> realcams;


	// collect all camera's in the config that are connected
	for (K4ACameraData cd : configuration.camera_data) {
#if 1 // xxxjack find() doesn't work??!?
		if(!cd.disabled)
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

void K4ACapture::_init_hardware_settings(std::vector<Type_api_camera>& camera_handles) {
	
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

void K4ACapture::_create_cameras(std::vector<Type_api_camera>& camera_handles, std::vector<std::string>& serials) {
	int serial_index = 0;
	for(uint32_t i=0; i<camera_handles.size(); i++) {
		assert (camera_handles[i] != nullptr);
#ifdef CWIPC_DEBUG
		std::cout << CLASSNAME << ": opening camera " << serials[i] << std::endl;
#endif
		// Found a Kinect camera. Create a default data entry for it.
		std::string serial(serials[serial_index++]);

		K4ACameraData& cd = get_camera_data(serial);
		if (cd.type != "kinect") {
			cwipc_k4a_log_warning("Camera " + cd.serial + " is type " + cd.type + " in stead of kinect");
		}
		int camera_index = cameras.size();
		if (!cd.disabled) {
			auto cam = new Type_our_camera(camera_handles[i], configuration, camera_index, cd);
			cameras.push_back(cam);
		}
	}
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
		if (camts > timestamp) timestamp = camts;
	}
	if (timestamp <= 0) {
		timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	}
	return timestamp;
}

bool K4ACapture::seek(uint64_t timestamp) {
	return false;
}