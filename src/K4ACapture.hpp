#ifndef cwipc_realsense_MFCapture_hpp
#define cwipc_realsense_MFCapture_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <k4a/k4a.h>

#include "K4AConfig.hpp"
#include "K4ABaseCapture.hpp"
#include "K4ACamera.hpp"


class K4ACapture : public K4ABaseCapture<k4a_device_t, K4ACamera> {
	typedef k4a_device_t Type_api_camera;
	typedef K4ACamera Type_our_camera;
public:
	static int count_devices();
	static K4ACapture* factory() { return new K4ACapture(); }
	// methods
	K4ACapture();
	virtual ~K4ACapture() {}
	virtual bool config_reload(const char* configFilename) override;
	bool seek(uint64_t timestamp) override;

protected:
	bool _capture_all_cameras() override;
	uint64_t _get_best_timestamp() override;
	virtual bool _apply_default_config() override;
private:
	bool _init_config_from_devices(std::vector<Type_api_camera>& camera_handles, std::vector<std::string>& serials); // Get initial configuration from attached hardware devices.
	void _update_config_from_devices(std::vector<std::string>& serials); // update config to match attached hardware
	void _init_hardware_settings(std::vector<Type_api_camera>& camera_handles); // initialize hardware parameters from configuration
	void _create_cameras(std::vector<Type_api_camera>& camera_handles, std::vector<std::string>& serials);
};
#endif // cwipc_realsense_MFCapture_hpp
