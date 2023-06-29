#ifndef cwipc_realsense_K4AOfflineCapture_hpp
#define cwipc_realsense_K4AOfflineCapture_hpp
#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <k4a/k4a.h>

#include "K4ABaseCapture.hpp"
#include "K4AOfflineCamera.hpp"

class K4AOfflineCapture : public K4ABaseCapture<k4a_playback_t, K4AOfflineCamera> {
	typedef k4a_playback_t Type_api_camera;
	typedef K4AOfflineCamera Type_our_camera;
public:
	static int count_devices() { return 0; }
	static K4AOfflineCapture* factory() { return new K4AOfflineCapture(); }
	// methods
	virtual ~K4AOfflineCapture() {}
	virtual bool config_reload(const char* configFilename) override;
	bool seek(uint64_t timestamp) override;

protected:
	K4AOfflineCapture();
	bool _capture_all_cameras() override;
	uint64_t _get_best_timestamp() override;
	virtual bool _apply_default_config() override { return false; }
private:
	void _create_cameras(std::vector<Type_api_camera>& camera_handles);
	bool _open_recording_files(std::vector<Type_api_camera>& camera_handles); // Open the recordings
private:
	// variables
	bool sync_inuse = false;
	int master_id = -1;

};
#endif // cwipc_realsense_RS2Offline_hpp
