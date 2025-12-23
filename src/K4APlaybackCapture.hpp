#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <k4a/k4a.h>

#include "K4ABaseCapture.hpp"
#include "K4APlaybackCamera.hpp"

class K4APlaybackCapture : public K4ABaseCapture<k4a_playback_t, K4APlaybackCamera> {
    typedef k4a_playback_t Type_api_camera;
    typedef K4APlaybackCamera Type_our_camera;

public:
    static int count_devices() {
        return 0;
    }

    static K4APlaybackCapture* factory() {
        return new K4APlaybackCapture();
    }

    // methods
    virtual ~K4APlaybackCapture();
    virtual bool config_reload_and_start_capturing(const char* configFilename) override;
    bool seek(uint64_t timestamp) override;

protected:
    K4APlaybackCapture();
    bool _capture_all_cameras() override;
    uint64_t _get_best_timestamp() override;
    virtual bool _apply_auto_config() override { return false; }

private:
    virtual bool _setup_camera_hardware_parameters() override final { return true; }
    void _create_cameras(std::vector<Type_api_camera>& camera_handles);
    bool _open_recording_files(std::vector<Type_api_camera>& camera_handles, const char *configFilename=nullptr); // Open the recordings

    // variables
    bool sync_inuse = false;
    int master_id = -1;
};
