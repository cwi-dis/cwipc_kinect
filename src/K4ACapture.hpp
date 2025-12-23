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
    virtual ~K4ACapture() {}
    virtual bool config_reload_and_start_capturing(const char* configFilename) override final;
    bool seek(uint64_t timestamp) override final;

protected:
    K4ACapture();
    bool _capture_all_cameras() override final;
    uint64_t _get_best_timestamp() override final;
    virtual bool _apply_auto_config() override final;

private:
    virtual bool _setup_camera_hardware_parameters() override final; // initialize hardware parameters from configuration
    bool _open_cameras();
    bool _create_cameras();
};
