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
    virtual bool config_reload(const char* configFilename) override;
    bool seek(uint64_t timestamp) override;

protected:
    K4ACapture();
    bool _capture_all_cameras() override;
    uint64_t _get_best_timestamp() override;
    virtual bool _apply_auto_config() override;

private:
    bool _init_hardware_settings(); // initialize hardware parameters from configuration
    bool _open_cameras();
    bool _create_cameras();
};
