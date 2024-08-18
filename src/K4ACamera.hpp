#pragma once

#include "K4ABaseCamera.hpp"

class K4ACamera : public K4ABaseCamera<k4a_device_t> {
    typedef k4a_device_t Type_api_camera;
    //const std::string CLASSNAME = "cwipc_kinect: K4ACamera";
    K4ACamera(const K4ACamera&);  // Disable copy constructor
    K4ACamera& operator=(const K4ACamera&); // Disable assignment

public:
    K4ACamera(Type_api_camera _handle, K4ACaptureConfig& configuration, int _camera_index, K4ACameraConfig& _camData);
    virtual ~K4ACamera() {}

    bool start() override;
    virtual void start_capturer() override;
    void stop() override;
    bool capture_frameset();

protected:
    virtual void _start_capture_thread() override;
    virtual void _capture_thread_main() override;
    k4a_image_t _uncompress_color_image(k4a_capture_t capture, k4a_image_t color_image) override;

    k4a_record_t recorder = nullptr;
private:
    bool _setup_device(k4a_device_configuration_t& device_config);
};
