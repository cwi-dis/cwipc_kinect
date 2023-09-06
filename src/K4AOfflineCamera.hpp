#pragma once

#include "K4ABaseCamera.hpp"

class K4AOfflineCamera : public K4ABaseCamera<k4a_playback_t> {
    typedef k4a_playback_t Type_api_camera;
    const std::string CLASSNAME = "cwipc_kinect: K4AOfflineCamera";

public:
    uint64_t current_frameset_timestamp = 0; //!< Unsure? How is this different from get_capture_timestamp()?

private:
    K4AOfflineCamera(const K4AOfflineCamera&);  // Disable copy constructor
    K4AOfflineCamera& operator=(const K4AOfflineCamera&); // Disable assignment

public:
    K4AOfflineCamera(Type_api_camera _handle, K4ACaptureConfig& configuration, int _camera_index, K4ACameraConfig& _camData);
    virtual ~K4AOfflineCamera() {}

    bool start() override;
    void start_capturer() override;
    void stop() override;
    bool capture_frameset(uint64_t master_timestamp);
    bool seek(uint64_t timestamp);

protected:
    virtual void _start_capture_thread() override;
    virtual void _capture_thread_main() override;
    k4a_image_t _uncompress_color_image(k4a_capture_t capture, k4a_image_t color_image) override;

private:
    bool _prepare_next_valid_frame();
    bool _prepare_cond_next_valid_frame(uint64_t master_timestamp);

    uint64_t max_delay = 0;
    int capture_id = 0;
    k4a_record_configuration_t file_config;
};
