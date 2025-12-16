//
//  defs.h
//
//  Created by Fons Kuijk on 12-12-18.
//
#pragma once

#include <cstdint>
#include <thread>

#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>

//
// Definitions of types used across cwipc_kinect, cwipc_codec and cwipc_util.
//
#include "cwipc_util/api_pcl.h"
#include "cwipc_util/internal.h"
#include <k4abt.h>

struct K4ACameraProcessingParameters {
    bool do_threshold = true;
    double threshold_near = 0.15;         // float, near point for distance threshold
    double threshold_far = 6.0;           // float, far point for distance threshold
    int depth_x_erosion = 0;        // How many valid depth pixels to remove in camera x direction
    int depth_y_erosion = 0;        // How many valid depth pixels to remove in camera y direction
    int32_t color_exposure_time = -1;     // default for manual: 40000;
    int32_t color_whitebalance = -1;   // default for manual: 3160; range(2500-12500)
    int32_t color_backlight_compensation = 0;     // default for manual: 0;
    int32_t color_brightness = 128;        // default for manual: 128;
    int32_t color_contrast = 5;          // default for manual: 5;
    int32_t color_saturation = 32;        // default for manual: 32;
    int32_t color_sharpness = 2;         // default for manual: 2;
    int32_t color_gain = 0;              // default for manual: 100;
    int32_t color_powerline_frequency = 2;     // default for manual: 2;
    bool map_color_to_depth = false; // default DEPTH_TO_COLOR
};

struct K4ACameraConfig : CwipcBaseCameraConfig {
    bool disabled = false;  // to easily disable cameras without altering to much the cameraconfig.
    void* handle = nullptr; // Will be set if camera is opened
    std::string serial;   // Serial number of this camera
    std::string type = "kinect";       // Camera type (must be realsense)
    std::string filename;   // Filename for offline captures
    pcl::shared_ptr<Eigen::Affine3d> trafo; //!< Transformation matrix from camera coorindates to world coordinates
    pcl::shared_ptr<Eigen::Affine3d> intrinsicTrafo;  //!< offline only: matrix to convert color to depth coordinates
    cwipc_vector cameraposition = { 0, 0, 0 };  //!< Position of this camera in real world coordinates
};

struct K4ACaptureConfig : CwipcBaseCaptureConfig {
    int color_height = 720;                     // width of color frame (720, 1080 and various other values allowed, see kinect docs)
    int depth_height = 576;                // width of depth frame (288, 576, 512 and 1024 allowed)
    int fps = 30;                         // capture fps (5, 15 and 30 allowed)
    bool greenscreen_removal = false;   // If true include greenscreen removal
    double height_min = 0.0;        // If height_min != height_max perform height filtering
    double height_max = 0.0;        // If height_min != height_max perform height filtering
    double radius_filter = 0.0;     // If radius_filter > 0 we will remove all points further than radius_filter from the (0,1,0) axis
    int single_tile = -1;       // if singletile >=0 all the points will be the specified integer

    std::string sync_master_serial = "";  // If empty run without sync. If non-empty this camera is the sync master
    bool ignore_sync = false;  // If true dont look at camera master/sub mode in files.
    K4ACameraProcessingParameters camera_processing;
    int bt_sensor_orientation = -1; // Override k4abt sensor_orientation (if >= 0)
    int bt_processing_mode = -1;  // Override k4abt processing_mode (if >= 0)
    std::string bt_model_path = "";     // Override k4abt model path
    std::string record_to_directory = ""; // If non-empty all camera streams will be recorded to this directory.
    bool new_timestamps = false; // If true new timestamps are generated (otherwise original timestamps from capture time)
    // We could probably also allow overriding GPU id and model path, but no need for now.
    // per camera data
    std::vector<K4ACameraConfig> all_camera_configs;
};

struct K4ACaptureConfig;

void cwipc_k4a_log_warning(std::string warning);
extern char** cwipc_k4a_warning_store;

bool cwipc_k4a_jsonfile2config(const char* filename, K4ACaptureConfig* config, std::string typeWanted);
bool cwipc_k4a_jsonbuffer2config(const char* filename, K4ACaptureConfig* config, std::string typeWanted);
std::string cwipc_k4a_config2string(K4ACaptureConfig* config);

#ifdef _WIN32
#include <Windows.h>

inline void _cwipc_setThreadName(std::thread* thr, const wchar_t* name) {
    HANDLE threadHandle = static_cast<HANDLE>(thr->native_handle());
    SetThreadDescription(threadHandle, name);
}

#else
inline void _cwipc_setThreadName(std::thread* thr, const wchar_t* name) {}
#endif
