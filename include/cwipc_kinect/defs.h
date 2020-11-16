//
//  defs.h
//
//  Created by Fons Kuijk on 12-12-18.
//

#ifndef cwipc_kinect_defs_h
#define cwipc_kinect_defs_h

#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>

// Define to enable optional dumping of RGB video frames (to test hardware sync)
#define WITH_DUMP_VIDEO_FRAMES

//
// Definitions of types used across cwipc_kinect, cwipc_codec and cwipc_util.
//
#include "cwipc_util/api_pcl.h"

struct K4ACameraSettings {
	bool do_threshold = true;
	double threshold_near = 0.15;         // float, near point for distance threshold
	double threshold_far = 6.0;           // float, far point for distance threshold
	int32_t color_exposure_time = -1;     // default for manual: 40000;
	int32_t color_whitebalance = 3 - 1;   // default for manual: 160;
	int32_t color_backlight_compensation = -1;     // default for manual: 0;
	int32_t color_brightness = -1;        // default for manual: 128;
	int32_t color_contrast = -1;          // default for manual: 5;
	int32_t color_saturation = -1;        // default for manual: 32;
	int32_t color_sharpness = -1;         // default for manual: 2;
	int32_t color_gain = -1;              // default for manual: 100;
	int32_t color_powerline_frequency = -1;     // default for manual: 2;
};

struct K4ACameraData {
	std::string serial;		// Serial number of this camera
	std::string type = "kinect";       // Camera type (must be realsense)
	boost::shared_ptr<Eigen::Affine3d> trafo;	//!< Transformation matrix from camera coorindates to world coordinates
	boost::shared_ptr<Eigen::Affine3d> intrinsicTrafo;	//!< offline only: matrix to convert color to depth coordinates
	cwipc_vector cameraposition;	//!< Position of this camera in real world coordinates
	cwipc_pcl_pointcloud cloud;	//!< Pointcloud most recently captured
};

struct K4ACaptureConfig {

	int color_height = 720;                     // width of color frame (720, 1080 and various other values allowed, see kinect docs)
	int depth_height = 576;                // width of depth frame (288, 576, 512 and 1024 allowed)
	int fps = 30;                         // capture fps (5, 15 and 30 allowed)
	bool greenscreen_removal = false;	  // If true include greenscreen removal
	bool depth_filtering = false;         // If true perform post filtering on depth frame
	double height_min = 0.0;			  // If height_min != height_max perform height filtering
	double height_max = 0.0;			  // If height_min != height_max perform height filtering
	double cloud_resolution = 0.0;        // Resolution of voxelized pointclouds

	std::string sync_master_serial = "";  // If empty run without sync. If non-empty this camera is the sync master
	// special features
	std::string cwi_special_feature = ""; // Specifier for temporary development specific feature

	K4ACameraSettings default_camera_settings;
	// realsense specific post processing filtering

	// per camera data
	std::vector<K4ACameraData> cameraData;
};
#endif /* cwipc_kinect_defs_h */
