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

	int width = 1280;
	int height = 720;
	int fps = 30;
	bool greenscreen_removal = false;	  // If true include greenscreen removal
	bool depth_filtering = false;         // If true perform post filtering on depth frame
	double height_min = 0.0;			  // If height_min != height_max perform height filtering
	double height_max = 0.0;			  // If height_min != height_max perform height filtering
	double cloud_resolution = 0.0;        // Resolution of voxelized pointclouds

	// special features
	std::string cwi_special_feature = ""; // Specifier for temporary development specific feature

	K4ACameraSettings default_camera_settings;
	// realsense specific post processing filtering

	// per camera data
	std::vector<K4ACameraData> cameraData;
};
#endif /* cwipc_kinect_defs_h */
