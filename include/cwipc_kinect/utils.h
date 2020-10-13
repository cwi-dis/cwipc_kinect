//
//  utils.h
//
//  Created by Fons Kuijk on 12-12-18.
//

#ifndef cwipw_realsense_utils_h
#define cwipw_realsense_utils_h
#pragma once

#ifndef _CWIPC_KINECT_EXPORT
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_KINECT_EXPORT __declspec(dllimport)
#else
#define _CWIPC_KINECT_EXPORT 
#endif
#endif

#include <cstdint>
#include "cwipc_util/api_pcl.h"

struct K4ACaptureConfig;

_CWIPC_KINECT_EXPORT void cwipc_k4a_log_warning(std::string warning);
_CWIPC_KINECT_EXPORT extern char **cwipc_k4a_warning_store;

_CWIPC_KINECT_EXPORT bool cwipc_k4a_file2config(const char* filename, K4ACaptureConfig* config);


_CWIPC_KINECT_EXPORT bool cwipc_k4a_noChromaRemoval(cwipc_pcl_point* p);

#endif /* cwipw_realsense_utils_h */
