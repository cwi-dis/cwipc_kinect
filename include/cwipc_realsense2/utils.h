//
//  utils.h
//
//  Created by Fons Kuijk on 12-12-18.
//

#ifndef cwipw_realsense_utils_h
#define cwipw_realsense_utils_h
#pragma once

#ifndef _CWIPC_REALSENSE2_EXPORT
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllimport)
#else
#define _CWIPC_REALSENSE2_EXPORT 
#endif
#endif

_CWIPC_REALSENSE2_EXPORT bool mf_file2config(const char* filename, MFCaptureConfig* config);

// store the current camera transformation setting into a xml document
_CWIPC_REALSENSE2_EXPORT void mf_config2file(const char* filename, MFCaptureConfig* config);

_CWIPC_REALSENSE2_EXPORT bool mf_noChromaRemoval(cwipc_pcl_point* p);

#endif /* cwipw_realsense_utils_h */
