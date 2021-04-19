//
//  utils.h
//
//  Created by Fons Kuijk on 12-12-18.
//

#ifndef cwipw_realsense_utils_h
#define cwipw_realsense_utils_h
#pragma once

#include <cstdint>
#include <thread>
#include "cwipc_util/api_pcl.h"

struct K4ACaptureConfig;

void cwipc_k4a_log_warning(std::string warning);
extern char **cwipc_k4a_warning_store;

bool cwipc_k4a_file2config(const char* filename, K4ACaptureConfig* config);

#ifdef _WIN32
#include <Windows.h>
inline void _cwipc_setThreadName(std::thread* thr, const wchar_t* name) {
	HANDLE threadHandle = static_cast<HANDLE>(thr->native_handle());
	SetThreadDescription(threadHandle, name);
}
#else
inline void _cwipc_setThreadName(std::thread* thr, const wchar_t* name) {}
#endif
#endif /* cwipw_realsense_utils_h */
