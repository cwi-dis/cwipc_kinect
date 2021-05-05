#ifndef cwipc_realsense_K4ABaseCapture_hpp
#define cwipc_realsense_K4ABaseCapture_hpp
#pragma once

#include <string>
#include <mutex>
#include <condition_variable>

#include <k4a/k4a.h>
#include <k4abt.h>

#include "K4AConfig.hpp"

template<typename Type_api_camera, class Type_our_camera>
class K4ABaseCapture {
public:
	K4ACaptureConfig configuration;	//!< Complete configuration read from cameraconfig.xml
	bool no_cameras = true;	//<! True of no cameras attached
	bool eof = false;	//<! True when end-of-file seen on pointcloud source

protected:
	std::string CLASSNAME;	//!< For error, warning and debug messages only
	std::vector<Type_our_camera*> cameras;	//<! Cameras used by this capturer

	bool want_auxdata_rgb = false;	//<! True after caller requests this auxiliary data
	bool want_auxdata_depth = false;	//<! True after caller requests this auxiliary data
	bool want_auxdata_skeleton = false;	//<! True after caller requests this auxiliary data
	bool stopped = false;	//<! True when stopping capture

	uint64_t starttime = 0;	//!< Used only for statistics messages
	int numberOfPCsProduced = 0;	//!< Used only for statistics messages
	
	cwipc* mergedPC = nullptr;	//<! Merged pointcloud from all cameras
	std::mutex mergedPC_mutex;	//<! Lock for all mergedPC-related data structures
	bool mergedPC_is_fresh;	//<! True if mergedPC contains a freshly-created pointcloud
	std::condition_variable mergedPC_is_fresh_cv;	//<! Condition variable for signalling freshly-created pointcloud
	bool mergedPC_want_new;	//<! Set to true to request a new pointcloud
	std::condition_variable mergedPC_want_new_cv;	//<! Condition variable for signalling we want a new pointcloud
	
//xxxjack	k4abt_skeleton_t skeleton;	//<! Skeleton 
public:
	K4ABaseCapture(const std::string& _Classname)
	:	CLASSNAME(_Classname)
	{

	}

};
#endif // cwipc_realsense_K4ABaseCapture_hpp