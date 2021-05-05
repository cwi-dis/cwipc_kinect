#ifndef cwipc_realsense_K4ABaseCamera_hpp
#define cwipc_realsense_K4ABaseCamera_hpp
#pragma once

#include <string>

template<typename Type_api_camera>
class K4ABaseCamera {
public:
	K4ABaseCamera(const std::string& _Classname, Type_api_camera _handle)
	:	CLASSNAME(_Classname),
		camera_handle(_handle)
	{

	}
	std::string CLASSNAME;
	Type_api_camera camera_handle;
};
#endif // cwipc_realsense_K4ABaseCamera_hpp