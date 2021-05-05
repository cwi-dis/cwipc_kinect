#ifndef cwipc_realsense_K4ABaseCapture_hpp
#define cwipc_realsense_K4ABaseCapture_hpp
#pragma once

#include <string>

template<typename Type_api_camera, class Type_our_camera>
class K4ABaseCapture {
public:
	K4ABaseCapture(const std::string& _Classname)
		: CLASSNAME(_Classname)
	{

	}
	std::string CLASSNAME;
};
#endif // cwipc_realsense_K4ABaseCapture_hpp