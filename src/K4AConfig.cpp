//
//  utils.cpp
//
//  Created by Fons Kuijk on 12-12-18.
//

//enable this to print memory statistics
#undef MEMORY_DEBUG 
#include "K4AConfig.hpp"
#include "tinyxml.h"
#ifdef MEMORY_DEBUG
	#include "psapi.h"
#endif

static std::string k4a_most_recent_warning;
char **cwipc_k4a_warning_store;

void cwipc_k4a_log_warning(std::string warning)
{
    std::cerr << "cwipc_kinect: Warning: " << warning << std::endl;
    if (cwipc_k4a_warning_store) {
        k4a_most_recent_warning = warning;
        *cwipc_k4a_warning_store = (char *)k4a_most_recent_warning.c_str();
    }
}

#ifdef MEMORY_DEBUG
void print_stats(std::string header) { 
	//This function can help debugging memory problems. Code taken from: https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
	//More info here: https://docs.microsoft.com/en-us/windows/win32/api/sysinfoapi/ns-sysinfoapi-memorystatusex
	//VALUES COME IN BYTES

	Sleep(500); //give time to the system to update
	//Total virtual memory
	MEMORYSTATUSEX memInfo;
	memInfo.dwLength = sizeof(MEMORYSTATUSEX);
	GlobalMemoryStatusEx(&memInfo);
	DWORDLONG totalVirtualMem = memInfo.ullTotalPageFile; // Note: The name "TotalPageFile" is a bit misleading here. In reality this parameter gives the "Virtual Memory Size", which is size of swap file plus installed RAM.

	//Virtual memory currently used:
	DWORDLONG virtualMemUsed = memInfo.ullTotalPageFile - memInfo.ullAvailPageFile;

	//Virtual Memory currently used by current process:
	PROCESS_MEMORY_COUNTERS_EX pmc;
	GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));
	SIZE_T virtualMemUsedByMe = pmc.PrivateUsage;

	//Total Physical Memory(RAM) :
	DWORDLONG totalPhysMem = memInfo.ullTotalPhys;

	//Physical Memory currently used :
	DWORDLONG physMemUsed = memInfo.ullTotalPhys - memInfo.ullAvailPhys;

	//Physical Memory currently used by current process:
	SIZE_T physMemUsedByMe = pmc.WorkingSetSize;
	std::cout << "STATS : " << header << std::endl;
	std::cout << "\t## RAM(MB): Total:" << totalPhysMem / 1000000 << " | Used:" << physMemUsed / 1000000 << "| UsedByMe:" << physMemUsedByMe / 1000000 << std::endl;
	std::cout << "\t## VRAM(MB): Total:" << totalVirtualMem / 1000000 << " | Used:" << virtualMemUsed / 1000000 << "| UsedByMe:" << virtualMemUsedByMe / 1000000 << std::endl;
}
#endif // MEMORY_DEBUG

// read and restore the camera transformation setting as stored in the configuration document
bool cwipc_k4a_file2config(const char* filename, K4ACaptureConfig* config)
{
	TiXmlDocument doc(filename);
	bool loadOkay = doc.LoadFile();
	if (!loadOkay)
	{
        cwipc_k4a_log_warning(std::string("Failed to load configfile ") + filename + ", using default matrices");
		return false;
	}

	TiXmlHandle docHandle(&doc);
	TiXmlElement* configElement = docHandle.FirstChild("file").FirstChild("CameraConfig").ToElement();
    int version = -1;
    configElement->QueryIntAttribute("version", &version);
    if (version != 2) {
        cwipc_k4a_log_warning(std::string("CameraConfig ") + filename + " is not version 2");
    }
	// get the system related information
	TiXmlElement* systemElement = configElement->FirstChildElement("system");
	if (systemElement) {
		systemElement->QueryIntAttribute("color_height", &(config->color_height));
		systemElement->QueryIntAttribute("depth_height", &(config->depth_height));
		systemElement->QueryIntAttribute("fps", &(config->fps));
		systemElement->QueryIntAttribute("single_tile", &(config->single_tile));
		const char* serial_str = systemElement->Attribute("sync_master_serial");
		if (serial_str && *serial_str) {
			config->sync_master_serial = serial_str;
		}
#ifdef notyet
		systemElement->QueryBoolAttribute("colormaster", &(config->colormaster));
#endif
		systemElement->QueryIntAttribute("color_exposure_time", &(config->camera_config.color_exposure_time));
		systemElement->QueryIntAttribute("color_whitebalance", &(config->camera_config.color_whitebalance));
		systemElement->QueryIntAttribute("color_backlight_compensation", &(config->camera_config.color_backlight_compensation));
		systemElement->QueryIntAttribute("color_brightness", &(config->camera_config.color_brightness));
		systemElement->QueryIntAttribute("color_contrast", &(config->camera_config.color_contrast));
		systemElement->QueryIntAttribute("color_saturation", &(config->camera_config.color_saturation));
		systemElement->QueryIntAttribute("color_sharpness", &(config->camera_config.color_sharpness));
		systemElement->QueryIntAttribute("color_gain", &(config->camera_config.color_gain));
		systemElement->QueryIntAttribute("color_powerline_frequency", &(config->camera_config.color_powerline_frequency));
		systemElement->QueryBoolAttribute("map_color_to_depth", &(config->camera_config.map_color_to_depth));
}

    // get the processing related information
    TiXmlElement* postprocessingElement = configElement->FirstChildElement("postprocessing");
    if (postprocessingElement) {
		postprocessingElement->QueryBoolAttribute("greenscreenremoval", &(config->greenscreen_removal));
		postprocessingElement->QueryDoubleAttribute("height_min", &(config->height_min));
		postprocessingElement->QueryDoubleAttribute("height_max", &(config->height_max));
		postprocessingElement->QueryDoubleAttribute("radius_filter", &(config->radius_filter));
        TiXmlElement* parameterElement = postprocessingElement->FirstChildElement("depthfilterparameters");
        if (parameterElement) {
			parameterElement->QueryBoolAttribute("do_threshold", &(config->camera_config.do_threshold));
			parameterElement->QueryDoubleAttribute("threshold_near", &(config->camera_config.threshold_near));
			parameterElement->QueryDoubleAttribute("threshold_far", &(config->camera_config.threshold_far));
			parameterElement->QueryIntAttribute("depth_x_erosion", &(config->camera_config.depth_x_erosion));
			parameterElement->QueryIntAttribute("depth_y_erosion", &(config->camera_config.depth_y_erosion));
		}
		TiXmlElement* btElement = configElement->FirstChildElement("skeleton");
		if (btElement) {
			btElement->QueryIntAttribute("sensor_orientation", &(config->bt_sensor_orientation));
			btElement->QueryIntAttribute("processing_mode", &(config->bt_processing_mode));
			const char* model_path = btElement->Attribute("model_path");
			if (model_path && *model_path) {
				config->bt_model_path = std::string(model_path);
			}
		}
    }
    
	bool allnewcameras = config->camera_data.size() == 0; // if empty we have to set up a new administration
	int registeredcameras = 0;

	// now get the per camera info
	TiXmlElement* cameraElement = configElement->FirstChildElement("camera");
	while (cameraElement)
	{
		const char * serial = cameraElement->Attribute("serial");
		K4ACameraData* cd;

		int i = 0;
		while (i < config->camera_data.size()) {
			if (config->camera_data[i].serial == serial) {
				cameraElement->QueryBoolAttribute("disabled", &(config->camera_data[i].disabled));
				cd = &config->camera_data[i];
				break;
			}
			i++;
		}
		if (i == config->camera_data.size()) {
			// this camera was not in the admin yet
			if (!allnewcameras)
				loadOkay = false;

			cd = new K4ACameraData();
			pcl::shared_ptr<Eigen::Affine3d> trafo(new Eigen::Affine3d());
			pcl::shared_ptr<Eigen::Affine3d> intrinsicTrafo(new Eigen::Affine3d());
			intrinsicTrafo->setIdentity();
			cd->serial = cameraElement->Attribute("serial");
			cameraElement->QueryBoolAttribute("disabled", &cd->disabled);
			cd->trafo = trafo;
			cd->intrinsicTrafo = intrinsicTrafo;
			cd->cameraposition = { 0, 0, 0 };
			config->camera_data.push_back(*cd);
			cd = &config->camera_data.back();
		}

        std::string type = cameraElement->Attribute("type");
        if (type != "") {
            cd->type = type;
        }

		auto camerafile_c = cameraElement->Attribute("filename");
		if (camerafile_c != NULL && camerafile_c[0] != '\0') {
			std::string camerafile(camerafile_c);
			if (camerafile.substr(0, 1) != "/") {
				// Relative path (so don''t use windows drive numbers;-)
				std::string filename_cpp(filename);
				size_t lastSlashPos = filename_cpp.find_last_of("/\\");
				if (lastSlashPos != std::string::npos) {
					camerafile = filename_cpp.substr(0, lastSlashPos + 1) + camerafile;
				}
			}
			cd->filename = camerafile;

		}
        
		TiXmlElement *trafo = cameraElement->FirstChildElement("trafo");
		if (trafo) {
			TiXmlElement *val = trafo->FirstChildElement("values");
			val->QueryDoubleAttribute("v00", &(*cd->trafo)(0, 0));
			val->QueryDoubleAttribute("v01", &(*cd->trafo)(0, 1));
			val->QueryDoubleAttribute("v02", &(*cd->trafo)(0, 2));
			val->QueryDoubleAttribute("v03", &(*cd->trafo)(0, 3));
			val->QueryDoubleAttribute("v10", &(*cd->trafo)(1, 0));
			val->QueryDoubleAttribute("v11", &(*cd->trafo)(1, 1));
			val->QueryDoubleAttribute("v12", &(*cd->trafo)(1, 2));
			val->QueryDoubleAttribute("v13", &(*cd->trafo)(1, 3));
			val->QueryDoubleAttribute("v20", &(*cd->trafo)(2, 0));
			val->QueryDoubleAttribute("v21", &(*cd->trafo)(2, 1));
			val->QueryDoubleAttribute("v22", &(*cd->trafo)(2, 2));
			val->QueryDoubleAttribute("v23", &(*cd->trafo)(2, 3));
			val->QueryDoubleAttribute("v30", &(*cd->trafo)(3, 0));
			val->QueryDoubleAttribute("v31", &(*cd->trafo)(3, 1));
			val->QueryDoubleAttribute("v32", &(*cd->trafo)(3, 2));
			val->QueryDoubleAttribute("v33", &(*cd->trafo)(3, 3));
		}
		else
			loadOkay = false;

		// load optional intrinsicTrafo element (only for offline usage)
		trafo = cameraElement->FirstChildElement("intrinsicTrafo");
		if (trafo) {
			TiXmlElement *val = trafo->FirstChildElement("values");
			val->QueryDoubleAttribute("v00", &(*cd->intrinsicTrafo)(0, 0));
			val->QueryDoubleAttribute("v01", &(*cd->intrinsicTrafo)(0, 1));
			val->QueryDoubleAttribute("v02", &(*cd->intrinsicTrafo)(0, 2));
			val->QueryDoubleAttribute("v03", &(*cd->intrinsicTrafo)(0, 3));
			val->QueryDoubleAttribute("v10", &(*cd->intrinsicTrafo)(1, 0));
			val->QueryDoubleAttribute("v11", &(*cd->intrinsicTrafo)(1, 1));
			val->QueryDoubleAttribute("v12", &(*cd->intrinsicTrafo)(1, 2));
			val->QueryDoubleAttribute("v13", &(*cd->intrinsicTrafo)(1, 3));
			val->QueryDoubleAttribute("v20", &(*cd->intrinsicTrafo)(2, 0));
			val->QueryDoubleAttribute("v21", &(*cd->intrinsicTrafo)(2, 1));
			val->QueryDoubleAttribute("v22", &(*cd->intrinsicTrafo)(2, 2));
			val->QueryDoubleAttribute("v23", &(*cd->intrinsicTrafo)(2, 3));
			val->QueryDoubleAttribute("v30", &(*cd->intrinsicTrafo)(3, 0));
			val->QueryDoubleAttribute("v31", &(*cd->intrinsicTrafo)(3, 1));
			val->QueryDoubleAttribute("v32", &(*cd->intrinsicTrafo)(3, 2));
			val->QueryDoubleAttribute("v33", &(*cd->intrinsicTrafo)(3, 3));
		}

		registeredcameras++;
		cameraElement = cameraElement->NextSiblingElement("camera");
	}
	if (config->camera_data.size() != registeredcameras)
		loadOkay = false;

    if (!loadOkay) {
        cwipc_k4a_log_warning("Available hardware camera configuration does not match configuration file");
    }
	return loadOkay;
}


