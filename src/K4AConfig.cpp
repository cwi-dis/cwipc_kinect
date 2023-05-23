//
//  utils.cpp
//
//  Created by Fons Kuijk on 12-12-18.
//

#include "K4AConfig.hpp"
#include "tinyxml.h"


#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#define _MY_JSON_GET(jsonobj, name, config, attr) if (jsonobj.contains(#name)) jsonobj.at(#name).get_to(config.attr)
#define _MY_JSON_PUT(jsonobj, name, config, attr) jsonobj[#name] = config.attr

//enable this to print memory statistics
#undef MEMORY_DEBUG 
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


void from_json(const json& json_data, K4ACaptureConfig& config) {
    // version and type should already have been checked.

    json system_data = json_data.at("system");
    _MY_JSON_GET(system_data, color_height, config, color_height);
    _MY_JSON_GET(system_data, depth_height, config, depth_height);
    _MY_JSON_GET(system_data, fps, config, fps);
    _MY_JSON_GET(system_data, single_tile, config, single_tile);
    _MY_JSON_GET(system_data, sync_master_serial, config, sync_master_serial);
    _MY_JSON_GET(system_data, color_exposure_time, config.camera_processing, color_exposure_time);
    _MY_JSON_GET(system_data, color_whitebalance, config.camera_processing, color_whitebalance);
    _MY_JSON_GET(system_data, color_brightness, config.camera_processing, color_brightness);
    _MY_JSON_GET(system_data, color_contrast, config.camera_processing, color_contrast);
    _MY_JSON_GET(system_data, color_saturation, config.camera_processing, color_saturation);
    _MY_JSON_GET(system_data, color_gain, config.camera_processing, color_gain);
    _MY_JSON_GET(system_data, color_powerline_frequency, config.camera_processing, color_powerline_frequency);
    _MY_JSON_GET(system_data, map_color_to_depth, config.camera_processing, map_color_to_depth);

    json postprocessing = json_data.at("postprocessing");
    _MY_JSON_GET(postprocessing, greenscreenremoval, config, greenscreen_removal);
    _MY_JSON_GET(postprocessing, height_min, config, height_min);
    _MY_JSON_GET(postprocessing, height_max, config, height_max);
    _MY_JSON_GET(postprocessing, radius_filter, config, radius_filter);

    json depthfilterparameters = postprocessing.at("depthfilterparameters");
    _MY_JSON_GET(depthfilterparameters, do_threshold, config.camera_processing, do_threshold);
    _MY_JSON_GET(depthfilterparameters, threshold_near, config.camera_processing, threshold_near);
    _MY_JSON_GET(depthfilterparameters, threshold_far, config.camera_processing, threshold_far);
    _MY_JSON_GET(depthfilterparameters, depth_x_erosion, config.camera_processing, depth_x_erosion);
    _MY_JSON_GET(depthfilterparameters, depth_y_erosion, config.camera_processing, depth_y_erosion);
    
    json skeleton = json_data.at("skeleton");
    _MY_JSON_GET(skeleton, sensor_orientation, config, bt_sensor_orientation);
    _MY_JSON_GET(skeleton, processing_mode, config, bt_processing_mode);
    _MY_JSON_GET(skeleton, model_path, config, bt_model_path);

    json cameras = json_data.at("camera");
    int camera_index = 0;
    for (json::iterator it = cameras.begin(); it != cameras.end(); it++) {
        json camera = *it;
        K4ACameraConfig cd;
        pcl::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
        default_trafo->setIdentity();
        cd.trafo = default_trafo;
        cd.intrinsicTrafo = default_trafo;
        _MY_JSON_GET(camera, serial, cd, serial);
        _MY_JSON_GET(camera, type, cd, type);
        _MY_JSON_GET(camera, disabled, cd, disabled);
        _MY_JSON_GET(camera, filename, cd, filename);
        if (camera.contains("trafo")) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 4; y++) {
                    (*cd.trafo)(x, y) = camera["trafo"][x][y];
                }
            }
        }
        if (camera.contains("intrinsicTrafo")) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 4; y++) {
                    (*cd.intrinsicTrafo)(x, y) = camera["intrinsicTrafo"][x][y];
                }
            }
        }
        // xxxjack should check whether the camera with this serial already exists
        config.all_camera_configs.push_back(cd);
        camera_index++;
    }
}

void to_json(json& json_data, const K4ACaptureConfig& config) {

    json cameras;
    int camera_index = 0;
    for (K4ACameraConfig cd : config.all_camera_configs) {
        json camera;
        _MY_JSON_PUT(camera, serial, cd, serial);
        _MY_JSON_PUT(camera, type, cd, type);
        camera["trafo"] = {
            {(*cd.trafo)(0, 0), (*cd.trafo)(0, 1), (*cd.trafo)(0, 2), (*cd.trafo)(0, 3)},
            {(*cd.trafo)(1, 0), (*cd.trafo)(1, 1), (*cd.trafo)(1, 2), (*cd.trafo)(1, 3)},
            {(*cd.trafo)(2, 0), (*cd.trafo)(2, 1), (*cd.trafo)(2, 2), (*cd.trafo)(2, 3)},
            {(*cd.trafo)(3, 0), (*cd.trafo)(3, 1), (*cd.trafo)(3, 2), (*cd.trafo)(3, 3)},
        };
        cameras[camera_index] = camera;
        camera_index++;
    }
    json_data["cameras"] = cameras;
#if xxxjack_notyet
    json depthfilterparameters;
    _MY_JSON_PUT(depthfilterparameters, do_decimation, config.camera_processing, do_decimation);
    _MY_JSON_PUT(depthfilterparameters, decimation_value, config.camera_processing, decimation_value);
    _MY_JSON_PUT(depthfilterparameters, do_threshold, config.camera_processing, do_threshold);
    _MY_JSON_PUT(depthfilterparameters, threshold_near, config.camera_processing, threshold_near);
    _MY_JSON_PUT(depthfilterparameters, threshold_far, config.camera_processing, threshold_far);
    _MY_JSON_PUT(depthfilterparameters, do_spatial, config.camera_processing, do_spatial);
    _MY_JSON_PUT(depthfilterparameters, spatial_iterations, config.camera_processing, spatial_iterations);
    _MY_JSON_PUT(depthfilterparameters, spatial_alpha, config.camera_processing, spatial_alpha);
    _MY_JSON_PUT(depthfilterparameters, spatial_delta, config.camera_processing, spatial_delta);
    _MY_JSON_PUT(depthfilterparameters, spatial_filling, config.camera_processing, spatial_filling);
    _MY_JSON_PUT(depthfilterparameters, do_temporal, config.camera_processing, do_temporal);
    _MY_JSON_PUT(depthfilterparameters, temporal_alpha, config.camera_processing, temporal_alpha);
    _MY_JSON_PUT(depthfilterparameters, temporal_delta, config.camera_processing, temporal_delta);
    _MY_JSON_PUT(depthfilterparameters, temporal_percistency, config.camera_processing, temporal_percistency);

    json postprocessing;
    postprocessing["depthfilterparameters"] = depthfilterparameters;

    _MY_JSON_PUT(postprocessing, greenscreenremoval, config, greenscreen_removal);
    _MY_JSON_PUT(postprocessing, height_min, config, height_min);
    _MY_JSON_PUT(postprocessing, height_max, config, height_max);
    json_data["postprocessing"] = postprocessing;

    json system_data;
    _MY_JSON_PUT(system_data, usb2width, config, usb2_width);
    _MY_JSON_PUT(system_data, usb2height, config, usb2_height);
    _MY_JSON_PUT(system_data, usb2fps, config, usb2_fps);
    _MY_JSON_PUT(system_data, usb3width, config, usb3_width);
    _MY_JSON_PUT(system_data, usb3height, config, usb3_height);
    _MY_JSON_PUT(system_data, usb3fps, config, usb3_fps);
    _MY_JSON_PUT(system_data, usb2allowed, config, usb2allowed);
    _MY_JSON_PUT(system_data, density_preferred, config, density);
    _MY_JSON_PUT(system_data, exposure, config, exposure);
    _MY_JSON_PUT(system_data, whitebalance, config, whitebalance);
    _MY_JSON_PUT(system_data, backlight_compensation, config, backlight_compensation);
    _MY_JSON_PUT(system_data, laser_power, config, laser_power);
    json_data["system"] = system_data;
#endif
    json_data["version"] = 3;
    json_data["type"] = "kinect";
}

bool cwipc_k4a_jsonfile2config(const char* filename, K4ACaptureConfig* config) {
    json json_data;
    try {
        std::ifstream f(filename);
        if (!f.is_open()) {
            cwipc_k4a_log_warning(std::string("CameraConfig ") + filename + " not found");
            return false;
        }
        json_data = json::parse(f);

        int version = 0;
        json_data.at("version").get_to(version);
        if (version != 3) {
            cwipc_k4a_log_warning(std::string("CameraConfig ") + filename + "ignored, is not version 3");
            return false;
        }
        std::string type;
        json_data.at("type").get_to(type);
        if (type != "kinect") {
            cwipc_k4a_log_warning(std::string("CameraConfig ") + filename + "ignored, is not kinect but " + type);
            return false;
        }
        from_json(json_data, *config);
    }
    catch (const std::exception& e) {
        cwipc_k4a_log_warning(std::string("CameraConfig ") + filename + ": exception " + e.what());
        return false;
    }
    json dbg_result;
    to_json(dbg_result, *config);
    std::cerr << "xxxjack debug json parse result: \n" << dbg_result << "\n";
    return true;
}

bool cwipc_k4a_jsonbuffer2config(const char* jsonBuffer, K4ACaptureConfig* config) {
    json json_data;
    try {
        json_data = json::parse(jsonBuffer);

        int version = 0;
        json_data.at("version").get_to(version);
        if (version != 3) {
            cwipc_k4a_log_warning(std::string("CameraConfig ") + "(inline buffer) " + "ignored, is not version 3");
            return false;
        }
        std::string type;
        json_data.at("type").get_to(type);
        if (type != "kinect") {
            cwipc_k4a_log_warning(std::string("CameraConfig ") + "(inline buffer) " + "ignored, is not kinect but " + type);
            return false;
        }
        from_json(json_data, *config);
    }
    catch (const std::exception& e) {
        cwipc_k4a_log_warning(std::string("CameraConfig ") + "(inline buffer) " + ": exception " + e.what());
        return false;
    }
    json dbg_result;
    to_json(dbg_result, *config);
    std::cerr << "xxxjack debug json parse result: \n" << dbg_result << "\n";
    return true;
}

std::string cwipc_k4a_config2jsonbuffer(K4ACaptureConfig* config) {
    json result;
    to_json(result, *config);
    return result.dump();
}

// read and restore the camera transformation setting as stored in the configuration document
bool cwipc_k4a_xmlfile2config(const char* filename, K4ACaptureConfig* config)
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
		systemElement->QueryIntAttribute("color_exposure_time", &(config->camera_processing.color_exposure_time));
		systemElement->QueryIntAttribute("color_whitebalance", &(config->camera_processing.color_whitebalance));
		systemElement->QueryIntAttribute("color_backlight_compensation", &(config->camera_processing.color_backlight_compensation));
		systemElement->QueryIntAttribute("color_brightness", &(config->camera_processing.color_brightness));
		systemElement->QueryIntAttribute("color_contrast", &(config->camera_processing.color_contrast));
		systemElement->QueryIntAttribute("color_saturation", &(config->camera_processing.color_saturation));
		systemElement->QueryIntAttribute("color_sharpness", &(config->camera_processing.color_sharpness));
		systemElement->QueryIntAttribute("color_gain", &(config->camera_processing.color_gain));
		systemElement->QueryIntAttribute("color_powerline_frequency", &(config->camera_processing.color_powerline_frequency));
		systemElement->QueryBoolAttribute("map_color_to_depth", &(config->camera_processing.map_color_to_depth));
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
			parameterElement->QueryBoolAttribute("do_threshold", &(config->camera_processing.do_threshold));
			parameterElement->QueryDoubleAttribute("threshold_near", &(config->camera_processing.threshold_near));
			parameterElement->QueryDoubleAttribute("threshold_far", &(config->camera_processing.threshold_far));
			parameterElement->QueryIntAttribute("depth_x_erosion", &(config->camera_processing.depth_x_erosion));
			parameterElement->QueryIntAttribute("depth_y_erosion", &(config->camera_processing.depth_y_erosion));
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
    
	bool allnewcameras = config->all_camera_configs.size() == 0; // if empty we have to set up a new administration
	int registeredcameras = 0;

	// now get the per camera info
	TiXmlElement* cameraElement = configElement->FirstChildElement("camera");
	while (cameraElement)
	{
		const char * serial = cameraElement->Attribute("serial");
		K4ACameraConfig* cd;

		int i = 0;
		while (i < config->all_camera_configs.size()) {
			if (config->all_camera_configs[i].serial == serial) {
				cameraElement->QueryBoolAttribute("disabled", &(config->all_camera_configs[i].disabled));
				cd = &config->all_camera_configs[i];
				break;
			}
			i++;
		}
		if (i == config->all_camera_configs.size()) {
			// this camera was not in the admin yet
			if (!allnewcameras)
				loadOkay = false;

			cd = new K4ACameraConfig();
			pcl::shared_ptr<Eigen::Affine3d> trafo(new Eigen::Affine3d());
			pcl::shared_ptr<Eigen::Affine3d> intrinsicTrafo(new Eigen::Affine3d());
			intrinsicTrafo->setIdentity();
			cd->serial = cameraElement->Attribute("serial");
			cameraElement->QueryBoolAttribute("disabled", &cd->disabled);
			cd->trafo = trafo;
			cd->intrinsicTrafo = intrinsicTrafo;
			cd->cameraposition = { 0, 0, 0 };
			config->all_camera_configs.push_back(*cd);
			cd = &config->all_camera_configs.back();
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
	if (config->all_camera_configs.size() != registeredcameras)
		loadOkay = false;

    if (!loadOkay) {
        cwipc_k4a_log_warning("Available hardware camera configuration does not match configuration file");
    }
	return loadOkay;
}


