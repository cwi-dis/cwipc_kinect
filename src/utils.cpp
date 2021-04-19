//
//  utils.cpp
//
//  Created by Fons Kuijk on 12-12-18.
//

#include "cwipc_kinect/private/K4AConfig.hpp"
#include "cwipc_kinect/private/utils.h"

#include "tinyxml.h"


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
		const char* serial_str = systemElement->Attribute("sync_master_serial");
		if (serial_str && *serial_str) {
			config->sync_master_serial = serial_str;
		}
#ifdef notyet
		systemElement->QueryBoolAttribute("colormaster", &(config->colormaster));
#endif
		systemElement->QueryIntAttribute("color_exposure_time", &(config->default_camera_settings.color_exposure_time));
		systemElement->QueryIntAttribute("color_whitebalance", &(config->default_camera_settings.color_whitebalance));
		systemElement->QueryIntAttribute("color_backlight_compensation", &(config->default_camera_settings.color_backlight_compensation));
		systemElement->QueryIntAttribute("color_brightness", &(config->default_camera_settings.color_brightness));
		systemElement->QueryIntAttribute("color_contrast", &(config->default_camera_settings.color_contrast));
		systemElement->QueryIntAttribute("color_saturation", &(config->default_camera_settings.color_saturation));
		systemElement->QueryIntAttribute("color_sharpness", &(config->default_camera_settings.color_sharpness));
		systemElement->QueryIntAttribute("color_gain", &(config->default_camera_settings.color_gain));
		systemElement->QueryIntAttribute("color_powerline_frequency", &(config->default_camera_settings.color_powerline_frequency));
}

    // get the processing related information
    TiXmlElement* postprocessingElement = configElement->FirstChildElement("postprocessing");
    if (postprocessingElement) {
		postprocessingElement->QueryBoolAttribute("greenscreenremoval", &(config->greenscreen_removal));
		postprocessingElement->QueryDoubleAttribute("height_min", &(config->height_min));
		postprocessingElement->QueryDoubleAttribute("height_max", &(config->height_max));
        TiXmlElement* parameterElement = postprocessingElement->FirstChildElement("depthfilterparameters");
        if (parameterElement) {
			parameterElement->QueryBoolAttribute("do_threshold", &(config->default_camera_settings.do_threshold));
			parameterElement->QueryDoubleAttribute("threshold_near", &(config->default_camera_settings.threshold_near));
			parameterElement->QueryDoubleAttribute("threshold_far", &(config->default_camera_settings.threshold_far));
			parameterElement->QueryIntAttribute("depth_x_erosion", &(config->default_camera_settings.depth_x_erosion));
			parameterElement->QueryIntAttribute("depth_y_erosion", &(config->default_camera_settings.depth_y_erosion));
		}
    }
    
	bool allnewcameras = config->cameraData.size() == 0; // if empty we have to set up a new administration
	int registeredcameras = 0;

	// now get the per camera info
	TiXmlElement* cameraElement = configElement->FirstChildElement("camera");
	while (cameraElement)
	{
		const char * serial = cameraElement->Attribute("serial");
		K4ACameraData* cd;

		int i = 0;
		while (i < config->cameraData.size()) {
			if (config->cameraData[i].serial == serial) {
				cd = &config->cameraData[i];
				break;
			}
			i++;
		}
		if (i == config->cameraData.size()) {
			// this camera was not in the admin yet
			if (!allnewcameras)
				loadOkay = false;

			cd = new K4ACameraData();
			pcl::shared_ptr<Eigen::Affine3d> trafo(new Eigen::Affine3d());
			pcl::shared_ptr<Eigen::Affine3d> intrinsicTrafo(new Eigen::Affine3d());
			intrinsicTrafo->setIdentity();
			cd->serial = cameraElement->Attribute("serial");
			cd->trafo = trafo;
			cd->intrinsicTrafo = intrinsicTrafo;
			cd->cameraposition = { 0, 0, 0 };
			config->cameraData.push_back(*cd);
			cd = &config->cameraData.back();
		}

        std::string type = cameraElement->Attribute("type");
        if (type != "") {
            cd->type = type;
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
	if (config->cameraData.size() != registeredcameras)
		loadOkay = false;

    if (!loadOkay) {
        cwipc_k4a_log_warning("Available hardware camera configuration does not match configuration file");
    }
	return loadOkay;
}


