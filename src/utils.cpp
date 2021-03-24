//
//  utils.cpp
//
//  Created by Fons Kuijk on 12-12-18.
//
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_KINECT_EXPORT __declspec(dllexport)
#endif
#include "cwipc_kinect/defs.h"
#include "cwipc_kinect/utils.h"

#include "tinyxml.h"

typedef struct HsvColor
{
	unsigned char h;
	unsigned char s;
	unsigned char v;
} HsvColor;

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

		/*std::cout << (*cd->trafo)(0, 0) << "\t" << (*cd->trafo)(0, 1) << "\t" << (*cd->trafo)(0, 2) << "\t" << (*cd->trafo)(0, 3) << std::endl;
		std::cout << (*cd->trafo)(1, 0) << "\t" << (*cd->trafo)(1, 1) << "\t" << (*cd->trafo)(1, 2) << "\t" << (*cd->trafo)(1, 3) << std::endl;
		std::cout << (*cd->trafo)(2, 0) << "\t" << (*cd->trafo)(2, 1) << "\t" << (*cd->trafo)(2, 2) << "\t" << (*cd->trafo)(2, 3) << std::endl;
		std::cout << (*cd->trafo)(3, 0) << "\t" << (*cd->trafo)(3, 1) << "\t" << (*cd->trafo)(3, 2) << "\t" << (*cd->trafo)(3, 3) << std::endl;*/

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

cwipc_pcl_point* hsvToRgb(HsvColor hsv, cwipc_pcl_point* pnt)
{
	unsigned char region, p, q, t;
	unsigned int h, s, v, remainder;

	if (hsv.s == 0)
	{
		pnt->r = hsv.v;
		pnt->g = hsv.v;
		pnt->b = hsv.v;
		return pnt;
	}

	// converting to 16 bit to prevent overflow
	h = hsv.h;
	s = hsv.s;
	v = hsv.v;

	region = h / 43;
	remainder = (h - (region * 43)) * 6;

	p = (v * (255 - s)) >> 8;
	q = (v * (255 - ((s * remainder) >> 8))) >> 8;
	t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

	switch (region)
	{
	case 0:
		pnt->r = v;
		pnt->g = t;
		pnt->b = p;
		break;
	case 1:
		pnt->r = q;
		pnt->g = v;
		pnt->b = p;
		break;
	case 2:
		pnt->r = p;
		pnt->g = v;
		pnt->b = t;
		break;
	case 3:
		pnt->r = p;
		pnt->g = q;
		pnt->b = v;
		break;
	case 4:
		pnt->r = t;
		pnt->g = p;
		pnt->b = v;
		break;
	default:
		pnt->r = v;
		pnt->g = p;
		pnt->b = q;
		break;
	}

	return pnt;
}

HsvColor rgbToHsv(cwipc_pcl_point* pnt)
{
	HsvColor hsv;
	unsigned char rgbMin, rgbMax;

	rgbMin = pnt->r < pnt->g ? (pnt->r < pnt->b ? pnt->r : pnt->b) : (pnt->g < pnt->b ? pnt->g : pnt->b);
	rgbMax = pnt->r > pnt->g ? (pnt->r > pnt->b ? pnt->r : pnt->b) : (pnt->g > pnt->b ? pnt->g : pnt->b);

	hsv.v = rgbMax;
	if (hsv.v == 0)
	{
		hsv.h = 0;
		hsv.s = 0;
		return hsv;
	}

	hsv.s = 255 * ((long)(rgbMax - rgbMin)) / hsv.v;
	if (hsv.s == 0)
	{
		hsv.h = 0;
		return hsv;
	}

	if (rgbMax == pnt->r)
		hsv.h = 0 + 43 * (pnt->g - pnt->b) / (rgbMax - rgbMin);
	else if (rgbMax == pnt->g)
		hsv.h = 85 + 43 * (pnt->b - pnt->r) / (rgbMax - rgbMin);
	else
		hsv.h = 171 + 43 * (pnt->r - pnt->g) / (rgbMax - rgbMin);

	return hsv;
}

bool cwipc_k4a_noChromaRemoval(cwipc_pcl_point* p)
{
	HsvColor hsv = rgbToHsv(p);

	if (hsv.h >= 60 && hsv.h <= 130) {
		if (hsv.s >= 0.15 && hsv.v >= 0.15) {
			// reducegreen
			if ((p->r * p->b) != 0 && (p->g * p->g) / (p->r * p->b) > 1.5) {
				p->r *= 1.4;
				p->b *= 1.4;
			}
			else {
				p->r *= 1.2;
				p->b *= 1.2;
			}
		}
		return !(hsv.s >= 0.4 && hsv.v >= 0.3);
	}
	return true;
}
