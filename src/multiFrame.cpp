//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

#include <chrono>
#include <cstdint>

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include "cwipc_realsense2/multiFrame.hpp"
#include "cwipc_realsense2/api.h"
#include "cwipc_realsense2/utils.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "cwipc_realsense2/stb_image_write.h"

MFCapture::MFCapture(const char *_configFilename)
{
	if (_configFilename) {
		configFilename = _configFilename;
	}
	else {
		configFilename = "cameraconfig.xml";
	}
	// Create librealsense context for managing all connected RealSense devices
	rs2::context ctx;
	auto devs = ctx.query_devices();
	const std::string platform_camera_name = "Platform Camera";
	//Sync messages to assign master and slave
#define WITH_NEW_SYNC
#ifndef WITH_OLD_SYNC
	uint8_t sync_default[24] = { 0x14,0x0,0xAB,0xCD,0x64,0x0,0x0,0x0,0x00,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0 };
	uint8_t sync_master[24] = { 0x14,0x0,0xAB,0xCD,0x64,0x0,0x0,0x0,0x01,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0 };
	uint8_t sync_slave[24] = { 0x14,0x0,0xAB,0xCD,0x64,0x0,0x0,0x0,0x02,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0 };
#endif // WITH_OLD_SYNC
	bool master_set = false;
	bool multiple_cameras = true; // xxxjack devs.size() > 1;
	rs2_error* e = nullptr;
	// prepare storage for camera data for each connected camera
	for (auto dev : devs) {
		if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
			boost::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
			default_trafo->setIdentity();
			MFConfigCamera cd;
			cd.serial = std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
			cd.trafo = default_trafo;
			cd.cloud = new_cwipc_pcl_pointcloud();
			cd.background = { 0, 0, 0 };
			cd.cameraposition = { 0, 0, 0 };
			configuration.cameraConfig.push_back(cd);

			MFCamera rsd;
			rsd.serial = cd.serial;
			rsd.usb = std::string(dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR));
			cameras.push_back(rsd);
#ifdef WITH_OLD_SYNC
			//Send sync messages
			if (!master_set)
			{
				rs2_send_and_receive_raw_data((rs2_device*)&dev, (void*)&sync_master, sizeof(sync_master), &e);
				master_set = true;
			}
			else {
				rs2_send_and_receive_raw_data((rs2_device*)&dev, (void*)&sync_slave, sizeof(sync_default), &e);
			}
#endif
#ifdef WITH_NEW_SYNC
			if (multiple_cameras) {
				auto allSensors = dev.query_sensors();
				bool foundSensorSupportingSync = false;
				for (auto sensor : allSensors) {
					if (sensor.supports(RS2_OPTION_INTER_CAM_SYNC_MODE)) {
						foundSensorSupportingSync = true;
						if (!master_set) {
							sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
							master_set = true;
						} else {
							sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
						}
					}
				}
				if (!foundSensorSupportingSync) {
					std::cout << "Warning: camera " << cd.serial << " does not support inter-camera-sync";
				}
			}
#endif
		}
	}

	if (configuration.cameraConfig.size() == 0) {
		// no camera connected, so we'll use a generated pointcloud instead
		GeneratedPC = generate_pcl();
		std::cout << "No cameras found, default production is a spinning generated pointcloud of " << GeneratedPC->size() << " data points\n";
	}
	else {
		// Read the configuration
		if (!mf_file2config(configFilename.c_str(), &configuration)) {

			// the configuration file did not fully match the current situation so we have to update the admin
			std::vector<std::string> serials;
			std::vector<MFConfigCamera> realcams;

			// collect serial numbers of all connected cameras
			for (auto dev : devs) {
				if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
					serials.push_back(std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)));
				}
			}
			
			// collect all camera's in the config that are connected
			for (MFConfigCamera cd : configuration.cameraConfig) {
				if ((find(serials.begin(), serials.end(), cd.serial) != serials.end()))
					realcams.push_back(cd);
				else
					std::cout << "WARNING: camera " << cd.serial << " is not connected\n";
			}
			// Reduce the active configuration to cameras that are connected
			configuration.cameraConfig = realcams;
		}
	}
	MergedPC = new_cwipc_pcl_pointcloud();

	// for an explanation of filtering see librealsense/doc/post-processing-filters.md and code in librealsense/src/proc 
	dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, configuration.decimation_value);

	spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, configuration.spatial_iterations);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, configuration.spatial_alpha);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, configuration.spatial_delta);
	spat_filter.set_option(RS2_OPTION_HOLES_FILL, configuration.spatial_filling);

	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, configuration.temporal_alpha);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, configuration.temporal_delta);
	temp_filter.set_option(RS2_OPTION_HOLES_FILL, configuration.temporal_percistency);

	// optionally set request for cwi_special_feature
	char* feature_request;
	feature_request = getenv("CWI_CAPTURE_FEATURE");
	if (feature_request != NULL)
		configuration.cwi_special_feature = feature_request;

	// find camerapositions
	for (int i = 0; i < configuration.cameraConfig.size(); i++) {
		cwipc_pcl_pointcloud pcptr(new_cwipc_pcl_pointcloud());
		cwipc_pcl_point pt;
		pt.x = 0;
		pt.y = 0;
		pt.z = 0;
		pcptr->push_back(pt);
		transformPointCloud(*pcptr, *pcptr, *configuration.cameraConfig[i].trafo);
		cwipc_pcl_point pnt = pcptr->points[0];
		configuration.cameraConfig[i].cameraposition.x = pnt.x;
		configuration.cameraConfig[i].cameraposition.y = pnt.y;
		configuration.cameraConfig[i].cameraposition.z = pnt.z;
	}


	// start the cameras
	for (int i = 0; i < cameras.size(); i++)
		camera_start(&cameras[i]);
	starttime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

MFCapture::~MFCapture() {
	for (MFCamera rsd : cameras)
		rsd.pipe.stop();
	std::cout << "stopped all camera's\n";
}

#if 0
// API function that triggers the capture and returns the merged pointcloud and timestamp
void multiFrame::get_pointcloud(uint64_t *timestamp, void **pointcloud)
{
	*timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	if (Configuration.cameraConfig.size() > 0) {
		for (cameradata cd : Configuration.cameraConfig)
			camera_action(cd);
		merge_views();

		if (MergedPC.get()->size() > 0) {
#ifdef CWIPC_DEBUG
			std::cout << "capturer produced a merged cloud of " << MergedPC->size() << " points in ringbuffer " << ring_index << "\n";
#endif
			*pointcloud = reinterpret_cast<void *> (&(MergedPC));
		}
		else {
#ifdef CWIPC_DEBUG
			std::cout << "\nWARNING: capturer did get an empty pointcloud\n\n";
#endif
			// HACK to make sure the encoder does not get an empty pointcloud 
			cwipc_pcl_point point;
			point.x = 1.0;
			point.y = 1.0;
			point.z = 1.0;
			point.rgb = 0.0;
			MergedPC->points.push_back(point);
			*pointcloud = reinterpret_cast<void *> (&(MergedPC));
		}
	}
	else {	// return a spinning generated mathematical pointcloud
		static float angle;
		angle += 0.031415;
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
		transformPointCloud(*GeneratedPC, *MergedPC, transform);
		*pointcloud = reinterpret_cast<void *> (&(MergedPC));
	}
	ring_index = ring_index < Configuration.ringbuffer_size - 1 ? ++ring_index : 0;
}
#else

// API function that triggers the capture and returns the merged pointcloud and timestamp
cwipc_pcl_pointcloud MFCapture::get_pointcloud(uint64_t *timestamp)
{
	*timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	if (cameras.size() > 0) {
		for (int i = 0; i < cameras.size(); i++)
			camera_action(i, timestamp);

		if (merge_views()->size() > 0) {
#ifdef DEBUG
			std::cout << "capturer produced a merged cloud of " << MergedPC->size() << " points in ringbuffer " << ring_index << "\n";
#endif
		}
		else {
#ifdef DEBUG
			std::cout << "\nWARNING: capturer did get an empty pointcloud\n\n";
#endif
			// HACK to make sure the encoder does not get an empty pointcloud 
			cwipc_pcl_point point;
			point.x = 1.0;
			point.y = 1.0;
			point.z = 1.0;
			point.rgb = 0.0;
			MergedPC->points.push_back(point);
		}
	}
	else {	// return a spinning generated mathematical pointcloud
		static float angle;
		angle += 0.031415;
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
		transformPointCloud(*GeneratedPC, *MergedPC, transform);
	}
	return MergedPC;
}

#endif

// return the merged cloud 
cwipc_pcl_pointcloud MFCapture::getPointCloud()
{
	return MergedPC;
}

// Configure and initialize caputuring of one camera
void MFCapture::camera_start(MFCamera* rsd)
{
	rs2::config cfg;
	if (rsd->usb[0] == '3') {
		std::cout << "starting camera ser no: " << rsd->serial << " in usb3 mode\n";
		cfg.enable_device(rsd->serial);
		cfg.enable_stream(RS2_STREAM_COLOR, configuration.usb3_width, configuration.usb3_height, RS2_FORMAT_RGB8, configuration.usb3_fps);
		cfg.enable_stream(RS2_STREAM_DEPTH, configuration.usb3_width, configuration.usb3_height, RS2_FORMAT_Z16, configuration.usb3_fps);
	}
	else {
		std::cout << "starting camera ser no: " << rsd->serial << " in usb2 mode\n";
		cfg.enable_device(rsd->serial);
		cfg.enable_stream(RS2_STREAM_COLOR, configuration.usb2_width, configuration.usb2_height, RS2_FORMAT_RGB8, configuration.usb2_fps);
		cfg.enable_stream(RS2_STREAM_DEPTH, configuration.usb2_width, configuration.usb2_height, RS2_FORMAT_Z16, configuration.usb2_fps);
	}
	rsd->pipe.start(cfg);		// Start streaming with the configuration just set
}

// get new frames from the camera and update the pointcloud of the camera's data 
void MFCapture::camera_action(int camera_index, uint64_t *timestamp)
{
	MFCamera* rsd = &cameras[camera_index];
	MFConfigCamera* cd = &configuration.cameraConfig[camera_index];
	rs2::pointcloud pc;
	rs2::points points;

	uint8_t camera_label = (uint8_t)1 << camera_index;

#ifdef POLLING
	// Poll to find if there is a next set of frames from the camera
	rs2::frameset frames;
	if (!rsd->pipe.poll_for_frames(&frames))
		return;
#else
	// Wait to find if there is a next set of frames from the camera
	rs2::frameset frames = rsd->pipe.wait_for_frames();
#endif

	rs2::depth_frame depth = frames.get_depth_frame();
	rs2::video_frame color = frames.get_color_frame();

	// On special request write video to png
	if (configuration.cwi_special_feature == "dumpvideoframes") {
		std::stringstream png_file;
		png_file <<  "videoframe_" << *timestamp - starttime << "_" << camera_index << ".png";
		stbi_write_png(png_file.str().c_str(), color.get_width(), color.get_height(),
			color.get_bytes_per_pixel(), color.get_data(), color.get_stride_in_bytes());
	}

	cd->cloud->clear();

	// Tell points frame to map to this color frame
	pc.map_to(color); // NB: This does not align the frames. That should be handled by setting resolution of cameras
	
	if (configuration.depth_filtering) { // Apply filters
		//depth = dec_filter.process(depth);          // decimation filter
		depth = depth_to_disparity.process(depth);  // transform into disparity domain
		depth = spat_filter.process(depth);         // spatial filter
		depth = temp_filter.process(depth);         // temporal filter
		depth = disparity_to_depth.process(depth);  // revert back to depth domain
	}
	points = pc.calculate(depth);

	// Generate new vertices and color vector
	auto vertices = points.get_vertices();

	unsigned char *colors = (unsigned char*)color.get_data();

	if (configuration.background_removal) {
		MFConfigCamera* cd = get_cameradata(rsd->serial);

		// Set the background removal window
        if (cd->background.z > 0.0) {
            rsd->maxz = cd->background.z;
			rsd->minz = 0.0;
			if (cd->background.x != 0.0) {
				rsd->minx = cd->background.x;
			}
			else {
				for (int i = 0; i < points.size(); i++) {
					double minz = 100;
					if (vertices[i].z != 0 && minz > vertices[i].z) {
						rsd->minz = vertices[i].z;
						rsd->minx = vertices[i].x;
					}
				}
			}
        }
		else {
			rsd->minz = 100.0;
			for (int i = 0; i < points.size(); i++) {
				if (vertices[i].z != 0 && rsd->minz > vertices[i].z) {
					rsd->minz = vertices[i].z;
					rsd->minx = vertices[i].x;
				}
			}
			rsd->maxz = 0.8f + rsd->minz;
		}

		// Make PointCloud
		for (int i = 0; i < points.size(); i++) {
			double x = rsd->minx - vertices[i].x; x *= x;
			double z = vertices[i].z;
			if (rsd->minz < z && z < rsd->maxz - x) { // Simple background removal, horizontally parabolic, vertically straight.
				cwipc_pcl_point pt;
				pt.x = vertices[i].x;
				pt.y = -vertices[i].y;
				pt.z = -z;
				int pi = i * 3;
				pt.r = colors[pi];
				pt.g = colors[pi + 1];
				pt.b = colors[pi + 2];
				pt.a = camera_label;
				if (!configuration.greenscreen_removal || mf_noChromaRemoval(&pt)) // chromakey removal
					cd->cloud->push_back(pt);
			}
		}
	}
	else {
		// Make PointCloud
		for (int i = 0; i < points.size(); i++) {
			cwipc_pcl_point pt;

			pt.x = vertices[i].x;
			pt.y = -vertices[i].y;
			pt.z = -vertices[i].z;
			int pi = i * 3;
			pt.r = colors[pi];
			pt.g = colors[pi + 1];
			pt.b = colors[pi + 2];
			pt.a = camera_label;
			if (!configuration.greenscreen_removal || mf_noChromaRemoval(&pt)) // chromakey removal
				cd->cloud->push_back(pt);
		}
	}
}

cwipc_pcl_pointcloud MFCapture::merge_views()
{
	cwipc_pcl_pointcloud aligned_cld(new_cwipc_pcl_pointcloud());
	MergedPC->clear();
	for (MFConfigCamera cd : configuration.cameraConfig) {
		cwipc_pcl_pointcloud cam_cld = cd.cloud;

		if (cam_cld->size() > 0) {
			transformPointCloud(*cam_cld, *aligned_cld, *cd.trafo);
			*MergedPC += *aligned_cld;
		}
	}

	if (configuration.cloud_resolution > 0) {
#ifdef CWIPC_DEBUG
		std::cout << "Points before reduction: " << cloud_ptr.get()->size() << endl;
#endif
		pcl::VoxelGrid<cwipc_pcl_point> grd;
		grd.setInputCloud(MergedPC);
		grd.setLeafSize(configuration.cloud_resolution, configuration.cloud_resolution, configuration.cloud_resolution);
		grd.setSaveLeafLayout(true);
		grd.filter(*MergedPC);

#ifdef DEBUG
		std::cout << "Points after reduction: " << cloud_ptr.get()->size() << endl;
#endif
	}
	return MergedPC;
}

MFConfigCamera* MFCapture::get_cameradata(std::string serial) {
	for (int i = 0; i < configuration.cameraConfig.size(); i++)
		if (configuration.cameraConfig[i].serial == serial)
			return &configuration.cameraConfig[i];
	return NULL;
}

MFCamera* MFCapture::get_realsensedata(std::string serial) {
	for (int i = 0; i < cameras.size(); i++)
		if (cameras[i].serial == serial)
			return &cameras[i];
	return NULL;
}

MFCamera MFCapture::newrealsensedata() {
	MFCamera rsd;
	return rsd;
}

// generate a mathematical pointcloud
cwipc_pcl_pointcloud MFCapture::generate_pcl()
{
	cwipc_pcl_pointcloud point_cloud_ptr(new_cwipc_pcl_pointcloud());
	uint8_t r(255), g(15), b(15);

	for (float z(-1.0f); z <= 1.0f; z += 0.005f) {
        float angle(0.0);
		while (angle <= 360.0) {
			cwipc_pcl_point point;
			point.x = 0.5f*cosf(pcl::deg2rad(angle))*(1.0f - z * z);
			point.y = sinf(pcl::deg2rad(angle))*(1.0f - z * z);
			point.z = z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			point_cloud_ptr->points.push_back(point);
            float r = sqrt(point.x*point.x + point.y*point.y);
            if (r > 0.0)
                angle += 0.27/r;
            else break;
		}
		if (z < 0.0) { r -= 1; g += 1; }
		else { g -= 1; b += 1; }
	}
	point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;
	return point_cloud_ptr;
}

