//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

// Define to try and use hardware sync to synchronize multiple cameras
#define WITH_INTER_CAM_SYNC

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD



// This is the dll source, so define external symbols as dllexport on windows.

#ifdef _WIN32
#define _CWIPC_KINECT_EXPORT __declspec(dllexport)
#define CWIPC_DLL_ENTRY __declspec(dllexport)
#endif

#include <chrono>

#include "cwipc_kinect/defs.h"
#include "cwipc_kinect/utils.h"
#include "cwipc_kinect/K4ACapture.hpp"
#include "cwipc_kinect/K4ACamera.hpp"


// Static variable used to print a warning message when we re-create an K4ACapture
// if there is another one open.
static int numberOfCapturersActive = 0;

K4ACapture::K4ACapture(int dummy)
:	numberOfPCsProduced(0),
    no_cameras(true),
	mergedPC_is_fresh(false),
	mergedPC_want_new(false)
{
	numberOfCapturersActive++;
	mergedPC = new_cwipc_pcl_pointcloud();
	starttime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

K4ACapture::K4ACapture(const char *configFilename)
:	numberOfPCsProduced(0),
    no_cameras(false),
	mergedPC_is_fresh(false),
	mergedPC_want_new(false)
{
	// First check that no other K4ACapture is active within this process (trying to catch programmer errors)
	numberOfCapturersActive++;
	if (numberOfCapturersActive > 1) {
		cwipc_k4a_log_warning("Attempting to create capturer while one is already active.");
	}
#ifdef notrs2

	// Determine how many realsense cameras (not platform cameras like webcams) are connected
	const std::string platform_camera_name = "Platform Camera";
	rs2::device_list devs = ctx.query_devices();
	int camera_count = 0;
	for(auto dev: devs) {
		if (dev.get_info(RS2_CAMERA_INFO_NAME) != platform_camera_name) {
			camera_count++;
		}
	}
#else
	int camera_count = k4a_device_get_installed_count();
#endif
	if (camera_count == 0) {
		// no camera connected, so we'll return nothing
		no_cameras = true;
		return;
	}
	std::vector<std::string> serials;
	k4a_device_t* camera_handles = new k4a_device_t[camera_count];
	bool any_failure = false;
	for (uint32_t i = 0; i < camera_count; i++) {
		if (k4a_device_open(i, &camera_handles[i]) != K4A_RESULT_SUCCEEDED) {
			cwipc_k4a_log_warning("k4a_device_open failed");
			any_failure = true;
			continue;
		}
		K4ACameraData cd;
		char serial_buf[64];
		size_t serial_buf_size = sizeof(serial_buf) / sizeof(serial_buf[0]);
		if (k4a_device_get_serialnum(camera_handles[i], serial_buf, &serial_buf_size) != K4A_RESULT_SUCCEEDED) {
			cwipc_k4a_log_warning("get_serialnum failed");
			any_failure = true;
			continue;
		}
		cd.serial = std::string(serial_buf);
		serials.push_back(serial_buf);
		boost::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
		default_trafo->setIdentity();
		cd.trafo = default_trafo;
		cd.intrinsicTrafo = default_trafo;
		cd.cloud = new_cwipc_pcl_pointcloud();
		cd.cameraposition = { 0, 0, 0 };
		configuration.cameraData.push_back(cd);
	}
	if (any_failure) return;

	//
	// Read the configuration. We do this only now because for historical reasons the configuration
	// reader is also the code that checks whether the configuration file contents match the actual
	// current hardware setup. To be fixed at some point.
	//
	if (configFilename == NULL) {
		configFilename = "cameraconfig.xml";
	}
	if (!cwipc_k4a_file2config(configFilename, &configuration)) {

		// the configuration file did not fully match the current situation so we have to update the admin
		std::vector<std::string> serials;
		std::vector<K4ACameraData> realcams;


		// collect all camera's in the config that are connected
		for (K4ACameraData cd : configuration.cameraData) {
#if 1 // xxxjack find() doesn't work??!?
			realcams.push_back(cd);
#else
			if ((find(serials.begin(), serials.end(), cd.serial) != serials.end()))
				realcams.push_back(cd);
			else
				cwipc_k4a_log_warning("Camera " + cd.serial + " is not connected");
#endif
		}
		// Reduce the active configuration to cameras that are connected
		configuration.cameraData = realcams;
	}

	// Set various camera hardware parameters (color)  //TODO: //should be set from the configfile
	for (int i = 0; i < camera_count; i++) {
		//options for color sensor
		k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, 40000); // Exposure_time (in microseconds)
		k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_MANUAL, 3160); // White_balance (2500-12500)
		k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, K4A_COLOR_CONTROL_MODE_MANUAL, 0); // Backlight_compensation 0=disabled | 1=enabled. Default=0
		k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_BRIGHTNESS, K4A_COLOR_CONTROL_MODE_MANUAL, 128); // Brightness. (0 to 255). Default=128.
		k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_CONTRAST, K4A_COLOR_CONTROL_MODE_MANUAL, 5); // Contrast (0-10). Default=5
		k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_SATURATION, K4A_COLOR_CONTROL_MODE_MANUAL, 32); // saturation (0-63). Default=32
		k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_SHARPNESS, K4A_COLOR_CONTROL_MODE_MANUAL, 2); // Sharpness (0-4). Default=2
		k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_GAIN, K4A_COLOR_CONTROL_MODE_MANUAL, 100); // Gain (0-255). Default=0
		k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, K4A_COLOR_CONTROL_MODE_MANUAL, 0); // Backlight_compensation 0=disabled | 1=enabled. Default=0
		k4a_device_set_color_control(camera_handles[i], K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, K4A_COLOR_CONTROL_MODE_MANUAL, 2); // Powerline_Frequency (1=50Hz, 2=60Hz). Default=2
	}

#ifdef _rs2_WITH_INTER_CAM_SYNC
	bool master_set = false;
	for (auto dev : devs) {
		if (camera_count > 1) {
			auto allSensors = dev.query_sensors();
			bool foundSensorSupportingSync = false;
			for (auto sensor : allSensors) {
				if (sensor.supports(RS2_OPTION_INTER_CAM_SYNC_MODE)) {
					foundSensorSupportingSync = true;
					if (!master_set) {
						sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
						master_set = true;
					}
					else {
						sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
					}
				}
			}
			if (!foundSensorSupportingSync) {
				cwipc_k4a_log_warning(std::string("Camera ") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) + " does not support inter-camera-sync");
			}
		}
	}
#endif // WITH_INTER_CAM_SYNC

	// Now we have all the configuration information. Open the cameras.
	_create_cameras(camera_handles, serials, camera_count);
	// We can now free camera_handles
	delete camera_handles;

	// Create an empty pointcloud just in case anyone calls get_mostRecentPointcloud() before one is generated.
	mergedPC = new_cwipc_pcl_pointcloud();

	// optionally set request for cwi_special_feature
	char* feature_request;
	feature_request = getenv("CWI_CAPTURE_FEATURE");
	if (feature_request != NULL)
		configuration.cwi_special_feature = feature_request;

	// find camerapositions
	for (int i = 0; i < configuration.cameraData.size(); i++) {
		cwipc_pcl_pointcloud pcptr(new_cwipc_pcl_pointcloud());
		cwipc_pcl_point pt;
		pt.x = 0;
		pt.y = 0;
		pt.z = 0;
		pcptr->push_back(pt);
		transformPointCloud(*pcptr, *pcptr, *configuration.cameraData[i].trafo);
		cwipc_pcl_point pnt = pcptr->points[0];
		configuration.cameraData[i].cameraposition.x = pnt.x;
		configuration.cameraData[i].cameraposition.y = pnt.y;
		configuration.cameraData[i].cameraposition.z = pnt.z;
	}

	//
	// start the cameras
	//
	for (auto cam : cameras) {
		if (!cam->start()) return;
	}
		
	starttime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	//
	// start the per-camera capture threads
	//
	for (auto cam: cameras)
		cam->start_capturer();
	//
	// start our run thread (which will drive the capturers and merge the pointclouds)
	//
	stopped = false;
	control_thread = new std::thread(&K4ACapture::_control_thread_main, this);
	_cwipc_setThreadName(control_thread, L"cwipc_kinect::K4ACapture::control_thread");
}

void K4ACapture::_create_cameras(k4a_device_t *camera_handles, std::vector<std::string> serials, uint32_t camera_count) {
	int serial_index = 0;
	for(uint32_t i=0; i<camera_count; i++) {
		if (camera_handles[i] == NULL) continue;
#ifdef CWIPC_DEBUG
		std::cout << "K4ACapture: opening camera " << serials[i] << std::endl;
#endif
		// Found a realsense camera. Create a default data entry for it.
		std::string serial(serials[serial_index++]);

		K4ACameraData& cd = get_camera_data(serial);
		if (cd.type != "kinect") {
			cwipc_k4a_log_warning("Camera " + serial + " is type " + cd.type + " in stead of kinect");
		}
		int camera_index = cameras.size();
		auto cam = new K4ACamera(camera_handles[i], configuration, camera_index, cd);
		cameras.push_back(cam);
	}
}

K4ACapture::~K4ACapture() {
    if (no_cameras) {
        numberOfCapturersActive--;
        return;
    }
	uint64_t stopTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	stopped = true;
	// Stop all cameras
	for (auto cam : cameras)
		cam->stop();
    mergedPC_is_fresh = true;
    mergedPC_want_new = false;
    mergedPC_is_fresh_cv.notify_all();
    mergedPC_want_new = true;
    mergedPC_want_new_cv.notify_all();
	control_thread->join();
	delete control_thread;
	std::cerr << "cwipc_kinect: stopped all cameras\n";
	// Delete all cameras (which will stop their threads as well)
	for (auto cam : cameras)
		delete cam;
	cameras.clear();
	std::cerr << "cwipc_kinect: deleted all cameras\n";
	// Print some minimal statistics of this run
	float deltaT = (stopTime - starttime) / 1000.0;
	std::cerr << "cwipc_kinect: ran for " << deltaT << " seconds, produced " << numberOfPCsProduced << " pointclouds at " << numberOfPCsProduced / deltaT << " fps." << std::endl;
	numberOfCapturersActive--;
}

// API function that triggers the capture and returns the merged pointcloud and timestamp
cwipc_pcl_pointcloud K4ACapture::get_pointcloud(uint64_t *timestamp)
{
    if (no_cameras) return nullptr;
	*timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	_request_new_pointcloud();
	// Wait for a fresh mergedPC to become available.
	// Note we keep the return value while holding the lock, so we can start the next grab/process/merge cycle before returning.
	cwipc_pcl_pointcloud rv;
	{
		std::unique_lock<std::mutex> mylock(mergedPC_mutex);
		mergedPC_is_fresh_cv.wait(mylock, [this]{return mergedPC_is_fresh; });
		mergedPC_is_fresh = false;
		numberOfPCsProduced++;
		rv = mergedPC;
	}
	_request_new_pointcloud();
	return rv;
}

float K4ACapture::get_pointSize()
{
    if (no_cameras) return 0;
	float rv = 99999;
	for (auto cam : cameras) {
		if (cam->pointSize < rv) rv = cam->pointSize;
	}
	if (rv > 9999) rv = 0;
	return rv;
}

bool K4ACapture::pointcloud_available(bool wait)
{
    if (no_cameras) return false;
	_request_new_pointcloud();
	std::this_thread::yield();
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	auto duration = std::chrono::seconds(wait?1:0);
	mergedPC_is_fresh_cv.wait_for(mylock, duration, [this]{return mergedPC_is_fresh; });
	return mergedPC_is_fresh;
}

void K4ACapture::_control_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "cwipc_kinect: K4ACapture: processing thread started" << std::endl;
#endif
	while(!stopped) {
		{
			std::unique_lock<std::mutex> mylock(mergedPC_mutex);
			mergedPC_want_new_cv.wait(mylock, [this]{ return mergedPC_want_new;});
		}
		if (stopped) {
			break;
		}
        assert (cameras.size() > 0);
        // Step one: grab frames from all cameras. This should happen as close together in time as possible,
        // because that gives use he biggest chance we have the same frame (or at most off-by-one) for each
        // camera.
		bool all_captures_ok = true;
        for(auto cam : cameras) {
			if (!cam->capture_frameset()) {
				all_captures_ok = false;
				continue;
			}
        }
		if (!all_captures_ok) {
			//std::cerr << "cwipc_kinect: K4ACapture: xxxjack not all captures succeeded. Retrying." << std::endl;
			std::this_thread::yield();
			continue;
		}
        // And get the best timestamp
        uint64_t timestamp = 0;
        for(auto cam: cameras) {
            uint64_t camts = cam->get_capture_timestamp();
            if (camts > timestamp) timestamp = camts;
        }
#ifdef WITH_DUMP_VIDEO_FRAMES
        // Step 2, if needed: dump image frames.
        if (configuration.cwi_special_feature == "dumpvideoframes") {
            for(auto cam : cameras) {
                std::stringstream png_file;
                png_file <<  "videoframe_" << timestamp - starttime << "_" << cam->camera_index << ".png";
                cam->dump_color_frame(png_file.str());
            }
        }
#endif // WITH_DUMP_VIDEO_FRAMES
        // Step 3: start processing frames to pointclouds, for each camera
        for(auto cam : cameras) {
            cam->create_pc_from_frames();
        }
        // Lock mergedPC already while we are waiting for the per-camera
        // processing threads. This so the main thread doesn't go off and do
        // useless things if it is calling available(true).
        std::unique_lock<std::mutex> mylock(mergedPC_mutex);
        // Step 4: wait for frame processing to complete.
        for(auto cam : cameras) {
            cam->wait_for_pc();
        }
        // Step 5: merge views
        mergedPC = new_cwipc_pcl_pointcloud();
        merge_views();
        if (mergedPC->size() > 0) {
#ifdef CWIPC_DEBUG
            std::cerr << "cwipc_kinect: capturer produced a merged cloud of " << mergedPC->size() << " points" << std::endl;
#endif
        } else {
#ifdef CWIPC_DEBUG
            std::cerr << "cwipc_kinect: Warning: capturer got an empty pointcloud\n";
#endif
#if 0
            // HACK to make sure the encoder does not get an empty pointcloud
            cwipc_pcl_point point;
            point.x = 1.0;
            point.y = 1.0;
            point.z = 1.0;
            point.rgb = 0.0;
            mergedPC->points.push_back(point);
#endif
        }
        // Signal that a new mergedPC is available. (Note that we acquired the mutex earlier)
        mergedPC_is_fresh = true;
        mergedPC_want_new = false;
        mergedPC_is_fresh_cv.notify_all();
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "wipc_kinect: K4ACapture: processing thread stopped" << std::endl;
#endif
}

// return the merged cloud 
cwipc_pcl_pointcloud K4ACapture::get_mostRecentPointCloud()
{
    if (no_cameras) return nullptr;
	// This call doesn't need a fresh pointcloud (Jack thinks), but it does need one that is
	// consistent. So we lock, but don't wait on the condition.
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	return mergedPC;
}

void K4ACapture::_request_new_pointcloud()
{
	std::unique_lock<std::mutex> mylock(mergedPC_mutex);
	if (!mergedPC_want_new && !mergedPC_is_fresh) {
		mergedPC_want_new = true;
		mergedPC_want_new_cv.notify_all();
	}
}

void K4ACapture::merge_views()
{
	cwipc_pcl_pointcloud aligned_cld(new_cwipc_pcl_pointcloud());
	mergedPC->clear();
	// Pre-allocate space in the merged pointcloud
	size_t nPoints = 0;
	for (K4ACameraData cd : configuration.cameraData) {
		cwipc_pcl_pointcloud cam_cld = cd.cloud;
		nPoints += cam_cld->size();
	}
	mergedPC->reserve(nPoints);
	// Now merge all pointclouds
	for (K4ACameraData cd : configuration.cameraData) {
		cwipc_pcl_pointcloud cam_cld = cd.cloud;
		*mergedPC += *cam_cld;
	}

	if (configuration.cloud_resolution > 0) {
#ifdef CWIPC_DEBUG
		std::cerr << "cwipc_kinect: Points before reduction: " << mergedPC->size() << std::endl;
#endif
		pcl::VoxelGrid<cwipc_pcl_point> grd;
		grd.setInputCloud(mergedPC);
		grd.setLeafSize(configuration.cloud_resolution, configuration.cloud_resolution, configuration.cloud_resolution);
		grd.setSaveLeafLayout(true);
		grd.filter(*mergedPC);

#ifdef CWIPC_DEBUG
		std::cerr << "cwipc_kinect: Points after reduction: " << mergedPC->size() << std::endl;
#endif
	}
}

K4ACameraData& K4ACapture::get_camera_data(std::string serial) {
	for (int i = 0; i < configuration.cameraData.size(); i++)
		if (configuration.cameraData[i].serial == serial)
			return configuration.cameraData[i];
	cwipc_k4a_log_warning("Unknown camera " + serial);
	abort();
}

K4ACamera* K4ACapture::get_camera(std::string serial) {
	for (auto cam : cameras)
		if (cam->serial == serial)
			return cam;
	return NULL;
}
