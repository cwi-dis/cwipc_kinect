//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//
#include <cstdlib>

// Define to get (a little) debug prints
#define CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_KINECT_EXPORT __declspec(dllexport)
#endif

#include "cwipc_kinect/defs.h"
#include "cwipc_kinect/utils.h"
#include "cwipc_kinect/K4ACamera.hpp"

#ifdef WITH_DUMP_VIDEO_FRAMES
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "cwipc_kinect/stb_image_write.h"
#endif

#ifdef _WIN32
#include <Windows.h>
void setThreadName(std::thread* thr, const wchar_t* name) {
	HANDLE threadHandle = static_cast<HANDLE>(thr->native_handle());
	SetThreadDescription(threadHandle, name);
}
#else
void setThreadName(std::thread& thr, const wchar_t* name) {}
#endif

K4ACamera::K4ACamera(k4a_device_t _handle, K4ACaptureConfig& configuration, int _camera_index, K4ACameraData& _camData)
:	pointSize(0), minx(0), minz(0), maxz(0),
	device_handle(_handle),
	camera_index(_camera_index),
	serial(_camData.serial),
	stopped(true),
	capture_started(false),
	camData(_camData),
	camSettings(configuration.default_camera_settings),
	captured_frame_queue(1),
	processing_frame_queue(1),
	camera_width(configuration.width),
	camera_height(configuration.height),
	camera_fps(configuration.fps),
	do_greenscreen_removal(configuration.greenscreen_removal),
	do_height_filtering(configuration.height_min != configuration.height_max),
	height_min(configuration.height_min),
	height_max(configuration.height_max),
	grabber_thread(nullptr)
{
#ifdef CWIPC_DEBUG
		std::cout << "K4ACapture: creating camera " << serial << std::endl;
#endif
	_init_filters();
}

K4ACamera::~K4ACamera()
{
#ifdef CWIPC_DEBUG
	std::cout << "K4ACamera: destroying " << serial << std::endl;
#endif
	assert(stopped);
}

void K4ACamera::_init_filters()
{
	if (!do_depth_filtering) return;
}

bool K4ACamera::capture_frameset()
{
	bool rv = captured_frame_queue.try_dequeue(current_frameset);
#if 1 // def CWIPC_DEBUG_THREAD
	if (rv) {
		uint64_t tsRGB = k4a_image_get_device_timestamp_usec(k4a_capture_get_color_image(current_frameset));
		uint64_t tsD = k4a_image_get_device_timestamp_usec(k4a_capture_get_depth_image(current_frameset));
		std::cerr << "frame forward: cam=" << serial << ", rgbseq=" << tsRGB << ", dseq=" << tsD << std::endl;
	}
#endif

	return rv;
}

// Configure and initialize caputuring of one camera
void K4ACamera::start()
{
	assert(stopped);
	k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	device_config.color_resolution = K4A_COLOR_RESOLUTION_720P; // xxxjack
	device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	device_config.camera_fps = K4A_FRAMES_PER_SECOND_30; // xxxjack
	device_config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture

	k4a_calibration_t calibration;
	if (K4A_RESULT_SUCCEEDED !=
		k4a_device_get_calibration(device_handle, device_config.depth_mode, device_config.color_resolution, &calibration))
	{
		std::cerr << "cwipc_kinect: Failed to get calibration" << std::endl;
		return;
	}
	transformation_handle = k4a_transformation_create(&calibration);

	if (k4a_device_start_cameras(device_handle, &device_config) != K4A_RESULT_SUCCEEDED) {
		std::cerr << "cwipc_kinect: failed to start camera " << serial << std::endl;
		return;
	}
	std::cerr << "cwipc_kinect: starting camera " << serial << ": " << camera_width << "x" << camera_height << "@" << camera_fps << std::endl;
	
	capture_started = true;
}

#ifdef notrs2
void K4ACamera::_computePointSize(rs2::pipeline_profile profile)
{

	// Get the 3D distance between camera and (0,0,0) or use 1m if unreasonable
	float tx = (*camData.trafo)(0,3);
	float ty = (*camData.trafo)(1,3);
	float tz = (*camData.trafo)(2,3);
	float dist = sqrt(tx*tx + ty*ty + tz*tz);
	if (dist == 0) dist = 1;

	// Now get the intrinsics for the depth stream
	auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	auto intrinsics = stream.get_intrinsics(); // Calibration data

	// Compute 2D coordinates of adjacent pixels in the middle of the field of view
	float pixel0[2], pixel1[2];
	pixel0[0] = camera_width / 2;
	pixel0[1] = camera_height / 2;
	if (do_depth_filtering && camSettings.do_decimation) {
		pixel1[0] = pixel0[0] + camSettings.decimation_value;
		pixel1[1] = pixel0[1] + camSettings.decimation_value;
	} else {
		pixel1[0] = pixel0[0] + 1;
		pixel1[1] = pixel0[1] + 1;
	}

	// Deproject to get 3D distance
	float point0[3], point1[3];
	rs2_deproject_pixel_to_point(point0, &intrinsics, pixel0, dist);
	rs2_deproject_pixel_to_point(point1, &intrinsics, pixel1, dist);
	float rv = sqrt(pow(point1[0]-point0[0], 2)+pow(point1[1]-point0[1], 2)+pow(point1[2]-point0[2], 2));
	pointSize = rv;
}
#endif

void K4ACamera::stop()
{
	assert(!stopped);
	stopped = true;
	if (grabber_thread) grabber_thread->join();
	if (processing_thread) processing_thread->join();
	if (capture_started) {
		k4a_device_stop_cameras(device_handle);
		k4a_transformation_destroy(transformation_handle);
	}
	capture_started = false;
	processing_done = true;
	processing_done_cv.notify_one();
	k4a_device_close(device_handle);
}

void K4ACamera::start_capturer()
{
	assert(stopped);
	stopped = false;
	_start_capture_thread();
	processing_thread = new std::thread(&K4ACamera::_processing_thread_main, this);
	setThreadName(processing_thread, L"cwipc_kinect::K4ACamera::processing_thread");
}

void K4ACamera::_start_capture_thread()
{
	grabber_thread = new std::thread(&K4ACamera::_capture_thread_main, this);
	setThreadName(grabber_thread, L"cwipc_kinect::K4ACamera::capture_thread");
}

void K4ACamera::_capture_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "frame capture: cam=" << serial << " thread started" << std::endl;
#endif
	while(!stopped) {
		k4a_capture_t capture_handle;
		if (k4a_capture_create(&capture_handle) != K4A_RESULT_SUCCEEDED) {
			cwipc_k4a_log_warning("k4a_capture_create failed");
			break;
		}
		k4a_wait_result_t ok = k4a_device_get_capture(device_handle, &capture_handle, K4A_WAIT_INFINITE);
		if (ok != K4A_RESULT_SUCCEEDED) {
			std::cerr << "frame capture: error " << ok << std::endl;
			cwipc_k4a_log_warning("k4a_device_get_capture failed");
			break;
		}
		assert(capture_handle != NULL);
#ifdef CWIPC_DEBUG_THREAD
		uint64_t tsRGB = k4a_image_get_device_timestamp_usec(k4a_capture_get_color_image(capture_handle));
		uint64_t tsD = k4a_image_get_device_timestamp_usec(k4a_capture_get_depth_image(capture_handle));
		std::cerr << "frame capture: cam=" << serial << ", rgbseq=" << tsRGB << ", dseq=" << tsD << std::endl;
#endif
		if (!captured_frame_queue.try_enqueue(capture_handle)) {
			// Queue is full. discard.
			std::cerr << "frame capture: drop frame from camera "<< serial << std::endl;
			k4a_capture_release(capture_handle);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "frame capture: cam=" << serial << " thread stopped" << std::endl;
#endif
}

void K4ACamera::_processing_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "frame processing: cam=" << serial << " thread started" << std::endl;
#endif
	while(!stopped) {
		k4a_capture_t processing_frameset;
		bool ok = processing_frame_queue.wait_dequeue_timed(processing_frameset, std::chrono::milliseconds(5000));
		if (!ok) {
			std::cerr << "cwipc_kinect: no frameset for 5 seconds, camera " << serial << std::endl;
			continue;
		}
		assert(processing_frameset);
		std::lock_guard<std::mutex> lock(processing_mutex);
		k4a_image_t depth = k4a_capture_get_depth_image(processing_frameset);
		k4a_image_t color = k4a_capture_get_color_image(processing_frameset);

		// Note: the following code uses color as the main source of resolution. To be decided.
		int color_image_width_pixels = k4a_image_get_width_pixels(color);
		int color_image_height_pixels = k4a_image_get_height_pixels(color);

		k4a_image_t transformed_depth = NULL;
		k4a_result_t sts;
		sts = k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, color_image_width_pixels, color_image_height_pixels, color_image_width_pixels * (int)sizeof(uint16_t), &transformed_depth);
		if (sts != K4A_RESULT_SUCCEEDED) {
			std::cerr << "cwipc_kinect: cannot create transformed depth image" << std::endl;
			break;
		}
		k4a_image_t point_cloud_image = NULL;
		sts = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, color_image_width_pixels, color_image_height_pixels, color_image_width_pixels * 3 * (int)sizeof(int16_t), &point_cloud_image);
		if (sts != K4A_RESULT_SUCCEEDED) {
			std::cerr << "cwipc_kinect: cannot create pointcloud image" << std::endl;
			break;
		}

		sts = k4a_transformation_depth_image_to_color_camera(transformation_handle, depth, transformed_depth);
		if (sts != K4A_RESULT_SUCCEEDED) {
			std::cerr << "cwipc_kinect: cannot transform depth image" << std::endl;
			break;
		}
		sts = k4a_transformation_depth_image_to_point_cloud(transformation_handle, transformed_depth, K4A_CALIBRATION_TYPE_COLOR, point_cloud_image);
		if (sts != K4A_RESULT_SUCCEEDED) {
			std::cerr << "cwipc_kinect: cannot create point cloud" << std::endl;
			break;
		}
		uint8_t* color_data = k4a_image_get_buffer(color);
		int16_t* point_cloud_image_data = (int16_t*)k4a_image_get_buffer(point_cloud_image);
		// Setup depth filtering, if needed
		int16_t min_depth = (int16_t)(camSettings.threshold_near * 1000);
		int16_t max_depth = (int16_t)(camSettings.threshold_far * 1000);
		// now loop over images and create points.
		camData.cloud->clear();
		camData.cloud->reserve(color_image_width_pixels * color_image_height_pixels);
		for (int i = 0; i < color_image_width_pixels * color_image_height_pixels; i++)
		{
			cwipc_pcl_point point;
			point.x = point_cloud_image_data[3 * i + 0];
			point.y = point_cloud_image_data[3 * i + 1];
			point.z = point_cloud_image_data[3 * i + 2];
			if (point.z == 0) continue;
			if (camSettings.do_threshold && (point.z < min_depth || point.z > max_depth)) continue;

			point.r = color_data[4 * i + 2];
			point.g = color_data[4 * i + 1];
			point.b = color_data[4 * i + 0];
			uint8_t alpha = color_data[4 * i + 3];

			if (point.r == 0 && point.g == 0 && point.b == 0 && alpha == 0) continue;

			transformPoint(point);
			if (do_height_filtering && (point.y < height_min || point.y > height_max)) continue;
			if (!do_greenscreen_removal || cwipc_k4a_noChromaRemoval(&point)) // chromakey removal
				camData.cloud->push_back(point);
		}
		// xxxjack free all allocated images
		k4a_image_release(point_cloud_image);
		k4a_image_release(color);
		k4a_image_release(depth);
		k4a_image_release(transformed_depth);
		k4a_capture_release(processing_frameset);
		if (camData.cloud->size() == 0) {
			std::cerr << "cwipc_kinect: warning: captured empty pointcloud from camera " << camData.serial << std::endl;
			//continue;
		}
		// Notify wait_for_pc that we're done.
		processing_done = true;
		processing_done_cv.notify_one(); 
#ifdef notrs2
		// Wait for next frame to process. Allow aborting in case of stopped becoming false...
		rs2::frameset processing_frameset;
		bool ok = processing_frame_queue.try_wait_for_frame(&processing_frameset, 1000);
		if (!ok) continue;

		std::lock_guard<std::mutex> lock(processing_mutex);

		if (do_depth_filtering) {
			processing_frameset = processing_frameset.apply_filter(aligner);
			if (camSettings.do_decimation) processing_frameset = processing_frameset.apply_filter(dec_filter);
			if (camSettings.do_threshold) processing_frameset = processing_frameset.apply_filter(threshold_filter);
			if (camSettings.do_spatial || camSettings.do_temporal) {
				processing_frameset = processing_frameset.apply_filter(depth_to_disparity);
				if (camSettings.do_spatial) processing_frameset = processing_frameset.apply_filter(spat_filter);
				if (camSettings.do_temporal) processing_frameset = processing_frameset.apply_filter(temp_filter);
				processing_frameset = processing_frameset.apply_filter(disparity_to_depth);
			}

		}

		rs2::depth_frame depth = processing_frameset.get_depth_frame();
		rs2::video_frame color = processing_frameset.get_color_frame();
		assert(depth);
		assert(color);
#ifdef CWIPC_DEBUG
		std::cerr << "frame processing: cam=" << serial << ", depthseq=" << depth.get_frame_number() << ", colorseq=" << depth.get_frame_number() << std::endl;
#endif

		// Calculate new pointcloud, map to the color images and get vertices and color indices
        pointcloud.map_to(color);
		auto points = pointcloud.calculate(depth);
		auto vertices = points.get_vertices();
		auto texture_coordinates = points.get_texture_coordinates();

		// Get some constants used later to map colors and such from rs2 to pcl pointclouds.
		const int texture_width = color.get_width();
		const int texture_height = color.get_height();
		const int texture_x_step = color.get_bytes_per_pixel();
		const int texture_y_step = color.get_stride_in_bytes();
		const unsigned char *texture_data = (unsigned char*)color.get_data();
		const uint8_t camera_label = (uint8_t)1 << camera_index;

		// Clear the previous pointcloud and pre-allocate space in the pointcloud (so we don't realloc)
		camData.cloud->clear();
		camData.cloud->reserve(points.size());
#ifdef WITH_MANUAL_BACKGROUND_REMOVAL
		// Note by Jack: this code is currently not correct, hasn't been updated
		// for the texture coordinate mapping.
		if (do_background_removal) {

			// Set the background removal window
			if (camData.background.z > 0.0) {
				maxz = camData.background.z;
				minz = 0.0;
				if (camData.background.x != 0.0) {
					minx = camData.background.x;
				}
				else {
					for (int i = 0; i < points.size(); i++) {
						double minz = 100;
						if (vertices[i].z != 0 && minz > vertices[i].z) {
							minz = vertices[i].z;
							minx = vertices[i].x;
						}
					}
				}
			}
			else {
				minz = 100.0;
				for (int i = 0; i < points.size(); i++) {
					if (vertices[i].z != 0 && minz > vertices[i].z) {
						minz = vertices[i].z;
						minx = vertices[i].x;
					}
				}
				maxz = 0.8f + minz;
			}
			// Make PointCloud
			for (int i = 0; i < points.size(); i++) {
				double x = minx - vertices[i].x; x *= x;
				double z = vertices[i].z;
				if (minz < z && z < maxz - x) { // Simple background removal, horizontally parabolic, vertically straight.
					cwipc_pcl_point pt;
					pt.x = vertices[i].x;
					pt.y = -vertices[i].y;
					pt.z = -z;
					if (do_height_filtering && ( pt.y < height_min || pt.y > height_max)) continue;
					int pi = i * 3;
					pt.r = texture_data[pi];
					pt.g = texture_data[pi + 1];
					pt.b = texture_data[pi + 2];
					pt.a = camera_label;
					if (!do_greenscreen_removal || cwipc_k4a_noChromaRemoval(&pt)) // chromakey removal
						camData.cloud->push_back(pt);
				}
			}
		}
		else
#endif // WITH_MANUAL_BACKGROUND_REMOVAL
		{
			// Make PointCloud
			for (int i = 0; i < points.size(); i++) {
				// Skip points with z=0 (they don't exist)
				if (vertices[i].z == 0) continue;

				cwipc_pcl_point pt;
				transformPoint(pt, vertices[i]);
				if (do_height_filtering && (pt.y < height_min || pt.y > height_max)) continue;
				float u = texture_coordinates[i].u;
				float v = texture_coordinates[i].v;
                int texture_x = int(0.5 + u*texture_width);
                int texture_y = int(0.5 + v*texture_height);
                // Unsure whether this ever happens: out-of-bounds u/v points are skipped
                if (texture_x <= 0 || texture_x >= texture_width-1) continue;
                if (texture_y <= 0 || texture_y >= texture_height-1) continue;
				int idx = texture_x * texture_x_step + texture_y * texture_y_step;
				pt.r = texture_data[idx];
				pt.g = texture_data[idx + 1];
				pt.b = texture_data[idx + 2];
                // Unexpectedly, this does happen: 100% black points don't actually exist.
                if (pt.r == 0 && pt.g == 0 && pt.b == 0) continue;
				pt.a = camera_label;
				if (!do_greenscreen_removal || cwipc_k4a_noChromaRemoval(&pt)) // chromakey removal
					camData.cloud->push_back(pt);
			}
		}
		if (camData.cloud->size() == 0) {
			std::cerr << "cwipc_kinect: warning: captured empty pointcloud from camera " << camData.serial << std::endl;
            //continue;
		}
		// Notify wait_for_pc that we're done.
		processing_done = true;
		processing_done_cv.notify_one();
#endif // notrs2
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "frame processing: cam=" << serial << " thread stopped" << std::endl;
#endif
}

void K4ACamera::transformPoint(cwipc_pcl_point& pt)
{
	float x = pt.x / 1000.0;
	float y = pt.y / 1000.0;
	float z = pt.z / 1000.0;
	pt.x = (*camData.trafo)(0,0)*x + (*camData.trafo)(0,1)*y + (*camData.trafo)(0,2)*z + (*camData.trafo)(0,3);
	pt.y = (*camData.trafo)(1,0)*x + (*camData.trafo)(1,1)*y + (*camData.trafo)(1,2)*z + (*camData.trafo)(1,3);
	pt.z = (*camData.trafo)(2,0)*x + (*camData.trafo)(2,1)*y + (*camData.trafo)(2,2)*z + (*camData.trafo)(2,3);
}

void K4ACamera::create_pc_from_frames()
{
	assert(current_frameset);
	processing_frame_queue.enqueue(current_frameset);
}

void K4ACamera::wait_for_pc()
{
	std::unique_lock<std::mutex> lock(processing_mutex);
	processing_done_cv.wait(lock, [this]{ return processing_done; });
	processing_done = false;
}

uint64_t K4ACamera::get_capture_timestamp()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void
K4ACamera::dump_color_frame(const std::string& filename)
{
#ifdef notrs2
#ifdef WITH_DUMP_VIDEO_FRAMES
		rs2::video_frame color = current_frameset.get_color_frame();
		stbi_write_png(filename.c_str(), color.get_width(), color.get_height(),
			color.get_bytes_per_pixel(), color.get_data(), color.get_stride_in_bytes());
#endif
#endif
}

