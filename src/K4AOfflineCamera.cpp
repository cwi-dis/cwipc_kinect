//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//
#include <cstdlib>

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD
#undef CWIPC_MEMORY_DEBUG

#ifdef CWIPC_MEMORY_DEBUG
#include <vld.h>
#endif

// This is the dll source, so define external symbols as dllexport on windows.

#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_KINECT_EXPORT __declspec(dllexport)
#endif

#include "cwipc_kinect/private/K4AOfflineCamera.hpp"
#include "turbojpeg.h"

typedef struct HsvColor
{
	unsigned char h;
	unsigned char s;
	unsigned char v;
} HsvColor;


static HsvColor rgbToHsv(cwipc_pcl_point* pnt)
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

static bool isNotGreen(cwipc_pcl_point* p)
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

K4AOfflineCamera::K4AOfflineCamera(recording_t _recording, K4ACaptureConfig& configuration, int _camera_index)
	: pointSize(0), minx(0), minz(0), maxz(0),
	playback_handle(_recording.handle),
	camera_index(_camera_index),
	stopped(true),
	camera_started(false),
	capture_started(false),
	camData(configuration.camera_data[camera_index]),
	serial(configuration.camera_data[camera_index].serial),
	filename(configuration.camera_data[camera_index].filename),
	camSettings(configuration.camera_config),
	captured_frame_queue(1),
	processing_frame_queue(1),
	current_frameset(NULL),
	color_height(configuration.color_height),
	depth_height(configuration.depth_height),
	camera_fps(configuration.fps),
	camera_sync_ismaster(serial == configuration.sync_master_serial),
	camera_sync_inuse(configuration.sync_master_serial != ""),
	do_greenscreen_removal(configuration.greenscreen_removal),
	do_height_filtering(configuration.height_min != configuration.height_max),
	height_min(configuration.height_min),
	height_max(configuration.height_max)
{
#ifdef CWIPC_DEBUG
	std::cout << "K4ACapture: creating camera " << serial << std::endl;
#endif
	assert(stopped);
	k4a_calibration_t calibration;
	if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback_handle, &calibration))
	{
		std::cerr << "cwipc_kinect: K4AOfflineCamera:  Failed to k4a_device_get_calibration" << std::endl;
		camera_started = false;
		return;
	}
	else {
		transformation_handle = k4a_transformation_create(&calibration);
		camera_started = true;
	}

	max_delay = 10 * 160; //we set a 160us delay between cameras to avoid laser interference. It is enough for 10x cameras
	_init_filters();
}

K4AOfflineCamera::~K4AOfflineCamera()
{
#ifdef CWIPC_DEBUG
	std::cout << "cwipc_kinect: K4AOfflineCamera: destroying " << serial << std::endl;
#endif
	assert(stopped);
}

void K4AOfflineCamera::_init_filters()
{
	if (!do_depth_filtering) return;
}

bool K4AOfflineCamera::prepare_next_valid_frame() {
	k4a_result_t result;
	k4a_stream_result_t stream_result;
	// Read the next capture into memory
	bool succeeded = false;
	while (!succeeded) {
		if (stopped) return false;
		assert(playback_handle);
		if (current_frameset != NULL)
			k4a_capture_release(current_frameset);
		stream_result = k4a_playback_get_next_capture(playback_handle, &current_frameset);
		if (stream_result == K4A_STREAM_RESULT_EOF)
		{
			if (current_frameset_timestamp == 0) {
				std::cerr << "cwipc_kinect: K4AOfflineCamera: ERROR: Recording file is empty: " << filename << std::endl;
				result = K4A_RESULT_FAILED;
			}
			else {
				std::cout << "cwipc_kinect: K4AOfflineCamera: Recording file " << filename << " reached EOF" << std::endl;
				eof = true;
			}
			break;
		}
		else if (stream_result == K4A_STREAM_RESULT_FAILED)
		{
			std::cerr << "cwipc_kinect: K4AOfflineCamera: ERROR: Failed to read first capture from file: " << filename << std::endl;
			result = K4A_RESULT_FAILED;
			break;
		}
		capture_id++;
		k4a_image_t color = k4a_capture_get_color_image(current_frameset);
		if (color == NULL) {
			std::cerr << "cwipc_kinect: K4AOfflineCamera: Color is missing in capture " << capture_id << " from " << filename << std::endl;
			continue;
		}

		k4a_image_t depth = k4a_capture_get_depth_image(current_frameset);
		if (depth == NULL) {
			std::cerr << "cwipc_kinect: K4AOfflineCamera: Depth is missing in capture " << capture_id << " from " << filename << std::endl;
			//color was not null so we have to release it
			k4a_image_release(color);
			continue;
		}
		current_frameset_timestamp = k4a_image_get_device_timestamp_usec(color);
		succeeded = true;

		k4a_image_release(color);
		k4a_image_release(depth);
	}
	return succeeded;
}

bool K4AOfflineCamera::prepare_cond_next_valid_frame(uint64_t master_timestamp) {
	/// <summary>
	/// returns -1 if there was a problem, 1 if satisfied condition, 2 if we need to update master frame
	/// </summary>
	/// <param name="file"></param>
	/// <param name="master_timestamp"></param>
	/// <param name="max_delay"></param>
	/// <returns></returns>
	bool satisfies_condition = false;
	//check if current frame already satisfies the condition
	if (current_frameset != NULL && (current_frameset_timestamp > master_timestamp)) {
		if (current_frameset_timestamp < (master_timestamp + max_delay)) //satisfies
		{
			return 1;
		}
		else { //update master
			return 2;
		}

	}
	//otherwise start process to find a frame that satisfies the condition.
	while (!satisfies_condition)
	{
		bool ok = prepare_next_valid_frame();
		if (!ok) break;
		if (current_frameset_timestamp > master_timestamp) {
			if (current_frameset_timestamp < (master_timestamp + max_delay)) {
				satisfies_condition = true;
			}
			else {  //it is a future frame, we need to update master frame
				return false;
			}
		}
	}
	return satisfies_condition;
}

bool K4AOfflineCamera::capture_frameset(uint64_t master_timestamp)
{
	if (stopped) return false;
	bool rv;
	if (camera_sync_inuse) {
		if (camera_sync_ismaster) {
			rv = prepare_next_valid_frame(); //TODO: use return value?
		}
		else {
			rv = prepare_cond_next_valid_frame(master_timestamp);
		}
	}
	else {
		rv = prepare_next_valid_frame(); //TODO: use return value?
	}


#ifdef CWIPC_DEBUG_THREAD
	if (current_frameset == NULL) {
		std::cerr << "cwipc_kinect: K4AOfflineCamera: " << camera_index << " forward NULL frame" << std::endl;
	}
	else {
		k4a_image_t color = k4a_capture_get_color_image(current_frameset);
		uint64_t tsRGB = k4a_image_get_device_timestamp_usec(color);
		k4a_image_release(color);
		k4a_image_t depth = k4a_capture_get_depth_image(current_frameset);
		uint64_t tsD = k4a_image_get_device_timestamp_usec(depth);
		k4a_image_release(depth);
		std::cerr << "cwipc_kinect: K4AOfflineCamera: forward frame: cam=" << serial << ", rgbseq=" << tsRGB << ", dseq=" << tsD << std::endl;
	}
#endif

	return rv;
}

#ifdef notrs2
void K4AOfflineCamera::_computePointSize(rs2::pipeline_profile profile)
{

	// Get the 3D distance between camera and (0,0,0) or use 1m if unreasonable
	float tx = (*camData.trafo)(0, 3);
	float ty = (*camData.trafo)(1, 3);
	float tz = (*camData.trafo)(2, 3);
	float dist = sqrt(tx * tx + ty * ty + tz * tz);
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
	}
	else {
		pixel1[0] = pixel0[0] + 1;
		pixel1[1] = pixel0[1] + 1;
	}

	// Deproject to get 3D distance
	float point0[3], point1[3];
	rs2_deproject_pixel_to_point(point0, &intrinsics, pixel0, dist);
	rs2_deproject_pixel_to_point(point1, &intrinsics, pixel1, dist);
	float rv = sqrt(pow(point1[0] - point0[0], 2) + pow(point1[1] - point0[1], 2) + pow(point1[2] - point0[2], 2));
	pointSize = rv;
}
#endif

void K4AOfflineCamera::stop()
{
	assert(!stopped);
	stopped = true;
	processing_frame_queue.try_enqueue(NULL);
	if (capture_started) {
		k4a_transformation_destroy(transformation_handle);
		camera_started = false;
	}
	if (processing_thread) processing_thread->join();
	delete processing_thread;
	processing_done = true;
	processing_done_cv.notify_one();
	if (current_frameset != NULL) {
		k4a_capture_release(current_frameset);
		current_frameset = NULL;
	}
	k4a_playback_close(playback_handle);
	playback_handle = NULL;
}

void K4AOfflineCamera::start_capturer()
{
	if (!camera_started) return;
	assert(stopped);
	stopped = false;
	capture_started = true;
	processing_thread = new std::thread(&K4AOfflineCamera::_processing_thread_main, this);
	_cwipc_setThreadName(processing_thread, L"cwipc_kinect::K4AOfflineCamera::processing_thread");
}

void K4AOfflineCamera::_processing_thread_main()
{
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "cwipc_kinect: K4AOfflineCamera: processing: cam=" << serial << " thread started" << std::endl;
#endif
	while (!stopped) {
		k4a_capture_t processing_frameset = NULL;
		k4a_image_t depth_image = NULL;
		k4a_image_t color_image = NULL;

		bool ok = processing_frame_queue.wait_dequeue_timed(processing_frameset, std::chrono::milliseconds(10000));
		if (processing_frameset == NULL) {
#ifdef CWIPC_DEBUG_THREAD
			std::cerr << "cwipc_kinect: K4AOfflineCamera: processing thread: null frameset" << std::endl;
#endif
			continue;
		}
		if (!ok) {
			std::cerr << "cwipc_kinect: K4AOfflineCamera: no frameset for 10 seconds, camera " << serial << std::endl;
			continue;
		}
#ifdef CWIPC_DEBUG_THREAD
		std::cerr << "cwipc_kinect: K4AOfflineCamera: processing: got frame for camera " << serial << std::endl;
#endif
		assert(processing_frameset);
		std::lock_guard<std::mutex> lock(processing_mutex);
		depth_image = k4a_capture_get_depth_image(processing_frameset);
		color_image = k4a_capture_get_color_image(processing_frameset);

		k4a_image_t uncompressed_color_image = NULL;
		int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
		int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
		if (k4a_image_get_format(color_image) == K4A_IMAGE_FORMAT_COLOR_MJPG) {
			//COLOR image is JPEG compressed. we need to convert the image to BGRA format.

			if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
				color_image_width_pixels,
				color_image_height_pixels,
				color_image_width_pixels * 4 * (int)sizeof(uint8_t),
				&uncompressed_color_image))
			{
				std::cerr << "Failed to create image buffer" << std::endl;
				return;
			}

			tjhandle tjHandle;
			tjHandle = tjInitDecompress();
			if (tjDecompress2(tjHandle,
				k4a_image_get_buffer(color_image),
				static_cast<unsigned long>(k4a_image_get_size(color_image)),
				k4a_image_get_buffer(uncompressed_color_image),
				color_image_width_pixels,
				0, // pitch
				color_image_height_pixels,
				TJPF_BGRA,
				TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0)
			{
				std::cerr << "Failed to decompress color frame" << std::endl;
				if (tjDestroy(tjHandle))
				{
					std::cerr << "Failed to destroy turboJPEG handle" << std::endl;
				}
				return;
			}
			if (tjDestroy(tjHandle))
			{
				std::cerr << "Failed to destroy turboJPEG handle" << std::endl;
			}
		}

		cwipc_pcl_pointcloud new_pointcloud = nullptr;
		if (camSettings.map_color_to_depth) {
			new_pointcloud = generate_point_cloud_color_to_depth(transformation_handle, depth_image, uncompressed_color_image);
		}
		else {
			new_pointcloud = generate_point_cloud_depth_to_color(transformation_handle, depth_image, uncompressed_color_image);
		}
		if (new_pointcloud != nullptr) {
			current_pointcloud = new_pointcloud;
#ifdef CWIPC_DEBUG_THREAD
			std::cerr << "cwipc_kinect: K4AOfflineCamera: camera " << serial << " produced " << camData.cloud->size() << " point" << std::endl;
#endif
			if (current_pointcloud->size() == 0) {
				std::cerr << "cwipc_kinect: K4AOfflineCamera: warning: captured empty pointcloud from camera " << camData.serial << std::endl;
				//continue;
			}
			// Notify wait_for_pc that we're done.
			processing_done = true;
			processing_done_cv.notify_one();
		}
		if (processing_frameset != NULL) k4a_capture_release(processing_frameset);
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "cwipc_kinect: K4AOfflineCamera: processing: cam=" << serial << " thread stopped" << std::endl;
#endif
}

cwipc_pcl_pointcloud K4AOfflineCamera::generate_point_cloud_color_to_depth(k4a_transformation_t transformation_handle,
	const k4a_image_t depth_image,
	const k4a_image_t color_image)
{
	int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
	int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
	k4a_image_t transformed_color_image = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
		depth_image_width_pixels,
		depth_image_height_pixels,
		depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
		&transformed_color_image))
	{
		std::cerr << "cwipc_kinect: K4AOfflineCamera: Failed to create transformed color image" << std::endl;
		return nullptr;
	}

	k4a_image_t point_cloud_image = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		depth_image_width_pixels,
		depth_image_height_pixels,
		depth_image_width_pixels * 3 * (int)sizeof(int16_t),
		&point_cloud_image))
	{
		std::cerr << "cwipc_kinect: K4AOfflineCamera: Failed to create point cloud image" << std::endl;
		return nullptr;
	}

	if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
		depth_image,
		color_image,
		transformed_color_image))
	{
		std::cerr << "cwipc_kinect: K4AOfflineCamera: Failed to compute transformed color image" << std::endl;
		return nullptr;
	}

	if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
		depth_image,
		K4A_CALIBRATION_TYPE_DEPTH,
		point_cloud_image))
	{
		std::cerr << "cwipc_kinect: K4AOfflineCamera: Failed to compute point cloud" << std::endl;
		return nullptr;
	}

	cwipc_pcl_pointcloud rv = generate_point_cloud(point_cloud_image, transformed_color_image);

	k4a_image_release(transformed_color_image);
	k4a_image_release(point_cloud_image);

	return rv;
}

cwipc_pcl_pointcloud K4AOfflineCamera::generate_point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
	const k4a_image_t depth_image,
	const k4a_image_t color_image)
{
	// transform color image into depth camera geometry
	int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
	int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
	k4a_image_t transformed_depth_image = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * (int)sizeof(uint16_t),
		&transformed_depth_image))
	{
		std::cerr << "cwipc_kinect: K4AOfflineCamera: Failed to create transformed depth image" << std::endl;
		return nullptr;
	}

	k4a_image_t point_cloud_image = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * 3 * (int)sizeof(int16_t),
		&point_cloud_image))
	{
		std::cerr << "cwipc_kinect: K4AOfflineCamera: Failed to create point cloud image" << std::endl;
		return nullptr;
	}

	if (K4A_RESULT_SUCCEEDED !=
		k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
	{
		std::cerr << "cwipc_kinect: K4AOfflineCamera: Failed to compute transformed depth image" << std::endl;
		return nullptr;
	}

	if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
		transformed_depth_image,
		K4A_CALIBRATION_TYPE_COLOR,
		point_cloud_image))
	{
		std::cerr << "cwipc_kinect: K4AOfflineCamera: Failed to compute point cloud" << std::endl;
		return nullptr;
	}

	cwipc_pcl_pointcloud rv = generate_point_cloud(point_cloud_image, color_image);

	k4a_image_release(transformed_depth_image);
	k4a_image_release(point_cloud_image);

	return rv;
}

cwipc_pcl_pointcloud K4AOfflineCamera::generate_point_cloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image)
{
	int width = k4a_image_get_width_pixels(point_cloud_image);
	int height = k4a_image_get_height_pixels(color_image);

	uint8_t* color_data = k4a_image_get_buffer(color_image);
	int16_t* point_cloud_image_data = (int16_t*)k4a_image_get_buffer(point_cloud_image);
	if (camSettings.do_threshold || camSettings.depth_x_erosion || camSettings.depth_y_erosion) {
		_filter_depth_data(point_cloud_image_data, width, height);
	}
	// Setup depth filtering, if needed
	// now loop over images and create points.
	cwipc_pcl_pointcloud new_cloud = new_cwipc_pcl_pointcloud();
	new_cloud->clear();
	new_cloud->reserve(width * height);
	for (int i = 0; i < width * height; i++)
	{
		int i_pc = i * 3;
		int i_rgba = i * 4;
		cwipc_pcl_point point;
		int16_t x = point_cloud_image_data[i_pc + 0];
		int16_t y = point_cloud_image_data[i_pc + 1];
		int16_t z = point_cloud_image_data[i_pc + 2];
		if (z == 0) continue;

		// color_data is BGR
		point.r = color_data[i_rgba + 2];
		point.g = color_data[i_rgba + 1];
		point.b = color_data[i_rgba + 0];
		point.a = (uint8_t)1 << camera_index;
		uint8_t alpha = color_data[i_rgba + 3];

		if (point.r == 0 && point.g == 0 && point.b == 0 && alpha == 0) continue;
		point.x = x;
		point.y = y;
		point.z = z;
		transformPoint(point);
		if (do_height_filtering && (point.y < height_min || point.y > height_max)) continue;
		if (!do_greenscreen_removal || isNotGreen(&point)) // chromakey removal
			new_cloud->push_back(point);
	}
#ifdef CWIPC_DEBUG_THREAD
	std::cerr << "cwipc_kinect: camera " << serial << " produced " << camData.cloud->size() << " point" << std::endl;
#endif
	return new_cloud;
}

void K4AOfflineCamera::_filter_depth_data(int16_t* depth_values, int width, int height) {
	int16_t min_depth = (int16_t)(camSettings.threshold_near * 1000);
	int16_t max_depth = (int16_t)(camSettings.threshold_far * 1000);
	int16_t* z_values = (int16_t*)calloc(width * height, sizeof(int16_t));
	// Pass one: Copy Z values to temporary buffer, but leave out-of-range values at zero.
	for (int i = 0; i < width * height; i++)
	{
		int i_pc = i * 3;
		int16_t z = depth_values[i_pc + 2];
		if (camSettings.do_threshold && (z <= min_depth || z >= max_depth)) continue;
		z_values[i] = z;
	}
	// Pass two: loop for zero pixels in temp buffer, and clear out x/y pixels adjacent in depth buffer
	int x_delta = camSettings.depth_x_erosion;
	int y_delta = camSettings.depth_y_erosion;
	if (x_delta || y_delta) {
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				if (z_values[x + y * width] != 0) continue;
				// Zero depth at (x, y). Clear out pixels 
				for (int ix = x - x_delta; ix <= x + x_delta; ix++) {
					if (ix < 0 || ix >= width) continue;
					int i_pc = (ix + y * width) * 3;
					depth_values[i_pc + 2] = 0;
				}
				for (int iy = y - y_delta; iy <= y + y_delta; iy++) {
					if (iy < 0 || iy >= height) continue;
					int i_pc = (x + iy * width) * 3;
					depth_values[i_pc + 2] = 0;
				}
			}
		}
	}
	else {
		// Pass three: clear out zero pixels from temporary buffer.
		for (int i = 0; i < width * height; i++)
		{
			if (z_values[i] != 0) continue;
			int i_pc = i * 3;
			depth_values[i_pc + 2] = 0;
		}
	}
	free(z_values);
}

void K4AOfflineCamera::transformPoint(cwipc_pcl_point& pt)
{
	float x = pt.x / 1000.0;
	float y = pt.y / 1000.0;
	float z = pt.z / 1000.0;
	pt.x = (*camData.trafo)(0, 0) * x + (*camData.trafo)(0, 1) * y + (*camData.trafo)(0, 2) * z + (*camData.trafo)(0, 3);
	pt.y = (*camData.trafo)(1, 0) * x + (*camData.trafo)(1, 1) * y + (*camData.trafo)(1, 2) * z + (*camData.trafo)(1, 3);
	pt.z = (*camData.trafo)(2, 0) * x + (*camData.trafo)(2, 1) * y + (*camData.trafo)(2, 2) * z + (*camData.trafo)(2, 3);
}

void K4AOfflineCamera::create_pc_from_frames()
{
	assert(current_capture);
	if (!processing_frame_queue.try_enqueue(current_frameset)) {
		std::cerr << "cwipc_kinect: K4AOfflineCamera:  camera " << serial << ": drop frame before processing" << std::endl;
		k4a_capture_release(current_frameset);
	}
	current_frameset = NULL;
}

void K4AOfflineCamera::wait_for_pc()
{
	std::unique_lock<std::mutex> lock(processing_mutex);
	processing_done_cv.wait(lock, [this] { return processing_done; });
	processing_done = false;
}

uint64_t K4AOfflineCamera::get_capture_timestamp()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void
K4AOfflineCamera::save_auxdata(cwipc* pc, bool rgb, bool depth)
{
	if (rgb) {
		std::string name = "rgb." + serial;
		k4a_image_t image = k4a_capture_get_color_image(current_frameset);
		if (image != NULL) {
			uint8_t* data_pointer = k4a_image_get_buffer(image);
			const size_t size = k4a_image_get_size(image);
			int width = k4a_image_get_width_pixels(image);
			int height = k4a_image_get_height_pixels(image);
			int stride = k4a_image_get_stride_bytes(image);
			int format = k4a_image_get_format(image);
			std::string description =
				"width=" + std::to_string(width) +
				",height=" + std::to_string(height) +
				",stride=" + std::to_string(stride) +
				",format=" + std::to_string(format);
			void* pointer = malloc(size);
			if (pointer) {
				memcpy(pointer, data_pointer, size);
				cwipc_auxiliary_data* ap = pc->access_auxiliary_data();
				ap->_add(name, description, pointer, size, ::free);
}
		}
	}
	if (depth) {
		std::string name = "depth." + serial;
		k4a_image_t image = k4a_capture_get_depth_image(current_frameset);
		if (image != NULL) {
			uint8_t* data_pointer = k4a_image_get_buffer(image);
			const size_t size = k4a_image_get_size(image);
			int width = k4a_image_get_width_pixels(image);
			int height = k4a_image_get_height_pixels(image);
			int stride = k4a_image_get_stride_bytes(image);
			int format = k4a_image_get_format(image);
			std::string description =
				"width=" + std::to_string(width) +
				",height=" + std::to_string(height) +
				",stride=" + std::to_string(stride) +
				",format=" + std::to_string(format);
			void* pointer = malloc(size);
			if (pointer) {
				memcpy(pointer, data_pointer, size);
				cwipc_auxiliary_data* ap = pc->access_auxiliary_data();
				ap->_add(name, description, pointer, size, ::free);
			}
		}
	}
}

void
K4AOfflineCamera::dump_color_frame(const std::string& filename)
{
#ifdef WITH_DUMP_VIDEO_FRAMES
	k4a_image_t color = k4a_capture_get_color_image(current_frameset);
	if (color != NULL) {
		int color_image_width_pixels = k4a_image_get_width_pixels(color);
		int color_image_height_pixels = k4a_image_get_height_pixels(color);
		int bytes_per_pixel = 4;
		uint8_t* color_data = k4a_image_get_buffer(color);
		int color_stride_bytes = k4a_image_get_stride_bytes(color);
		long timestamp = k4a_image_get_device_timestamp_usec(color);

		stbi_write_png(filename.c_str(), color_image_width_pixels, color_image_height_pixels,
			bytes_per_pixel, color_data, color_stride_bytes);

		std::cout << "cwipc_kinect: dumped image. Camera: " << camera_index << " t=" << timestamp << std::endl;
		k4a_image_release(color);
	}
	else {
		std::cerr << "cwipc_kinect: error: dumping image. serial: " << camData.serial << std::endl;
	}
#endif
}

