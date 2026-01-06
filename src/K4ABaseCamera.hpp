#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <k4arecord/record.h>
#include <k4abt.h>

#include "cwipc_util/api_pcl.h"
#include "cwipc_kinect/api.h"
#include "cwipc_util/capturers.hpp"

#include "K4AConfig.hpp"
#include "readerwriterqueue.h"

//opencv xxxnacho
#include <opencv2/opencv.hpp>

// Check which K4ABT version we have
# if K4ABT_VERSION_MAJOR >= 1 && K4ABT_VERSION_MINOR >= 1
#define K4ABT_SUPPORTS_MODEL_PATH
#endif

// Define to get (a little) debug prints
#undef CWIPC_DEBUG
#undef CWIPC_DEBUG_THREAD

typedef union {
    /** XY or array representation of vector */
    struct _bgra {
        uint8_t b; /**< b component of a vector */
        uint8_t g; /**< g component of a vector */
        uint8_t r; /**< r component of a vector */
        uint8_t a; /**< a component of a vector */
    } bgra;        /**< B, G, R, A representation of a vector */

    uint8_t v[4];  /**< Array representation of a vector */
} cwi_bgra_t;

template<typename Type_api_camera> 
class K4ABaseCamera : public CwipcBaseCamera {
public:
    K4ABaseCamera(const std::string& _Classname, Type_api_camera _handle, K4ACaptureConfig& _configuration, int _camera_index, K4ACameraConfig& _camData)
    :   CwipcBaseCamera(_Classname + ": " + _camData.serial, "kinect"),
        configuration(_configuration),
        camera_handle(_handle),
        camera_config(_camData),
        camera_index(_camera_index),
        serial(_camData.serial),
        captured_frame_queue(1),
        processing_frame_queue(1),
        camera_sync_ismaster(serial == configuration.sync_master_serial),
        camera_sync_inuse(configuration.sync_master_serial != ""),
        do_height_filtering(configuration.height_min != configuration.height_max)
    {

    }

    virtual ~K4ABaseCamera() {
        _log_debug("Destroying camera object");
        assert(camera_stopped);

        if (tracker_handle) {
            k4abt_tracker_shutdown(tracker_handle);
            k4abt_tracker_destroy(tracker_handle);
            tracker_handle = nullptr;
        }
    }
    /// Step 1 in starting: tell the camera we are going to start. Called for all cameras.
    virtual bool pre_start_all_cameras() final {
        if (!_init_filters()) {
            return false;
        }
        if (!_init_hardware_for_this_camera()) {
            return false;
        }
        return true;
     }
    /// Step 2 in starting: starts the camera. Called for all cameras. 
    virtual bool start_camera() = 0;
    /// Step 3 in starting: starts the capturer. Called after all cameras have been started.
    virtual void start_camera_streaming() = 0;
    /// Step 4, called after all capturers have been started.
    virtual void post_start_all_cameras() final {}
    virtual void pre_stop_camera() final {}
    virtual void stop_camera() = 0;

    virtual bool is_sync_master() final {
        return camera_sync_ismaster;
    }
protected:
    // internal API that is "shared" with other implementations (realsense, kinect)
    virtual bool _init_hardware_for_this_camera() = 0;
    virtual bool _init_filters() override final {
        // K4A API does not implement any filtering, so nothing to initialize
        // xxxjack or should we initialize the XY table here?
        return true;
    }

    virtual void _apply_filters() final {
        // xxxjack to be implemented. and used.
    }

    virtual bool _init_tracker() override final {
        if (tracker_handle != NULL) {
            return true;
        }

        k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
        tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;

        if (configuration.bt_processing_mode >= 0) {
            tracker_config.processing_mode = (k4abt_tracker_processing_mode_t)configuration.bt_processing_mode;
        }

        if (configuration.bt_sensor_orientation >= 0) {
            tracker_config.sensor_orientation = (k4abt_sensor_orientation_t)configuration.bt_sensor_orientation;
        }

#ifdef K4ABT_SUPPORTS_MODEL_PATH
        if (configuration.bt_model_path != "") {
            tracker_config.model_path = configuration.bt_model_path.c_str();
        }
#endif

        auto sts = k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker_handle);
        const char* processingModesStr[] = { "GPU", "CPU", "GPU_CUDA", "GPU_TENSORRT", "GPU_DIRECTML" };
        const char* sensorOrientationsStr[] = { "Default", "Clockwise_90", "CounterClockwise_90", "Flip_180" };

        if (sts != K4A_RESULT_SUCCEEDED) {
            _log_error("Body tracker initialization failed: " + std::to_string(sts));
            return false;
        } else {
            _log_trace(std::string("Body tracker initialized. Processing mode: ") +
                processingModesStr[tracker_config.processing_mode] +
                ", Sensor orientation: " + sensorOrientationsStr[tracker_config.sensor_orientation]
            );
        }

        return true;
    }
    // xxxjack _prepare_config_for_starting_camera() has different signatures for camera/recording.


public:
    virtual bool request_skeleton_auxdata(bool _skl) final {
        want_auxdata_skeleton = _skl;

        if (want_auxdata_skeleton) {
            return _init_tracker();
        }
        else {
            return false;
        }
    }

    virtual void create_pc_from_frames() final {
        assert(current_frameset);

        if (!processing_frame_queue.try_enqueue(current_frameset)) {
            _log_warning("processing frame queue full, dropping frame");
            k4a_capture_release(current_frameset);
        }

        current_frameset = NULL;
    }

    virtual void wait_for_pc() final {
        std::unique_lock<std::mutex> lock(processing_mutex);
        processing_done_cv.wait(lock, [this] { return processing_done; });
        processing_done = false;
    }

    virtual uint64_t get_capture_timestamp() final {
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }

    virtual cwipc_pcl_pointcloud get_current_pointcloud() final {
        return current_pointcloud;
    }

    virtual void save_auxdata_images(cwipc* pc, bool rgb, bool depth) final {
        k4a_image_t color_image = k4a_capture_get_color_image(current_frameset);
        k4a_image_t depth_image = k4a_capture_get_depth_image(current_frameset);
        if (color_image == nullptr || depth_image == nullptr) return;
        int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
        int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
        int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
        int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
        if (rgb) {
            std::string name = "rgb." + serial;
            color_image = _uncompress_color_image(current_frameset, color_image);
            if (configuration.camera_processing.map_color_to_depth) {
                k4a_image_t transformed_color_image = NULL;
                k4a_result_t status;

                status = k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                    depth_image_width_pixels,
                    depth_image_height_pixels,
                    depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
                    &transformed_color_image
                );

                if (status != K4A_RESULT_SUCCEEDED) {
                    _log_error("Failed to create transformed color image: " + std::to_string(status));
                    return;
                }

                status = k4a_transformation_color_image_to_depth_camera(transformation_handle,
                    depth_image,
                    color_image,
                    transformed_color_image
                );

                if (status != K4A_RESULT_SUCCEEDED) {
                    _log_error("Failed to compute transformed color image: " + std::to_string(status));
                    return;
                }


                k4a_image_release(color_image);
                color_image = transformed_color_image;
            }
            uint8_t* data_pointer = k4a_image_get_buffer(color_image);
            const size_t size = k4a_image_get_size(color_image);
            int width = k4a_image_get_width_pixels(color_image);
            int height = k4a_image_get_height_pixels(color_image);
            int stride = k4a_image_get_stride_bytes(color_image);
            int format = k4a_image_get_format(color_image);

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

            k4a_image_release(color_image);
        }

        if (depth) {
            std::string name = "depth." + serial;
            if (!configuration.camera_processing.map_color_to_depth) {
                k4a_image_t transformed_depth_image = NULL;
                k4a_result_t status;
                status = k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                    color_image_width_pixels,
                    color_image_height_pixels,
                    color_image_width_pixels * (int)sizeof(uint16_t),
                    &transformed_depth_image
                );

                if (status != K4A_RESULT_SUCCEEDED) {
                    _log_error("Failed to create transformed depth image: " + std::to_string(status));
                    return;
                }

                status = k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image);
                if (status != K4A_RESULT_SUCCEEDED) {
                    _log_error("Failed to compute transformed depth image: " + std::to_string(status));
                    return;
                }
                k4a_image_release(depth_image);
                depth_image = transformed_depth_image;
            }

            uint8_t* data_pointer = k4a_image_get_buffer(depth_image);
            const size_t size = k4a_image_get_size(depth_image);
            int width = k4a_image_get_width_pixels(depth_image);
            int height = k4a_image_get_height_pixels(depth_image);
            int stride = k4a_image_get_stride_bytes(depth_image);
            int format = k4a_image_get_format(depth_image);

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

    virtual void save_auxdata_skeleton(cwipc* pc) final {
        int n_skeletons = skeletons.size();
        size_t size_str = sizeof(cwipc_skeleton_collection) + n_skeletons * (int)K4ABT_JOINT_COUNT * sizeof(cwipc_skeleton_joint);
        cwipc_skeleton_collection* skl = (cwipc_skeleton_collection*)malloc(size_str);

        if (skl != NULL) {
            skl->n_skeletons = n_skeletons;
            skl->n_joints = (int)K4ABT_JOINT_COUNT;
            cwipc_skeleton_joint* p = skl->joints;
            for (auto s : skeletons) {
                for (auto j : s.joints) {
                    p->confidence = (int)j.confidence_level;
                    p->x = j.position.xyz.x;
                    p->y = j.position.xyz.y;
                    p->z = j.position.xyz.z;
                    p->q_w = j.orientation.wxyz.w;
                    p->q_x = j.orientation.wxyz.x;
                    p->q_y = j.orientation.wxyz.y;
                    p->q_z = j.orientation.wxyz.z;
                    p++;
                }
            }

            std::string name = "skeleton." + serial;
            cwipc_auxiliary_data* ap = pc->access_auxiliary_data();
            ap->_add(name, "", (void*)skl, size_str, ::free);
        }
    }

    bool map2d3d(int x_2d, int y_2d, int d_2d, float* out3d)
    {
        float depth = d_2d; // 1000.0; // Note: this comes out of the Depth image, so it is in millimeters

        k4a_float2_t* xy_table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);
        int width = k4a_image_get_width_pixels(xy_table);
        int height = k4a_image_get_height_pixels(xy_table);
        if (y_2d < 0 || y_2d >= height || x_2d < 0 || x_2d >= width) {
            _log_error("map2d3d: requested pixel out of bounds");
            return false;
        }
        int idx = y_2d * width + x_2d;
        cwipc_pcl_point point;
        point.x = xy_table_data[idx].xy.x * depth;
        point.y = xy_table_data[idx].xy.y * depth;
        point.z = depth;
        if (configuration.camera_processing.map_color_to_depth) {
            transformDepthToColorPoint(point);
        }
        transformPoint(point); //transforming from camera to world coordinates
        out3d[0] = point.x;
        out3d[1] = point.y;
        out3d[2] = point.z;
        return true;
    }
protected:


    virtual void _start_capture_thread() = 0;
    virtual void _capture_thread_main() = 0;

    virtual void _processing_thread_main() final {
        _log_debug_thread("processing thread started for camera " + serial);
        while (!camera_stopped) {
            k4a_capture_t processing_frameset = NULL;
            k4a_image_t depth_image = NULL;
            k4a_image_t color_image = NULL;

            bool ok = processing_frame_queue.wait_dequeue_timed(processing_frameset, std::chrono::milliseconds(10000));
            if (processing_frameset == NULL) {
            _log_debug_thread("processing thread stopping for camera " + serial);
                continue;
            }

            if (!ok) {
                _log_warning("processing thread dequeue timeout");
                continue;
            }

            _log_debug_thread("processing thread got frameset for camera " + serial);
            assert(processing_frameset);
            std::lock_guard<std::mutex> lock(processing_mutex);

            // use body tracker for skeleton extraction
            if (want_auxdata_skeleton && tracker_handle) {
                //
                // Push frameset into the tracker. Wait indefinitely for the result.
                //
                k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker_handle, processing_frameset, K4A_WAIT_INFINITE);
                if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT) {
                  // It should never hit timeout when K4A_WAIT_INFINITE is set.
                  _log_warning("k4abt_tracker_enqueue_capture: timeout");
                } else if (queue_capture_result == K4A_WAIT_RESULT_FAILED) {
                  _log_warning("k4abt_tracker_enqueue_capture: failed");
                }

                //
                // Now pop the result. Again wait indefinitely.
                //
                k4abt_frame_t body_frame = NULL;
                skeletons.clear();
                k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker_handle, &body_frame, K4A_WAIT_INFINITE);

                if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED) {
                    uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
                    _log_debug_thread("processing thread: detected " + std::to_string(num_bodies) + " bodies");
                    if (num_bodies > 0) {
                        // Transform each 3d joints from 3d depth space to 2d color image space
                        for (uint32_t i = 0; i < num_bodies; i++) {
                            //printf("- Person[%u]:\n", i);
                            k4abt_skeleton_t skeleton;
                            auto sts = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);

                            if (sts != K4A_RESULT_SUCCEEDED) {
                                _log_error("Get body from body frame failed");
                                break;
                            }

                            for (int joint_id = 0; joint_id < (int)K4ABT_JOINT_COUNT; joint_id++) {
                                k4a_float3_t::_xyz pos = skeleton.joints[joint_id].position.xyz; //millimiters
                                cwipc_pcl_point point;
                                point.x = pos.x;
                                point.y = pos.y;
                                point.z = pos.z;

                                if (!configuration.camera_processing.map_color_to_depth) {
                                    transformDepthToColorPoint(point);
                                }

                                transformPoint(point);
                                pos.x = point.x;
                                pos.y = point.y;
                                pos.z = point.z;
                                skeleton.joints[joint_id].position.xyz = pos;
                                //k4abt_joint_confidence_level_t confidence = skeleton.joints[joint_id].confidence_level;
                                //k4a_quaternion_t orientation = skeleton.joints[joint_id].orientation;
                                //std::cout << "\tJoint " << joint_id << " : \t(" << pos.x << "," << pos.y << "," << pos.z << ")\t\tconfidence_level = " << confidence << "\t orientation_wxyz: (" << orientation.wxyz.w << "," << orientation.wxyz.x << "," << orientation.wxyz.y << "," << orientation.wxyz.z << ")" << std::endl;
                            }

                            skeletons.push_back(skeleton);
                        }
                    }
                } else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT) {
                    _log_warning("k4abt_tracker_pop_result: timeout");
                } else {
                    _log_warning("k4abt_tracker_pop_result: failed");
                }

                if (body_frame != nullptr) {
                    k4abt_frame_release(body_frame);
                }
            }

            // get depth and color images. Apply filters and uncompress color image if needed
            depth_image = k4a_capture_get_depth_image(processing_frameset);

            _filter_depth_image(depth_image); //filtering depthmap => better now because if we map depth to color then we need to filter more points.
            color_image = k4a_capture_get_color_image(processing_frameset);
            color_image = _uncompress_color_image(processing_frameset, color_image);

            //generate pointclouds
            cwipc_pcl_pointcloud new_pointcloud = nullptr;
            if (configuration.camera_processing.map_color_to_depth) {
                new_pointcloud = generate_point_cloud_color_to_depth(depth_image, color_image);
            } else {
                new_pointcloud = generate_point_cloud_depth_to_color(depth_image, color_image);
            }

            if (new_pointcloud != nullptr) {
                current_pointcloud = new_pointcloud;
                _log_debug_thread("generated pointcloud with " + std::to_string(current_pointcloud->size()) + " points");

                if (current_pointcloud->size() == 0) {
                    _log_warning("generated pointcloud has zero points");
                    //continue;
                }

                // Notify wait_for_pc that we're done.
                processing_done = true;
                processing_done_cv.notify_one();
            }

            if (depth_image) {
                k4a_image_release(depth_image);
            }

            depth_image = nullptr;
            if (color_image) {
                k4a_image_release(color_image);
            }

            color_image = nullptr;
            if (processing_frameset) {
                k4a_capture_release(processing_frameset);
            }

            processing_frameset = nullptr;
        }

        _log_debug_thread("processing thread exiting");
    }

    virtual void _filter_depth_image(k4a_image_t depth_image) {
        if (configuration.camera_processing.do_threshold) {
            uint16_t* depth_buffer = (uint16_t*)(void*)k4a_image_get_buffer(depth_image);

            int width = k4a_image_get_width_pixels(depth_image);
            int height = k4a_image_get_height_pixels(depth_image);

            int16_t min_depth = (int16_t)(configuration.camera_processing.threshold_near * 1000);
            int16_t max_depth = (int16_t)(configuration.camera_processing.threshold_far * 1000);
            min_depth = (min_depth > 1) ? min_depth : 1; // min depth should be > minimum_operating_range (250-500 for kinect). 0 means no data.
            max_depth = (max_depth > min_depth) ? max_depth : 5460; // max depth should be >min_depth otherwise we use the maximum_operating_range (5460 for kinect).

            // First we create the depth material from the depth_data
            cv::Mat depth_in(height, width, CV_16UC1, depth_buffer);

            // DISTANCE FILTER: we create a mask to filter data using the distance thresholds
            cv::Mat mask = cv::Mat::ones(height, width, CV_8UC1);
            cv::inRange(depth_in, min_depth, max_depth, mask); //we check if the depth values are in the wanted range and create a mask

            // EROSION FILTER: if erosion wanted, we will erode the mask using a kernel.
            int x_delta = configuration.camera_processing.depth_x_erosion;
            int y_delta = configuration.camera_processing.depth_y_erosion;
            if (x_delta || y_delta) {
                cv::Mat kernel = cv::getStructuringElement(CV_16UC1, cv::Size(x_delta*2+1, y_delta*2+1), cv::Point(-1, -1)); //we want erosion on 4 directions. +-x, +-y.
                cv::erode(mask, mask, kernel, cv::Point(-1, -1), 1);
            }

            //APPLYING THE MASK:
            cv::Mat depth_out;
            depth_in.copyTo(depth_out, mask); //we apply the mask

            //copy filtered_depthmap data back to initial depth_buffer
            memcpy(depth_buffer, depth_out.data, depth_out.step * depth_out.rows);
        }
    }

    virtual void _filter_depth_data(int16_t* depth_values, int width, int height) final {
        int16_t min_depth = (int16_t)(configuration.camera_processing.threshold_near * 1000);
        int16_t max_depth = (int16_t)(configuration.camera_processing.threshold_far * 1000);
        int16_t *z_values = (int16_t *)calloc(width * height, sizeof(int16_t));

        // Pass one: Copy Z values to temporary buffer, but leave out-of-range values at zero.
        for (int i = 0; i < width * height; i++) {
            int i_pc = i * 3;
            int16_t z = depth_values[i_pc + 2];

            if (configuration.camera_processing.do_threshold && (z <= min_depth || z >= max_depth)) {
                continue;
            }

            z_values[i] = z;
        }

        /*cv::Mat depth_in(height, width, CV_16UC1, z_values);
        cv::imwrite("test/depth_th.png", depth_in);*/

        // Pass two: loop for zero pixels in temp buffer, and clear out x/y pixels adjacent in depth buffer
        int x_delta = configuration.camera_processing.depth_x_erosion;
        int y_delta = configuration.camera_processing.depth_y_erosion;

        if (x_delta || y_delta) {
            for (int x = 0; x < width; x++) {
                for (int y = 0; y < height; y++) {
                    if (z_values[x + y * width] != 0) {
                        continue;
                    }

                    // Zero depth at (x, y). Clear out pixels 
                    for (int ix = x - x_delta; ix <= x + x_delta; ix++) {
                        if (ix < 0 || ix >= width) {
                            continue;
                        }

                        int i_pc = (ix + y * width) * 3;
                        depth_values[i_pc + 2] = 0;
                    }

                    for (int iy = y - y_delta; iy <= y + y_delta; iy++) {
                        if (iy < 0 || iy >= height) {
                            continue;
                        }

                        int i_pc = (x + iy * width) * 3;
                        depth_values[i_pc + 2] = 0;
                    }
                }
            }
        } else {
          // Pass three: clear out zero pixels from temporary buffer.
            for (int i = 0; i < width * height; i++) {
                if (z_values[i] != 0) {
                    continue;
                }

                int i_pc = i * 3;
                depth_values[i_pc + 2] = 0;
            }
        }

        free(z_values);
    }

    //virtual void _computePointSize() = 0;
    virtual cwipc_pcl_pointcloud generate_point_cloud_color_to_depth(const k4a_image_t depth_image, const k4a_image_t color_image) final {
        int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
        int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
        k4a_image_t transformed_color_image = NULL;
        k4a_result_t status;

        status = k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
            depth_image_width_pixels,
            depth_image_height_pixels,
            depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
            &transformed_color_image
        );

        if (status != K4A_RESULT_SUCCEEDED) {
          _log_error("Failed to create transformed color image: " + std::to_string(status));
          return nullptr;
        }

        status = k4a_transformation_color_image_to_depth_camera(transformation_handle,
            depth_image,
            color_image,
            transformed_color_image
        );

        if (status != K4A_RESULT_SUCCEEDED) {
            _log_error("Failed to compute transformed color image: " + std::to_string(status));
            return nullptr;
        }

        cwipc_pcl_pointcloud rv;


        rv = generate_point_cloud_v2(depth_image, transformed_color_image);

        k4a_image_release(transformed_color_image);

        return rv;
    }

    virtual cwipc_pcl_pointcloud generate_point_cloud_depth_to_color(const k4a_image_t depth_image, const k4a_image_t color_image) final {
        // transform color image into depth camera geometry
        int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
        int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
        k4a_image_t transformed_depth_image = NULL;
        k4a_result_t status;

        status = k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
            color_image_width_pixels,
            color_image_height_pixels,
            color_image_width_pixels * (int)sizeof(uint16_t),
            &transformed_depth_image
        );

        if (status != K4A_RESULT_SUCCEEDED) {
            _log_error("Failed to create transformed depth image: " + std::to_string(status));
            return nullptr;
        }

        status = k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image);
        if (status != K4A_RESULT_SUCCEEDED) {
            _log_error("Failed to compute transformed depth image: " + std::to_string(status));
            return nullptr;
        }

        cwipc_pcl_pointcloud rv;

        rv = generate_point_cloud_v2(transformed_depth_image, color_image);

        k4a_image_release(transformed_depth_image);

        return rv;
    }

    virtual cwipc_pcl_pointcloud generate_point_cloud(const k4a_image_t point_cloud_image, const k4a_image_t color_image) final {
        int width = k4a_image_get_width_pixels(point_cloud_image);
        int height = k4a_image_get_height_pixels(color_image);

        uint8_t * color_data = k4a_image_get_buffer(color_image);
        int16_t* point_cloud_image_data = (int16_t*)k4a_image_get_buffer(point_cloud_image);

        // now loop over images and create points.
        cwipc_pcl_pointcloud new_cloud = new_cwipc_pcl_pointcloud();
        new_cloud->clear();
        new_cloud->reserve(width * height);

        for (int i = 0; i < width * height; i++) {
            int i_pc = i * 3;
            int i_rgba = i * 4;
            cwipc_pcl_point point;
            int16_t x = point_cloud_image_data[i_pc + 0];
            int16_t y = point_cloud_image_data[i_pc + 1];
            int16_t z = point_cloud_image_data[i_pc + 2];

            if (z == 0) {
                continue;
            }

            // color_data is BGR
            point.r = color_data[i_rgba + 2];
            point.g = color_data[i_rgba + 1];
            point.b = color_data[i_rgba + 0];

            if (configuration.single_tile >= 0) {
                point.a = configuration.single_tile;
            } else {
                point.a = (uint8_t)1 << camera_index;
            }
            uint8_t alpha = color_data[i_rgba + 3];

            if (point.r == 0 && point.g == 0 && point.b == 0 && alpha == 0) {
                continue;
            }

            point.x = x;
            point.y = y;
            point.z = z;

            if (configuration.camera_processing.map_color_to_depth) {
              transformDepthToColorPoint(point);
            }

            transformPoint(point);

            if (configuration.radius_filter > 0.0) { // apply radius filter
                if(!isPointInRadius(point, configuration.radius_filter)) {
                    continue;
                }
            }

            if (do_height_filtering && (point.y < configuration.height_min || point.y > configuration.height_max)) {
                continue;
            }

            if (!configuration.greenscreen_removal || isNotGreen(&point)) {// chromakey removal
                new_cloud->push_back(point);
            }
        }
        _log_debug_thread("produced " + std::to_string(new_cloud->size()) + " points");
        return new_cloud;
    }

    virtual void create_xy_table(const k4a_calibration_t* calibration) {
        int width;
        int height;

        if (configuration.camera_processing.map_color_to_depth) {
            width = calibration->depth_camera_calibration.resolution_width;
            height = calibration->depth_camera_calibration.resolution_height;
        } else {
            width = calibration->color_camera_calibration.resolution_width;
            height = calibration->color_camera_calibration.resolution_height;
        }

        k4a_result_t status;
        status = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
            width,
            height,
            width * (int)sizeof(k4a_float2_t),
            &xy_table
        );

        if (status != K4A_RESULT_SUCCEEDED) {
            _log_error("Failed to create XY table image: " + std::to_string(status));
            return;
        }

        k4a_float2_t* table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);

        k4a_float2_t p;
        k4a_float3_t ray;
        int valid;

        for (int y = 0, idx = 0; y < height; y++) {
            p.xy.y = (float)y;

            for (int x = 0; x < width; x++, idx++) {
                p.xy.x = (float)x;

                if (configuration.camera_processing.map_color_to_depth) {
                    k4a_calibration_2d_to_3d(
                        calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid
                    );
                } else {
                    k4a_calibration_2d_to_3d(
                        calibration, &p, 1.f, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ray, &valid
                    );
                }

                if (valid) {
                    table_data[idx].xy.x = ray.xyz.x;
                    table_data[idx].xy.y = ray.xyz.y;
                } else {
                    table_data[idx].xy.x = nanf("");
                    table_data[idx].xy.y = nanf("");
                }
            }
        }
    }

    virtual cwipc_pcl_pointcloud generate_point_cloud_v2(const k4a_image_t depth_image, k4a_image_t color_image) {
        int width = k4a_image_get_width_pixels(depth_image);
        int height = k4a_image_get_height_pixels(depth_image);

        float min_depth = (configuration.camera_processing.threshold_near);
        float max_depth = (configuration.camera_processing.threshold_far);

        //access depth and color data
        uint16_t* depth_data = (uint16_t*)(void*)k4a_image_get_buffer(depth_image);
        k4a_float2_t* xy_table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);
        cwi_bgra_t* color_data = (cwi_bgra_t*)(void*)k4a_image_get_buffer(color_image);

        cwipc_pcl_pointcloud new_cloud = new_cwipc_pcl_pointcloud();
        new_cloud->clear();
        new_cloud->reserve(width * height);

        for (int i = 0; i < width * height; i++) {
            if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y)) {
                cwipc_pcl_point point;
                point.x = xy_table_data[i].xy.x * (float)depth_data[i];
                point.y = xy_table_data[i].xy.y * (float)depth_data[i];
                point.z = (float)depth_data[i];
                point.b = color_data[i].bgra.b;
                point.g = color_data[i].bgra.g;
                point.r = color_data[i].bgra.r;
                point.a = (uint8_t)1 << camera_index;

                if (configuration.camera_processing.map_color_to_depth) {
                    transformDepthToColorPoint(point);
                }
                transformPoint(point); //transforming from camera to world coordinates

                if (do_height_filtering && (point.y < configuration.height_min || point.y > configuration.height_max)) { //height filtering
                    continue;
                }

                if (configuration.radius_filter > 0.0) { // apply radius filter
                    if (!isPointInRadius(point, configuration.radius_filter)) {
                        continue;
                    }
                }

                if (configuration.greenscreen_removal && !isNotGreen(&point)) { //chromakey removal
                    continue;
                }

                //point passed all filters, so we add this to the pointcloud
                new_cloud->push_back(point);
            }
        }
        _log_debug_thread("produced " + std::to_string(new_cloud->size()) + " points");
        return new_cloud;
    }

    virtual void transformPoint(cwipc_pcl_point& pt) final {
        float x = pt.x / 1000.0;
        float y = pt.y / 1000.0;
        float z = pt.z / 1000.0;
        pt.x = (*camera_config.trafo)(0,0)*x + (*camera_config.trafo)(0,1)*y + (*camera_config.trafo)(0,2)*z + (*camera_config.trafo)(0,3);
        pt.y = (*camera_config.trafo)(1,0)*x + (*camera_config.trafo)(1,1)*y + (*camera_config.trafo)(1,2)*z + (*camera_config.trafo)(1,3);
        pt.z = (*camera_config.trafo)(2,0)*x + (*camera_config.trafo)(2,1)*y + (*camera_config.trafo)(2,2)*z + (*camera_config.trafo)(2,3);
    }

    virtual void transformDepthToColorPoint(cwipc_pcl_point& pt) final {
        float x = pt.x;
        float y = pt.y;
        float z = pt.z;
        float *rotation = depth_to_color_extrinsics.rotation;
        float *translation = depth_to_color_extrinsics.translation;
        pt.x = rotation[0] * x + rotation[1] * y + rotation[2] * z + translation[0];
        pt.y = rotation[3] * x + rotation[4] * y + rotation[5] * z + translation[1];
        pt.z = rotation[6] * x + rotation[7] * y + rotation[8] * z + translation[2];
    }
    /// Optionally uncompress color image.
    /// For code simplicity this always returns a value k4a_image_t, even though it may be wrong.
    virtual k4a_image_t _uncompress_color_image(k4a_capture_t capture, k4a_image_t color_image) = 0;
public:
    float pointSize = 0;  //<! (Approximate) 3D cellsize of pointclouds captured by this camera
    std::string serial; //<! Serial number for this camera
    bool eof = false; //<! True when end of file reached on this camera stream
    int camera_index;

protected:
    K4ACaptureConfig& configuration;
    Type_api_camera camera_handle;
    bool camera_stopped = true;  //<! True when stopping
    bool camera_started = false;  //<! True when camera hardware is grabbing
    std::thread* camera_processing_thread = nullptr; //<! Handle for thread that runs processing loop
    std::thread* camera_capturer_thread = nullptr;  //<! Handle for thread that rungs grabber (if applicable)
    K4ACameraConfig& camera_config; //<! Per-camera data for this camera
    bool want_auxdata_skeleton = false; //<! True if caller wants skeleton auxdata
    std::vector<k4abt_skeleton_t> skeletons; //<! Skeletons extracted using the body tracking sdk
    cwipc_pcl_pointcloud current_pointcloud = nullptr;  //<! Most recent grabbed pointcloud
    k4a_transformation_t transformation_handle = nullptr; //<! k4a structure describing relationship between RGB and D cameras
    moodycamel::BlockingReaderWriterQueue<k4a_capture_t> captured_frame_queue;  //<! Frames from capture-thread, waiting to be inter-camera synchronized
    moodycamel::BlockingReaderWriterQueue<k4a_capture_t> processing_frame_queue;  //<! Synchronized frames, waiting for processing thread
    k4a_capture_t current_frameset = nullptr; //<! Current frame being moved from captured_frame_queue to processing_frame_queue
    bool camera_sync_ismaster;  //<! Parameter from camData xxxjack needs to go
    bool camera_sync_inuse; //<! Parameter from camData xxxjack needs to go
    bool do_height_filtering; //<! Parameter from camData xxxjack needs to go
    std::mutex processing_mutex;  //<! Exclusive lock for frame to pointcloud processing.
    std::condition_variable processing_done_cv; //<! Condition variable signalling pointcloud ready
    bool processing_done = false; //<! Boolean for processing_done_cv

    k4abt_tracker_t tracker_handle = nullptr; //<! Handle to k4abt skeleton tracker
    k4a_calibration_t sensor_calibration; //<! k4a calibration data read from hardware camera or recording
    k4a_calibration_extrinsics_t depth_to_color_extrinsics; //<! k4a calibration data read from hardware camera or recording
    k4a_image_t xy_table = NULL;
    std::string record_to_file; //<! If non-empty: file to record the captured streams to.
};
