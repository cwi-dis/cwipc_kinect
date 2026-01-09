#pragma once

#include <string>
#include <mutex>
#include <condition_variable>

#include <k4a/k4a.h>
#include <k4abt.h>

#include "cwipc_util/capturers.hpp"

#include "K4AConfig.hpp"

/** Base class for capturers that are implemented with the K4A API
 * 
 * Type_api_camera: The type representing a single camera in the K4A API (k4a_device_t or k4a_playback_t)
 * Type_our_camera: The type representing our wrapper around a single K4A camera (subclass of K4ABaseCamera)
*/
template<typename Type_api_camera, class Type_our_camera> 
class K4ABaseCapture : public CwipcBaseCapture {
public:
    //
    // Public API methods (mainly overrides from CwipcBaseCapture)
    //
    /// Subclasses need to implement static factory().
    /// Subclasses need to implement static count_devices().

    using CwipcBaseCapture::CwipcBaseCapture;
    virtual ~K4ABaseCapture() {
        uint64_t stopTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        _unload_cameras();
    }

    virtual int get_camera_count() override final { 
        return cameras.size(); 
    }

    virtual bool is_valid() override final { 
        return cameras.size() > 0; 
    }
    

    virtual bool config_reload_and_start_capturing(const char* configFilename) override final{
        _unload_cameras();

        //
        // Read the configuration.
        //
        if (!_apply_config(configFilename)) {
            return false;
        }

        auto camera_config_count = configuration.all_camera_configs.size();
        if (camera_config_count == 0) {
            return false;
        }

        //
        // Initialize hardware capture setting (for all cameras)
        //
        if (!_init_hardware_for_all_cameras()) {
            // xxxjack we should really close all cameras too...
            camera_config_count = 0;
            return false;
        }

        _setup_inter_camera_sync();

        // Now we have all the configuration information. Create our K4ACamera objects.
        if (!_create_cameras()) {
            _unload_cameras();
            return false;
        }

        if (!_check_cameras_connected()) {
            _unload_cameras();
            return false;
        }
        _start_cameras();

        //
        // start our run thread (which will drive the capturers and merge the pointclouds)
        //
        stopped = false;
        control_thread = new std::thread(&K4ABaseCapture::_control_thread_main, this);
        _cwipc_setThreadName(control_thread, L"cwipc_kinect::K4ABaseCapture::control_thread");

        return true;
    }

    virtual std::string config_get() {
        return configuration.to_string();
    }

    virtual void request_auxiliary_data(bool rgb, bool depth, bool timestamps, bool skeleton) override final {
        configuration.auxData.want_auxdata_rgb = rgb;
        configuration.auxData.want_auxdata_depth = depth;
        configuration.auxData.want_auxdata_skeleton = skeleton;
    }


    virtual bool pointcloud_available(bool wait) override final {
        if (!is_valid()) {
            return false;
        }

        _request_new_pointcloud();
        std::this_thread::yield();
        std::unique_lock<std::mutex> mylock(mergedPC_mutex);
        auto duration = std::chrono::seconds(wait ? 1 : 0);

        mergedPC_is_fresh_cv.wait_for(mylock, duration, [this] {
            return mergedPC_is_fresh;
        });

        return mergedPC_is_fresh;
    }

    virtual cwipc* get_pointcloud() override final {
        if (!is_valid()) {
          _log_warning("get_pointcloud: returning NULL, no cameras");
          return nullptr;
        }

        _request_new_pointcloud();
        // Wait for a fresh mergedPC to become available.
        // Note we keep the return value while holding the lock, so we can start the next grab/process/merge cycle before returning.
        cwipc* rv;

        {
            std::unique_lock<std::mutex> mylock(mergedPC_mutex);
            mergedPC_is_fresh_cv.wait(mylock, [this] {
                return mergedPC_is_fresh;
            });

            assert(mergedPC_is_fresh);
            mergedPC_is_fresh = false;
            rv = mergedPC;
            mergedPC = nullptr;

            if (rv == nullptr) {
              _log_warning("get_pointcloud: returning NULL, even though mergedPC_is_fresh");
            }
        }

        _request_new_pointcloud();
        return rv;
    }

    virtual float get_pointSize() override final {
        if (!is_valid()) {
            return 0;
        }

        float rv = 99999;

        for (auto cam : cameras) {
            if (cam->pointSize < rv) {
                rv = cam->pointSize;
            }
        }

        if (rv > 9999) {
            rv = 0;
        }

        return rv;
    }

    virtual bool map2d3d(int tile, int x_2d, int y_2d, int d_2d, float* out3d) override final
    {
        for (auto cam : cameras) {
            if (tile == (1 << cam->camera_index)) {
                return cam->map2d3d(x_2d, y_2d, d_2d, out3d);
            }
        }
        return false;
    }
    virtual bool mapcolordepth(int tile, int u, int v, int* out2d) override final
    {
        // For Kinect the RGB and D images have the same coordinate system.
        out2d[0] = u;
        out2d[1] = v;
        return true;
    }

    bool eof() override {
        return _eof;
    }
    virtual bool seek(uint64_t timestamp) override = 0;

protected:
    /// Load configuration from file or string.
    virtual bool _apply_config(const char *configFilename) override final {
        K4ACaptureConfig newConfig;
        newConfig.auxData = configuration.auxData; // preserve auxdata requests
        configuration = newConfig;
        if (configFilename == NULL || *configFilename == '\0') {
            configFilename = "cameraconfig.json";
        }
        configurationCurrentFilename = configFilename;

        if (strcmp(configFilename, "auto") == 0) {
            // Special case 1: string "auto" means auto-configure all cameras.
            configurationCurrentFilename = "";
            return _apply_auto_config();
        }

        if (configFilename[0] == '{') {
            // Special case 2: a string starting with { is considered a JSON literal
            configurationCurrentFilename = "";
            return configuration.from_string(configFilename, type);
        }

        // Otherwise we check the extension. It can be .json.
        const char *extension = strrchr(configFilename, '.');
        if (extension != nullptr && strcmp(extension, ".json") == 0) {
            return configuration.from_file(configFilename, type);
        }

        return false;
    }
    /// Load default configuration based on hardware cameras connected.
    virtual bool _apply_auto_config() override = 0;
    /// Get configuration for a single camera, by serial number.
    virtual K4ACameraConfig* get_camera_config(std::string serial) final {
        for (int i = 0; i < configuration.all_camera_configs.size(); i++) {
            if (configuration.all_camera_configs[i].serial == serial) {
                return &configuration.all_camera_configs[i];
            }
        }

        _log_warning("Unknown camera " + serial);
        return nullptr;
    }
    /// Create our wrapper around a single camera. Here because it needs to be templated.
    virtual inline Type_our_camera *_create_single_camera(Type_api_camera _handle, K4ACaptureConfig& configuration, int _camera_index, K4ACameraConfig& _camData) final {
        return new Type_our_camera(_handle, configuration, _camera_index, _camData);
    }


    /// Setup camera synchronization (if needed).
    virtual bool _setup_inter_camera_sync() override final {
        // Nothing to do for K4A: real cameras need some setup, but it is done
        // in K4ACamera::_prepare_config_for_starting_camera().
        return true;
    }
    /// xxxjack another one?
    virtual void _initial_camera_synchronization() {
    }

    /// Create the per-camera capturers.
    virtual bool _create_cameras() override = 0;
    /// Setup camera hardware parameters (white balance, etc).
    virtual bool _init_hardware_for_all_cameras() override = 0;
    /// Check that all cameras are connected.
    virtual bool _check_cameras_connected() override = 0;
    /// Start all cameras.
    virtual void _start_cameras() override final {
        //
        // start the cameras. First start all non-sync-master cameras, then start the sync-master camera.
        //
        bool start_error = false;
        for (auto cam: cameras) {
            if (!cam->pre_start_all_cameras()) {
                start_error = true;
            }
        }
        // start the cameras in a specific order: first all non-masters, then all masters.
        for (auto cam : cameras) {
            if (cam->is_sync_master()) {
                continue;
            }

            if (!cam->start_camera()) {
                start_error = true;
            }
        }

        for (auto cam : cameras) {
            if (!cam->is_sync_master()) {
                continue;
            }

            if (!cam->start_camera()) {
                start_error = true;
            }
        }
        if (start_error) {
            _log_error("Not all cameras could be started");
            _unload_cameras();
            return;
        }
        for (auto cam: cameras) {
            cam->post_start_all_cameras();
        }

        //
        // start the per-camera capture threads. Master camera has to be started latest
        //
        for (auto cam : cameras) {
            if (cam->is_sync_master()) {
                continue;
            }

            cam->start_camera_streaming();
        }

        for (auto cam : cameras) {
            if (!cam->is_sync_master()) {
                continue;
            }

            cam->start_camera_streaming();
        }
        
        for (auto cam: cameras) {
            cam->post_start_all_cameras();
        }
    }

    /// Stop and unload all cameras and release all resources.
    virtual void _unload_cameras() override final {
        _stop_cameras();

        // Delete all cameras
        for (auto cam : cameras)
          delete cam;
        cameras.clear();
        _log_debug("deleted all cameras");
    }

    /// Stop all cameras.
    virtual void _stop_cameras() override final {
        stopped = true;
        mergedPC_is_fresh = true;
        mergedPC_want_new = false;
        mergedPC_is_fresh_cv.notify_all();
        mergedPC_want_new = true;
        mergedPC_want_new_cv.notify_all();

        _log_debug("stopping all cameras");

        if (control_thread && control_thread->joinable()) {
            control_thread->join();
        }

        delete control_thread;
        control_thread = nullptr;

        // Stop all cameras
        for (auto cam : cameras) {
            cam->pre_stop_camera();
        }
        for (auto cam : cameras) {
            cam->stop_camera();
        }

        mergedPC_is_fresh = false;
        mergedPC_want_new = false;
        _log_debug("stopped all cameras");
    }


    virtual Type_our_camera* get_camera(std::string serial) final {
        for (auto cam : cameras) {
            if (cam->serial == serial) {
                return cam;
            }
        }

        return NULL;
    }

protected:
    
    /// Anything that needs to be done to get the camera streams synchronized after opening.
    /// (Realsense Playback seeks all streams to the same timecode, the earliest one present
    /// in each stream)


    virtual void _control_thread_main() final {
        _log_debug_thread("processing thread started");
        _initial_camera_synchronization();
        while (!stopped) {
            {
                std::unique_lock<std::mutex> mylock(mergedPC_mutex);
                mergedPC_want_new_cv.wait(mylock, [this] { 
                    return mergedPC_want_new; 
                });
            }

            //check EOF:
            for (auto cam : cameras) {
                if (cam->end_of_stream_reached) {
                    _eof = true;
                    stopped = true;
                    break;
                }
            }

            if (stopped) {
                break;
            }

            assert(cameras.size() > 0);
            // Step one: grab frames from all cameras. This should happen as close together in time as possible,
            // because that gives use he biggest chance we have the same frame (or at most off-by-one) for each
            // camera.
            uint64_t timestamp = 0;
            bool all_captures_ok = _capture_all_cameras(timestamp);

            if (!all_captures_ok) {
                std::this_thread::yield();
                continue;
            }

            if (stopped) {
                break;
            }

            // And get the best timestamp
            if (configuration.new_timestamps) {
                timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            }

            // Step 2 - Create pointcloud, and save rgb/depth images if wanted
            cwipc_pcl_pointcloud pcl_pointcloud = new_cwipc_pcl_pointcloud();
            char* error_str = NULL;
            cwipc* newPC = cwipc_from_pcl(pcl_pointcloud, timestamp, &error_str, CWIPC_API_VERSION);

            if (newPC == nullptr) {
                _log_debug("Failed to create cwipc from pcl pointcloud: " + std::string(error_str ? error_str : "unknown error"));
                break;
            }

            if (configuration.auxData.want_auxdata_rgb || configuration.auxData.want_auxdata_depth) {
                for (auto cam : cameras) {
                    cam->save_auxdata_images(newPC, configuration.auxData.want_auxdata_rgb, configuration.auxData.want_auxdata_depth);
                }
            }

            if (configuration.auxData.want_auxdata_skeleton) {
                for (auto cam : cameras) {
                    cam->save_auxdata_skeleton(newPC);
                }
            }

            if (stopped) {
                break;
            }

            // Step 3: start processing frames to pointclouds, for each camera
            for (auto cam : cameras) {
                cam->create_pc_from_frames();
            }

            if (stopped) {
                break;
            }

            // Lock mergedPC already while we are waiting for the per-camera
            // processing threads. This so the main thread doesn't go off and do
            // useless things if it is calling available(true).
            std::unique_lock<std::mutex> mylock(mergedPC_mutex);
            if (mergedPC && mergedPC_is_fresh) {
                mergedPC->free();
                mergedPC = nullptr;
            }

            if (stopped) {
                break;
            }

            mergedPC = newPC;

            // Step 4: wait for frame processing to complete.
            for (auto cam : cameras) {
                cam->wait_for_pc();
            }

            if (stopped) {
                break;
            }

            // Step 5: merge views
            merge_views();

            if (mergedPC->access_pcl_pointcloud()->size() > 0) {
                _log_debug("merged pointcloud has " + std::to_string(mergedPC->access_pcl_pointcloud()->size()) + " points");
            } else {
                _log_warning("merged pointcloud is empty");
            }
            // Signal that a new mergedPC is available. (Note that we acquired the mutex earlier)
            mergedPC_is_fresh = true;
            mergedPC_want_new = false;
            mergedPC_is_fresh_cv.notify_all();
        }

        _log_debug_thread("processing thread stopped");
    }

    virtual bool _capture_all_cameras(uint64_t& timestamp) = 0;

    virtual void merge_views() final {
        cwipc_pcl_pointcloud aligned_cld(mergedPC->access_pcl_pointcloud());
        aligned_cld->clear();

        // Pre-allocate space in the merged pointcloud
        size_t nPoints = 0;
        for (auto cam : cameras) {
            cwipc_pcl_pointcloud cam_cld = cam->get_current_pointcloud();
            if (cam_cld == nullptr) {
                _log_warning("Returned NULL cloud, ignoring");
                continue;
            }

            nPoints += cam_cld->size();
        }

        aligned_cld->reserve(nPoints);

        // Now merge all pointclouds
        for (auto cam : cameras) {
            cwipc_pcl_pointcloud cam_cld = cam->get_current_pointcloud();

            if (cam_cld == nullptr) {
                continue;
            }

            *aligned_cld += *cam_cld;
        }

        if (aligned_cld->size() != nPoints) {
            _log_error("Combined pointcloud has different number of points than expected");
        }
    }

    virtual void _request_new_pointcloud() final {
        std::unique_lock<std::mutex> mylock(mergedPC_mutex);

        if (!mergedPC_want_new && !mergedPC_is_fresh) {
            mergedPC_want_new = true;
            mergedPC_want_new_cv.notify_all();
        }
    }

public:
    // public attributes. Mainly public so they can be accessed
    // from the Camera class.
    K4ACaptureConfig configuration;  //!< Configuration of this capturer
    std::string configurationCurrentFilename;  //!< Configuration filename

protected:
    // protected attibutes. Accessible from subclasses.
    std::vector<Type_our_camera*> cameras;  //<! Cameras used by this capturer

    bool stopped = false; //<! True when stopping capture
    bool _eof = false; //<! True when end-of-file seen on pointcloud source
    
    cwipc* mergedPC = nullptr;  //<! Merged pointcloud from all cameras
    std::mutex mergedPC_mutex;  //<! Lock for all mergedPC-related data structures

    bool mergedPC_is_fresh = false; //<! True if mergedPC contains a freshly-created pointcloud
    std::condition_variable mergedPC_is_fresh_cv; //<! Condition variable for signalling freshly-created pointcloud

    bool mergedPC_want_new = false; //<! Set to true to request a new pointcloud
    std::condition_variable mergedPC_want_new_cv; //<! Condition variable for signalling we want a new pointcloud

    std::thread* control_thread = nullptr;
};
