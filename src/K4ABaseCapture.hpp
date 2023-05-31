#ifndef cwipc_realsense_K4ABaseCapture_hpp
#define cwipc_realsense_K4ABaseCapture_hpp
#pragma once

#include <string>
#include <mutex>
#include <condition_variable>

#include <k4a/k4a.h>
#include <k4abt.h>

#include "K4AConfig.hpp"

template<typename Type_api_camera, class Type_our_camera>
class K4ABaseCapture {
public:
	K4ACaptureConfig configuration;	//!< Complete configuration read from cameraconfig.xml
	int camera_count = 0;
	bool eof = false;	//<! True when end-of-file seen on pointcloud source

protected:
	std::string CLASSNAME;	//!< For error, warning and debug messages only
	std::vector<Type_our_camera*> cameras;	//<! Cameras used by this capturer

	bool want_auxdata_rgb = false;	//<! True after caller requests this auxiliary data
	bool want_auxdata_depth = false;	//<! True after caller requests this auxiliary data
	bool want_auxdata_skeleton = false;	//<! True after caller requests this auxiliary data
	bool stopped = false;	//<! True when stopping capture

	uint64_t starttime = 0;	//!< Used only for statistics messages
	int numberOfPCsProduced = 0;	//!< Used only for statistics messages
	
	cwipc* mergedPC = nullptr;	//<! Merged pointcloud from all cameras
	std::mutex mergedPC_mutex;	//<! Lock for all mergedPC-related data structures
	bool mergedPC_is_fresh = false;	//<! True if mergedPC contains a freshly-created pointcloud
	std::condition_variable mergedPC_is_fresh_cv;	//<! Condition variable for signalling freshly-created pointcloud
	bool mergedPC_want_new = false;	//<! Set to true to request a new pointcloud
	std::condition_variable mergedPC_want_new_cv;	//<! Condition variable for signalling we want a new pointcloud

	std::thread* control_thread = nullptr;
	
public:
	K4ABaseCapture(const std::string& _Classname)
	:	CLASSNAME(_Classname)
	{

	}
	
	virtual bool config_reload(const char* configFilename) = 0;

	virtual std::string config_get() {
		return cwipc_k4a_config2string(&configuration);
	}

	virtual ~K4ABaseCapture() {
		if (camera_count == 0) {
			return;
		}
		uint64_t stopTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		
		_unload_cameras();
		

		// Print some minimal statistics of this run
		float deltaT = (stopTime - starttime) / 1000.0;
		std::cerr << CLASSNAME << ": ran for " << deltaT << " seconds, produced " << numberOfPCsProduced << " pointclouds at " << numberOfPCsProduced / deltaT << " fps." << std::endl;
	}

	void _unload_cameras() {
		if (camera_count == 0) return;
		stop();

		// Delete all cameras (which will stop their threads as well)
		for (auto cam : cameras)
			delete cam;
		cameras.clear();
		std::cerr << CLASSNAME << ": deleted all cameras\n";
		camera_count = 0;
	}

	virtual void stop() final {
		stopped = true;
		mergedPC_is_fresh = true;
		mergedPC_want_new = false;
		mergedPC_is_fresh_cv.notify_all();
		mergedPC_want_new = true;
		mergedPC_want_new_cv.notify_all();
		std::cerr << CLASSNAME << ": stopped all cameras\n";
		if (control_thread) control_thread->join();
		delete control_thread;
		// Stop all cameras
		for (auto cam : cameras)
			cam->stop();
	}
	
	virtual void request_image_auxdata(bool _rgb, bool _depth) final {
		want_auxdata_rgb = _rgb;
		want_auxdata_depth = _depth;
	}
	
	virtual void request_skeleton_auxdata(bool _skl) final {
		want_auxdata_skeleton = _skl;
		for (auto cam : cameras) {
			cam->request_skeleton_auxdata(_skl);
		}
	}

	virtual Type_our_camera* get_camera(std::string serial) final {
		for (auto cam : cameras)
			if (cam->serial == serial)
				return cam;
		return NULL;
	}

	virtual float get_pointSize() final {
		if (camera_count == 0) return 0;
		float rv = 99999;
		for (auto cam : cameras) {
			if (cam->pointSize < rv) rv = cam->pointSize;
		}
		if (rv > 9999) rv = 0;
		return rv;
	}
	
	virtual bool pointcloud_available(bool wait) final {
		if (camera_count == 0) return false;
		_request_new_pointcloud();
		std::this_thread::yield();
		std::unique_lock<std::mutex> mylock(mergedPC_mutex);
		auto duration = std::chrono::seconds(wait ? 1 : 0);
		mergedPC_is_fresh_cv.wait_for(mylock, duration, [this] {return mergedPC_is_fresh; });
		return mergedPC_is_fresh;
	}
	
	virtual cwipc* get_pointcloud() final {
		if (camera_count == 0) {
			cwipc_k4a_log_warning("get_pointcloud: returning NULL, no cameras");
			return nullptr;
		}
		_request_new_pointcloud();
		// Wait for a fresh mergedPC to become available.
		// Note we keep the return value while holding the lock, so we can start the next grab/process/merge cycle before returning.
		cwipc* rv;
		{
			std::unique_lock<std::mutex> mylock(mergedPC_mutex);
			mergedPC_is_fresh_cv.wait(mylock, [this] {return mergedPC_is_fresh; });
			assert(mergedPC_is_fresh);
			mergedPC_is_fresh = false;
			numberOfPCsProduced++;
			rv = mergedPC;
			mergedPC = nullptr;
			if (rv == nullptr) {
				cwipc_k4a_log_warning("get_pointcloud: returning NULL, even though mergedPC_is_fresh");
			}
		}
		_request_new_pointcloud();
		return rv;
	}

	virtual bool seek(uint64_t timestamp) = 0;
	
	virtual cwipc* get_mostRecentPointCloud() final {
		if (camera_count == 0) return nullptr;
		// This call cdoesn't need a fresh pointcloud (Jack thinks), but it does need one that is
		// consistent. So we lock, but don't wait on the condition.
		std::unique_lock<std::mutex> mylock(mergedPC_mutex);
		return mergedPC;
	}
	
	virtual K4ACameraConfig* get_camera_config(std::string serial) final {
		for (int i = 0; i < configuration.all_camera_configs.size(); i++)
			if (configuration.all_camera_configs[i].serial == serial)
				return &configuration.all_camera_configs[i];
		cwipc_k4a_log_warning("Unknown camera " + serial);
		return nullptr;
	}

protected:
	virtual bool _apply_default_config() = 0;
	virtual bool _apply_config(const char *configFilename) final {
		if (configFilename == NULL) {
			configFilename = "cameraconfig.xml";
		}
		if (strcmp(configFilename, "auto") == 0) {
			// Special case 1: string "auto" means auto-configure all realsense cameras.
			return _apply_default_config();
		}
		if (configFilename[0] == '{') {
			// Special case 2: a string starting with { is considered a JSON literal
			return cwipc_k4a_jsonbuffer2config(configFilename, &configuration);
		}
		// Otherwise we check the extension. It can be .xml or .json.
		const char* extension = strrchr(configFilename, '.');
		if (strcmp(extension, ".xml") == 0) {
			return cwipc_k4a_xmlfile2config(configFilename, &configuration);
		}
		if (strcmp(extension, ".json") == 0) {
			return cwipc_k4a_jsonfile2config(configFilename, &configuration);
		}
		return false;
	}
	
	virtual void _init_camera_positions() final {
		// find camerapositions
		for (int i = 0; i < configuration.all_camera_configs.size(); i++) {
			cwipc_pcl_pointcloud pcptr(new_cwipc_pcl_pointcloud());
			cwipc_pcl_point pt;
			pt.x = 0;
			pt.y = 0;
			pt.z = 0;
			pcptr->push_back(pt);
			transformPointCloud(*pcptr, *pcptr, *configuration.all_camera_configs[i].trafo);
			cwipc_pcl_point pnt = pcptr->points[0];
			configuration.all_camera_configs[i].cameraposition.x = pnt.x;
			configuration.all_camera_configs[i].cameraposition.y = pnt.y;
			configuration.all_camera_configs[i].cameraposition.z = pnt.z;
		}
	}

	virtual void _start_cameras() final {
		//
		// start the cameras. First start all non-sync-master cameras, then start the sync-master camera.
		//
		bool start_error = false;
		for (auto cam : cameras) {
			if (cam->is_sync_master()) continue;
			if (!cam->start()) {
				start_error = true;
			}
		}
		for (auto cam : cameras) {
			if (!cam->is_sync_master()) continue;
			if (!cam->start()) {
				start_error = true;
			}
		}
		if (start_error) {
			cwipc_k4a_log_warning("Not all cameras could be started");
			_unload_cameras();
			return;
		}

		starttime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		//
		// start the per-camera capture threads. Master camera has to be started latest
		//
		for (auto cam : cameras) {
			if (cam->is_sync_master()) continue;
			cam->start_capturer();
		}
		for (auto cam : cameras) {
			if (!cam->is_sync_master()) continue;
			cam->start_capturer();
		}
	}
	
	virtual bool _capture_all_cameras() = 0;
	
	virtual uint64_t _get_best_timestamp() = 0;
	
	virtual void _control_thread_main() final {
#ifdef CWIPC_DEBUG_THREAD
		std::cerr << CLASSNAME << ": processing thread started" << std::endl;
#endif
		while (!stopped) {
			{
				std::unique_lock<std::mutex> mylock(mergedPC_mutex);
				mergedPC_want_new_cv.wait(mylock, [this] { return mergedPC_want_new; });
			}
			//check EOF:
			for (auto cam : cameras) {
				if (cam->eof) {
					eof = true;
					stopped = true;
					break;
				}
			}
			if (stopped) break;
			assert(cameras.size() > 0);
			// Step one: grab frames from all cameras. This should happen as close together in time as possible,
			// because that gives use he biggest chance we have the same frame (or at most off-by-one) for each
			// camera.
			bool all_captures_ok = _capture_all_cameras();
			if (!all_captures_ok) {
				//std::cerr << CLASSNAME << ": xxxjack not all captures succeeded. Retrying." << std::endl;
				std::this_thread::yield();
				continue;
			}
			if (stopped) break;
			// And get the best timestamp
			uint64_t timestamp = _get_best_timestamp();
			// xxxjack current_ts = timestamp;
		
			// Step 2 - Create pointcloud, and save rgb/depth images if wanted
			cwipc_pcl_pointcloud pcl_pointcloud = new_cwipc_pcl_pointcloud();
			char* error_str = NULL;
			cwipc* newPC = cwipc_from_pcl(pcl_pointcloud, timestamp, &error_str, CWIPC_API_VERSION);
			if (newPC == nullptr) {
				std::cerr << CLASSNAME << ": cwipc_from_pcl returned error: " << error_str << std::endl;
				break;
			}

			if (want_auxdata_rgb || want_auxdata_depth) {
				for (auto cam : cameras) {
					cam->save_auxdata_images(newPC, want_auxdata_rgb, want_auxdata_depth);
				}
			}
			if (want_auxdata_skeleton) {
				for (auto cam : cameras) {
					cam->save_auxdata_skeleton(newPC);
				}
			}

			if (stopped) break;

			// Step 3: start processing frames to pointclouds, for each camera
			for (auto cam : cameras) {
				cam->create_pc_from_frames();
			}
			if (stopped) break;
			// Lock mergedPC already while we are waiting for the per-camera
			// processing threads. This so the main thread doesn't go off and do
			// useless things if it is calling available(true).
			std::unique_lock<std::mutex> mylock(mergedPC_mutex);
			if (mergedPC && mergedPC_is_fresh) {
				mergedPC->free();
				mergedPC = nullptr;
			}
			if (stopped) break;
			mergedPC = newPC;

			// Step 4: wait for frame processing to complete.
			for (auto cam : cameras) {
				cam->wait_for_pc();
			}
			if (stopped) break;
			// Step 5: merge views
			merge_views();
			if (mergedPC->access_pcl_pointcloud()->size() > 0) {
#ifdef CWIPC_DEBUG
				std::cerr << CLASSNAME << ": capturer produced a merged cloud of " << mergedPC->size() << " points" << std::endl;
#endif
			} else {
#ifdef CWIPC_DEBUG
				std::cerr << CLASSNAME << ": Warning: capturer got an empty pointcloud\n";
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
		std::cerr << CLASSNAME << ": processing thread stopped" << std::endl;
#endif
	}
	
	virtual void merge_views() final {
		cwipc_pcl_pointcloud aligned_cld(mergedPC->access_pcl_pointcloud());
		aligned_cld->clear();

		// Pre-allocate space in the merged pointcloud
		size_t nPoints = 0;
		for (auto cam : cameras) {
			cwipc_pcl_pointcloud cam_cld = cam->get_current_pointcloud();
			if (cam_cld == nullptr) {
				cwipc_k4a_log_warning("Camera " + cam->serial + " returned NULL cloud, ignoring");
				continue;
			}
			nPoints += cam_cld->size();
		}
		aligned_cld->reserve(nPoints);

		// Now merge all pointclouds
		for (auto cam : cameras) {
			cwipc_pcl_pointcloud cam_cld = cam->get_current_pointcloud();
			if (cam_cld == nullptr) continue;
			*aligned_cld += *cam_cld;
		}

#ifdef xxNacho_skeleton_DEBUG

		cwipc_pcl_pointcloud skl = new_cwipc_pcl_pointcloud();
		size_t sk_points = 32;
		skl->reserve(sk_points);
		//merge skeletons using average
		if (configuration.cameraData[0].skeletons.size() > 0) {
			//printf("skeletons merge:\n");
			for (int joint_id = 0; joint_id < 32; joint_id++) {
				cwipc_pcl_point joint_pos;
				//printf("Joint %i:\n", joint_id);
				int numsk = 0;
				for (K4ACameraConfig cd : configuration.cameraData) {
					if (cd.skeletons.size() > 0) {
						numsk++;
						joint_pos.x += cd.skeletons[0].joints[joint_id].position.xyz.x;
						joint_pos.y += cd.skeletons[0].joints[joint_id].position.xyz.y;
						joint_pos.z += cd.skeletons[0].joints[joint_id].position.xyz.z;
						int color[3] = {0, 0, 0};
						if (cd.skeletons[0].joints[joint_id].confidence_level == K4ABT_JOINT_CONFIDENCE_HIGH) {
							color[1] = 255;
						}
						else if (cd.skeletons[0].joints[joint_id].confidence_level == K4ABT_JOINT_CONFIDENCE_MEDIUM) {
							color[0] = 255;
							color[1] = 165;
						}
						else {
							color[0] = 255;
						}
						joint_pos.r = color[0];
						joint_pos.g = color[1];
						joint_pos.b = color[2];
						joint_pos.a = 8;
						//printf("\tCam %s (%f,%f,%f)\n", cd.serial,cd.skeletons[0].joints[joint_id].position.xyz.x, cd.skeletons[0].joints[joint_id].position.xyz.y, cd.skeletons[0].joints[joint_id].position.xyz.z);
					}
				}
				joint_pos.x /= numsk; //configuration.cameraData.size();
				joint_pos.y /= numsk; //configuration.cameraData.size();
				joint_pos.z /= numsk; //configuration.cameraData.size();
				skl->push_back(joint_pos);
				//printf("Fusion (%f,%f,%f)\n", joint_pos.x, joint_pos.y, joint_pos.z);
				//printf("%f %f %f\n", joint_pos.x, joint_pos.y, joint_pos.z);
			}
			uint64_t t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			std::string fn = "skeleton_" + std::to_string(t) + ".ply";
			cwipc* rv = cwipc_from_pcl(skl, t, NULL, CWIPC_API_VERSION);
			char* error = NULL;
			int ok = cwipc_write(fn.c_str(), rv, &error);
			if (ok==0)
				std::cout << "Writed skeleton = " << fn << std::endl;
		}
		else {
			std::cout << "No skeleton found" << std::endl;
			for (int i = 0; i < 32; i++) {
				cwipc_pcl_point joint_pos;
				joint_pos.a = 4;
				skl->push_back(joint_pos);
			}
		}
		*mergedPC += *skl;

#endif // xxNacho_skeleton_DEBUG
		if (aligned_cld->size() != nPoints) {
			cwipc_k4a_log_warning("Combined pointcloud has different number of points than expected");
		}
	}
	
	virtual void _request_new_pointcloud() final {
		std::unique_lock<std::mutex> mylock(mergedPC_mutex);
		if (!mergedPC_want_new && !mergedPC_is_fresh) {
			mergedPC_want_new = true;
			mergedPC_want_new_cv.notify_all();
		}
	}
};
#endif // cwipc_realsense_K4ABaseCapture_hpp