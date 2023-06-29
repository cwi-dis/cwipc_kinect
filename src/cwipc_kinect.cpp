
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_KINECT_EXPORT __declspec(dllexport)
#endif

#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"
#include "cwipc_kinect/api.h"
#include "K4AConfig.hpp"
#include "K4ACapture.hpp"
#include "K4AOfflineCapture.hpp"


// Global variables (constants, really)


cwipc_vector* add_vectors(cwipc_vector a, cwipc_vector b, cwipc_vector *result) {
	if (result) {
		result->x = a.x + b.x;
		result->y = a.y + b.y;
		result->z = a.z + b.z;
	}
	return result;
}
cwipc_vector* diff_vectors(cwipc_vector a, cwipc_vector b, cwipc_vector *result) {
	if (result) {
		result->x = a.x - b.x;
		result->y = a.y - b.y;
		result->z = a.z - b.z;
	}
	return result;
}
double len_vector(cwipc_vector v) {
	return v.x * v.x + v.y * v.y + v.z * v.z;
}
cwipc_vector* mult_vector(double factor, cwipc_vector *v) {
	if (v) {
		v->x *= factor;
		v->y *= factor;
		v->z *= factor;
	}
	return v;
}
cwipc_vector* norm_vector(cwipc_vector *v) {
	double len = len_vector(*v);
	if (len > 0)
		mult_vector(1.0/len, v);
	return v;
}
double dot_vectors(cwipc_vector a, cwipc_vector b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
cwipc_vector* cross_vectors(cwipc_vector a, cwipc_vector b, cwipc_vector *result) {
	if (result) {
		result->x = a.y*b.z - a.z*b.y;
		result->y = a.z*b.x - a.x*b.z;
		result->z = a.x*b.y - a.y*b.x;
	}
	return result;
}

class cwipc_source_kinect_impl : public cwipc_tiledsource {
protected:
    K4ACapture *m_grabber;
    cwipc_source_kinect_impl(K4ACapture *obj)
    : m_grabber(obj)
    {}
public:
    cwipc_source_kinect_impl(const char *configFilename=NULL)
		: m_grabber(K4ACapture::factory())
	{ 
		m_grabber->config_reload(configFilename);
	}

    ~cwipc_source_kinect_impl()
	{
        delete m_grabber;
        m_grabber = NULL;
    }

    bool is_valid() {
        return m_grabber->camera_count > 0;
    }
    
    void free() 
	{
        delete m_grabber;
        m_grabber = NULL;
    }

	virtual size_t get_config(char* buffer, size_t size)
	{
		auto config = m_grabber->config_get();
		if (buffer == nullptr) {
			return config.length();
		}
		if (size < config.length()) {
			return 0;
		}
		memcpy(buffer, config.c_str(), config.length());
		return config.length();
	}

    bool eof() 
	{
    	return m_grabber->eof;
    }

    bool available(bool wait)
	{
    	if (m_grabber == NULL) return false;
    	return m_grabber->pointcloud_available(wait);
    }

    cwipc* get()
	{
        if (m_grabber == NULL) return NULL;
        cwipc* rv = m_grabber->get_pointcloud();
		return rv;
    }

	bool seek(uint64_t timestamp) {
		return false;
	}
    
    int maxtile()
    {
        if (m_grabber == NULL) return 0;
        int nCamera = m_grabber->configuration.all_camera_configs.size();
        if (nCamera <= 1) {
            // Using a single camera or no camera.
            return nCamera;
        }
        return 1<<nCamera;
    }
    
    bool get_tileinfo(int tilenum, struct cwipc_tileinfo *tileinfo) {
        if (m_grabber == NULL)
			return false;

        int nCamera = m_grabber->configuration.all_camera_configs.size();

		if (nCamera == 0) { // No camera
			return false;
		}
        if (tilenum < 0 || tilenum >= (1<<nCamera))
			return false;

		// nCamera > 0
		cwipc_vector camcenter = { 0, 0, 0 };

		// calculate the center of all cameras
		for (auto camdat : m_grabber->configuration.all_camera_configs) {
			add_vectors(camcenter, camdat.cameraposition, &camcenter);
		}
		mult_vector(1.0 / nCamera, &camcenter);

		// calculate normalized direction vectors from the center towards each camera
		std::vector<cwipc_vector> camera_directions;
		for (auto camdat : m_grabber->configuration.all_camera_configs) {
			cwipc_vector normal;
			diff_vectors(camdat.cameraposition, camcenter, &normal);
			norm_vector(&normal);
			camera_directions.push_back(normal);
		}

		// add all cameradirections that contributed
		int ncontribcam = 0;
		int lastcontribcamid = 0;
		cwipc_vector tile_direction = { 0, 0, 0 };
		for (int i = 0; i < m_grabber->configuration.all_camera_configs.size(); i++) {
			uint8_t camera_label = (uint8_t)1 << i;
			if (tilenum == 0 || (tilenum & camera_label)) {
				add_vectors(tile_direction, camera_directions[i], &tile_direction);
				ncontribcam++;
				lastcontribcamid = i;
			}
		}
		norm_vector(&tile_direction);
		
		if (tileinfo) {
			tileinfo->normal = tile_direction;
			tileinfo->cameraName = NULL;
			tileinfo->ncamera = ncontribcam;
			tileinfo->cameraMask = tilenum;
			if (ncontribcam == 1) {
				// A single camera contributed to this
				tileinfo->cameraName = (char *)m_grabber->configuration.all_camera_configs[lastcontribcamid].serial.c_str();
			}
		}
		return true;
    }

	void request_auxiliary_data(const std::string& name) override {
		cwipc_tiledsource::request_auxiliary_data(name);
		m_grabber->request_image_auxdata(
			auxiliary_data_requested("rgb"),
			auxiliary_data_requested("depth"));
		m_grabber->request_skeleton_auxdata(auxiliary_data_requested("skeleton"));
		std::cout << "cwipc_kinect: Requested auxdata rgb=" << auxiliary_data_requested("rgb") << ", depth=" << auxiliary_data_requested("depth") << ", skeleton=" << auxiliary_data_requested("skeleton") << std::endl;
	}
};


class cwipc_source_k4aoffline_impl : public cwipc_tiledsource{
protected:
	K4AOfflineCapture *m_offline;
public:
	cwipc_source_k4aoffline_impl(const char* configFilename = NULL)
		: m_offline(K4AOfflineCapture::factory())
	{
		m_offline->config_reload(configFilename);
	}

	~cwipc_source_k4aoffline_impl()
	{
		delete m_offline;
		m_offline = NULL;
	}

	bool is_valid() {
		return m_offline->camera_count > 0;
	}

	void free()
	{
		delete m_offline;
		m_offline = NULL;
	}

	bool eof()
	{
		return m_offline->eof;
	}

	bool available(bool wait)
	{
		if (m_offline == NULL) return false;
		return m_offline->pointcloud_available(wait);
	}

	cwipc* get()
	{
		if (m_offline == NULL) return NULL;
		cwipc* rv = m_offline->get_pointcloud();
		return rv;
	}

	bool seek(uint64_t timestamp) {
		if (m_offline == NULL) return NULL;
		bool rv = m_offline->seek(timestamp);
		return rv;
	}

	int maxtile()
	{
		if (m_offline == NULL) return 0;
		int nCamera = m_offline->configuration.all_camera_configs.size();
		if (nCamera <= 1) {
			// Using a single camera or no camera.
			return nCamera;
		}
		return 1 << nCamera;
	}

	bool get_tileinfo(int tilenum, struct cwipc_tileinfo* tileinfo) {
		if (m_offline == NULL)
			return false;

		int nCamera = m_offline->configuration.all_camera_configs.size();

		if (nCamera == 0) { // No camera
			return false;
		}
		if (tilenum < 0 || tilenum >= (1 << nCamera))
			return false;

		// nCamera > 0
		cwipc_vector camcenter = { 0, 0, 0 };

		// calculate the center of all cameras
		for (auto camdat : m_offline->configuration.all_camera_configs) {
			add_vectors(camcenter, camdat.cameraposition, &camcenter);
		}
		mult_vector(1.0 / nCamera, &camcenter);

		// calculate normalized direction vectors from the center towards each camera
		std::vector<cwipc_vector> camera_directions;
		for (auto camdat : m_offline->configuration.all_camera_configs) {
			cwipc_vector normal;
			diff_vectors(camdat.cameraposition, camcenter, &normal);
			norm_vector(&normal);
			camera_directions.push_back(normal);
		}

		// add all cameradirections that contributed
		int ncontribcam = 0;
		int lastcontribcamid = 0;
		cwipc_vector tile_direction = { 0, 0, 0 };
		for (int i = 0; i < m_offline->configuration.all_camera_configs.size(); i++) {
			uint8_t camera_label = (uint8_t)1 << i;
			if (tilenum == 0 || (tilenum & camera_label)) {
				add_vectors(tile_direction, camera_directions[i], &tile_direction);
				ncontribcam++;
				lastcontribcamid = i;
			}
		}
		norm_vector(&tile_direction);

		if (tileinfo) {
			tileinfo->normal = tile_direction;
			tileinfo->cameraName = NULL;
			tileinfo->ncamera = ncontribcam;
			tileinfo->cameraMask = tilenum;
			if (ncontribcam == 1) {
				// A single camera contributed to this
				tileinfo->cameraName = (char*)m_offline->configuration.all_camera_configs[lastcontribcamid].serial.c_str();
			}
		}
		return true;
	}

	void request_auxiliary_data(const std::string& name) override {
		cwipc_tiledsource::request_auxiliary_data(name);
		m_offline->request_image_auxdata(
			auxiliary_data_requested("rgb"),
			auxiliary_data_requested("depth"));
		m_offline->request_skeleton_auxdata(auxiliary_data_requested("skeleton"));
		std::cout << "cwipc_kinect: Requested auxdata rgb=" << auxiliary_data_requested("rgb") << ", depth=" << auxiliary_data_requested("depth") << ", skeleton=" << auxiliary_data_requested("skeleton") << std::endl;
	}
};

//
// C-compatible entry points
//

cwipc_tiledsource* cwipc_kinect(const char *configFilename, char **errorMessage, uint64_t apiVersion)
{
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
			char* msgbuf = (char*)malloc(1024);
			snprintf(msgbuf, 1024, "cwipc_kinect: incorrect apiVersion 0x%08" PRIx64 " expected 0x%08" PRIx64 "..0x%08" PRIx64 "", apiVersion, CWIPC_API_VERSION_OLD, CWIPC_API_VERSION);
			*errorMessage = msgbuf;
		}
		return NULL;
	}
//xxxjack 	if (!MFCapture_versionCheck(errorMessage)) return NULL;
    cwipc_k4a_warning_store = errorMessage;
	cwipc_source_kinect_impl *rv = new cwipc_source_kinect_impl(configFilename);
    cwipc_k4a_warning_store = NULL;
    // If the grabber found cameras everything is fine
    if (rv && rv->is_valid()) return rv;
    delete rv;
    if (errorMessage && *errorMessage == NULL) {
        *errorMessage = (char *)"cwipc_kinect: no kinect cameras found";
    }
    return NULL;
}

cwipc_tiledsource* cwipc_k4aoffline(const char* configFilename, char** errorMessage, uint64_t apiVersion)
{
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
			char* msgbuf = (char*)malloc(1024);
			snprintf(msgbuf, 1024, "cwipc_k4aoffline: incorrect apiVersion 0x%08" PRIx64 " expected 0x%08" PRIx64 "..0x%08" PRIx64 "", apiVersion, CWIPC_API_VERSION_OLD, CWIPC_API_VERSION);
			*errorMessage = msgbuf;
		}
		return NULL;
	}
	cwipc_source_k4aoffline_impl* rv = new cwipc_source_k4aoffline_impl(configFilename);
	// If the grabber found cameras everything is fine
	if (rv && rv->is_valid()) return rv;
	delete rv;
	if (errorMessage && *errorMessage == NULL) {
		*errorMessage = (char*)"cwipc_kinect: no kinect cameras found";
	}
	return NULL;
}

//void cwipc_offline_free(cwipc_offline* obj)
//{
//	obj->free();
//}
//
//cwipc_tiledsource* cwipc_offline_get_source(cwipc_offline* obj)
//{
//	return obj->get_source();
//}

