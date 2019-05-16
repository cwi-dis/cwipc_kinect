
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_REALSENSE2_EXPORT __declspec(dllexport)
#endif

#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"
#include "cwipc_realsense2/api.h"

#include "cwipc_realsense2/multiFrame.hpp"

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

class cwipc_source_realsense2_impl : public cwipc_tiledsource {
private:
    multiFrame *m_grabber;
public:
    cwipc_source_realsense2_impl(const char *configFilename=NULL)
		: m_grabber(NULL)
	{ 
		m_grabber = new multiFrame(configFilename); 
	}

    ~cwipc_source_realsense2_impl()
	{
        delete m_grabber;
    }

    void free() 
	{
        delete m_grabber;
    }

    bool eof() 
	{
    	return false;
    }

    bool available(bool wait)
	{
    	return m_grabber != NULL;
    }

    cwipc* get()
	{
        if (m_grabber == NULL) return NULL;
        uint64_t timestamp;
        cwipc_pcl_pointcloud pc = m_grabber->get_pointcloud(&timestamp);
        if (pc == NULL) return NULL;
        return cwipc_from_pcl(pc, timestamp, NULL);
    }
    
    int maxtile()
    {
        if (m_grabber == NULL) return 0;
        int nCamera = m_grabber->configuration.camera_data.size();
        if (nCamera <= 1) {
            // Using a single camera or synthetic grabber. 1 tile only.
            return 1;
        }
        return 1<<nCamera;
    }
    
    bool get_tileinfo(int tilenum, struct cwipc_tileinfo *tileinfo, int infoVersion) {
        if (m_grabber == NULL)
			return false;
        if (infoVersion != CWIPC_TILEINFO_VERSION)
            return false;

        int nCamera = m_grabber->configuration.camera_data.size();
        if (tilenum < 0 || tilenum >= (1<<nCamera))
			return false;

		if (nCamera == 0 || tilenum == 0) { // The synthetic camera...
			cwipc_tileinfo info = { {0, 0, 0} };
			if (tileinfo) {
				*tileinfo = info;
				return true;
			}
			else
				return false;
		}

		// nCamera > 0
		cwipc_vector camcenter = { 0, 0, 0 };

		// calculate the center of all cameras
		for (auto camdat : m_grabber->configuration.camera_data)
			add_vectors(camcenter, camdat.cameraposition, &camcenter);
		mult_vector(1.0 / nCamera, &camcenter);

		// calculate normalized direction vectors from the center towards each camera
		std::vector<cwipc_vector> camera_directions;
		for (auto camdat : m_grabber->configuration.camera_data) {
			cwipc_vector normal;
			diff_vectors(camdat.cameraposition, camcenter, &normal);
			norm_vector(&normal);
			camera_directions.push_back(normal);
		}

		// add all cameradirections that contributed
		cwipc_vector tile_direction = { 0, 0, 0 };
		for (int i = 0; i < m_grabber->configuration.camera_data.size(); i++) {
			uint8_t camera_label = (uint8_t)1 << i;
			if (tilenum & camera_label)
				add_vectors(tile_direction, camera_directions[i], &tile_direction);
		}
		norm_vector(&tile_direction);

		if (tileinfo) {
			tileinfo->normal = tile_direction;
			return true;
		}
		return false;
    }
};

//
// C-compatible entry points
//

cwipc_tiledsource* cwipc_realsense2(char **errorMessage)
{
	return new cwipc_source_realsense2_impl();
}

cwipc_tiledsource* cwipc_realsense2_ex(const char *configFilename, char **errorMessage)
{
	return new cwipc_source_realsense2_impl(configFilename);
}
