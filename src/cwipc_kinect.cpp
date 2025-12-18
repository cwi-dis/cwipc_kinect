
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_KINECT_EXPORT __declspec(dllexport)
#endif

#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"
#include "cwipc_util/internal.h"
#include "cwipc_util/vectors.h"
#include "cwipc_kinect/api.h"
#include "K4AConfig.hpp"
#include "K4ACapture.hpp"
#include "K4APlaybackCapture.hpp"


static bool _api_versioncheck(char **errorMessage, uint64_t apiVersion) {
    if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
        char* msgbuf = (char*)malloc(1024);
        snprintf(msgbuf, 1024, "cwipc_kinect: incorrect apiVersion 0x%08" PRIx64 " expected 0x%08" PRIx64 "..0x%08" PRIx64 "", apiVersion, CWIPC_API_VERSION_OLD, CWIPC_API_VERSION);
        if (errorMessage) {
            *errorMessage = msgbuf;
        }
        cwipc_log(LOG_ERROR, "cwipc_kinect", msgbuf + 13);
        return false;
    }
    return true;
}

// Global variables (constants, really)


class cwipc_source_kinect_impl : public cwipc_tiledsource {
protected:
    K4ACapture *m_grabber; cwipc_source_kinect_impl(K4ACapture *obj) : m_grabber(obj) {}

public:
    cwipc_source_kinect_impl(const char *configFilename=NULL) : m_grabber(K4ACapture::factory()) {
        m_grabber->config_reload(configFilename);
    }

    ~cwipc_source_kinect_impl() {
        delete m_grabber;
        m_grabber = NULL;
    }

    bool is_valid() {
        return m_grabber->camera_count > 0;
    }

    void free() {
        delete m_grabber;
        m_grabber = NULL;
    }

    virtual size_t get_config(char* buffer, size_t size) override
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

    virtual bool reload_config(const char* configFile) override {
        return m_grabber->config_reload(configFile);
    }

    bool eof() {
        return m_grabber->eof;
    }

    bool available(bool wait) {
        if (m_grabber == NULL) {
            return false;
        }

        return m_grabber->pointcloud_available(wait);
    }

    cwipc* get() {
        if (m_grabber == NULL) {
            return NULL;
        }

        cwipc* rv = m_grabber->get_pointcloud();
        return rv;
    }

    bool seek(uint64_t timestamp) {
        return false;
    }

    int maxtile() {
        if (m_grabber == NULL) {
            return 0;
        }

        int nCamera = m_grabber->configuration.all_camera_configs.size();
        if (nCamera <= 1) {
            // Using a single camera or no camera.
            return nCamera;
        }

        return nCamera + 1;
    }

    bool get_tileinfo(int tilenum, struct cwipc_tileinfo *tileinfo) {
        if (m_grabber == NULL) {
            return false;
        }

        int nCamera = m_grabber->configuration.all_camera_configs.size();

        if (nCamera == 0) { // No camera
            return false;
        }

        if (tilenum < 0 || tilenum >= nCamera+1) {
            return false;
        }
        if (tilenum == 0) {
            // Special case: the whole pointcloud
            if (tileinfo) {
                tileinfo->normal = { 0, 0, 0 };
                tileinfo->cameraName = NULL;
                tileinfo->ncamera = nCamera;
                tileinfo->cameraMask = 0; // All cameras contributes to this
            }
            return true;
        }
        K4ACameraConfig &cameraConfig = m_grabber->configuration.all_camera_configs[tilenum-1];
        if (tileinfo) {
            tileinfo->normal = cameraConfig.cameraposition; // Use the camera position as the normal
            tileinfo->cameraName = (char *)cameraConfig.serial.c_str();
            tileinfo->ncamera = 1; // Only one camera contributes to this
            tileinfo->cameraMask = (uint8_t)1 << (tilenum-1); // Only this camera contributes
        }
        return true;
    }

    void request_auxiliary_data(const std::string &name) override {
        cwipc_tiledsource::request_auxiliary_data(name);

        m_grabber->request_image_auxdata(
            auxiliary_data_requested("rgb"),
            auxiliary_data_requested("depth")
        );

        m_grabber->request_skeleton_auxdata(auxiliary_data_requested("skeleton"));
        std::cout << "cwipc_kinect: Requested auxdata rgb=" << auxiliary_data_requested("rgb") << ", depth=" << auxiliary_data_requested("depth") << ", skeleton=" << auxiliary_data_requested("skeleton") << std::endl;
    }

    bool auxiliary_operation(const std::string op, const void* inbuf, size_t insize, void* outbuf, size_t outsize) override {
        // For test purposes, really...
        std::cerr << "xxxjack cwipc_kinect aux-op " << op << std::endl;
        if (op != "map2d3d") return false;
        if (inbuf == nullptr || insize != 4 * sizeof(float)) return false;
        if (outbuf == nullptr || outsize != 3 * sizeof(float)) return false;
        float* infloat = (float*)inbuf;
        float* outfloat = (float*)outbuf;
        int tilenum = (int)infloat[0];
        int x_2d = (int)infloat[1];
        int y_2d = (int)infloat[2];
        float d_2d = infloat[3];

        return m_grabber->map2d3d(tilenum, x_2d, y_2d, d_2d, outfloat);
    }
};

class cwipc_source_k4aplayback_impl : public cwipc_tiledsource {
protected:
    K4APlaybackCapture *m_grabber;

public:
    cwipc_source_k4aplayback_impl(const char* configFilename = NULL) : m_grabber(K4APlaybackCapture::factory()) {
        m_grabber->config_reload(configFilename);
    }

    ~cwipc_source_k4aplayback_impl() {
        delete m_grabber;
        m_grabber = NULL;
    }

    bool is_valid() {
        return m_grabber->camera_count > 0;
    }

    void free() {
        delete m_grabber;
        m_grabber = NULL;
    }

    bool eof() {
        return m_grabber->eof;
    }

    bool available(bool wait) {
        if (m_grabber == NULL) {
            return false;
        }

        return m_grabber->pointcloud_available(wait);
    }

    cwipc *get() {
        if (m_grabber == NULL) {
            return NULL;
        }

        cwipc *rv = m_grabber->get_pointcloud();
        return rv;
    }

    bool seek(uint64_t timestamp) {
        if (m_grabber == NULL) {
            return NULL;
        }

        bool rv = m_grabber->seek(timestamp);
        return rv;
    }

    int maxtile() {
        if (m_grabber == NULL) {
            return 0;
        }

        int nCamera = m_grabber->configuration.all_camera_configs.size();

        if (nCamera <= 1) {
            // Using a single camera or no camera.
            return nCamera;
        }

        return 1 << nCamera;
    }

    bool get_tileinfo(int tilenum, struct cwipc_tileinfo *tileinfo) {
        if (m_grabber == NULL) {
            return false;
        }

        int nCamera = m_grabber->configuration.all_camera_configs.size();

        // No camera
        if (nCamera == 0) {
            return false;
        }

        if (tilenum < 0 || tilenum >= (1 << nCamera)) {
            return false;
        }

        // nCamera > 0
        cwipc_vector camcenter = {0, 0, 0};

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
        cwipc_vector tile_direction = {0, 0, 0};

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

    virtual bool reload_config(const char* configFile) override {
        return m_grabber->config_reload(configFile);
    }

    virtual size_t get_config(char* buffer, size_t size) override {
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

    void request_auxiliary_data(const std::string& name) override {
        cwipc_tiledsource::request_auxiliary_data(name);

        m_grabber->request_image_auxdata(
            auxiliary_data_requested("rgb"),
            auxiliary_data_requested("depth")
        );
        m_grabber->request_skeleton_auxdata(auxiliary_data_requested("skeleton"));

        std::cout << "cwipc_kinect: Requested auxdata rgb=" << auxiliary_data_requested("rgb") << ", depth=" << auxiliary_data_requested("depth") << ", skeleton=" << auxiliary_data_requested("skeleton") << std::endl;
    }

    bool auxiliary_operation(const std::string op, const void* inbuf, size_t insize, void* outbuf, size_t outsize) override {
        // For test purposes, really...
        if (op != "map2d3d") return false;
        if (inbuf == nullptr || insize != 4 * sizeof(float)) return false;
        if (outbuf == nullptr || outsize != 3 * sizeof(float)) return false;
        float* infloat = (float*)inbuf;
        float* outfloat = (float*)outbuf;
        int tilenum = (int)infloat[0];
        int x_2d = (int)infloat[1];
        int y_2d = (int)infloat[2];
        float d_2d = infloat[3];

        return m_grabber->map2d3d(tilenum, x_2d, y_2d, d_2d, outfloat);
    }
};

//
// C-compatible entry points
//

cwipc_tiledsource* cwipc_kinect(const char *configFilename, char **errorMessage, uint64_t apiVersion) {
    if (! _api_versioncheck(errorMessage,  apiVersion)) {
        return NULL;
    }
    cwipc_source_kinect_impl *rv = new cwipc_source_kinect_impl(configFilename);

    // If the grabber found cameras everything is fine
    if (rv && rv->is_valid()) {
        return rv;
    }

    delete rv;
    cwipc_log(LOG_ERROR, "cwipc_kinect", "no kinect cameras found");
    if (errorMessage && *errorMessage == NULL) {
        *errorMessage = (char *)"cwipc_kinect: no kinect cameras found";
    }

    return NULL;
}

cwipc_tiledsource* cwipc_k4aplayback(const char* configFilename, char** errorMessage, uint64_t apiVersion) {
    if (! _api_versioncheck(errorMessage,  apiVersion)) {
        return NULL;
    }
    cwipc_source_k4aplayback_impl* rv = new cwipc_source_k4aplayback_impl(configFilename);

    // If the grabber found cameras everything is fine
    if (rv && rv->is_valid()) {
        return rv;
    }

    delete rv;

    cwipc_log(LOG_ERROR, "cwipc_k4aplayback", "cannot open recording");
    if (errorMessage && *errorMessage == NULL) {
        *errorMessage = (char *)"cwipc_k4aplayback: cannot open recording";
    }
    return NULL;
}

//
// These static variables only exist to ensure the initializer is called, which registers our camera type.
//
int _cwipc_dummy_kinect_initializer = _cwipc_register_capturer("kinect", K4ACapture::count_devices, cwipc_kinect);
int _cwipc_dummy_kinect_playback_initializer = _cwipc_register_capturer("kinect_playback", nullptr, cwipc_k4aplayback);
// For backward compatibility
int _cwipc_dummy_kinect_offline_initializer = _cwipc_register_capturer("kinect_offline", nullptr, cwipc_k4aplayback);
