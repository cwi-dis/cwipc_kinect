//
//  pcl_align.hpp
//
//  Created by Fons Kuijk on 12-12-18.
//

#ifndef pcl_align_hpp
#define pcl_align_hpp
#pragma once

#include "multiFrame.hpp"

#define GLFW_INCLUDE_GLU
#include "GLFW/glfw3.h"

#include <string>
#include <sstream>
#include <iostream>
#include <pcl/io/ply_io.h>

class window
{
public:
	std::function<void(bool)>           on_left_mouse = [](bool) {};
	std::function<void(double, double)> on_mouse_scroll = [](double, double) {};
	std::function<void(double, double)> on_mouse_move = [](double, double) {};
	std::function<void(int)>            on_key_release = [](int) {};

	window(int width, int height, const char* title) : _width(width), _height(height)
	{
		glfwInit();
		win = glfwCreateWindow(width, height, title, nullptr, nullptr);
		if (!win)
			throw std::runtime_error("Could not open OpenGL window, please check your graphic drivers or use the textual SDK tools");
		glfwMakeContextCurrent(win);
		glfwSetWindowUserPointer(win, this);
		glfwSetMouseButtonCallback(win, [](GLFWwindow * win, int button, int action, int mods) {
			auto s = (window*)glfwGetWindowUserPointer(win);
			if (button == 0) s->on_left_mouse(action == GLFW_PRESS);
		});

		glfwSetScrollCallback(win, [](GLFWwindow * win, double xoffset, double yoffset) {
			auto s = (window*)glfwGetWindowUserPointer(win);
			s->on_mouse_scroll(xoffset, yoffset);
		});

		glfwSetCursorPosCallback(win, [](GLFWwindow * win, double x, double y) {
			auto s = (window*)glfwGetWindowUserPointer(win);
			s->on_mouse_move(x, y);
		});

		glfwSetKeyCallback(win, [](GLFWwindow * win, int key, int scancode, int action, int mods) {
			auto s = (window*)glfwGetWindowUserPointer(win);
			if (0 == action) // on key release
				s->on_key_release(key);
		});
	}

	float width() const { return float(_width); }
	float height() const { return float(_height); }

	operator bool()
	{
		glPopMatrix();
		glfwSwapBuffers(win);

		auto res = !glfwWindowShouldClose(win);

		glfwPollEvents();
		glfwGetFramebufferSize(win, &_width, &_height);

		// Clear the framebuffer
		glClear(GL_COLOR_BUFFER_BIT);
		glViewport(0, 0, _width, _height);

		// Draw the images
		glPushMatrix();
		glfwGetWindowSize(win, &_width, &_height);
		glOrtho(0, _width, _height, 0, -1, +1);

		return res;
	}

	~window()
	{
		glfwDestroyWindow(win);
		glfwTerminate();
	}

	operator GLFWwindow*() { return win; }

private:
	GLFWwindow* win;
	int _width, _height;
};

// Struct for managing rotation of pointcloud view
struct glfw_state {
	glfw_state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0), ml(false), offset(0.f) {}
	double yaw;
	double pitch;
	double last_x;
	double last_y;
	bool ml;
	float offset;
};

struct cam_data {
	string serial;
	boost::shared_ptr<Eigen::Affine3d> trafo;
	boost::shared_ptr<PointCloudT> cloud;
};

bool do_align = false;
bool rotation = true;
int aligncamera = 0;
Eigen::Vector4f mergedcenter;	// Needed to automatically center the merged cloud
Eigen::Vector4f cloudcenter;		// Needed to be able to rotate around the cloud's centre of mass
vector<cam_data> CamData;		// Storage of per camera data for reloaded camera data

string ext(".ply");

void printhelp() {
	cout << "\nThe cloud rendered by this application will automatically be centered with the view origin.\n";
	cout << "To examine the pointcloud use the mouse: leftclick and move to rotate, use the mouse wheel to zoom.\n";
	cout << "Use \"esc\" to reset the position of the (fused) cloud.\n";

	cout << "\naction keys for alignment of camera clouds:\n";
	cout << "\t\"a\" to toggle between \"life\" and \"alignment mode\"\n";
	cout << "\t\"1-9\" to select the camera to align\n";
	cout << "\t\"r\" to start cloud rotate mode\n";
	cout << "\t\"t\" to start cloud translate mode\n";
	cout << "\t\"esc\" to reset the cloud transformation of the active camera\n";
	cout << "\t\"s\" to save the configuration and snapshots of each camera to files\n";
	cout << "\t\"l\" to load a configuration and snapshots of each camera from files to (re)align\n";
	cout << "\t\"h\" to print this help\n";
	cout << "\t\"q\" to quit\n";
}

// store the current camera transformation setting into a xml document
void config2file(multiFrame& multiframe)
{
	TiXmlDocument doc;
	doc.LinkEndChild(new TiXmlDeclaration("1.0", "", ""));

	TiXmlElement* root = new TiXmlElement("file");
	doc.LinkEndChild(root);

	TiXmlElement* file = new TiXmlElement("CameraConfig");
	root->LinkEndChild(file);
	file->SetDoubleAttribute("resolution", multiframe.getSpatialResolution());
	file->SetAttribute("ringbuffersize", multiframe.getRingbufferSize());
	file->SetAttribute("greenscreenremoval", multiframe.getGreenScreen());
	file->SetAttribute("tiling", multiframe.getTiling());
	file->SetDoubleAttribute("tilingresolution", multiframe.getTilingResolution());

	for (cam_data ccfg : CamData) {
		TiXmlElement* cam = new TiXmlElement("camera");
		cam->SetAttribute("serial", ccfg.serial.c_str());
		file->LinkEndChild(cam);

		TiXmlElement* trafo = new TiXmlElement("trafo");
		cam->LinkEndChild(trafo);

		TiXmlElement* val = new TiXmlElement("values");
		val->SetDoubleAttribute("v00", (*ccfg.trafo)(0, 0));
		val->SetDoubleAttribute("v01", (*ccfg.trafo)(0, 1));
		val->SetDoubleAttribute("v02", (*ccfg.trafo)(0, 2));
		val->SetDoubleAttribute("v03", (*ccfg.trafo)(0, 3));
		val->SetDoubleAttribute("v10", (*ccfg.trafo)(1, 0));
		val->SetDoubleAttribute("v11", (*ccfg.trafo)(1, 1));
		val->SetDoubleAttribute("v12", (*ccfg.trafo)(1, 2));
		val->SetDoubleAttribute("v13", (*ccfg.trafo)(1, 3));
		val->SetDoubleAttribute("v20", (*ccfg.trafo)(2, 0));
		val->SetDoubleAttribute("v21", (*ccfg.trafo)(2, 1));
		val->SetDoubleAttribute("v22", (*ccfg.trafo)(2, 2));
		val->SetDoubleAttribute("v23", (*ccfg.trafo)(2, 3));
		val->SetDoubleAttribute("v30", (*ccfg.trafo)(3, 0));
		val->SetDoubleAttribute("v31", (*ccfg.trafo)(3, 1));
		val->SetDoubleAttribute("v32", (*ccfg.trafo)(3, 2));
		val->SetDoubleAttribute("v33", (*ccfg.trafo)(3, 3));
		trafo->LinkEndChild(val);
	}
	doc.SaveFile("cameraconfig.xml");
}

void cloud2file(boost::shared_ptr<PointCloudT> pntcld, string filename)
{
	if (!pntcld) return;
	int size = pntcld->size();
	if (size <= 0) return;

	std::ofstream myfile(filename.c_str());
	myfile << "ply\n" << "format ascii 1.0\nelement vertex " << size << "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

	std::ostringstream oss;
	for (int i = 0; i < size; i++) {
		oss << (
			std::to_string((*pntcld)[i].x) + " " +
			std::to_string((*pntcld)[i].y) + " " +
			std::to_string((*pntcld)[i].z) + " " +
			std::to_string((*pntcld)[i].r) + " " +
			std::to_string((*pntcld)[i].g) + " " +
			std::to_string((*pntcld)[i].b) + "\n").c_str();
	}
	myfile << oss.str();
	myfile.close();
}

bool load_ply_of_camera(cam_data camera)
{
	// Load the cloud and save it into the global list of models
	if (pcl::io::loadPLYFile<PointT>(camera.serial + ext, *camera.cloud) == -1)
	{
		string msg = "Error loading cloud from file " + camera.serial + ext + "\n";
		PCL_ERROR(msg.c_str());
		return false;
	}
	return true;
}

bool load_config()
{
	TiXmlDocument doc("cameraconfig.xml");
	bool loadOkay = doc.LoadFile();
	if (!loadOkay)
	{
		std::cout << "WARNING: Failed to load cameraconfig.xml\n";
		if (CamData.size() > 1)
			std::cout << "\tCaptured pointclouds will be merged based on unregistered camera clouds\n";
		return false;
	}
	CamData.clear();

	TiXmlHandle docHandle(&doc);
	TiXmlElement* configElement = docHandle.FirstChild("file").FirstChild("CameraConfig").ToElement();
	TiXmlElement* cameraElement = configElement->FirstChildElement("camera");
	while (cameraElement)
	{
		cam_data* cc = new cam_data();
		boost::shared_ptr<PointCloudT> empty_pntcld(new PointCloudT());
		boost::shared_ptr<Eigen::Affine3d> trafo(new Eigen::Affine3d());
		cc->serial = cameraElement->Attribute("serial");
		cc->cloud = empty_pntcld;
		cc->trafo = trafo;

		TiXmlElement *trafoElement = cameraElement->FirstChildElement("trafo");
		if (trafoElement) {
			TiXmlElement *val = trafoElement->FirstChildElement("values");
			val->QueryDoubleAttribute("v00", &(*cc->trafo)(0, 0));
			val->QueryDoubleAttribute("v01", &(*cc->trafo)(0, 1));
			val->QueryDoubleAttribute("v02", &(*cc->trafo)(0, 2));
			val->QueryDoubleAttribute("v03", &(*cc->trafo)(0, 3));
			val->QueryDoubleAttribute("v10", &(*cc->trafo)(1, 0));
			val->QueryDoubleAttribute("v11", &(*cc->trafo)(1, 1));
			val->QueryDoubleAttribute("v12", &(*cc->trafo)(1, 2));
			val->QueryDoubleAttribute("v13", &(*cc->trafo)(1, 3));
			val->QueryDoubleAttribute("v20", &(*cc->trafo)(2, 0));
			val->QueryDoubleAttribute("v21", &(*cc->trafo)(2, 1));
			val->QueryDoubleAttribute("v22", &(*cc->trafo)(2, 2));
			val->QueryDoubleAttribute("v23", &(*cc->trafo)(2, 3));
			val->QueryDoubleAttribute("v30", &(*cc->trafo)(3, 0));
			val->QueryDoubleAttribute("v31", &(*cc->trafo)(3, 1));
			val->QueryDoubleAttribute("v32", &(*cc->trafo)(3, 2));
			val->QueryDoubleAttribute("v33", &(*cc->trafo)(3, 3));
		}
		CamData.push_back(*cc);
		cameraElement = cameraElement->NextSiblingElement("camera");
	}
	return true;
}

bool load_data() {
	if (!load_config())
		return false;
	for (auto camera : CamData)
		if (!load_ply_of_camera(camera)) {
			cerr << "Could not load a .ply file for camera " << camera.serial << " as specified in the configuration file\n";
			return false;
		}
	return true;
}

// Handle the OpenGL setup needed to display all pointclouds
void draw_pointcloud(window& app, glfw_state& app_state, multiFrame& multiframe)
{
	// OpenGL commands that prep screen
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
	glClear(GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60, app.width() / app.height(), 0.01f, 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

	glTranslatef(0.0, 0.0, app_state.offset*0.05f);
	glRotated(app_state.pitch, 1, 0, 0);
	glRotated(app_state.yaw, 0, 1, 0);
	glTranslatef(-mergedcenter.x(), -mergedcenter.y(), -mergedcenter.z());
	glPushMatrix();
	glPointSize(app.width() / 640);
	glEnable(GL_DEPTH_TEST);
	glBegin(GL_POINTS);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

	// draw the pointcloud(s)
	if (do_align) {
		for (int i = 0; i < CamData.size(); i++) {
			PointCloudT::Ptr pcptr(new PointCloudT);
			transformPointCloud(*(CamData[i].cloud.get()), *pcptr, *CamData[i].trafo);
			for (auto pnt : pcptr->points) {
				float col[3];
				if (i == aligncamera) {	// highlight the cloud of the selected camera
					col[0] = 0.2 + pnt.r / 320.0;
					col[1] = 0.2 + pnt.g / 320.0;
					col[2] = 0.2 + pnt.b / 320.0;
				}
				else {
					col[0] = pnt.r / 320.0;
					col[1] = pnt.g / 320.0;
					col[2] = pnt.b / 320.0;
				}
				glColor3fv(col);
				float vert[] = { pnt.x, pnt.y, pnt.z };
				glVertex3fv(vert);
			}
		}
	}
	else {
		if (multiframe.getNumberOfCameras() > 0) {
			for (auto pnt : multiframe.getPointCloud()->points) {
				float col[] = { (float)pnt.r / 256.f, (float)pnt.g / 256.f, (float)pnt.b / 256.f };
				glColor3fv(col);
				float vert[] = { pnt.x, pnt.y, pnt.z };
				glVertex3fv(vert);
			}
		}
		else {
			for (int i = 0; i < CamData.size(); i++) {
				PointCloudT::Ptr pcptr(new PointCloudT);
				transformPointCloud(*(CamData[i].cloud.get()), *pcptr, *CamData[i].trafo);
				for (auto pnt : pcptr->points) {
					float col[] = { (float)pnt.r / 256.f, (float)pnt.g / 256.f, (float)pnt.b / 256.f };
					glColor3fv(col);
					float vert[] = { pnt.x, pnt.y, pnt.z };
					glVertex3fv(vert);
				}
			}
		}
	}

	// OpenGL cleanup
	glEnd();
	glPopMatrix();
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glPushMatrix();
}

// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window& app, glfw_state& app_state, multiFrame& multiframe)
{
	app.on_left_mouse = [&](bool pressed) {
		app_state.ml = pressed;
	};

	app.on_mouse_scroll = [&](double xoffset, double yoffset) {
		if (do_align) {
			Eigen::Affine3d *transform = CamData[aligncamera].trafo.get();
			if (rotation) {
				(*transform).rotate(Eigen::AngleAxisd(yoffset / 100.0, Eigen::Vector3d::UnitZ()));
			}
			else {
				(*transform).translate(Eigen::Vector3d(0.0, 0.0, -yoffset / 100.0));
			}
		}
		else {
			app_state.offset -= static_cast<float>(yoffset);
		}
	};

	app.on_mouse_move = [&](double x, double y) {
		if (app_state.ml) {
			if (do_align) {
				Eigen::Affine3d *transform = CamData[aligncamera].trafo.get();
				double dx = (x - app_state.last_x) / (0.25 * app.width());
				double dy = -(y - app_state.last_y) / (0.25 * app.width());
				if (rotation) {
					(*transform).translate(Eigen::Vector3d(cloudcenter.x(), cloudcenter.y(), cloudcenter.z()));
					(*transform).rotate(Eigen::AngleAxisd(dx, Eigen::Vector3d::UnitY()));
					(*transform).rotate(Eigen::AngleAxisd(-dy, Eigen::Vector3d::UnitX()));
					(*transform).translate(-Eigen::Vector3d(cloudcenter.x(), cloudcenter.y(), cloudcenter.z()));
				}
				else {
					(*transform).translate(Eigen::Vector3d(dx, dy, 0.0));
				}
			}
			else {
				app_state.yaw += (x - app_state.last_x) / 10.0;
				app_state.pitch += (y - app_state.last_y) / 10.0;
				app_state.pitch = std::max(app_state.pitch, -85.0);
				app_state.pitch = std::min(app_state.pitch, +85.0);
			}
		}
		app_state.last_x = x;
		app_state.last_y = y;
	};

	app.on_key_release = [&](int key) {
		if (key == 256) { // Escape is interpreted as a reset of the transformation
			if (do_align) {
				Eigen::Affine3d *transform = CamData[aligncamera].trafo.get();
				(*transform).setIdentity();
			}
			else {
				app_state.yaw = app_state.pitch = 0;
				app_state.offset = 0.0;
			}
		}
		else if (key == 65) {	// key = "a": start/stop Alignment
			if (do_align) {
				do_align = false;
			}
			else {
				if (multiframe.getNumberOfCameras() > 0) {
					CamData.clear();
					for (int i = 0; i < multiframe.getNumberOfCameras(); i++) {
						cam_data *cam = new cam_data();
						cam->serial = multiframe.getCameraSerial(i);
						cam->trafo = multiframe.getCameraTransform(i);
						cam->cloud = multiframe.getCameraCloud(i);
						CamData.push_back(*cam);
					}
				}
				do_align = true;
				if (aligncamera > CamData.size())
					aligncamera = 0;
				pcl::compute3DCentroid(*CamData[aligncamera].cloud, cloudcenter);
			}
		}
		else if (key == 72) {	// key = "h": print help
			printhelp();
		}
		else if (key == 76) {	// key = "l": load previous result
			if (load_data() && CamData.size() > 0)
				do_align = true;
			else {
				cerr << "\nError: Data could not be loaded\n";
				return;
			}
			if (aligncamera > CamData.size())
				aligncamera = 0;
			pcl::compute3DCentroid(*CamData[aligncamera].cloud, cloudcenter);
		}
		else if (key == 81) {	// key = "q": Quit program
			multiframe.~multiFrame();
			exit(0);
		}
		else if (key == 82) {	// key = "r": Rotate
			rotation = true;
		}
		else if (key == 83) {	// key = "s": Save config and snapshots to file
			config2file(multiframe);
			for (int i = 0; i < CamData.size(); i++)
				cloud2file(CamData[i].cloud, CamData[i].serial + ".ply");
		}
		else if (key == 84) {	// key = "t": Translate
			rotation = false;
		}
		else if (key >= 49 && key < CamData.size() + 49) {	// key = "1-9": select a camera
			aligncamera = key - 49;
			pcl::compute3DCentroid(*CamData[aligncamera].cloud, cloudcenter);
		}
		else if (key == 73) {	// key =\"i": dump frames for icp processing
			for (int i = 0; i < CamData.size(); i++) {
				PointCloudT::Ptr point_cloud_ptr(new PointCloudT);
				boost::shared_ptr<PointCloudT> aligned_cld(point_cloud_ptr);

				transformPointCloud(*CamData[i].cloud.get(), *aligned_cld, *CamData[i].trafo);

				cloud2file(aligned_cld, "pcl_aligned_" + CamData[i].serial + ".ply");
				cloud2file(CamData[i].cloud, "pcl_original_" + CamData[i].serial + ".ply");
			}
		}
	};
}

#endif /* pcl_align_hpp */