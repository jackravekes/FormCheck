#pragma once

#define GLEW_STATIC
#include <Kinect.h>
#include <vector>
#include <string>
#include <irrKlang.h>
#include "applicationconstants.h"
#include "../deps/SDL/SDL_opengl.h"


class application {
public:
	application(unsigned int width = applicationconstants::DefaultWidth_, unsigned int height = applicationconstants::DefaultHeight_);
	~application();
	application(application & other);
	int Run();
	bool GetRunning();
private:
	void getJointInfo(const unsigned int &bodyCount, IBody **bodies);
	int initializeKinect();
	int m_width, m_height;				// Dimensions of application
	IKinectSensor* kinect_sensor;				// Kinect sensor
	HANDLE m_depthStream;				// Stream for Depth data
	HANDLE m_infraredStream;				// Stream for infrared data

	bool m_running;						// Denotes prepared instance of Application
	bool m_paused;						// Denotes paused instance of Application
	void* m_context;					// GL's context
};