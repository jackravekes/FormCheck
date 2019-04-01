#pragma once

#if !__APPLE__
#define GLEW_STATIC
#include <Windows.h>
#include <Ole2.h>
//#include <gl/glew.h>
#include "../deps/SDL/SDL_opengl.h"

#endif

// Determines whether someone is in the way of the Kinect
// Crossed state invalid is triggered when the Kinect does
// not collect data.
enum CrossedState {
	CrossedState_True,
	CrossedState_False,
	CrossedState_Invalid
};

namespace applicationconstants {
	const unsigned int DefaultWidth_		= 1280;
	const unsigned int DefaultHeight_		= 960;

	const float MaxDepth_					= 4300.f;
}
