#include <Windows.h>
#include <Ole2.h>
#include <gl/GL.h>
#include <gl/GLU.h>
#include <iostream>

//#include <irrKlang.h>

//#include <gl/glut.h>
#include <Kinect.h>
#include <MMSystem.h>
#include "../headers/application.h"
//using namespace std; // For error reporting.

/*
	Default constructor to initialize the application instance.
*/
application::application(unsigned int width, unsigned int height)
{

	// Initialize Kinect sensor
	//this->m_running = initializeKinect();
	//if (!this->m_running) {
	//	system("pause");
	//	exit(EXIT_FAILURE);
	//}
	// Store application variables
	this->m_width = width;
	this->m_height = height;
	this->m_paused = false;
}

application::~application()
{
	//Later
}

application::application(application & other) {
	//I can do this for real later.
}

/**void application::Run() {
	//actual program goes here. Collect joint information, make calculations accordingly.
	for (int i = 0; i < 30; i++) {
		initializeKinect();
	}
	return;
}*/

bool application::GetRunning(){
	return true;
}

template < typename T, size_t N >
size_t countof( T ( & arr )[ N ] )
{
    return std::extent< T[ N ] >::value;
}

int application::Run()
{
    IKinectSensor *sensor = nullptr;
    IBodyFrameReader *bodyFrameReader = nullptr;

    //Get the default Kinect sensor
    HRESULT hr = GetDefaultKinectSensor(&sensor);

    //If the function succeeds, open the sensor
    if (SUCCEEDED(hr)) {
        hr = sensor->Open();
		//std::cout << "opened the sensor" << std::endl;
        if (SUCCEEDED(hr)) {
            //Get a body frame source from which we can get our body frame reader
            IBodyFrameSource *bodyFrameSource = nullptr;

            hr = sensor->get_BodyFrameSource(&bodyFrameSource);

            if (SUCCEEDED(hr)) {
                hr = bodyFrameSource->OpenReader(&bodyFrameReader);
				//std::cout << "got a body frame source" << std::endl;
            }

            //We're done with bodyFrameSource, so we'll release it
        }
    }

    if (sensor == nullptr || FAILED(hr)) {
        std::cerr << "Cannot find any sensors.\n";
        return E_FAIL;
    }

    while (bodyFrameReader != nullptr) {
        IBodyFrame *bodyFrame = nullptr;
        hr = bodyFrameReader->AcquireLatestFrame(&bodyFrame);
		//std::cout << "acquired the latest frame" << std::endl;
        if (SUCCEEDED(hr)) {
            IBody *bodies[BODY_COUNT] = {0};
            hr = bodyFrame->GetAndRefreshBodyData(countof(bodies), bodies);
			//std::cout << "hr is " << hr << std::endl;


			if (countof(bodies) == 0) {
				//std::cout << "no bodies detected" << std::endl;
			}
            if (SUCCEEDED(hr)) {
                getJointInfo(countof(bodies), bodies);
                //After body processing is done, we're done with our bodies so release them.
                for (unsigned int bodyIndex = 0; bodyIndex < countof(bodies); bodyIndex++) {
                    //release somehow
                }
            }
        }
        else if (sensor) {
            BOOLEAN isSensorAvailable = false;
			//std::cout << hr << std::endl;
            hr = sensor->get_IsAvailable(&isSensorAvailable);

            if (SUCCEEDED(hr) && isSensorAvailable == false) {
                std::cerr << "No available sensor is found.\n";
            }
        }
        else {
            std::cerr << "Trouble reading the body frame.\n";
        }
    }

    return 0;
}

void application::getJointInfo(const unsigned int &bodyCount, IBody **bodies)
{
	//std::cout << "beginning of getJointInfo" << bodyCount << std::endl;

    for (unsigned int bodyIndex = 0; bodyIndex < bodyCount; bodyIndex++) {
        IBody *body = bodies[bodyIndex];
        //Get the tracking status for the body, if it's not tracked we'll skip it
        BOOLEAN isTracked = false;
        HRESULT hr = body->get_IsTracked(&isTracked);
        if (FAILED(hr) || isTracked == false) {
            continue;
        }
		//std::cout << "beginning of getJointInfo" << bodyCount << std::endl;
        //If we're here the body is tracked so lets get the joint properties for this skeleton
        Joint joints[JointType_Count];
        hr = body->GetJoints(countof(joints), joints);

        if (SUCCEEDED(hr)) {
            //Let's print the head's position
            const CameraSpacePoint &headPos = joints[JointType_Head].Position;
            const CameraSpacePoint &leftHandPos = joints[JointType_HandLeft].Position;

            //Let's check if the use has his hand up
            if (leftHandPos.Y >= headPos.Y) {
                std::cout << "LEFT HAND UP!!\n";
				/*irrklang::createSoundDeviceList();
				irrklang::ISoundEngine* engine = nullptr;//irrklang::createIrrKlangDevice();

 				if (!engine)
   				std::cout << "engine not initialized properly \n";

				 // play some sound stream, looped
				 engine->play2D("Please_lower_your_hand.wav", true);*/
            }
			else {
				std::cout << "LEFT HAND NOT UP!!\n";
			}
            HandState leftHandState;
            hr = body->get_HandLeftState(&leftHandState);
            if (SUCCEEDED(hr)) {
                if (leftHandState == HandState_Closed) {
                    std::cout << "CLOSED HAND\n";
                }
                else if (leftHandState == HandState_Open) {
                    std::cout << "OPEN HAND\n";
                }
                else if (leftHandState == HandState_Lasso) {
                    std::cout << "PEW PEW HANDS\n";
                }
                else if (leftHandState == HandState_NotTracked) {
                    std::cout << "HAND IS NOT TRACKED\n";
                }
                else if (leftHandState == HandState_Unknown) {
                    std::cout << "HANDS STATE IS UNKNOWN\n";
                }
            }
        }
		else {
			std::cout << hr << std::endl;
		}
    }
}