//------------------------------------------------------------------------------
// <copyright file="BodyBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "BodyBasics.h"
#include <cmath>
# include <stdio.h>
# include <math.h>
# include <stdlib.h>

//Audio
#include <Mmsystem.h>
#include <Windows.h>
#include <queue> 
#include <string>

//Balance board
#include "pch.h"
#include <stdio.h>
#include <tchar.h>
#include "SerialClass.h"	// Library described above
#include <string>
#include <regex>
#include <iostream> 
#include <sstream>
#include <thread>
using namespace std;

static const float c_JointThickness = 3.0f;
static const float c_TrackedBoneThickness = 6.0f;
static const float c_InferredBoneThickness = 1.0f;
static const float c_HandSize = 30.0f;

// Rep Tracking Parameters
int repCount = -1;
int lowerRepThreshold = -20;
int upperRepThreshold = 0;
int initialHandLeftCM = -1; 
int initialHandRightCM = -1;
int initial = 0;
enum repState {up, down};
repState currRep = up;

double currLeftLegAngle = 180;
double currRightLegAngle = 180;

//Audio
queue<string> audioToPlay;

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(    
	_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR lpCmdLine,
    _In_ int nShowCmd
)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    CBodyBasics application;
    application.Run(hInstance, nShowCmd);
}

/// <summary>
/// Constructor
/// </summary>
CBodyBasics::CBodyBasics() :
    m_hWnd(NULL),
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0LL),
    m_pKinectSensor(NULL),
    m_pCoordinateMapper(NULL),
    m_pBodyFrameReader(NULL),
    m_pD2DFactory(NULL),
    m_pRenderTarget(NULL),
    m_pBrushJointTracked(NULL),
    m_pBrushJointInferred(NULL),
    m_pBrushBoneTracked(NULL),
    m_pBrushBoneInferred(NULL)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }
}
  

/// <summary>
/// Destructor
/// </summary>
CBodyBasics::~CBodyBasics()
{
    DiscardDirect2DResources();

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    // done with body frame reader
    SafeRelease(m_pBodyFrameReader);

    // done with coordinate mapper
    SafeRelease(m_pCoordinateMapper);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CBodyBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"BodyBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        NULL,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CBodyBasics::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);

    // Main message loop
    while (WM_QUIT != msg.message)
    {
        Update();

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
void CBodyBasics::Update()
{
    if (!m_pBodyFrameReader)
    {
        return;
    }

    IBodyFrame* pBodyFrame = NULL;

    HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;

        hr = pBodyFrame->get_RelativeTime(&nTime);

        IBody* ppBodies[BODY_COUNT] = {0};

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
        }

        if (SUCCEEDED(hr))
        {
            ProcessBody(nTime, BODY_COUNT, ppBodies);
        }

        for (int i = 0; i < _countof(ppBodies); ++i)
        {
            SafeRelease(ppBodies[i]);
        }
    }

    SafeRelease(pBodyFrame);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CBodyBasics* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CBodyBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CBodyBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D
            D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

            // Get and initialize the default Kinect sensor
            InitializeDefaultSensor();
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;
    }

    return FALSE;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CBodyBasics::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get coordinate mapper and the body reader
        IBodyFrameSource* pBodyFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
        }

        SafeRelease(pBodyFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Handle new body data
/// <param name="nTime">timestamp of frame</param>
/// <param name="nBodyCount">body data count</param>
/// <param name="ppBodies">body data in frame</param>
/// </summary>
void CBodyBasics::ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies)
{
    if (m_hWnd)
    {
        HRESULT hr = EnsureDirect2DResources();

        if (SUCCEEDED(hr) && m_pRenderTarget && m_pCoordinateMapper)
        {
            m_pRenderTarget->BeginDraw();
            m_pRenderTarget->Clear();

            RECT rct;
            GetClientRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rct);
            int width = rct.right;
            int height = rct.bottom;

            for (int i = 0; i < nBodyCount; ++i)
            {
                IBody* pBody = ppBodies[i];
                if (pBody)
                {
                    BOOLEAN bTracked = false;
                    hr = pBody->get_IsTracked(&bTracked);

                    if (SUCCEEDED(hr) && bTracked)
                    {
                        Joint joints[JointType_Count]; 
                        D2D1_POINT_2F jointPoints[JointType_Count];

                        hr = pBody->GetJoints(_countof(joints), joints);
						bool trackedJoints[JointType_Count] = { 0 }; // Creates a boolean array of size JointType_Count that's all false


                        if (SUCCEEDED(hr))
                        {
                            for (int j = 0; j < _countof(joints); ++j)
                            {
								if (joints[j].TrackingState == TrackingState_Tracked) {
									if (joints[j].JointType == JointType_Head) { trackReps(joints[j], joints); }
									trackedJoints[j] = true;
								}
                                jointPoints[j] = BodyToScreen(joints[j].Position, width, height);
                            }
							
							// Only try to fix form if lifter is currently doing the exercise
							if (currRep == down) {
								// Tests go here:
								if ((trackedJoints[JointType_HandLeft] && trackedJoints[JointType_HandRight])) {
									//checkHeadPosition(joints, trackedJoints);
								}
								if ((trackedJoints[JointType_HandLeft] && trackedJoints[JointType_HandRight])) {
									updateHandPosition(joints, trackedJoints, initialHandLeftCM, initialHandRightCM);
								}
								if ((trackedJoints[JointType_KneeLeft] && trackedJoints[JointType_AnkleLeft]) 
									|| (trackedJoints[JointType_KneeRight] && trackedJoints[JointType_KneeRight])) {
									checkKnees(joints, trackedJoints);
								}
								if ((trackedJoints[JointType_KneeLeft] && trackedJoints[JointType_AnkleLeft] && trackedJoints[JointType_HipLeft])
									|| trackedJoints[JointType_KneeRight] && trackedJoints[JointType_AnkleRight] && trackedJoints[JointType_HipRight]) {
									updateSquatDepth(joints, trackedJoints);
								}
								
							}
								
							else {
								currLeftLegAngle = 180;
								currRightLegAngle = 180;
							}
                            DrawBody(joints, jointPoints);
							if (!audioToPlay.empty()) {
								playAudioFeedback();
							}
                        }
                    }
                }
            }

            hr = m_pRenderTarget->EndDraw();

            // Device lost, need to recreate the render target
            // We'll dispose it now and retry drawing
            if (D2DERR_RECREATE_TARGET == hr)
            {
                hr = S_OK;
                DiscardDirect2DResources();
            }
        }
    }
}

void CBodyBasics::trackReps(const Joint& head, Joint joints[JointType_Count])
{
	int headYinCM = head.Position.Y * 100;
	if (repCount == -1)
	{
		repCount++;
		upperRepThreshold = headYinCM;
		lowerRepThreshold += upperRepThreshold;
		//Gather starting data. Start with audio. Please get in starting position and wait for the tone to start exercising.
		if (joints[JointType_FootLeft].TrackingState == TrackingState_Tracked && joints[JointType_FootRight].TrackingState) {
			if (abs(joints[JointType_FootLeft].Position.X) < abs(joints[JointType_ShoulderLeft].Position.X) - .03) {
				repCount--;
				//feet not far enough apart.
				WCHAR szStatusMessage[120];
				TCHAR* headText = TEXT("Widen your stance");
				StringCchPrintf(szStatusMessage, _countof(szStatusMessage), headText);
				SetStatusMessage(szStatusMessage, 2000, true);
				return;
			}
		}
		else {
			repCount--;
			WCHAR szStatusMessage[120];
			TCHAR* headText = TEXT("Feet not in frame");
			StringCchPrintf(szStatusMessage, _countof(szStatusMessage), headText);
			SetStatusMessage(szStatusMessage, 2000, true);
			return;
		}
		if (joints[JointType_HandLeft].Position.Y < (joints[JointType_ShoulderLeft].Position.Y - .07)) {
			initialHandLeftCM = joints[JointType_HandLeft].Position.Z * 100;
			initialHandRightCM = joints[JointType_HandRight].Position.Z * 100;
			repCount--;
			WCHAR szStatusMessage[120];
			TCHAR* headText = TEXT("Get in proper starting position with the bar resting on your shoulders");
			StringCchPrintf(szStatusMessage, _countof(szStatusMessage), headText);
			SetStatusMessage(szStatusMessage, 2000, true);
			return;	
		}

		std::thread boardThread(board);
	}
	
	if (currRep == up && headYinCM < lowerRepThreshold)
	{
		currRep = down;
	}

	if (currRep == down && headYinCM > upperRepThreshold)
	{
		checkSquatDepth();
		repCount++;
		if ((repCount % 10 == 0) && (repCount != 0)) {
			audioToPlay.push("audio/good_job.wav");
		}

		currRep = up;
		
	}

	//WCHAR szStatusMessage[120];
	//LPCTSTR stringFormat = TEXT("%s %d");
	//TCHAR* headText = TEXT("Head Location: ");
	//TCHAR* repText = TEXT("Rep Count: ");
	//StringCchPrintf(szStatusMessage, _countof(szStatusMessage), stringFormat, repText, repCount);
	//SetStatusMessage(szStatusMessage, 33, false);
}

int CBodyBasics::board()
{
	Serial* SP = new Serial("...");    // adjust in library as needed

	//if (SP->IsConnected())
		//printf("We're connected");

	char incomingData[256] = "";			// don't forget to pre-allocate memory
	//printf("%s\n",incomingData);
	int dataLength = 255;
	int readResult = 0;





	while (true) {
		if (SP->IsConnected())
		{
			readResult = SP->ReadData(incomingData, dataLength);
			//printf("Bytes read: (0 means no data available) %i\n", readResult);
			incomingData[readResult] = 0;
			//printf("%s", incomingData);

			if (readResult == 18) {

				std::stringstream ss;
				ss << incomingData;
				std::string temp;
				int weights[4];
				for (int i = 0; i < 4; i++) {
					ss >> temp;
					if (std::stringstream(temp) >> weights[i])
						temp = "";
				}

				/*
				std::cout << "weight 1 = " << weights[0] << std::endl;
				std::cout << "weight 2 = " << weights[1] << std::endl;
				std::cout << "weight 3 = " << weights[2] << std::endl;
				std::cout << "weight 3 = " << weights[3] << std::endl;
				*/
				float lr = float(weights[0] + weights[3]) / float(weights[1] + weights[2]);
				float fb = float(weights[0] + weights[1]) / float(weights[2] + weights[3]);

				if (lr > 1.5) {

				WCHAR szStatusMessage[120];
				TCHAR* headText = TEXT("Shift your weight to the right");
				LPCTSTR pszFormat = TEXT("%s %d.");
				StringCchPrintf(szStatusMessage, _countof(szStatusMessage), headText);
				//SetStatusMessage(szStatusMessage, 2000, true);
				audioToPlay.push("audio/imbalanced_weight_right_leg.wav");

				}
				if (lr < .5) { 
					WCHAR szStatusMessage[120];
					TCHAR* headText = TEXT("Shift your weight to the left");
					LPCTSTR pszFormat = TEXT("%s %d.");
					StringCchPrintf(szStatusMessage, _countof(szStatusMessage), headText);
					//SetStatusMessage(szStatusMessage, 2000, true);
					audioToPlay.push("audio/imbalanced_weight_left_leg.wav");
				}
				/*if (fb > 1.5) {
					WCHAR szStatusMessage[120];
					TCHAR* headText = TEXT("Shift your weight back");
					LPCTSTR pszFormat = TEXT("%s %d.");
					StringCchPrintf(szStatusMessage, _countof(szStatusMessage), headText);
					//SetStatusMessage(szStatusMessage, 2000, true);
					audioToPlay.push("audio/shift_weight_back.wav");

				}
				if (fb < .5) {
					WCHAR szStatusMessage[120];
					TCHAR* headText = TEXT("Shift your weight forward");
					LPCTSTR pszFormat = TEXT("%s %d.");
					StringCchPrintf(szStatusMessage, _countof(szStatusMessage), headText);
					//SetStatusMessage(szStatusMessage, 2000, true);
					audioToPlay.push("audio/shift_weight_forward.wav");

				}*/

			}




		}


		Sleep(50);

	}

	return 0;
}
void CBodyBasics::checkHeadPosition(Joint joints[JointType_Count], bool trackedJoints[JointType_Count]) {
	//Not really working that well, frankly. 
	if (trackedJoints[JointType_SpineMid] && trackedJoints[JointType_Neck] && trackedJoints[JointType_Head]) {
		double SpineMidX = joints[JointType_SpineMid].Position.X;
		double NeckX = joints[JointType_Neck].Position.X;
		double HeadX = joints[JointType_Head].Position.X;

		double SpineMidY = joints[JointType_SpineMid].Position.Y;
		double NeckY = joints[JointType_Neck].Position.Y;
		double HeadY = joints[JointType_Head].Position.Y;

		double SpineMidZ = joints[JointType_SpineMid].Position.Z;
		double NeckZ = joints[JointType_Neck].Position.Z;
		double HeadZ = joints[JointType_Head].Position.Z;

		double vectorSpineNeckX = SpineMidX - NeckX;
		double vectorSpineNeckY = SpineMidY - NeckY;
		double vectorSpineNeckZ = SpineMidZ - NeckZ;

		double vectorNeckHeadX = HeadX - NeckX;
		double vectorNeckHeadY = HeadY - NeckY;
		double vectorNeckHeadZ = HeadZ - NeckZ;

		double dotSpineNeck = (vectorSpineNeckX * vectorNeckHeadX)
			+ (vectorSpineNeckY * vectorNeckHeadY)
			+ (vectorSpineNeckZ * vectorNeckHeadZ);

		double magSpineNeck = sqrt(pow(vectorSpineNeckX, 2)
			+ pow(vectorSpineNeckY, 2) + pow(vectorSpineNeckZ, 2));

		double magNeckHead = sqrt(pow(vectorNeckHeadX, 2)
			+ pow(vectorNeckHeadY, 2) + pow(vectorNeckHeadZ, 2));
			double backAngle = 200 - (180.0 / 3.1415) * acos(dotSpineNeck / (magSpineNeck * magNeckHead));
			if (backAngle > 15) {
				WCHAR szStatusMessage[120];
				TCHAR* headText = TEXT("Keep your head in neutral position.");
				LPCTSTR pszFormat = TEXT("%s %d.");
				StringCchPrintf(szStatusMessage, _countof(szStatusMessage), pszFormat, headText, backAngle);
				SetStatusMessage(szStatusMessage, 2000, true);
			}
		
	}
}
void CBodyBasics::updateHandPosition(Joint joints[JointType_Count], bool trackedJoints[JointType_Count], int initialHandLeftCM, int initialHandRightCM) {
	if (trackedJoints[JointType_HandLeft] && trackedJoints[JointType_HandRight]) {
		int handLeftCM = joints[JointType_HandLeft].Position.Z * 100;
		int handRightCM = joints[JointType_HandRight].Position.Z * 100;
		if (initialHandLeftCM == -1 || initialHandRightCM == -1) {
			//initial didn't update properly.
			return;
		}
		if ((initialHandLeftCM - handLeftCM) >= 15 || (initialHandRightCM - handRightCM) >= 15) {
			WCHAR szStatusMessage[120];
			TCHAR* headText = TEXT("Keep your spine in a neutral position.");
			StringCchPrintf(szStatusMessage, _countof(szStatusMessage), headText);
			SetStatusMessage(szStatusMessage, 2000, true);

			audioToPlay.push("audio/back_rounding.wav");
		}
	}
	return;
}
void CBodyBasics::checkKnees(Joint joints[JointType_Count], bool trackedJoints[JointType_Count])
{
	if (trackedJoints[JointType_KneeLeft] && trackedJoints[JointType_AnkleLeft]) {
		int kneeLeftCM = joints[JointType_KneeLeft].Position.X * 100;
		int ankleLeftCM = joints[JointType_AnkleLeft].Position.X * 100;
		if (abs(kneeLeftCM - ankleLeftCM) >= 8) {
			WCHAR szStatusMessage[120];
			TCHAR* headText = TEXT("Make sure your knees are over your feet.");
			StringCchPrintf(szStatusMessage, _countof(szStatusMessage), headText);
			SetStatusMessage(szStatusMessage, 2000, true);

			audioToPlay.push("audio/knees_over_feet.wav");
		}
	}
	else if (trackedJoints[JointType_KneeRight] && trackedJoints[JointType_AnkleRight]) {
		int kneeRightCM = joints[JointType_KneeRight].Position.X * 100;
		int ankleRightCM = joints[JointType_AnkleRight].Position.X * 100;
		if (abs(kneeRightCM - ankleRightCM) >= 8) {
			WCHAR szStatusMessage[120];
			TCHAR* headText = TEXT("Make sure your knees are over your feet.");
			StringCchPrintf(szStatusMessage, _countof(szStatusMessage), headText);
			SetStatusMessage(szStatusMessage, 2000, true);

			audioToPlay.push("audio/knees_over_feet.wav");
		}
	}
}

void CBodyBasics::updateSquatDepth(Joint joints[JointType_Count], bool trackedJoints[JointType_Count])
{
	if (trackedJoints[JointType_KneeLeft] && trackedJoints[JointType_AnkleLeft] && trackedJoints[JointType_HipLeft]) {
		double kneeLeftX = joints[JointType_KneeLeft].Position.X;
		double ankleLeftX = joints[JointType_AnkleLeft].Position.X;
		double hipLeftX = joints[JointType_HipLeft].Position.X;

		double kneeLeftY = joints[JointType_KneeLeft].Position.Y;
		double ankleLeftY = joints[JointType_AnkleLeft].Position.Y;
		double hipLeftY = joints[JointType_HipLeft].Position.Y;

		double kneeLeftZ = joints[JointType_KneeLeft].Position.Z;
		double ankleLeftZ = joints[JointType_AnkleLeft].Position.Z;
		double hipLeftZ = joints[JointType_HipLeft].Position.Z;

		double vectorAnkleKneeLeftX = kneeLeftX - ankleLeftX;
		double vectorAnkleKneeLeftY = kneeLeftY - ankleLeftY;
		double vectorAnkleKneeLeftZ = kneeLeftZ - ankleLeftZ;

		double vectorKneeHipLeftX = hipLeftX - kneeLeftX;
		double vectorKneeHipLeftY = hipLeftY - kneeLeftY;
		double vectorKneeHipLeftZ = hipLeftZ - kneeLeftZ;

		double dotAnkleKneeHip = vectorAnkleKneeLeftX * vectorKneeHipLeftX
			+ vectorAnkleKneeLeftY * vectorKneeHipLeftY
			+ vectorAnkleKneeLeftZ * vectorKneeHipLeftZ;

		double magAnkleKnee = sqrt(pow(vectorAnkleKneeLeftX, 2)
			+ pow(vectorAnkleKneeLeftY, 2) + pow(vectorAnkleKneeLeftZ, 2));

		double magKneeHip = sqrt(pow(vectorKneeHipLeftX, 2)
			+ pow(vectorKneeHipLeftY, 2) + pow(vectorKneeHipLeftZ, 2));

		double leftLegAngle = 200 - (180.0 / 3.1415) * acos(dotAnkleKneeHip / (magAnkleKnee * magKneeHip));
		if (leftLegAngle < currLeftLegAngle) {
			currLeftLegAngle = leftLegAngle;
		}
	}
	if (trackedJoints[JointType_KneeRight] && trackedJoints[JointType_AnkleRight] && trackedJoints[JointType_HipRight]) {
		double kneeRightX = joints[JointType_KneeRight].Position.X;
		double ankleRightX = joints[JointType_AnkleRight].Position.X;
		double hipRightX = joints[JointType_HipRight].Position.X;

		double kneeRightY = joints[JointType_KneeRight].Position.Y;
		double ankleRightY = joints[JointType_AnkleRight].Position.Y;
		double hipRightY = joints[JointType_HipRight].Position.Y;

		double kneeRightZ = joints[JointType_KneeRight].Position.Z;
		double ankleRightZ = joints[JointType_AnkleRight].Position.Z;
		double hipRightZ = joints[JointType_HipRight].Position.Z;

		double vectorAnkleKneeRightX = kneeRightX - ankleRightX;
		double vectorAnkleKneeRightY = kneeRightY - ankleRightY;
		double vectorAnkleKneeRightZ = kneeRightZ - ankleRightZ;

		double vectorKneeHipRightX = hipRightX - kneeRightX;
		double vectorKneeHipRightY = hipRightY - kneeRightY;
		double vectorKneeHipRightZ = hipRightZ - kneeRightZ;

		double dotAnkleKneeHip = (vectorAnkleKneeRightX * vectorKneeHipRightX)
			+ (vectorAnkleKneeRightY * vectorKneeHipRightY)
			+ (vectorAnkleKneeRightZ * vectorKneeHipRightZ);

		double magAnkleKnee = sqrt(pow(vectorAnkleKneeRightX, 2)
			+ pow(vectorAnkleKneeRightY, 2) + pow(vectorAnkleKneeRightZ, 2));

		double magKneeHip = sqrt(pow(vectorKneeHipRightX, 2)
			+ pow(vectorKneeHipRightY, 2) + pow(vectorKneeHipRightZ, 2));

		double rightLegAngle = 200 - (180.0 / 3.1415) * acos(dotAnkleKneeHip / (magAnkleKnee * magKneeHip));
		if (rightLegAngle < currRightLegAngle) {
			currRightLegAngle = rightLegAngle;
		}
	}
	//string s = to_string(currRightLegAngle);
	//wstring stemp = std::wstring(s.begin(), s.end());
	//LPCWSTR sw = stemp.c_str();
	//OutputDebugString(sw);
	//OutputDebugStringW(L"\n");
}

// Checks if squat depth is okay 
void CBodyBasics::checkSquatDepth() {
	if ((currRightLegAngle > 97) || (currLeftLegAngle > 97)) {
		WCHAR szStatusMessage[120];
		TCHAR* headText = TEXT("Try to squat deeper.");
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), headText);
		SetStatusMessage(szStatusMessage, 2000, true);

		audioToPlay.push("audio/try_to_squat_deeper.wav");
	}
	else if ((currRightLegAngle < 70) || (currLeftLegAngle < 70)) {

		WCHAR szStatusMessage[120];

		TCHAR* headText = TEXT("You squatted too deep.");
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), headText);
		SetStatusMessage(szStatusMessage, 2000, true);

		audioToPlay.push("audio/squatted_too_deep.wav");
	}
}

// Play the audio feedback
void CBodyBasics::playAudioFeedback() {
	if (!audioToPlay.empty()) {
		string audioFile = audioToPlay.front();
		wstring stemp = wstring(audioFile.begin(), audioFile.end());
		PlaySound(stemp.c_str(), NULL, SND_FILENAME | SND_ASYNC | SND_NOSTOP);
		audioToPlay.pop();
	}
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CBodyBasics::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    INT64 now = GetTickCount64();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }

    return false;
}

/// <summary>
/// Ensure necessary Direct2d resources are created
/// </summary>
/// <returns>S_OK if successful, otherwise an error code</returns>
HRESULT CBodyBasics::EnsureDirect2DResources()
{
    HRESULT hr = S_OK;

    if (m_pD2DFactory && !m_pRenderTarget)
    {
        RECT rc;
        GetWindowRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rc);  

        int width = rc.right - rc.left;
        int height = rc.bottom - rc.top;
        D2D1_SIZE_U size = D2D1::SizeU(width, height);
        D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
        rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
        rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

        // Create a Hwnd render target, in order to render to the window set in initialize
        hr = m_pD2DFactory->CreateHwndRenderTarget(
            rtProps,
            D2D1::HwndRenderTargetProperties(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), size),
            &m_pRenderTarget
        );

        if (FAILED(hr))
        {
            SetStatusMessage(L"Couldn't create Direct2D render target!", 10000, true);
            return hr;
        }

        // light green
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0.27f, 0.75f, 0.27f), &m_pBrushJointTracked);

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Yellow, 1.0f), &m_pBrushJointInferred);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 1.0f), &m_pBrushBoneTracked);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Gray, 1.0f), &m_pBrushBoneInferred);
    }

    return hr;
}

/// <summary>
/// Dispose Direct2d resources 
/// </summary>
void CBodyBasics::DiscardDirect2DResources()
{
    SafeRelease(m_pRenderTarget);

    SafeRelease(m_pBrushJointTracked);
    SafeRelease(m_pBrushJointInferred);
    SafeRelease(m_pBrushBoneTracked);
    SafeRelease(m_pBrushBoneInferred);
}

/// <summary>
/// Converts a body point to screen space
/// </summary>
/// <param name="bodyPoint">body point to tranform</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
/// <returns>point in screen-space</returns>
D2D1_POINT_2F CBodyBasics::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height)
{
    // Calculate the body's position on the screen
    DepthSpacePoint depthPoint = {0};
    m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

    float screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
    float screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;

    return D2D1::Point2F(screenPointX, screenPointY);
}

/// <summary>
/// Draws a body 
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
void CBodyBasics::DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints)
{
    // Draw the bones

    // Torso
    DrawBone(pJoints, pJointPoints, JointType_Head, JointType_Neck);
    DrawBone(pJoints, pJointPoints, JointType_Neck, JointType_SpineShoulder);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_SpineMid);
    DrawBone(pJoints, pJointPoints, JointType_SpineMid, JointType_SpineBase);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderRight);
    DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderLeft);
    DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipRight);
    DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipLeft);
    
    // Right Arm    
    DrawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_ElbowRight);
    DrawBone(pJoints, pJointPoints, JointType_ElbowRight, JointType_WristRight);
    DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_HandRight);
    DrawBone(pJoints, pJointPoints, JointType_HandRight, JointType_HandTipRight);
    DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_ThumbRight);

    // Left Arm
    DrawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_ElbowLeft);
    DrawBone(pJoints, pJointPoints, JointType_ElbowLeft, JointType_WristLeft);
    DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_HandLeft);
    DrawBone(pJoints, pJointPoints, JointType_HandLeft, JointType_HandTipLeft);
    DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_ThumbLeft);

    // Right Leg
    DrawBone(pJoints, pJointPoints, JointType_HipRight, JointType_KneeRight);
    DrawBone(pJoints, pJointPoints, JointType_KneeRight, JointType_AnkleRight);
    DrawBone(pJoints, pJointPoints, JointType_AnkleRight, JointType_FootRight);

    // Left Leg
    DrawBone(pJoints, pJointPoints, JointType_HipLeft, JointType_KneeLeft);
    DrawBone(pJoints, pJointPoints, JointType_KneeLeft, JointType_AnkleLeft);
    DrawBone(pJoints, pJointPoints, JointType_AnkleLeft, JointType_FootLeft);

    // Draw the joints
    for (int i = 0; i < JointType_Count; ++i)
    {
        D2D1_ELLIPSE ellipse = D2D1::Ellipse(pJointPoints[i], c_JointThickness, c_JointThickness);

        if (pJoints[i].TrackingState == TrackingState_Inferred)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointInferred);
        }
        else if (pJoints[i].TrackingState == TrackingState_Tracked)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointTracked);
        }
    }
}

/// <summary>
/// Draws one bone of a body (joint to joint)
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="joint0">one joint of the bone to draw</param>
/// <param name="joint1">other joint of the bone to draw</param>
void CBodyBasics::DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1)
{
    TrackingState joint0State = pJoints[joint0].TrackingState;
    TrackingState joint1State = pJoints[joint1].TrackingState;

    // If we can't find either of these joints, exit
    if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
    {
        return;
    }

    // Don't draw if both points are inferred
    if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
    {
        return;
    }

    // We assume all drawn bones are inferred unless BOTH joints are tracked
    if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
    {
        m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneTracked, c_TrackedBoneThickness);
    }
    else
    {
        m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneInferred, c_InferredBoneThickness);
    }
}