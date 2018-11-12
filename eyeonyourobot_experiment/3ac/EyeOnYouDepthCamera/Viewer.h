/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

// Standard Library
#include "stdafx.h"

#pragma comment(lib, "ws2_32.lib") // link winsock2
#include <thread>
#include <windows.h>

#ifndef _NITE_USER_VIEWER_H_
#define _NITE_USER_VIEWER_H_

#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <string.h>
#include <cstring>
#include <cstdlib>

#include "RobotDrive.h"
#include "PIDRun.h"
#include "VotingPID.h"
#include "ServerSocket.h"
#include "ServerSocketRunPID.h"
#include "ColorMemory.h"

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "OpenNI.h"
#include "NiTE.h"

#define MAX_DEPTH 10000

using namespace std;
using namespace openni;

enum DisplayModes
{
	DISPLAY_MODE_OVERLAY,
	DISPLAY_MODE_DEPTH,
	DISPLAY_MODE_IMAGE
};

class SampleViewer
{
public:
	//SampleViewer(const char* strSampleName);
	SampleViewer(const char* strSampleName, openni::Device& device, openni::VideoStream& depth, openni::VideoStream& color);
	virtual ~SampleViewer();

	virtual openni::Status Init(int argc, char **argv);
	virtual openni::Status Run();	//Does not return

protected:
	virtual void Display();
	virtual void DisplayPostDraw(){};	// Overload to draw over the screen image

	virtual void OnKey(unsigned char key, int x, int y);

	virtual openni::Status InitOpenGL(int argc, char **argv);
	void InitOpenGLHooks();

	void Finalize();

	// richardyctsai
	openni::VideoFrameRef		m_depthFrame;
	openni::VideoFrameRef		m_colorFrame;

	//openni::Device&			m_device;
	openni::VideoStream&			m_depthStream;
	openni::VideoStream&			m_colorStream;
	openni::VideoStream**		m_streams;
	
private:
	SampleViewer(const SampleViewer&);
	SampleViewer& operator=(SampleViewer&);

	static SampleViewer* ms_self;
	static void glutIdle();
	static void glutDisplay();
	static void glutKeyboard(unsigned char key, int x, int y);

	float				m_pDepthHist[MAX_DEPTH];
	char				m_strSampleName[ONI_MAX_STR];

	unsigned int		m_nTexMapX;
	unsigned int		m_nTexMapY;
	DisplayModes		m_eViewState;
	openni::RGB888Pixel*	m_pTexMap;
	int			m_width;
	int			m_height;

	openni::Device&		m_device;
	nite::UserTracker* m_pUserTracker;

	nite::UserId m_poseUser;
	uint64_t m_poseTime;
};


#endif // _NITE_USER_VIEWER_H_
