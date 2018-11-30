/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#ifndef _NITE_USER_VIEWER_H_
#define _NITE_USER_VIEWER_H_

#include "OpenNI.h"
#include "NiTE.h"


#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <string.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <ctime>


// Socket library


// Socket Server Header
//#include "ServerSocket.h"
#include "ServerSocketRunPID.h"
#include "PIDRun.h"
#include "VotingPID.h"

// // OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/imgproc/imgproc.hpp>

//?????
#include <stdio.h>

#define MAX_DEPTH 10000


using namespace std;
using namespace openni;


enum DisplayModes
{
	DISPLAY_MODE_OVERLAY,
	DISPLAY_MODE_DEPTH,
	DISPLAY_MODE_IMAGE
};

class EoyViewer
{
public:
	EoyViewer(const char* strSampleName, openni::Device& device, openni::VideoStream& depth, openni::VideoStream& color);
	virtual ~EoyViewer();

	virtual openni::Status Init(int argc, char **argv);
	virtual openni::Status Run();	//Does not return


protected:
	virtual void Display();
	virtual void DisplayPostDraw(){};	// Overload to draw over the screen image

	virtual void OnKey(unsigned char key, int x, int y);

	virtual openni::Status InitOpenGL(int argc, char **argv);
	void InitOpenGLHooks();

	void Finalize();

	// tingyaun
	openni::VideoFrameRef		m_depthFrame;
	openni::VideoFrameRef		m_colorFrame;

	//openni::device&			m_device;
	openni::Device&		m_device;
	openni::VideoStream&		m_depthStream;
	openni::VideoStream&		m_colorStream;
	openni::VideoStream**		m_streams;
	

private:
	EoyViewer(const EoyViewer&);
	EoyViewer& operator=(EoyViewer&);

	static EoyViewer* ms_self;
	static void glutIdle();
	static void glutDisplay();
	static void glutKeyboard(unsigned char key, int x, int y);

	float				m_pDepthHist[MAX_DEPTH];
	char			m_strSampleName[ONI_MAX_STR];
	
	unsigned int		m_nTexMapX;
	unsigned int		m_nTexMapY;

	// for openGl multi-stream
	DisplayModes		m_eViewState;
	openni::RGB888Pixel*	m_pTexMap;
	int			m_width;  // for OpenGL muti-streams
	int			m_height; // for OpenGL multi-streams

	nite::UserTracker* m_pUserTracker;

	nite::UserId m_poseUser;
	uint64_t m_poseTime;
};


#endif // _NITE_USER_VIEWER_H_
