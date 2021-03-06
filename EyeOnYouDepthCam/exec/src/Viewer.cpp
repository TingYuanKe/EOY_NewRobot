/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/
#if (defined _WIN32)
#define PRIu64 "llu"
#else
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#endif

#include "Viewer.h"
//#include "robot_control.h"
#include "robot_control_turtlebot.h"
#include "OpenNI.h"
#include "NiTE.h"

#if (ONI_PLATFORM == ONI_PLATFORM_MACOSX)
        #include <GLUT/glut.h>
#else
        #include <GL/glut.h>
#endif

#include <NiteSampleUtilities.h>

using namespace std;

#define GL_WIN_SIZE_X	1280
#define GL_WIN_SIZE_Y	1024
#define TEXTURE_SIZE	512

#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

EoyViewer* EoyViewer::ms_self = NULL;

bool g_drawSkeleton = true;
bool g_drawCenterOfMass = false;
bool g_drawStatusLabel = true;
bool g_drawBoundingBox = false;
bool g_drawBackground = true;
bool g_drawDepth = true;
bool g_drawFrameId = true;
bool g_runRobotTracking = true;
bool g_robotArmMode = false;

int g_nXRes = 0, g_nYRes = 0;

// time to hold in pose to exit program. In milliseconds.
const int g_poseTimeoutToExit = 2000;

// TingYuan: global time
char time_str[13];

// TingYuan: File output stream
ofstream csvfile;
string csvfilename = "/home/newrobot/data/SkeletonData/VSFile_Buffer.csv";

// ***************Read id and pairing result**********************
ifstream resultfile;
string resultfilename = "/home/newrobot/data/SkeletonData/result.csv";
string line;
int confidenceOfResult = -1;

char s_FollowingTarget[30] = "chwang"; //host name
int i_FollowingTarget = -1;
int LastMovingAction = 0; // 0: stop, 1: forward, 2: backward, 3: turn right, 4: turn left, 5:spin right, 6:spin left

bool b_StopRobotTracking = true; 

const int RobotVelocity = 2400;
const int SPEED_LIMIT = 2400;
const int ROTATE_LIMIT = 1100;
const int INTERVAL_VELOCITY_PLUS = 200;
const int INTERVAL_VELOCITY_MINUS = 400;
const int SPEED_WHEEL_DIFF = SPEED_LIMIT * 0.4;

double lastRoll = 0;
double lastPitch = 0;
double lastYaw = 0;

int RobotRGBImageCount = 0;
clock_t previousTime;
double wantedDuration = 1;
double duration;


typedef struct tm SYSTEMTIME;
void GetLocalTime(SYSTEMTIME *st);
SYSTEMTIME st;

	
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

//*********************************************************

void EoyViewer::glutIdle()
{
	glutPostRedisplay();
}
void EoyViewer::glutDisplay()
{
	EoyViewer::ms_self->Display();
}
void EoyViewer::glutKeyboard(unsigned char key, int x, int y)
{
	EoyViewer::ms_self->OnKey(key, x, y);
}

EoyViewer::EoyViewer(const char* strSampleName, openni::Device& device, openni::VideoStream& depth, openni::VideoStream& color) :
	m_device(device), m_depthStream(depth), m_colorStream(color), m_streams(NULL), m_eViewState(DEFAULT_DISPLAY_MODE), m_pTexMap(NULL)
{
	ms_self = this;
	strncpy(m_strSampleName, strSampleName, ONI_MAX_STR);
	m_pUserTracker = new nite::UserTracker;
}
EoyViewer::~EoyViewer()
{
	Finalize();

	delete[] m_pTexMap;

	ms_self = NULL;
}


void EoyViewer::Finalize()
{
	delete m_pUserTracker;
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
}


void rosInit(int argc, char** argv){
	rosInit(argc, argv, "newrobot");
	//armUp();
	cout << "Successfully connect to the Robot";
}


void rosFinalize() {
	stopMotion();

	//if (g_robotArmMode)
	//	armDown();

	cout << "Stop connect to the Robot";
}


openni::Status EoyViewer::Init(int argc, char **argv)
{	
	previousTime = clock();


	// init 2 video streams
	openni::VideoMode depthVideoMode;
	openni::VideoMode colorVideoMode;

	m_pTexMap = NULL;

	openni::Status rc = openni::OpenNI::initialize();

	if (m_depthStream.isValid() && m_colorStream.isValid())
	{
		depthVideoMode = m_depthStream.getVideoMode();
		colorVideoMode = m_colorStream.getVideoMode();

		int depthWidth = depthVideoMode.getResolutionX();
		int depthHeight = depthVideoMode.getResolutionY();
		int colorWidth = colorVideoMode.getResolutionX();
		int colorHeight = colorVideoMode.getResolutionY();

		if (depthWidth == colorWidth &&
			depthHeight == colorHeight)
		{
			m_width = depthWidth;
			m_height = depthHeight;
		}
		else
		{
			printf("Error - expect color and depth to be in same resolution: D: %dx%d, C: %dx%d\n",
				depthWidth, depthHeight,
				colorWidth, colorHeight);
			return openni::STATUS_ERROR;
		}
	}
	else if (m_depthStream.isValid())
	{
		depthVideoMode = m_depthStream.getVideoMode();
		m_width = depthVideoMode.getResolutionX();
		m_height = depthVideoMode.getResolutionY();
	}
	else if (m_colorStream.isValid())
	{
		colorVideoMode = m_colorStream.getVideoMode();
		m_width = colorVideoMode.getResolutionX();
		m_height = colorVideoMode.getResolutionY();
	}
	else
	{
		printf("Error - expects at least one of the streams to be valid...\n");
		return openni::STATUS_ERROR;
	}


	if (rc != openni::STATUS_OK)
	{
		printf("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError());
		return rc;
	}

	const char* deviceUri = openni::ANY_DEVICE;
	for (int i = 1; i < argc-1; ++i)
	{
		if (strcmp(argv[i], "-device") == 0)
		{
			deviceUri = argv[i+1];
			break;
		}
	}

	rc = m_device.open(deviceUri);
	if (rc != openni::STATUS_OK)
	{
		printf("Failed to open device\n%s\n", openni::OpenNI::getExtendedError());
		return rc;
	}
	printf("RGB Init Succesllfuly\n", openni::OpenNI::getExtendedError());
	m_streams = new openni::VideoStream*[2];
	m_streams[0] = &m_depthStream;
	m_streams[1] = &m_colorStream;

	// Texture map init
	m_nTexMapX = MIN_CHUNKS_SIZE(m_width, TEXTURE_SIZE);
	m_nTexMapY = MIN_CHUNKS_SIZE(m_height, TEXTURE_SIZE);
	m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];


	nite::NiTE::initialize();

	if (m_pUserTracker->create(&m_device) != nite::STATUS_OK)
	{
		return openni::STATUS_ERROR;
	}


	return InitOpenGL(argc, argv);

}


openni::Status EoyViewer::Run()	//Does not return
{
	glutMainLoop();

	return openni::STATUS_OK;
}


float Colors[][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
int colorCount = 3;

#define MAX_USERS 15
#define NAME_LEN 100
#define HIST_LEN 125

bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
char g_userStatusLabels[MAX_USERS][100] = {{0}};
char g_generalMessage[512] = {0};



// for PID memory
char g_userNameLabels[MAX_USERS][NAME_LEN] = {{0}};
char g_userNameLabelsByHist[MAX_USERS][NAME_LEN] = {{0}};
char g_userColorNameLabels[MAX_USERS][NAME_LEN] = {{0}};
bool g_userNameConfidence[MAX_USERS] = {false};
int g_userColorHistogramLabels[MAX_USERS][HIST_LEN] = {0};
int g_userTagID[MAX_USERS];

int g_userUpdateCount[MAX_USERS] = {0};

// clean all g_user variable array 
void cleanUserBuffer (int idx) {
	for (int i = 0; i < NAME_LEN; i++){
		g_userNameLabels[idx][i] = 0;
		g_userNameLabelsByHist[idx][i] = 0;
		g_userColorNameLabels[idx][i] = 0;
	}

	for (int i = 0; i < HIST_LEN; i++){
		g_userColorHistogramLabels[idx][i] = 0;

	}
	g_userNameConfidence[idx] = false;
	g_userUpdateCount[idx] = 0;

	cout <<"=======Clean user" << idx << "buffer=======" << endl;


}

#define USER_MESSAGE(msg) {\
	sprintf(g_userStatusLabels[user.getId()], "%s", msg);\
	printf("[%08" PRIu64 "] User #%d:\t%s\n", ts, user.getId(), msg);}


void updateUserState(const nite::UserData& user, uint64_t ts)
{	
	if (user.getCenterOfMass().z >3000) {
		//cout << "dis:" << user.getCenterOfMass().z <<endl;
		return;
	}

	if (user.isNew())
	{
		USER_MESSAGE("New, initial memory buffer");
	}
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		printf("[%08" PRIu64 "] User #%d:\tVisible\n", ts, user.getId());
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		printf("[%08" PRIu64 "] User #%d:\tOut of Scene\n", ts, user.getId());
	else if (user.isLost())
	{
		USER_MESSAGE("Lost");


	}
	g_visibleUsers[user.getId()] = user.isVisible();


	if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
			break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Skeleton Tracking!")
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			break;
		}
	}
}


#ifndef USE_GLES
void glPrintString(void *font, const char *str)
{
	int i,l = (int)strlen(str);

	for(i=0; i<l; i++)
	{   
		glutBitmapCharacter(font,*str++);
	}   
}
#endif


void DrawStatusLabel(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
	int color = user.getId() % colorCount;
	glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);

	float x,y;
	pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &x, &y);
	x *= GL_WIN_SIZE_X/(float)g_nXRes;
	y *= GL_WIN_SIZE_Y/(float)g_nYRes;
	char *msg = g_userStatusLabels[user.getId()];
	glRasterPos2i(x-((strlen(msg)/2)*8),y);
	glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
}


void DrawFrameId(int frameId)
{
	char buffer[80] = "";
	sprintf(buffer, "%d", frameId);
	glColor3f(1.0f, 0.0f, 0.0f);
	glRasterPos2i(20, 20);
	glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);
}

void DrawMode(char* mode) 
{
	char buffer[80] = "";
	sprintf(buffer, "%s", mode);
	glColor3f(0.2f, 1.0f, 0.2f);
	//TODO
	glRasterPos2i(20, 60);
	glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);
}


void DrawCenterOfMass(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
	glColor3f(1.0f, 1.0f, 1.0f);

	float coordinates[3] = {0};

	pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &coordinates[0], &coordinates[1]);


	coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;
	glPointSize(8);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_POINTS, 0, 1);
}


void DrawBoundingBox(const nite::UserData& user)
{
	glColor3f(1.0f, 1.0f, 1.0f);

	float coordinates[] =
	{
		user.getBoundingBox().max.x, user.getBoundingBox().max.y, 0,
		user.getBoundingBox().max.x, user.getBoundingBox().min.y, 0,
		user.getBoundingBox().min.x, user.getBoundingBox().min.y, 0,
		user.getBoundingBox().min.x, user.getBoundingBox().max.y, 0,
	};
	coordinates[0]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[3]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[4]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[6]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[7]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[9]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[10] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	glPointSize(2);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_LINE_LOOP, 0, 4);

}



void DrawLimb(nite::UserTracker* pUserTracker, const nite::SkeletonJoint& joint1, const nite::SkeletonJoint& joint2, int color)
{
	float coordinates[6] = {0};
	pUserTracker->convertJointCoordinatesToDepth(joint1.getPosition().x, joint1.getPosition().y, joint1.getPosition().z, &coordinates[0], &coordinates[1]);
	pUserTracker->convertJointCoordinatesToDepth(joint2.getPosition().x, joint2.getPosition().y, joint2.getPosition().z, &coordinates[3], &coordinates[4]);

	coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[3] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[4] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	if (joint1.getPositionConfidence() == 1 && joint2.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else if (joint1.getPositionConfidence() < 0.5f || joint2.getPositionConfidence() < 0.5f)
	{
		return;
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glPointSize(2);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_LINES, 0, 2);

	glPointSize(10);
	if (joint1.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_POINTS, 0, 1);

	if (joint2.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glVertexPointer(3, GL_FLOAT, 0, coordinates+3);
	glDrawArrays(GL_POINTS, 0, 1);
}


void DrawSkeleton(nite::UserTracker* pUserTracker, const nite::UserData& userData)
{
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_HEAD), userData.getSkeleton().getJoint(nite::JOINT_NECK), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HAND), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT), userData.getId() % colorCount);
}


void GetLocalTime(SYSTEMTIME *st) {
    if (st)
    {
        struct tm *pst = NULL;
        time_t t = time(NULL);
        pst = localtime(&t);
        memcpy(st,pst,sizeof(SYSTEMTIME));
        st->tm_year += 1900;
    }
}


void DrawSystemTime()
{
	SYSTEMTIME st;
	GetLocalTime(&st);
	memset(time_str, '\0', 13);
	snprintf(time_str, 9, "%02d:%02d:%02d", st.tm_hour, st.tm_min, st.tm_sec);

	char buffer[80] = "";
	sprintf(buffer, "%s", time_str);
	glColor3f(0.0f, 0.0f, 1.0f);
	glRasterPos2i(20, 40);
	glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);

}


void toEulerAngle(float w, float x, float y, float z, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (w * x + y * z);
	double cosr = +1.0 - 2.0 * (x * x + y * y);
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (w * y - z * x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (w * z + x * y);
	double cosy = +1.0 - 2.0 * (y * y + z * z);
	yaw = atan2(siny, cosy);
}


void WriteSkeletonInfo(int ID, ofstream& csvout, const nite::UserData& userData, char* time_str)
{
	const nite::SkeletonJoint& WH_JointPos = userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND);

	const nite::SkeletonJoint& rJoint = userData.getSkeleton().getJoint(nite::JOINT_TORSO);
	const nite::Quaternion& tr = rJoint.getOrientation();
	double roll = 0;
	double pitch = 0;
	double yaw = 0;
	toEulerAngle(tr.w, tr.x, tr.y, tr.z, roll, pitch, yaw);
	if (roll < 0.02)
		roll = 0;
	if (pitch < 0.02)
		pitch = 0;
	if (yaw < 0.02)
		yaw = 0;

	//cout << roll << "  " << pitch  << "   " << yaw  << endl;
	csvout << WH_JointPos.getPosition().x / 1000.0 << "," << WH_JointPos.getPosition().y / 1000.0 << "," << WH_JointPos.getPosition().z / 1000.0 << ","
		<< roll << "," << pitch << "," << yaw << ","
		<< ID << "," << time_str << "\n";

	lastRoll = roll;
	lastPitch = pitch;
	lastYaw = yaw;
}


void StopRobotTracking_2() {
	stopMotion();
	LastMovingAction = 0;
}



void RunRobotTracking(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
	float thresholdMaxZ = 1600.0;
	float thresholdMinZ = 1350.0;

	float thresholdMaxX;
	float thresholdMinX;

	glColor3f(1.0f, 1.0f, 1.0f);

	float coordinates[3] = { 0 };
	coordinates[0] =  user.getCenterOfMass().x;
	coordinates[1] =  user.getCenterOfMass().y;
	coordinates[2] =  user.getCenterOfMass().z;

	pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &coordinates[0], &coordinates[1]);
		
		if (coordinates[2] < thresholdMaxZ) {
		thresholdMaxX = 210.0;
		thresholdMinX = 140.0;
	}
	else if (coordinates[2] >= thresholdMaxZ && coordinates[2] <= 2700.0) {
		thresholdMaxX = coordinates[2] * 0.085 + 65.5;
		thresholdMinX = coordinates[2] * (-0.085) + 284.5;
	}
	else if(coordinates[2] > 2700.0) {
		thresholdMaxX = 295.0;
		thresholdMinX =  75.0;
	}


	//host in the center
	if (coordinates[0] <= thresholdMaxX && coordinates[0] >= thresholdMinX) {
		//host in the range
		if (coordinates[2] <= thresholdMaxZ && coordinates[2] >= thresholdMinZ && LastMovingAction != 0) {
			b_StopRobotTracking = true;
			StopRobotTracking_2();
		}
		//host far away from iRobot in Z-Dim
		else if (coordinates[2] > thresholdMaxZ  && LastMovingAction != 1) {
			LastMovingAction = 1;
			//setSpeed(RobotVelocity, RobotVelocity);
			goForward();
		}
		//host cloesd to iRobot in Z-Dim
		else if (coordinates[2] < thresholdMinZ  && LastMovingAction != 2) {
			LastMovingAction = 2;
			//setSpeed(RobotVelocity, RobotVelocity);
			goBack();
		}
	}
	//host in the right viewing field of iRobot 
	else if (coordinates[0] < thresholdMinX) {
		if (coordinates[2] <= thresholdMaxZ && LastMovingAction != 5) {
				LastMovingAction = 5;
				//setSpeed(ROTATE_LIMIT, ROTATE_LIMIT);
				spinRight();		
				//cout << "spin right!!!!" <<endl;
			
		}
		//host far away from iRobot (turn right)
		else if (coordinates[2] > thresholdMaxZ && LastMovingAction != 3) {
			LastMovingAction = 3;
			//turnLeftRight(RobotVelocity + RobotVelocity *0.25, RobotVelocity-SPEED_WHEEL_DIFF);
			turnLeftRight(false);
			//cout << "turn right!!" <<endl;
		}
	}
	////host in the left viewing field of iRobot
	else if (coordinates[0] > thresholdMaxX) {
		if (coordinates[2] <= thresholdMaxZ && LastMovingAction != 6) {
				LastMovingAction = 6;
				//setSpeed(ROTATE_LIMIT, ROTATE_LIMIT);
				spinLeft();
			}
		
		else if (coordinates[2] > thresholdMaxZ && LastMovingAction != 4) {
			LastMovingAction = 4;
			//turnLeftRight(RobotVelocity - SPEED_WHEEL_DIFF, RobotVelocity + RobotVelocity *0.25);
			turnLeftRight(left);
		}
	}
}


void GetResultOfPID()
{
	// 1. csvfile.close();
	if (PIDRun::getKeepSkeleton() == true)
	{
		// Save buffer file
		csvfile.close();
		// Create new file and from vsfile_Buffer.csv
		ifstream fin(csvfilename);
		string csvfilenamepairing = "/home/newrobot/data/SkeletonData//VSFile.csv";
		ofstream fout(csvfilenamepairing);
		string line;
		while (getline(fin, line)) fout << line << '\n';
		//cout << "buffer complete!!!!!!!!!!!!!" << endl;

		PIDRun::setKeepSkeleton(false);
		PIDRun::setExecutePID(true);
		
		csvfile.open(csvfilename);
	}

	// Read result.csv to tag profile on top of the head
	if (PIDRun::getTagProfile() == true) {
		resultfile.open(resultfilename);
		if (resultfile.is_open()) {
			// read ID and position of head joint and then putText
			getline(resultfile, line, '\n');
			resultfile.close();
		}

		istringstream templine(line);
		string data, nameReadFile;

		double x, y;
		int idx = 0;
		memset(g_userTagID, 0, sizeof(g_userTagID));

		while (getline(templine, data, ',')) {
			if (idx == 0) {
				if (data.compare("1") == 0)
					confidenceOfResult = 1;
				else if (data.compare("0") == 0)
					confidenceOfResult = 0;

				//confidenceOfResult = 1;
				cout << "data.c_str(): " << data.c_str() << "  confidenceOfResult: " << confidenceOfResult << endl;
			}
			if (idx != 0 && idx % 2 == 1) {
				//if(confidenceOfResult)
					VotingPID::setID(data.c_str());
				
				cout << "id: " << data.c_str();

				int idx = stoi(data.c_str());
				g_userTagID[idx] = 1;
			}
			else if (idx != 0 && idx % 2 == 0) {
				nameReadFile = data.c_str();
				//if (confidenceOfResult)
					VotingPID::setnameVotingWithIndex(VotingPID::getID(), VotingPID::votingOfPID(VotingPID::getID(), nameReadFile));
				cout << ", name: " << data.c_str() << endl;
			}
			idx += 1;
		}
		PIDRun::setTagProfile(false);
	}
}



//char g_generalMessage[100] = { 0 };


#define USER_COLOR(msg) {\
	sprintf(g_userColorNameLabels[userData.getId()], "%s", msg);}

#define USER_CONFIDENCE(msg) {\
	g_userNameConfidence[userData.getId()] = msg;}

#define USER_NAME(msg) {\
	sprintf(g_userNameLabels[userData.getId()], "%s", msg);}

#define USER_NAME_HIST(msg) {\
	sprintf(g_userNameLabelsByHist[userData.getId()], "%s", msg);}



void updateIdentity(const char *NAME, const bool CONFIDENCE, openni::VideoFrameRef arg_m_colorFrame, nite::UserTracker* pUserTracker, const nite::UserData& userData, uint64_t ts)
{	
	// combine string : ID : name
	char str[80];
	sprintf(str, "%d", userData.getId());
	strcat(str, ": ");
	strcat(str, NAME);

	g_userUpdateCount[userData.getId()] ++;
	
	USER_NAME(str);
	USER_CONFIDENCE(CONFIDENCE);

	// richardyctsai
	// get host ID from host name
	if (g_userNameConfidence[userData.getId()] == true) {
		if (strcmp(NAME, s_FollowingTarget) == 0) {
			i_FollowingTarget = userData.getId();
		}
		USER_NAME(str);
	}
	else {
		if (strcmp(NAME, s_FollowingTarget) == 0) {
			i_FollowingTarget = userData.getId();
		}
		strcat(str, "\n(Hist)");
		USER_NAME_HIST(str);
	}
}


void DrawIdentity(nite::UserTracker* pUserTracker, const nite::UserData& userData)
{
	
	int color = userData.getId() % colorCount;
	glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);

	const nite::SkeletonJoint& jointHead = userData.getSkeleton().getJoint(nite::JOINT_HEAD);
	
	float x, y;
	pUserTracker->convertJointCoordinatesToDepth(userData.getCenterOfMass().x, userData.getCenterOfMass().y, userData.getCenterOfMass().z, &x, &y);
	x *= GL_WIN_SIZE_X / (float)g_nXRes;
	y *= GL_WIN_SIZE_Y / (float)g_nYRes;

	char *msg = g_userNameLabels[userData.getId()];
	glRasterPos2i(x - ((strlen(msg) / 2) * 8), y-80);

	// Tag unknown or PID name
	if (userData.getCenterOfMass().z >3300){
		const char* unknow_identity = "x";
		glPrintString(GLUT_BITMAP_TIMES_ROMAN_24, unknow_identity);
	}
	else if (g_userUpdateCount[userData.getId()] < 1) { 
		const char* unknow_identity = "unknown";
		glPrintString(GLUT_BITMAP_TIMES_ROMAN_24, unknow_identity);
	}
	else {
		glPrintString(GLUT_BITMAP_TIMES_ROMAN_24, msg);
		
	}	

}


void DrawIdentityByHist(openni::VideoFrameRef arg_m_colorFrame, nite::UserTracker* pUserTracker, const nite::UserData& userData)
{
	int color = userData.getId() % colorCount;
	glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);

	const nite::SkeletonJoint& jointHead = userData.getSkeleton().getJoint(nite::JOINT_HEAD);

	float x, y;
	pUserTracker->convertJointCoordinatesToDepth(userData.getCenterOfMass().x, userData.getCenterOfMass().y, userData.getCenterOfMass().z, &x, &y);
	x *= GL_WIN_SIZE_X / (float)g_nXRes;
	y *= GL_WIN_SIZE_Y / (float)g_nYRes;
	char *msg = g_userNameLabelsByHist[userData.getId()];
	glRasterPos2i(x - ((strlen(msg) / 2) * 8), y - 80);

	//Tag unknown or PID name
	if (userData.getCenterOfMass().z >3300){
		const char* unknow_identity = "x";
		glPrintString(GLUT_BITMAP_TIMES_ROMAN_24, unknow_identity);
	}
	else if (g_userUpdateCount[userData.getId()] < 1) { 
		const char* unknow_identity = "unknown";
		glPrintString(GLUT_BITMAP_TIMES_ROMAN_24, unknow_identity);
	}
	else {
		glPrintString(GLUT_BITMAP_TIMES_ROMAN_24, msg);
		
	}	

}


void DrawUserColor(openni::VideoFrameRef arg_m_colorFrame, nite::UserTracker* pUserTracker, const nite::UserData& userData, uint64_t ts)
{
	// richardyctsai
	float clothPosX, clothPosY;
	pUserTracker->convertJointCoordinatesToDepth(userData.getCenterOfMass().x, userData.getCenterOfMass().y, userData.getCenterOfMass().z, &clothPosX, &clothPosY);

	const nite::SkeletonJoint& jointLK = userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE);
	float PantsPosX, PantsPosY;
	pUserTracker->convertJointCoordinatesToDepth(jointLK.getPosition().x, jointLK.getPosition().y, jointLK.getPosition().z, &PantsPosX, &PantsPosY);

	const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)arg_m_colorFrame.getData();
	int idx = ((arg_m_colorFrame.getWidth() * (int)clothPosY + 1) + (int)clothPosX);
	if (idx < 0)
		idx = 0;
	
	int r = pImageRow[idx].r;
	int g = pImageRow[idx].g;
	int b = pImageRow[idx].b;

	ColorMemory::setRGB(r, g, b);
	float* hsl = ColorMemory::RGBToHSL();

	char str[80];
	sprintf(str, "%d", userData.getId());
	strcat(str, ": ");
	strcat(str, "Your Cloth's Color is ");
	strcat(str, ColorMemory::ColorClassification().c_str());
	USER_COLOR(str);


	int color = userData.getId() % colorCount;
	glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);

	const nite::SkeletonJoint& jointHead = userData.getSkeleton().getJoint(nite::JOINT_HEAD);

	float x, y;
	pUserTracker->convertJointCoordinatesToDepth(jointHead.getPosition().x, jointHead.getPosition().y, jointHead.getPosition().z, &x, &y);
	x *= GL_WIN_SIZE_X / (float)g_nXRes;
	y *= GL_WIN_SIZE_Y / (float)g_nYRes;
	char *msg = g_userColorNameLabels[userData.getId()];
	glRasterPos2i(x - ((strlen(msg) / 2) * 8), y + 100);
	glPrintString(GLUT_BITMAP_TIMES_ROMAN_24, msg);
}


void EoyViewer::Display()
{	
	// tingyuanke
	int changedIndex ;
	openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
	if (rc != openni::STATUS_OK)
	{
		printf("Wait failed\n");
		return;
	}

	switch (changedIndex)
	{
	case 0:
		m_depthStream.readFrame(&m_depthFrame); break;
	case 1:
		m_colorStream.readFrame(&m_colorFrame); break;
	default:
		printf("Error in wait\n");
	}

	nite::UserTrackerFrameRef userTrackerFrame;
	openni::VideoFrameRef depthFrame;

	nite::Status rcUserTracker = m_pUserTracker->readFrame(&userTrackerFrame);
	if (rcUserTracker != nite::STATUS_OK)
	{
		printf("GetNextData failed\n");
		return;
	}

	depthFrame = userTrackerFrame.getDepthFrame();

	if (m_pTexMap == NULL)
	{
		// Texture map init
		m_nTexMapX = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionX(), TEXTURE_SIZE);
		m_nTexMapY = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionY(), TEXTURE_SIZE);
		m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];
	}

	const nite::UserMap& userLabels = userTrackerFrame.getUserMap();

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);

	if (depthFrame.isValid() && g_drawDepth)
	{
		calculateHistogram(m_pDepthHist, MAX_DEPTH, depthFrame);
	}

	memset(m_pTexMap, 0, m_nTexMapX*m_nTexMapY*sizeof(openni::RGB888Pixel));

	// richardyctsai
	// check if we need to draw image frame to texture
	if ((m_eViewState == DISPLAY_MODE_OVERLAY ||
		m_eViewState == DISPLAY_MODE_IMAGE) && m_colorFrame.isValid())
	{
		const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)m_colorFrame.getData();
		openni::RGB888Pixel* pTexRow = m_pTexMap + m_colorFrame.getCropOriginY() * m_nTexMapX;
		int rowSize = m_colorFrame.getStrideInBytes() / sizeof(openni::RGB888Pixel);

		for (int y = 0; y < m_colorFrame.getHeight(); ++y)
		{
			const openni::RGB888Pixel* pImage = pImageRow;
			openni::RGB888Pixel* pTex = pTexRow + m_colorFrame.getCropOriginX();

			for (int x = 0; x < m_colorFrame.getWidth(); ++x, ++pImage, ++pTex)
			{
				*pTex = *pImage;
			}

			pImageRow += rowSize;
			pTexRow += m_nTexMapX;
		}
	}

	float factor[3] = {1, 1, 1};
	// check if we need to draw depth frame to texture
	if (depthFrame.isValid() && g_drawDepth)
	{
		const nite::UserId* pLabels = userLabels.getPixels();

		const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame.getData();
		openni::RGB888Pixel* pTexRow = m_pTexMap + depthFrame.getCropOriginY() * m_nTexMapX;
		int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

		for (int y = 0; y < depthFrame.getHeight(); ++y)
		{
			const openni::DepthPixel* pDepth = pDepthRow;
			openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();

			for (int x = 0; x < depthFrame.getWidth(); ++x, ++pDepth, ++pTex, ++pLabels)
			{
				if (*pDepth != 0)
				{
					if (*pLabels == 0)
					{
						if (!g_drawBackground)
						{
							factor[0] = factor[1] = factor[2] = 0;

						}
						else
						{
							factor[0] = Colors[colorCount][0];
							factor[1] = Colors[colorCount][1];
							factor[2] = Colors[colorCount][2];
						}
					}
					else
					{
						factor[0] = Colors[*pLabels % colorCount][0];
						factor[1] = Colors[*pLabels % colorCount][1];
						factor[2] = Colors[*pLabels % colorCount][2];
					}
//					// Add debug lines - every 10cm
// 					else if ((*pDepth / 10) % 10 == 0)
// 					{
// 						factor[0] = factor[2] = 0;
// 					}

					int nHistValue = m_pDepthHist[*pDepth];
					pTex->r = nHistValue*factor[0];
					pTex->g = nHistValue*factor[1];
					pTex->b = nHistValue*factor[2];

					factor[0] = factor[1] = factor[2] = 1;
				}
			}

			pDepthRow += rowSize;
			pTexRow += m_nTexMapX;
		}
	}

	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_nTexMapX, m_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, m_pTexMap);

	// Display the OpenGL texture map
	glColor4f(1,1,1,1);

	glEnable(GL_TEXTURE_2D);
	glBegin(GL_QUADS);

	g_nXRes = depthFrame.getVideoMode().getResolutionX();
	g_nYRes = depthFrame.getVideoMode().getResolutionY();

	// upper left
	glTexCoord2f(0, 0);
	glVertex2f(0, 0);
	// upper right
	glTexCoord2f((float)g_nXRes/(float)m_nTexMapX, 0);
	glVertex2f(GL_WIN_SIZE_X, 0);
	// bottom right
	glTexCoord2f((float)g_nXRes/(float)m_nTexMapX, (float)g_nYRes/(float)m_nTexMapY);
	glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	// bottom left
	glTexCoord2f(0, (float)g_nYRes/(float)m_nTexMapY);
	glVertex2f(0, GL_WIN_SIZE_Y);

	glEnd();
	glDisable(GL_TEXTURE_2D);

	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();

	// TODO
	// Read result.csv to tag profile on top of the head
	 GetResultOfPID();

        bool trackingTargetLost = true;

	for (int i = 0; i < users.getSize(); ++i)
	{
		const nite::UserData& user = users[i];

		// TODO : draw name (after GetResultOfPID)
		if (confidenceOfResult == 1 && g_userTagID[user.getId()] && user.getCenterOfMass().z <3300 && user.isVisible())
		{
			updateIdentity(VotingPID::getnameVotingWithIndex(user.getId()).c_str(), true, m_colorFrame, m_pUserTracker, user, userTrackerFrame.getTimestamp());
		}
		else if (confidenceOfResult == 0 && g_userTagID[user.getId()] && user.getCenterOfMass().z <3300 && user.isVisible())
		{
			updateIdentity(VotingPID::getnameVotingWithIndex(user.getId()).c_str(), false, m_colorFrame, m_pUserTracker, user, userTrackerFrame.getTimestamp());
		}
		
		updateUserState(user, userTrackerFrame.getTimestamp());

		if (user.isNew() && user.getCenterOfMass().z <3300)
		{	
			cleanUserBuffer(user.getId());
			VotingPID::cleanVotingBufferWithIndex(user.getId());

			m_pUserTracker->startSkeletonTracking(user.getId());
			m_pUserTracker->startPoseDetection(user.getId(), nite::POSE_CROSSED_HANDS);
		}
		else if (!user.isLost())
		{
			if (g_drawStatusLabel)
			{
				// TODO
				//DrawStatusLabel(m_pUserTracker, user);
				if (g_userNameConfidence[user.getId()] == true && g_userTagID[user.getId()] )
				{
					DrawIdentity(m_pUserTracker, user);
					//DrawUserColor(m_colorFrame, m_pUserTracker, user, userTrackerFrame.getTimestamp());
				}	
				else if (g_userNameConfidence[user.getId()] == false && g_userTagID[user.getId()])
				{
					//DrawIdentity(m_pUserTracker, user);
					DrawIdentityByHist(m_colorFrame, m_pUserTracker, user);
					//DrawUserColor(m_colorFrame, m_pUserTracker, user, userTrackerFrame.getTimestamp());
				}
				
			}

			if (g_drawCenterOfMass)
			{
				DrawCenterOfMass(m_pUserTracker, user);
			}

			//TODO draw bounding box when get the host name 
			if (g_drawBoundingBox && (user.getId() == i_FollowingTarget) && user.getCenterOfMass().z <3300)
			{
				DrawBoundingBox(user);
			}

			//TODO : call tracking 
			// if (g_runRobotTracking && user.getId() == i_FollowingTarget) { // if (g_runRobotTracking) 
			// 	RunRobotTracking(m_pUserTracker, user);
			// } //&& (user.getId() == i_FollowingTarget)
			if (g_runRobotTracking && (userTrackerFrame.getFrameIndex() % 3 == 0 && (user.getId() == i_FollowingTarget))  ) { // if (g_runRobotTracking) 
				trackingTargetLost = false;
				RunRobotTracking(m_pUserTracker, user);
			}

			//skeleton sensing
			if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED && g_drawSkeleton && users[i].getCenterOfMass().z < 2200 && users[i].isVisible()) 
			{
				//cout << "user id :" << users[i].getId() << " z :" << users[i].getCenterOfMass().z <<endl;
				DrawSkeleton(m_pUserTracker, user);

				/*TODO write skeleton csv*/
				
				//get system time
				GetLocalTime(&st);
				//sprintf_s(time_str, 13, "%02d:%02d:%02d:%03d", st.tm_hour, st.tm_min, st.tm_sec, st.wMilliseconds);
				memset(time_str, '\0', 9); //xx:xx:xx
				snprintf(time_str, 9, "%02d:%02d:%02d", st.tm_hour, st.tm_min, st.tm_sec);
				
				WriteSkeletonInfo(user.getId(), csvfile, user, time_str);
			}
		}

		if (m_poseUser == 0 || m_poseUser == user.getId())
		{
			const nite::PoseData& pose = user.getPose(nite::POSE_CROSSED_HANDS);

			if (pose.isEntered())
			{
				// Start timer
				sprintf(g_generalMessage, "In exit pose. Keep it for %d second%s to exit\n", g_poseTimeoutToExit/1000, g_poseTimeoutToExit/1000 == 1 ? "" : "s");
				//printf("Counting down %d second to exit\n", g_poseTimeoutToExit/1000);
				m_poseUser = user.getId();
				m_poseTime = userTrackerFrame.getTimestamp();
			}
			else if (pose.isExited())
			{
				memset(g_generalMessage, 0, sizeof(g_generalMessage));
				//printf("Count-down interrupted\n");
				m_poseTime = 0;
				m_poseUser = 0;
			}
			else if (pose.isHeld())
			{
				// tick
				if (userTrackerFrame.getTimestamp() - m_poseTime > g_poseTimeoutToExit * 1000)
				{
					//printf("Count down complete. Exit...\n");
					StopRobotTracking_2();
					Finalize();

					exit(2);
				}
			}
		}
	}
	if (g_runRobotTracking && userTrackerFrame.getFrameIndex() % 3 == 0 && trackingTargetLost)
	{
		StopRobotTracking_2();
	} 

	confidenceOfResult = -1;
	// Screen Visualization
	if (g_drawFrameId)
	{
		DrawFrameId(userTrackerFrame.getFrameIndex());
		//draw SystemTime time
		DrawSystemTime();
	}
	if (g_runRobotTracking)
	{
		DrawMode("Robot Tracking Mode");
	}
	else {
		DrawMode("Manual Controll Mode");
	}

	if (g_generalMessage[0] != '\0')
	{
		char *msg = g_generalMessage;
		glColor3f(1.0f, 0.0f, 0.0f);
		glRasterPos2i(100, 20);
		glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
	}



	// Swap the OpenGL display buffers
	glutSwapBuffers();

}

void EoyViewer::OnKey(unsigned char key, int /*x*/, int /*y*/)
{
	switch (key)
	{
	case 27:
		Finalize();
		rosFinalize();
		exit (1);
	case 'z':
		// Draw skeleton?
		g_drawSkeleton = !g_drawSkeleton;
		break;
	case 'l':
		// Draw user status label?
		g_drawStatusLabel = !g_drawStatusLabel;
		break;
	case 'c':
		// Draw center of mass?
		g_drawCenterOfMass = !g_drawCenterOfMass;
		break;
	case 'x':
		// Draw bounding box?
		g_drawBoundingBox = !g_drawBoundingBox;
		break;
	case 'b':
		// Draw background?
		g_drawBackground = !g_drawBackground;
		break;
	case 'v':
		// Draw depth?
		g_drawDepth = !g_drawDepth;
		break;
	case 'f':
		// Draw frame ID
		g_drawFrameId = !g_drawFrameId;
		break;
	case '1': // Depth + RGB Stream Mode
		m_eViewState = DISPLAY_MODE_OVERLAY;
		m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		break;
	case '2': // Depth Stream Mode
		m_eViewState = DISPLAY_MODE_DEPTH;
		m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
		break;
	case '3': // RGB Stream Mode
		m_eViewState = DISPLAY_MODE_IMAGE;
		m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
		break;
	case 'm': // flip the scene (mirror)
		m_depthStream.setMirroringEnabled(!m_depthStream.getMirroringEnabled());
		m_colorStream.setMirroringEnabled(!m_colorStream.getMirroringEnabled());
		break;
	case 'r': // change mode
		g_runRobotTracking = !g_runRobotTracking;
		stopMotion();	
		break;
	// Robot control keyword
	case 'u': // armup and arm down
		/*
		(g_robotArmMode)? armDown(): armUp();
		g_robotArmMode = !g_robotArmMode;
		if (g_robotArmMode) 
			printf ("Arm Up");
		else
			cout << "Arm Down";
		*/
		break;
	case 32 : // stop robot
		stopMotion();
		break;
	case 'w':
		if (!g_runRobotTracking) 
			//setSpeed(SPEED_LIMIT, SPEED_LIMIT);
			goForward();
		break;
	case 's':
		if (!g_runRobotTracking) 
			//setSpeed(SPEED_LIMIT, SPEED_LIMIT);
			goBack();
		break;
	case 'a':
		if (!g_runRobotTracking) 
			//setSpeed(ROTATE_LIMIT, ROTATE_LIMIT);
			spinLeft();
		break;
	case 'd':
		if (!g_runRobotTracking) 
			//setSpeed(ROTATE_LIMIT, ROTATE_LIMIT);
			spinRight();
		break;
	case 'q':
		if (!g_runRobotTracking) 
			//turnLeftRight(SPEED_LIMIT - SPEED_WHEEL_DIFF, SPEED_LIMIT);
			turnLeftRight(true);
		break;
	case 'e':
		if (!g_runRobotTracking)
			//turnLeftRight(SPEED_LIMIT, SPEED_LIMIT - SPEED_WHEEL_DIFF);
			turnLeftRight(false);
		break;

	}
}

openni::Status EoyViewer::InitOpenGL(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow (m_strSampleName);
	// 	glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);
	glTranslatef (4., 0., 0.);

	InitOpenGLHooks();

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	return openni::STATUS_OK;

}


void EoyViewer::InitOpenGLHooks()
{
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
}
