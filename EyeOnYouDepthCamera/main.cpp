/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/
// Modified by Richard Yi-Chia Tsai @ 2017/11/17 and 2017/12/31 and 2018/01/03 and 2018/03/08
// EyeOnYou Robot

#include "Viewer.h"

DWORD WINAPI DriveRobotThreadFunc(void* data);
DWORD WINAPI RunPIDThreadFunc(void* data);

int main(int argc, char** argv)
{
	// 0. Initialize server socket to conduct interprocess communication with java-based robot
	HANDLE threadRobot = CreateThread(NULL, 0, DriveRobotThreadFunc, NULL, 0, NULL);
	HANDLE threadServer = CreateThread(NULL, 0, RunPIDThreadFunc, NULL, 0, NULL);
	int WaitForSocket = 1;
	while (WaitForSocket > 0) {
		cout << WaitForSocket << endl;
		WaitForSocket--;
		Sleep(1000);
	}

	openni::Status rc = openni::STATUS_OK;

	openni::Device device;
	openni::VideoStream depth, color;
	const char* deviceURI = openni::ANY_DEVICE;
	if (argc > 1)
	{
		deviceURI = argv[1];
	}

	rc = openni::OpenNI::initialize();

	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

	rc = device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	rc = depth.create(device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = depth.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			depth.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	rc = color.create(device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		rc = color.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
			color.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	if (!depth.isValid() || !color.isValid())
	{
		printf("SimpleViewer: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}

	SampleViewer sampleViewer("EyeOnYou Depth Camera Sensing", device, depth, color);

	rc = sampleViewer.Init(argc, argv);
	if (rc != openni::STATUS_OK)
	{
		openni::OpenNI::shutdown();
		return 3;
	}
	sampleViewer.Run();
}

DWORD WINAPI DriveRobotThreadFunc(void* data) {
	cout << "Native ServerSocketDriveRobot server starting" << endl;

	// Start server socket listener
	ServerSocket* server = new ServerSocket();
	server->startThread();

	// Wait for server socket to terminate
	WaitForSingleObject(server->getThread(), INFINITE);
	return 0;
}

DWORD WINAPI RunPIDThreadFunc(void* data) {
	cout << "Native ServerSocketRunPID server starting" << endl;

	// Start server socket listener
	ServerSocketRunPID* server = new ServerSocketRunPID();
	server->startThread();

	// Wait for server socket to terminate
	WaitForSingleObject(server->getThread(), INFINITE);
	return 0;
}