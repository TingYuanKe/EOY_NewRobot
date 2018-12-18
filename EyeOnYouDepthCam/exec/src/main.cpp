/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include "Viewer.h"

#include <pthread.h>

#include <cstdio>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <iostream> //lib use in c++
#include <unistd.h>
#include <signal.h>


// PID Server Socket thread 
void* RunPIDThreadFunc(void* data);
void StopRobotHandler(int sig);

int main(int argc, char** argv)
{
	// change current working directory
	chdir("/home/newrobot/catkin_ws/devel");
	
	//ctrl + c singal handler to stop robot
	signal(SIGINT,StopRobotHandler);

	//****************Init ROS node******************************
	pthread_t threadServer;
	pthread_create(&threadServer,NULL,RunPIDThreadFunc,NULL);
	int WaitForSocket = 1;
	while (WaitForSocket > 0) {
		cout << WaitForSocket << endl;
		WaitForSocket--;
		sleep(1);
	}

		//****************Init ROS node******************************
   	rosInit(argc, argv);
	cout << "\n=====Successfully Init ROS=====\n";



	//****************Init Openni and Nite***********************
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

	// init device
	rc = device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	// init depth stream
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

	// init rgb stream
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

	// double check
	if (!depth.isValid() || !color.isValid())
	{
		printf("SimpleViewer: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}


	//****************Show Visualization*************************
	EoyViewer EoyViewer("EoyViewer Visualization", device, depth, color);

	rc = EoyViewer.Init(argc, argv);
	if (rc != openni::STATUS_OK)
	{
		return 1;
	}
	EoyViewer.Run();


}


void* RunPIDThreadFunc(void* data) {
	cout << "=======ServerSocketRunPID Server Init========" << endl;

	// Start server socket listener
	ServerSocketRunPID* server = new ServerSocketRunPID();
	server->startThread();

	// Wait for server socket to terminate
	void* status;

	if(pthread_join(server->getThread(),&status)){
		printf("Failed to join thread\n");
	}
	
	return 0;
}


void StopRobotHandler(int sig) {
	rosFinalize();
	exit(1);
}