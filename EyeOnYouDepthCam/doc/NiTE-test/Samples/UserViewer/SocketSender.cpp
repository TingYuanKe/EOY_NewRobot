#include "stdafx.h"
#include "SocketSender.h"
#include "Viewer.h"

SocketSender::SocketSender(int client) {
	this->clientSocket = client;
}


void SocketSender::sendHostnameResponse() {
	char hostname[32];
	gethostname(hostname, 32);

	char buffer[512];
	buffer[0] = 0;
	sprintf( 
		buffer, 
		"<Response><Name>HostnameResponse</Name><Hostname>%s</Hostname></Response>\n", hostname);

	sendXMLToClient(buffer);
}

// Unused ??
//void SocketSender::sendMemoryResponse() {
//	MEMORYSTATUS memory;
//	GlobalMemoryStatus(&memory);
//
//	char buffer[512];
//	buffer[0] = 0;
//	sprintf( 
//		buffer, 
//		"<Response><Name>MemoryResponse</Name><TotalPhysicalMemory>%i</TotalPhysicalMemory></Response>\n", 
//		memory.dwTotalPhys);
//
//	sendXMLToClient(buffer);
//}

// Send command to iRobot: Unused ??
//void SocketSender::sendRandomNumberResponse() {
//	char buffer[512];
//	buffer[0] = 0;
//
//	char response_drivetowhere[512] = "";
//	int response_driveunit = 0;
//
//	memset(response_drivetowhere, 0, sizeof(response_drivetowhere));
//	strncat(response_drivetowhere, RobotDrive::getDrivetowhere(),512);
//	response_driveunit = RobotDrive::getDriveunit();
//
//	sprintf(buffer, "%s,%i\n", response_drivetowhere, response_driveunit);
//	sendXMLToClient(buffer);
//}


// TODO : PIDRun module

void SocketSender::sendKinectKeepSkeletonResponse() {
	char buffer[512];
	buffer[0] = 0;

	if (PIDRun::getExecutePID() == true) {
		sprintf(buffer, "runPID\n");
		sendXMLToClient(buffer);
		PIDRun::setExecutePID(false);
		// cout << "Successfully complete sending runPID command to EyeOnYouServer!" << endl;
	}
}


void SocketSender::sendKinectTagProfileResponse() {
	char buffer[512];
	buffer[0] = 0;

	if (PIDRun::getTagProfile() == false) {
		sprintf(buffer, "finishedTagging\n");
		sendXMLToClient(buffer);
		// cout << "Successfully complete sending tagProfile result to EyeOnYouServer!" << endl;
	}
}


void SocketSender::sendXMLToClient(char xml[]) {
	// Send some XML to the client
	int len = strlen(xml);
    int sent = send( clientSocket, xml, len, 0 );
	// cout << "*run PID* has been sent!!!" << endl;

    if ( sent == -1 ) {
        // printf( "send failed\n");
        close( clientSocket );
        return;
    }

	// printf( "%i bytes sent to client\n", sent);
}
