#ifndef ROBOTDRIVE_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define ROBOTDRIVE_H

#include <string>

using namespace std;

const int Max_Char_Drivetowhere = 512;

class RobotDrive {
	protected:
		static char drivetowhere[Max_Char_Drivetowhere];
		static int driveunit;

	public:
		//RobotDrive(char*  init_drivetowhere, int init_driveunit);
		static char* getDrivetowhere();
		static int getDriveunit();
		static void setDrivetowhere(char* rec_drivetowhere);
		static void setDriveunit(int rec_driveunit);
};

#endif