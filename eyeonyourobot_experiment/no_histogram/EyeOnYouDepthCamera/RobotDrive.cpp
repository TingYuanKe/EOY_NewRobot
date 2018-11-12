#include "RobotDrive.h"

char RobotDrive::drivetowhere[Max_Char_Drivetowhere] = "stop";
int RobotDrive::driveunit = 0;

//RobotDrive::RobotDrive(char*  init_drivetowhere, int init_driveunit)
//{
//	memset(RobotDrive::drivetowhere, 0, sizeof(RobotDrive::drivetowhere));
//	strcat_s(RobotDrive::drivetowhere, Max_Char_Drivetowhere, init_drivetowhere);
//
//	RobotDrive::driveunit = init_driveunit;
//}

char* RobotDrive::getDrivetowhere()
{
	return drivetowhere;
}

int RobotDrive::getDriveunit()
{
	return driveunit;
}

void RobotDrive::setDrivetowhere(char* rec_drivetowhere)
{
	memset(drivetowhere, 0, sizeof(drivetowhere));
	strcat_s(drivetowhere, Max_Char_Drivetowhere, rec_drivetowhere);
}

void RobotDrive::setDriveunit(int rec_driveunit)
{
	driveunit = rec_driveunit;
}

