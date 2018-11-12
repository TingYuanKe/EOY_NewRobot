#include <stdlib.h>
#include "PIDRun.h"

bool PIDRun::keepSkeleton = false;
bool PIDRun::executePID = false;
bool PIDRun::tagProfile = false;

bool PIDRun::getKeepSkeleton()
{
	return keepSkeleton;
}

void PIDRun::setKeepSkeleton(bool rec_keepSkeleton)
{
	keepSkeleton = rec_keepSkeleton;
}

bool PIDRun::getExecutePID()
{
	return executePID;
}

void PIDRun::setExecutePID(bool rec_executePID)
{
	executePID = rec_executePID;
}

bool PIDRun::getTagProfile()
{
	return tagProfile;
}

void PIDRun::setTagProfile(bool rec_tagProfile)
{
	tagProfile = rec_tagProfile;
}

//void PIDRun::systemCallCmd(char* cmd)
//{
//	system(cmd);
//}
