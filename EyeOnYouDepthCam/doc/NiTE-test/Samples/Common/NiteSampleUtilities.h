/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Common Utilities for Samples                         *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#ifndef _NITE_SAMPLE_UTILITIES_H_
#define _NITE_SAMPLE_UTILITIES_H_

#include <stdio.h>
#include <OpenNI.h>

#ifdef WIN32
#include <conio.h>
int wasKeyboardHit()
{
	return (int)_kbhit();
}

#else // linux

#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
int wasKeyboardHit()
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	// don't echo and don't wait for ENTER
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	
	// make it non-blocking (so we can check without waiting)
	if (0 != fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK))
	{
		return 0;
	}

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	if (0 != fcntl(STDIN_FILENO, F_SETFL, oldf))
	{
		return 0;
	}

	if(ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}
#endif // WIN32

#endif // _NITE_SAMPLE_UTILITIES_H_
