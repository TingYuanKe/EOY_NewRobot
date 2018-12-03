#include "stdafx.h"
#include "CThread.h"
#include <pthread.h>
CThread::CThread() {
	m_fRunning = false;
}

void CThread::startThread() {

	int iResult = pthread_create(&m_hThread,NULL,ThreadFunc,this);

	if ( iResult == 0 ) {
		m_fRunning = true;
		
		printf("Thread started\n");

	}
	else printf("Failed to create thread\n");

}

void CThread::waitForExit() {
	if ( !m_fRunning )
		return;
	void* status;

	if(pthread_join(m_hThread,&status)){
		printf("Failed to join thread\n");
	}
}

bool CThread::isRunning() {
	return m_fRunning;
}

void* CThread::ThreadFunc(void* param) {
	// The param is the address of the object
	CThread* pto = (CThread*)param;

	// Call the subclass' thread function
	pto->m_ThreadFunc();
	pto->m_fRunning = false;
	return 0;
}

int CThread::m_ThreadFunc() {
	// function overload:
	// actually m_ThreadFunc would be implemented in the inherit class ( ServerSocketRunPID )
	return 0;
}

