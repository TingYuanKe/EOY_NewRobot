#include "stdafx.h"
#include "CThread.h"

CThread::CThread() {
	m_fRunning = false;
	m_hThread = NULL;
	m_threadId = 0;
}

void CThread::startThread() {
	m_hThread = (HANDLE)_beginthreadex(
						NULL, 0,
						(PCTHREAD_THREADFUNC)ThreadFunc,
						this, 0,
						&m_threadId);

	if ( m_hThread != NULL ) {
		m_fRunning = true;
		printf("Thread started\n");
		return;
	}

	printf("Failed to create thread\n");
}

void CThread::waitForExit() {
	if ( m_hThread == NULL )
		return;

	WaitForSingleObject(m_hThread, INFINITE);
	CloseHandle(m_hThread);
}

BOOL CThread::isRunning() {
	return m_fRunning;
}

DWORD WINAPI CThread::ThreadFunc(LPVOID param) {
	// The param is the address of the object
	CThread* pto = (CThread*)param;

	// Call the subclass' overidden thread function
	DWORD dwResult = pto->m_ThreadFunc();
	pto->m_fRunning = false;
	return dwResult;
}

DWORD CThread::m_ThreadFunc() {
	// subclass overrides and implements this
	return 0;
}

