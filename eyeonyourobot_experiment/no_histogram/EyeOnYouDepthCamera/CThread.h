#if !defined(CTHREAD_INCLUDED)
#define CTHREAD_INCLUDED

typedef unsigned (WINAPI* PCTHREAD_THREADFUNC)(void* threadParam);

class CThread {
public:
	CThread();
	void startThread();
	static DWORD WINAPI ThreadFunc(LPVOID param);
	void waitForExit();
	BOOL isRunning();

protected:
	virtual DWORD m_ThreadFunc();
	HANDLE m_hThread;
	unsigned int m_threadId;
	BOOL m_fRunning;
};
#endif
