#if !defined(CTHREAD_INCLUDED)
#define CTHREAD_INCLUDED
#include <pthread.h>


class CThread {
public:
	CThread();
	void startThread();
	static void* ThreadFunc(void* param);
	void waitForExit();
	bool isRunning();

protected:
	virtual int m_ThreadFunc();
	pthread_t m_hThread;
	bool m_fRunning;
};
#endif
