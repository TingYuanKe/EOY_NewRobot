#include "CThread.h"
#include "SocketListener.h"
#include "SocketSender.h"

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT_STR "8081"

class ServerSocketRunPID : public CThread {
protected:
	WSADATA wsaData;
	SOCKET ListenSocket, ClientSocket;
	struct addrinfo *result, hints;
	char recvbuf[DEFAULT_BUFLEN];
	int iResult, iSendResult;
	int recvbuflen;
	BOOL listening;
	SocketSender* sender;
	SocketListener* listener;
	static bool runPID;

public:
	ServerSocketRunPID();
	SocketSender* getSender();
	SocketListener* getListener();
	HANDLE getThread();
	DWORD m_ThreadFunc();
	static void setrunPID(bool rec_runPID);
	static bool getrunPID();
};

