#include "CThread.h"
#include "SocketListener.h"
#include "SocketSender.h"

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT_STR "8080"

class ServerSocket : public CThread {
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

public:
	ServerSocket();
	SocketSender* getSender();
	SocketListener* getListener();
	HANDLE getThread();
	DWORD m_ThreadFunc();
};

