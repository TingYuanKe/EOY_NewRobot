#include "CThread.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
//#include "SocketListener.h"
//#include "SocketSender.h"

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT_STR "8080"

class ServerSocket : public CThread {
protected:
    //WSADATA wsaData;
    int ListenSocket, ClientSocket;
    struct addrinfo *result, hints;
    char recvbuf[DEFAULT_BUFLEN];
    int iResult, iSendResult;
    int recvbuflen;
	bool listening;
	//SocketSender* sender;
	//SocketListener* listener;

public:
	ServerSocket();
	//SocketSender* getSender();
	//SocketListener* getListener();
	HANDLE getThread();
	DWORD m_ThreadFunc();
};

