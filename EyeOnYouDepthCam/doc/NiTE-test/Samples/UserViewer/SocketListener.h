#if !defined(SOCKET_LISTENER__INCLUDED_)
#define SOCKET_LISTENER__INCLUDED_

#include "CThread.h"
#include "SocketSender.h"


#define DEFAULT_BUFLEN 512

class SocketListener : public CThread {
protected:
    int ClientSocket;
    struct addrinfo *result, hints;
    char recvbuf[DEFAULT_BUFLEN];
    int iResult, iSendResult;
    int recvbuflen;
	bool listening;
	SocketSender* sender;

public:
	SocketListener(int socket);
	void setSender(SocketSender* sender);
	int m_ThreadFunc();

protected:
	void HandleMessage(char* xml);
};

#endif
