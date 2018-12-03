#if !defined(SOCKET_SENDER__INCLUDED_)
#define SOCKET_SENDER__INCLUDED_

#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#define DEFAULT_BUFLEN 512

class SocketSender {
protected:
    int clientSocket;
    struct addrinfo *result, hints;
    char recvbuf[DEFAULT_BUFLEN];
    int iResult, iSendResult;
    int recvbuflen;

public:
	SocketSender(int client);

	void sendHostnameResponse();
	void sendMemoryResponse();
	void sendRandomNumberResponse();
	void sendKinectKeepSkeletonResponse();
	void sendKinectTagProfileResponse();

protected:
	void sendXMLToClient(char* xml);
};

#endif