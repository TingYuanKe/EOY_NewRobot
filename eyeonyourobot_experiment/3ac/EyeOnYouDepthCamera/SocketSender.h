#if !defined(SOCKET_SENDER__INCLUDED_)
#define SOCKET_SENDER__INCLUDED_

#define DEFAULT_BUFLEN 512

class SocketSender {
protected:
    WSADATA wsaData;
    SOCKET clientSocket;
    struct addrinfo *result, hints;
    char recvbuf[DEFAULT_BUFLEN];
    int iResult, iSendResult;
    int recvbuflen;

public:
	SocketSender(SOCKET client);

	void sendHostnameResponse();
	void sendMemoryResponse();
	void sendRandomNumberResponse();
	void sendKinectKeepSkeletonResponse();
	void sendKinectTagProfileResponse();

protected:
	void sendXMLToClient(char* xml);
};

#endif