#include "CThread.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
//#include "SocketListener.h"
//#include "SocketSender.h"

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT_STR "8081"

class ServerSocketRunPID : public CThread {
protected:
	int ListenSocket, ClientSocket;
	struct addrinfo *result, hints;
	char recvbuf[DEFAULT_BUFLEN];
	int iResult, iSendResult;
	int recvbuflen;
	bool listening;
	//SocketSender* sender;
	//SocketListener* listener;
	static bool runPID;

public:
	ServerSocketRunPID();
	//SocketSender* getSender();
	//SocketListener* getListener();
	int getThread();
	unsigned long m_ThreadFunc();
	static void setrunPID(bool rec_runPID);
	static bool getrunPID();
};

