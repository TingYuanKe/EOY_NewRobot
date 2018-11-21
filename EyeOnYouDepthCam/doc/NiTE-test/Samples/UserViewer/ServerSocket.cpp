#include "stdafx.h"
#include "ServerSocket.h"

ServerSocket::ServerSocket() {
	ListenSocket = -1;
	ClientSocket = -1;
	recvbuflen = DEFAULT_BUFLEN;
	listening = true;
	result = NULL;
	sender = NULL;
	listener = NULL;
}
/*
SocketSender* ServerSocket::getSender() {
	return sender;
};

SocketListener* ServerSocket::getListener() {
	return listener;
};
*/
HANDLE ServerSocket::getThread() {
	return this->m_hThread;
}

DWORD ServerSocket::m_ThreadFunc() {
	// Define socket type
    bzero(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, DEFAULT_PORT_STR, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed\n");
        return 1;
    }

    // Create a SOCKET for listening client
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket < 0) {
        printf("socket failed\n");
        freeaddrinfo(result);
        return 1;
    }

    // Setup the TCP listening socket
    iResult = bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult < 0) {
        printf("bind failed\n");
        freeaddrinfo(result);
        close(ListenSocket);
        return 1;
    }

    freeaddrinfo(result);

    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult < 0) {
        printf("listen failed\n");
        close(ListenSocket);
        return 1;
    }
	
	printf("Listing for clients on port %s \n", DEFAULT_PORT_STR);
	while ( listening ) {
		// Accept a client socket
		ClientSocket = accept(ListenSocket, NULL, NULL);
		if (ClientSocket >= 0) {
			printf("Client connected\n");

			// Check the sender/listener socket is existed or not
			if ( sender != NULL ) {
				try { delete sender; } catch (...) { }
			}
			if ( listener != NULL ) {
				try { delete listener; } catch (...) { }
			}

			// Setup the socket sender and listener
			//sender = new SocketSender(ClientSocket);
			//listener = new SocketListener(ClientSocket);
			//listener->setSender(sender);
			//listener->startThread();
		}
		else {
			printf("accept failed\n");
			listening = false;
			close(ListenSocket);
		}
	}

	return 0;
}

