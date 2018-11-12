#include "stdafx.h"
#include "ServerSocket.h"

ServerSocket::ServerSocket() {
	ListenSocket = INVALID_SOCKET;
	ClientSocket = INVALID_SOCKET;
	recvbuflen = DEFAULT_BUFLEN;
	listening = true;
	result = NULL;
	sender = NULL;
	listener = NULL;
}

SocketSender* ServerSocket::getSender() {
	return sender;
};

SocketListener* ServerSocket::getListener() {
	return listener;
};

HANDLE ServerSocket::getThread() {
	return this->m_hThread;
}

DWORD ServerSocket::m_ThreadFunc() {
	// Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed\n");
        return 1;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, DEFAULT_PORT_STR, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed\n");
        WSACleanup();
        return 1;
    }

    // Create a SOCKET for connecting to server
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed\n");
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }

    // Setup the TCP listening socket
    iResult = bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed\n");
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    freeaddrinfo(result);

    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed\n");
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }
	
	printf("Listing for clients on port %s \n", DEFAULT_PORT_STR);
	while ( listening ) {
		// Accept a client socket
		ClientSocket = accept(ListenSocket, NULL, NULL);
		if (ClientSocket != INVALID_SOCKET) {
			printf("Client connected\n");

			// Create socket listener and sender to handle this client
			if ( sender != NULL ) {
				try { delete sender; } catch (...) { }
			}
			if ( listener != NULL ) {
				try { delete listener; } catch (...) { }
			}

			// Setup the socket sender and listener
			sender = new SocketSender(ClientSocket);
			listener = new SocketListener(ClientSocket);
			listener->setSender(sender);
			listener->startThread();
		}
		else {
			int error = WSAGetLastError();
			printf("accept failed\n");
			switch ( error ) {
				case 10093:
					listening = false;
					try {
						closesocket(ListenSocket);
					}
					catch (...) { }
					return 1;
			}
		}
	}

	return 0;
}

