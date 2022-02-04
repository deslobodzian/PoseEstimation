//
// Created by DSlobodzian on 11/12/2021.
//

#pragma once

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

class Server {

private:
    int socket_;
    int port_;
    socklen_t clientLength_;
    struct sockaddr_in serverAddr_;
    struct sockaddr_in clientAddr_;
    struct hostent *hostp_; // Host info
    char buf[1024];
    char *hostAddrp_;
    int optval;
    int n; //message byte size

public:
    Server(const std::string& addr, int port) {
        port_ = port;

        socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_ < 0) {
            std::cout << "ERROR: Couldn't open socket." << std::endl;
        }

        optval = 1;
        setsockopt(socket_,
                   SOL_SOCKET,
                   SO_REUSEADDR,
                   (const void*) &optval,
                   sizeof(int));

        bzero((char* ) &serverAddr_, sizeof(serverAddr_));
        serverAddr_.sin_family = AF_INET;
        serverAddr_.sin_addr.s_addr = inet_addr("10.56.87.59");
        serverAddr_.sin_port = htons((unsigned short)27002);

        if (bind(socket_, ((struct sockaddr *) &serverAddr_), sizeof(serverAddr_)) < 0) {
            std::cout << "ERROR: Couldn't bind socket" << std::endl;
        }

        bzero((char* ) &clientAddr_, sizeof(clientAddr_));
        clientAddr_.sin_family = AF_INET;
        clientAddr_.sin_addr.s_addr = inet_addr("10.56.87.2");
        clientAddr_.sin_port = htons((unsigned short)27001);
        clientLength_ = sizeof(clientAddr_);
    }
    ~Server() = default;

    int receive() {
        while (true) {
            bzero(buf, 1024);
            n = recvfrom(socket_,
                         buf,
                         1024,
                         0,
                         (struct sockaddr*) &clientAddr_,
                         &clientLength_);
	    std::string s(buf, sizeof(buf));
	    std::cout << s << "\n";
            if (n < 0) {
                std::cout << "ERROR: Couldn't receive from client." << std::endl;
                break;
            }
        }
    }
    int send(std::string msg) {
        bzero(buf, BUFFER_SIZE);
        msg.copy(buf, BUFFER_SIZE);
        return sendto(socket_, buf, strlen(buf), 0, (struct sockaddr*) &clientAddr_, clientLength_);
    }
};
