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

#define BUFFER_SIZE 1024

class Server {

private:
    int socket_;
    int host_port_;
    int client_port_;
    socklen_t clientLength_;
    struct sockaddr_in serverAddr_;
    struct sockaddr_in clientAddr_;
    struct hostent *hostp_; // Host info
    char buf[BUFFER_SIZE];
    char *hostAddrp_;
    int optval;
    int n; //message byte size

public:
    Server(const std::string& host, int host_port, const std::string& client, int client_port) {
        char h[BUFFER_SIZE];
        host.copy(h, host.length()+1);
        char c[BUFFER_SIZE];
        client.copy(c, client.length()+1);
        host_port_ = host_port;
        client_port_ = client_port;

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
        serverAddr_.sin_port = htons((unsigned short)host_port_);

        if (bind(socket_, ((struct sockaddr *) &serverAddr_), sizeof(serverAddr_)) < 0) {
            std::cout << "ERROR: Couldn't bind socket" << std::endl;
        }

        bzero((char* ) &clientAddr_, sizeof(clientAddr_));
        clientAddr_.sin_family = AF_INET;
        clientAddr_.sin_addr.s_addr = inet_addr("10.56.87.2");
        clientAddr_.sin_port = htons((unsigned short)client_port_);
        clientLength_ = sizeof(clientAddr_);
    }

    ~Server() = default;

    int receive() {
        bzero(buf, BUFFER_SIZE);
        n = recvfrom(socket_,
                     buf,
                     BUFFER_SIZE,
                     0,
                     (struct sockaddr*) &clientAddr_,
                     &clientLength_);
	    std::string s(buf, sizeof(buf));
	    std::cout << s << "\n";
        if (n < 0) {
            std::cout << "ERROR: Couldn't receive from client." << std::endl;
        }
    }

    int send(std::string msg) {
        bzero(buf, BUFFER_SIZE);
        msg.copy(buf, BUFFER_SIZE);
        return sendto(socket_, buf, strlen(buf), 0, (struct sockaddr*) &clientAddr_, clientLength_);
    }
};
