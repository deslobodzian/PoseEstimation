//
// Created by DSlobodzian on 2/4/2022.
//

#ifndef POSE_ESTIMATION_UDP_CLIENT_HPP
#define POSE_ESTIMATION_UDP_CLIENT_HPP

#define BUFFER_SIZE 1024

#include <iostream>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

class Client {
private:
    int socket_;
    int port_;
    int data_;
    int server_length_;
    struct sockaddr_in server_address_;
    char *hostname_;
    char buf[BUFFER_SIZE];


public:
    Client(std::string& server_address, int port) {
        port_ = port;
        socket_ = socket(AF_INET, SOCK_DGRAM, 0);

        bzero((char *) &server_address_, sizeof(server_address_));
        server_address_.sin_family = AF_INET;
        server_address_.sin_addr.s_addr = inet_addr(server_address);
        server_address_.sin_port = htons(port_);
    }

    bool send_message(std::string msg) {
        bzero(buf, BUFFER_SIZE);
        msg.copy(buf, BUFFER_SIZE);
        server_length_ = sizeof(server_address_);
        return !(sendto(socket_, buf, strlen(buf), 0, &server_address_, server_length_) < 0);
    }

    bool receive_message() {
        int n = recvfrom(socket_, buf, strlen(buf), 0, &server_address_, &server_length_);
        if (n < 0) {
            return false;
        }
        printf("Received data: %s", buf);
        return true;
    }

}

#endif //POSE_ESTIMATION_UDP_CLIENT_HPP
