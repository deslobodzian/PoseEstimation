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

struct frame {
    long millis;
    double est_x;
    double est_y;
    double est_heading;
    bool has_target;
    double goal_distance;
    double goal_angle;
    frame(long m, double x, double y, double heading, bool target, double goal_dist, double goal_ang) {
        millis = m;
        est_x = x;
        est_y = y;
        est_heading = heading;
        has_target = target;
        goal_distance = goal_dist;
        goal_angle = goal_ang;
    }
    std::string to_udp_string() {
        std::string value = std::to_string(millis) + ";" +
                std::to_string(est_x) + ";" +
                std::to_string(est_y) + ";" +
                std::to_string(est_heading) +  ";" +
                std::to_string(has_target) +  ";" +
                std::to_string(goal_distance) +  ";" +
                std::to_string(goal_angle);
        return value;
    }
};

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
    char receive_buf[BUFFER_SIZE];
    char *hostAddrp_;
    int optval;
    int n; 


public:
    Server(const std::string& host, int host_port, const std::string& client, int client_port) {
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
        serverAddr_.sin_addr.s_addr = inet_addr(host.c_str());
        serverAddr_.sin_port = htons((unsigned short)host_port_);

        if (bind(socket_, ((struct sockaddr *) &serverAddr_), sizeof(serverAddr_)) < 0) {
            std::cout << "ERROR: Couldn't bind socket" << std::endl;
        }

        bzero((char* ) &clientAddr_, sizeof(clientAddr_));
        clientAddr_.sin_family = AF_INET;
        clientAddr_.sin_addr.s_addr = inet_addr(client.c_str());
        clientAddr_.sin_port = htons((unsigned short)client_port_);
        clientLength_ = sizeof(clientAddr_);
    }

    ~Server() = default;

    int receive() {
        bzero(receive_buf, BUFFER_SIZE);
        n = recvfrom(socket_,
                     receive_buf,
                     BUFFER_SIZE,
                     0,
                     (struct sockaddr*) &clientAddr_,
                     &clientLength_);
        if (n < 0) {
            std::cout << "ERROR: Couldn't receive from client." << std::endl;
        }
    }

    int send(std::string msg) {
        bzero(buf, BUFFER_SIZE);
        msg.copy(buf, BUFFER_SIZE);
        return sendto(socket_, buf, strlen(buf), 0, (struct sockaddr*) &clientAddr_, clientLength_);
    }
    int send(frame &frame) {
        return send(frame.to_udp_string());
    }

    std::string get_message() {
        std::string s(receive_buf, sizeof(receive_buf));
        return s;
    }

};
