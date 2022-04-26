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
#include <iostream>
#include "vision/tracked_object_info.hpp"
#include "localization/particle_filter.hpp"
#include <cmath>

#define BUFFER_SIZE 1024

struct output_frame {

    enum FRAME_TYPE {
        DEFAULT = 0,
        TRACKED_OBJECTS = 1
    };

    FRAME_TYPE type;
    std::vector<TrackedObjectInfo> tracked_objects;

    output_frame() {
        type = DEFAULT;
    }

    output_frame(std::vector<TrackedObjectInfo> tracked_objects) {
        type = TRACKED_OBJECTS;
        this->tracked_objects = tracked_objects;
    }

    std::string to_packet() {
        std::string packet;
        switch (type) {
            case DEFAULT:
                packet = std::to_string(DEFAULT) + ";";
                return packet;
            case TRACKED_OBJECTS:
                packet = std::to_string(TRACKED_OBJECTS) + ";";
                for (auto object : tracked_objects) {
                    packet.append(object.to_packet());
                }
                return packet;
        }
    }
};

struct input_frame{
    enum FRAME_TYPE {
        DEFAULT = 0,
        INIT = 1
    };
    long millis;
    ControlInput input;
    double init_pose[3]; // initial position [x, y, theta]
    input_frame() {
        millis = 0;
        input.dx = 0;
        input.dy = 0;
        input.d_theta = 0;
        init_pose[0] = 0;
        init_pose[1] = 0;
        init_pose[2] = 0;
    };
    input_frame(std::vector<std::string> values) {
        FRAME_TYPE type = (FRAME_TYPE) atof(values.at(0).c_str());
        switch (type) {
            case DEFAULT:
                millis = atof(values.at(1).c_str());
                input.dx = atof(values.at(2).c_str());
                input.dy = atof(values.at(3).c_str());
                input.d_theta = atof(values.at(4).c_str());
                break;
            case INIT:
                init_pose[0] = atof(values.at(1).c_str());
                init_pose[1] = atof(values.at(2).c_str());
                init_pose[2] = atof(values.at(3).c_str());
                break;
        }
    }
};

class UDPServer {

private:
    int server_socket_;
    int client_socket_;
    int host_port_ = 27002;
    int client_port_ = 27001;
    socklen_t clientLength_;
    socklen_t serverLength_;
    struct sockaddr_in serverAddr_;
    struct sockaddr_in clientAddr_;
    struct hostent *hostp_; // Host info
    char buf[BUFFER_SIZE];
    char receive_buf[BUFFER_SIZE];
    char *hostAddrp_;
    int optval;
    int n;
    std::string host_ = "10.56.87.20";
    std::string client_ = "10.56.87.2";
    input_frame latest_frame_;
    input_frame prev_frame_;
    input_frame init_pose_;
    output_frame data_frame_;

    std::thread data_thread_;
    std::thread recv_thread_;

public:
    UDPServer() {
        server_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        client_socket_ = socket(AF_INET, SOCK_DGRAM, 0);

        if (server_socket_ < 0) {
            error("ERROR: Couldn't open socket");
        }

        optval = 1;
        setsockopt(server_socket_,
                   SOL_SOCKET,
                   SO_REUSEADDR,
                   (const void*) &optval,
                   sizeof(int));
        setsockopt(client_socket_,
                   SOL_SOCKET,
                   SO_REUSEADDR,
                   (const void*) &optval,
                   sizeof(int));

        // server
        bzero((char* ) &serverAddr_, sizeof(serverAddr_));
        serverAddr_.sin_family = AF_INET;
        serverAddr_.sin_addr.s_addr = inet_addr(host_.c_str());
        serverAddr_.sin_port = htons((unsigned short)host_port_);
        serverLength_ = sizeof(serverLength_);

        // client
        bzero((char* ) &clientAddr_, sizeof(clientAddr_));
        clientAddr_.sin_family = AF_INET;
        clientAddr_.sin_addr.s_addr = inet_addr(client_.c_str());
        clientAddr_.sin_port = htons((unsigned short)client_port_);
        clientLength_ = sizeof(clientAddr_);
        // listening on socket
        if (bind(server_socket_, ((struct sockaddr *) &serverAddr_), sizeof(serverAddr_)) < 0) {
            error("ERROR: Couldn't bind send socket");
        }
    }

    ~UDPServer() = default;

    int receive() {
        bzero(receive_buf, BUFFER_SIZE);
        n = recvfrom(server_socket_,
                     receive_buf,
                     BUFFER_SIZE,
                     0,
                     (struct sockaddr*) &serverAddr_,
                     &serverLength_);
        if (n < 0) {
            error("ERROR: Couldn't receive from client");
        }
    }

    int send(std::string msg) {
        bzero(buf, BUFFER_SIZE);
        msg.copy(buf, BUFFER_SIZE);
        return sendto(client_socket_, buf, strlen(buf), 0, (struct sockaddr*) &clientAddr_, clientLength_);
    }

    int send(output_frame &frame) {
        return send(frame.to_packet());
    }

    std::vector<std::string> split( const std::string& str, char delimiter = ';' ) {
        std::vector<std::string> result ;
        std::istringstream stm(str) ;
        std::string fragment ;
        while( std::getline( stm, fragment, delimiter ) ) result.push_back(fragment) ;
        return result ;
    }


    std::string get_message() {
        std::string s(receive_buf, sizeof(receive_buf));
        std::vector<std::string> values = split(s);
        return s;
    }

    input_frame get_new_frame() {
        std::string s(receive_buf, sizeof(receive_buf));
        std::vector<std::string> values = split(s);
        if (values.size() >= 5) {
            if (atof(values.at(0).c_str()) == 0) {
                init_pose_ = input_frame(values);
                return input_frame();
            } else {
                return input_frame(values);
            }
        }
    }

    void receive_frame() {
        if (receive() > 0) {
            error("No frame");
        } else {
            input_frame incoming_frame = get_new_frame();
            if (incoming_frame.millis > latest_frame_.millis) {
//                info("Received frame");
                prev_frame_ = latest_frame_;
                latest_frame_ = incoming_frame;
                double dt = latest_frame_.millis - prev_frame_.millis;
//                std::cout << "[INFO] Frame DT {" << dt << "}\n";
            }
        }
    }

    input_frame get_latest_frame() {
        return latest_frame_;
    }

    input_frame get_init_pose_frame() {
        return init_pose_;
    }

    void set_data_frame(output_frame &frame) {
        data_frame_ = frame;
    }

    void receive_thread() {
        while (true) {
            receive_frame();
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }
    }
    void data_processing_thread() {
        while (true) {
            send(data_frame_);
	        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }
    }

    void start_thread() {
        data_thread_ = std::thread(&UDPServer::data_processing_thread, this);
//        recv_thread_ = std::thread(&Server::receive_thread, this);
    }
};
