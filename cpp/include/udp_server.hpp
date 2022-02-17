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

struct output_frame {
    long millis;
    double est_x;
    double est_y;
    double est_heading;
    bool has_target;
    double goal_distance;
    double goal_angle;
    output_frame() {
        millis = 0;
        est_x = 0;
        est_y = 0;
        est_heading = 0;
        has_target = false;
        goal_distance = 0;
        goal_angle = 0;
    }
    output_frame(long m, double x, double y, double heading, bool target, double goal_dist, double goal_ang) {
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

struct input_frame{
    int id;
    long millis;
    double u[3]; // odometry [dx, dy, dTheta]
    double init_pose[3]; // initial position [x, y, theta]
    input_frame() {
        id = -1;
        millis = 0;
        u[0] = 0;
        u[1] = 0;
        u[2] = 0;
        init_pose[0] = 0;
        init_pose[1] = 0;
        init_pose[2] = 0;
    };
    input_frame(std::vector<std::string> values) {
        if (atof(values.at(0).c_str()) == 0) {
            id = 0;
            init_pose[0] = atof(values.at(1).c_str());
            init_pose[1] = atof(values.at(2).c_str());
            init_pose[2] = atof(values.at(3).c_str());
        } else {
            id = 1;
            millis = atof(values.at(1).c_str());
            u[0] = atof(values.at(2).c_str());
            u[1] = atof(values.at(3).c_str());
            u[2] = atof(values.at(4).c_str());
        }
    }
};


class Server {

private:
    int socket_;
    int host_port_ = 27002;
    int client_port_ = 27001;
    socklen_t clientLength_;
    struct sockaddr_in serverAddr_;
    struct sockaddr_in clientAddr_;
    struct hostent *hostp_; // Host info
    char buf[BUFFER_SIZE];
    char receive_buf[BUFFER_SIZE];
    char *hostAddrp_;
    int optval;
    int n;
    std::string host_ = "10.56.87.59";
    std::string client_ = "10.56.87.2";
    input_frame latest_frame_;
    input_frame prev_frame_;
    input_frame init_pose_;
    output_frame data_frame_;
    bool has_init_pose_ = false;
    bool real_data_started_ = false;

    std::thread data_thread_;
    std::thread recv_thread_;

public:
    Server() {
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
        serverAddr_.sin_addr.s_addr = inet_addr(host_.c_str());
        serverAddr_.sin_port = htons((unsigned short)host_port_);

        if (bind(socket_, ((struct sockaddr *) &serverAddr_), sizeof(serverAddr_)) < 0) {
            std::cout << "ERROR: Couldn't bind socket" << std::endl;
        }

        bzero((char* ) &clientAddr_, sizeof(clientAddr_));
        clientAddr_.sin_family = AF_INET;
        clientAddr_.sin_addr.s_addr = inet_addr(client_.c_str());
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

    std::vector<std::string> split( const std::string& str, char delimiter = ';' ) {
        std::vector<std::string> result ;
        std::istringstream stm(str) ;
        std::string fragment ;
        while( std::getline( stm, fragment, delimiter ) ) result.push_back(fragment) ;
        return result ;
    }

    int send(output_frame &frame) {
        info("Sent frame");
        return send(frame.to_udp_string());
    }

    std::string get_message() {
        std::string s(receive_buf, sizeof(receive_buf));
        std::vector<std::string> values = split(s);
        return s;
    }
    input_frame get_new_frame() {
        std::string s(receive_buf, sizeof(receive_buf));
        std::vector<std::string> values = split(s);
        if (atof(values.at(0).c_str()) == 0) {
            init_pose_ = input_frame(values);
            has_init_pose_ = true;
            return input_frame();
        } else {
            real_data_started_ = true;
            return input_frame(values);
        }
    }

    void receive_frame() {
//        debug("running frame");
        if (receive() > 0) {
            error("No frame");
        } else {
//            input_frame incoming_frame = get_new_frame();
//            if (incoming_frame.millis > latest_frame_.millis && incoming_frame.id == 1) {
//                info("Received frame");
//                prev_frame_ = latest_frame_;
//                latest_frame_ = incoming_frame;
////            double dt = latest_frame_.millis - prev_frame_.millis;
////            std::cout << "[INFO] Frame DT {" << dt << "}\n";
//            }
        }
    }

    input_frame get_latest_frame() {
        return latest_frame_;
    }
    input_frame get_init_pose_frame() {
        return init_pose_;
    }

    bool received_init_pose() {
        return has_init_pose_;
    }

    bool real_data_started() {
        return real_data_started_;
    }

    void set_data_frame(output_frame &frame) {
        data_frame_ = frame;
    }

    void receive_thread() {
        while (true) {
            receive();
            std::string s = "why";
            send(s);
            //receive_frame();
        }
    }
    void data_processing_thread() {
        while (true) {
//            send(data_frame_);
//            if (send(data_frame_) < 0) {
//                error("message failed");
//            } else {
//            }
        }
    }

    void start_thread() {
//        data_thread_ = std::thread(&Server::data_processing_thread, this);
        recv_thread_ = std::thread(&Server::receive_thread, this);
    }
};