//
// Created by DSlobodzian on 4/19/2022.
//

#ifndef POSE_ESTIMATION_ZMQ_SERVER_HPP
#define POSE_ESTIMATION_ZMQ_SERVER_HPP

#include "zmq/zmq.hpp"
#include <iostream>
#include "utils.hpp"

class ZMQServer {
private:
    zmq::socket_t _socket;
    std::string _address = "tcp://*:27001";
public:
    ZMQServer() {
        zmq::context_t ctx;
        _socket = zmq::socket_t(ctx, zmq::socket_type::pair);
        // bind socket
        info("Binding Socket: " + _address);
        _socket.bind(_address);
        info("Bind success");
    }
    ~ZMQServer() = default;

    void send_message(std::string message) {
        _socket.send(zmq::buffer(message), zmq::send_flags::dontwait);
    }

    void receive_message() {
        zmq::message_t incoming_frame;
        _socket.recv(incoming_frame, zmq::recv_flags::none);
        info("Incoming frame {" + incoming_frame.str() + "}");
    }
};
#endif //POSE_ESTIMATION_ZMQ_SERVER_HPP
