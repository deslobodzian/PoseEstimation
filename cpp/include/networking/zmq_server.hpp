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
    std::string _address = "tcp://0.0.0.0:27001";
    zmq::context_t _ctx;
public:
    ZMQServer() {
        _socket = zmq::socket_t(_ctx, zmq::socket_type::dealer);
        // bind socket
        info("Binding Socket: " + _address);
        _socket.bind(_address);
        info("Bind success");
    }

    ~ZMQServer() = default;

    void send_message(std::string message) {
        info("sending");
        _socket.send(zmq::buffer(message), zmq::send_flags::dontwait);
    }

    void receive_message() {
        zmq::message_t incoming_frame;
        _socket.recv(incoming_frame, zmq::recv_flags::none);
        info("Incoming frame {" + incoming_frame.str() + "}");
    }
};
#endif //POSE_ESTIMATION_ZMQ_SERVER_HPP
