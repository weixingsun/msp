#include <Client2.hpp>
#include <asio.hpp>

#include <iostream>

namespace msp {
namespace client2 {

struct SerialPortImpl {
    SerialPortImpl() : port(io) {
        msg_in.data.resize(256);
//        payload.reserve(256); // MSPv1
    }

    asio::io_service io;     ///<! io service
    asio::serial_port port;  ///<! port for serial device

    const std::vector<uint8_t> h = {'$','M','<'};

    // MSPv1 header
    union {
        std::array<uint8_t, 5> data;
        struct {
            uint8_t preamble[2];
            uint8_t direction;
            uint8_t size;
            uint8_t id;
        } header;
    } header;

//    ByteVector payload;
    RawMessage msg_in;

//    // MSPv2 header
//    union Hv2 {
//        std::array<uint8_t, 8> data;
//        struct h {
//            uint8_t preamble[2];
//            uint8_t direction;
//            uint8_t flag;
//            uint16_t id;
//            uint16_t size;
//        };
//    };
};

Client::Client() : port(new SerialPortImpl) { }

Client::~Client() {
    disconnect();
}

void Client::connect(const std::string &device, const size_t baudrate) {
    port->port.open(device);

    port->port.set_option(asio::serial_port::baud_rate(baudrate));
    port->port.set_option(asio::serial_port::parity(asio::serial_port::parity::none));
    port->port.set_option(asio::serial_port::character_size(asio::serial_port::character_size(8)));
    port->port.set_option(asio::serial_port::stop_bits(asio::serial_port::stop_bits::one));

    start();
}

void Client::disconnect() {
    stop();
    port->port.close();
}

void Client::start() {
    reader = std::thread([this]{
        running = true;
        while(running) {
            const RawMessage& msg = read();
            //std::cout << "addr: " << &msg.data[0] << std::endl;
            std::cout << "got: " << uint(msg.id) << " " << uint(msg.data.size()) << std::endl;
            if(next_req.front()) {
                // decode and call callback
                next_req.front()->decode(msg.data);
                next_cb.front()->call(*next_req.front());
                //next_cb->call(*(next_req.front()->get()));
            }
            else {
                std::cout << "raw msg" << std::endl;
                // call callback with raw data
                static_cast<Callback<ByteVector>&>(*next_cb.front()).callback(msg.data);
            }
            next_req.pop();
            next_cb.pop();
        }
    });
}

void Client::stop() {
    running = false;
}

void Client::send(const Response &response, const bool wait_ack, const std::function<void(msp::ID)> &callback) {
    //
}

void Client::send(const RawMessage &msg, const bool wait_ack, const std::function<void(const RawMessage&)> &callback) {
    write(msg.id, msg.data);
}

const ByteVector& Client::receive(ID id, const double timeout) {
    write(uint8_t(id));

    const RawMessage& msg = read();

    return msg.data;
}

void Client::write(const uint8_t id, const ByteVector& data) {
    asio::write(port->port, asio::buffer(port->h));
    asio::write(port->port, asio::buffer({uint8_t(data.size())}));
    asio::write(port->port, asio::buffer({id}));
    asio::write(port->port, asio::buffer(data));
    asio::write(port->port, asio::buffer({crc(id, data)}));
}

const RawMessage& Client::read() {
    uint8_t c;
    // find start of header
    while(true) {
        // find '$'
        for(c=0; c!='$'; asio::read(port->port, asio::buffer(&c,1)));
        // check 'M'
        asio::read(port->port, asio::buffer(&c,1));
        if(c=='M') { break; }
    }

    asio::read(port->port, asio::buffer(&c,1));
    std::cout << "dir: " << c << std::endl;

    asio::read(port->port, asio::buffer(&c,1));
    std::cout << "size: " << std::hex << uint(c) << std::endl;
    port->header.header.size = c;

    asio::read(port->port, asio::buffer(&c,1));
    std::cout << "id: " << std::hex << uint(c) << std::endl;
    port->msg_in.id = uint8_t(c);

    port->msg_in.data.resize(port->header.header.size);
    asio::read(port->port, asio::buffer(port->msg_in.data));

    return port->msg_in;
}

uint8_t Client::crc(const uint8_t id, const ByteVector& data) {
    uint8_t crc = uint8_t(data.size())^id;
    for(const uint8_t d : data) { crc = crc^d; }
    return crc;
}

} // namespace client
} // namespace msp
