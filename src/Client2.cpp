#include <Client2.hpp>
#include <asio.hpp>

#include <iostream>

namespace msp {
namespace client2 {

struct SerialPortImpl {
    SerialPortImpl() : port(io) {
        payload.reserve(256); // MSPv1
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

    ByteVector payload;

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

Client::~Client() { }

void Client::connect(const std::string &device, const size_t baudrate) {
    port->port.open(device);

    port->port.set_option(asio::serial_port::baud_rate(baudrate));
    port->port.set_option(asio::serial_port::parity(asio::serial_port::parity::none));
    port->port.set_option(asio::serial_port::character_size(asio::serial_port::character_size(8)));
    port->port.set_option(asio::serial_port::stop_bits(asio::serial_port::stop_bits::one));
}

void Client::disconnect() { }

//void Client::start() { }

//void Client::stop() { }

void Client::send(const Response &response, const bool wait_ack, const std::function<void(msp::ID)> &callback) {
    //
}

void Client::send(const RawMessage &msg, const bool wait_ack, const std::function<void(const RawMessage&)> &callback) {
    write(msg.id, msg.data);
}

const ByteVector& Client::receive(ID id, const double timeout) {
    write(uint8_t(id));

    const ByteVector& data = read();

    return data;
}

void Client::write(const uint8_t id, const ByteVector& data) {
    asio::write(port->port, asio::buffer(port->h));
    asio::write(port->port, asio::buffer({uint8_t(data.size())}));
    asio::write(port->port, asio::buffer({id}));
    asio::write(port->port, asio::buffer(data));
    asio::write(port->port, asio::buffer({crc(id, data)}));
}

const ByteVector& Client::read() {
    asio::read(port->port, asio::buffer(port->header.data));

    port->payload.resize(port->header.header.size);
    asio::read(port->port, asio::buffer(port->payload));

    return port->payload;
}

uint8_t Client::crc(const uint8_t id, const ByteVector& data) {
    uint8_t crc = uint8_t(data.size())^id;
    for(const uint8_t d : data) { crc = crc^d; }
    return crc;
}

} // namespace client
} // namespace msp
