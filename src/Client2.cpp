#include <Client2.hpp>
#include <asio.hpp>

namespace msp {
namespace client2 {

struct SerialPortImpl {
    SerialPortImpl() : port(io) { }

    asio::io_service io;     ///<! io service
    asio::serial_port port;  ///<! port for serial device
};

Client::Client() : port(new SerialPortImpl) {

}

Client::~Client() {

}

void Client::connect(const std::string &device, const size_t baudrate) {
    port->port.open(device);

    port->port.set_option(asio::serial_port::baud_rate(baudrate));
    port->port.set_option(asio::serial_port::parity(asio::serial_port::parity::none));
    port->port.set_option(asio::serial_port::character_size(asio::serial_port::character_size(8)));
    port->port.set_option(asio::serial_port::stop_bits(asio::serial_port::stop_bits::one));
}

void Client::disconnect() { }

void Client::start() { }

void Client::stop() { }

void send(const RawMessage &response, const bool wait_ack, const std::function<void(const RawMessage&)> &callback) {

}

//void Client::receive(msp::Request &request, const double timeout) {

//}

} // namespace client
} // namespace msp
