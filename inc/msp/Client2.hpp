#ifndef CLIENT2_HPP
#define CLIENT2_HPP

#include <memory>
#include <types.hpp>

namespace msp {
namespace client2 {

struct SerialPortImpl;

class Client {
// methods
public:
    Client();

    ~Client();

    void connect(const std::string &device, const size_t baudrate=115200);

    void disconnect();

    void start();

    void stop();

    void send(const Response &response, const bool wait_ack=true, const std::function<void(const msp::ID&)> &callback = nullptr) {

    }

    void send(const RawMessage &response, const bool wait_ack=true, const std::function<void(const RawMessage&)> &callback = nullptr);

    template<typename T>
    void receive(Request &request, const double timeout = 0, const std::function<void(const T&)> &callback = nullptr) {

    }

    void receive(RawMessage &request, const double timeout = 0, const std::function<void(const RawMessage&)> &callback = nullptr);

    template<typename T>
    T receive(msp::ID &id, const double timeout = 0, const std::function<void(const T&)> &callback = nullptr) {

    }

    ByteVector receive(ID &id, const double timeout = 0);

private:

// members
private:
    // I/O
    std::unique_ptr<SerialPortImpl> port;
};

} // namespace client
} // namespace msp

#endif // CLIENT2_HPP
