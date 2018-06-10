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

    void send(const Response &response, const bool wait_ack=true, const std::function<void(msp::ID)> &callback = nullptr);

    void send(const RawMessage &msg, const bool wait_ack=true, const std::function<void(const RawMessage&)> &callback = nullptr);

    const ByteVector& receive(ID id, const double timeout = 0);

    template<typename M>
    const std::shared_ptr<M> receive(const double timeout = 0) {
        std::shared_ptr<M> m(new M());
        const ByteVector& data = receive(m->id(), timeout);
        m->decode(data);
        return m;
    }

    static uint8_t crc(const uint8_t id, const ByteVector& data);

private:
    void write(const uint8_t id, const ByteVector& data = ByteVector());

    const ByteVector &read();

private:
    // I/O
    std::unique_ptr<SerialPortImpl> port;
};

} // namespace client
} // namespace msp

#endif // CLIENT2_HPP
