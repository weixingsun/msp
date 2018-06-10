#ifndef CLIENT2_HPP
#define CLIENT2_HPP

#include <memory>
#include <types.hpp>
#include <thread>
#include <atomic>

#include <iostream>

namespace msp {
namespace client2 {

struct SerialPortImpl;

class CallbackBase {
public:
    virtual void call(const msp::Request &req) = 0;

    virtual ~CallbackBase();
};

CallbackBase::~CallbackBase() { }

template <typename T>
class Callback : public CallbackBase {
public:
    typedef std::function<void(const T&)> callback_t;

    Callback(const callback_t &cb) : callback(cb) { }

    void call(const msp::Request &req) {
        callback(dynamic_cast<const T&>(req));
    }

private:
    std::function<void(const T&)> callback;
};

class Client {
// methods
public:
    Client();

    ~Client();

    void connect(const std::string &device, const size_t baudrate=115200);

    void disconnect();

    void start() {
        reader = std::thread([this]{
            running = true;
            while(running) {
                const RawMessage& msg = read();
                std::cout << "addr: " << &msg.data[0] << std::endl;
                std::cout << "got: " << uint(msg.id) << " " << uint(msg.data.size()) << std::endl;
                next_req->decode(msg.data);
                next_cb->call(*next_req);
            }
        });
    }

    void stop() {
        running = false;
    }

    void send(const Response &response, const bool wait_ack=true, const std::function<void(msp::ID)> &callback = nullptr);

    void send(const RawMessage &msg, const bool wait_ack=true, const std::function<void(const RawMessage&)> &callback = nullptr);

    const ByteVector& receive(ID id, const double timeout = 0);

    template<typename T>
    void receive(const std::function<void(const T&)> &callback) {
        // set callback and allocate memory for type
        next_cb = std::make_shared<Callback<T>>(callback);
        next_req = std::make_shared<T>();
        write(uint8_t(next_req->id()));
    }

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

    const RawMessage& read();

private:
    // I/O
    std::unique_ptr<SerialPortImpl> port;

    std::thread reader;
    std::atomic_bool running;

    std::shared_ptr<Request> next_req;
    //std::shared_ptr<const std::function<void(const std::shared_ptr<const Request>)> > next_cb;
    std::shared_ptr<CallbackBase> next_cb;
};

} // namespace client
} // namespace msp

#endif // CLIENT2_HPP
