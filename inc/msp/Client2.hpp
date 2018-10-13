#ifndef CLIENT2_HPP
#define CLIENT2_HPP

#include <memory>
#include <types.hpp>
#include <thread>
#include <atomic>
#include <queue>
#include <functional>
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

    void call(const Request &req) {
        callback(dynamic_cast<const T&>(req));
    }

    std::function<void(const T&)> callback;
};

template <typename T>
class Queue {
public:
    Queue(std::size_t size = 0) : max_size(size) { }

    const T & front() {
        return queue.front();
    }

    const T & back() {
        return queue.back();
    }

    void insert(const T & v) {
        // remove the oldest element to fit maximum size
        if(max_size && queue.size()>max_size) {
            queue.pop();
        }
        queue.push(v);
    }

    void remove() {
        queue.pop();
    }

    void setMaximumSize(const std::size_t max_size) {
        // remove oldest elements to fit maximum size
        while(queue.size()>max_size) {
            queue.pop();
        }
        this->max_size = max_size;
    }

private:
    std::queue<T> queue;
    std::size_t max_size;
};

class Client {
public:
    Client();

    ~Client();

    void connect(const std::string &device, const size_t baudrate=115200);

    void disconnect();

    void send(const Response &response, const bool wait_ack=true, const std::function<void(msp::ID)> &callback = nullptr);

    void send(const RawMessage &msg, const bool wait_ack=true, const std::function<void(const RawMessage&)> &callback = nullptr);

    const ByteVector& receive(ID id, const double timeout = 0);

    template<typename T>
    void receive(const std::function<void(const T&)> &callback) {
        // set callback and allocate memory for type
//        next_cb = std::make_shared<Callback<T>>(callback);
//        next_req = std::make_shared<T>();
        next_cb.insert(std::make_shared<Callback<T>>(callback));
        next_req.insert(std::make_shared<T>());
        write(uint8_t(next_req.back()->id()));
    }

    void receive(const uint8_t id, const std::function<void(const ByteVector&)> &callback) {
        // set callback and allocate memory for type
//        next_cb = std::make_shared<Callback<ByteVector>>(callback);
//        next_req = nullptr;
        next_cb.insert(std::make_shared<Callback<ByteVector>>(callback));
        next_req.insert(nullptr);
        write(id);
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
    void start();

    void stop();

    void write(const uint8_t id, const ByteVector& data = ByteVector());

    const RawMessage& read();

private:
    // I/O
    std::unique_ptr<SerialPortImpl> port;

    std::thread reader;
    std::atomic_bool running;

//    std::shared_ptr<Request> next_req;
//    std::shared_ptr<CallbackBase> next_cb;

//    std::queue<std::shared_ptr<Request>> next_req;
//    std::queue<std::shared_ptr<CallbackBase>> next_cb;
    Queue<std::shared_ptr<Request>> next_req;
    Queue<std::shared_ptr<CallbackBase>> next_cb;
};

} // namespace client
} // namespace msp

#endif // CLIENT2_HPP
