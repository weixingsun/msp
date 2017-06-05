#include <Client.hpp>

#include <iostream>

namespace msp {
namespace client {

Client::Client() : port(io), running(false), print_warnings(false) { }

Client::~Client() {
    for(const std::pair<msp::ID, msp::Request*> d : subscribed_requests)
        delete d.second;

    for(const std::pair<msp::ID, SubscriptionBase*> s : subscriptions)
        delete s.second;
}

void Client::connect(const std::string &device, const uint baudrate) {
    port.open(device);

    port.set_option(asio::serial_port::baud_rate(baudrate));
    port.set_option(asio::serial_port::parity(asio::serial_port::parity::none));
    port.set_option(asio::serial_port::character_size(asio::serial_port::character_size(8)));
    port.set_option(asio::serial_port::stop_bits(asio::serial_port::stop_bits::one));
}

void Client::waitForOneMessage() {
    mutex_buffer.lock();
    //parser_state = PREAMBLE;
    //while(parser_state!=END) {
        std::cout << "state: " << parser_state << std::endl;
        // register handler for incomming messages
        switch (parser_state) {
        case PREAMBLE:
            asio::async_read_until(port, buffer, "$M",
                std::bind(&Client::onPreamble, this, std::placeholders::_1, std::placeholders::_2));
            break;
        case HEADER:
            //exit(EXIT_FAILURE);
            asio::async_read(port, buffer, asio::transfer_exactly(3),
                std::bind(&Client::onHeader, this, std::placeholders::_1, std::placeholders::_2));
            break;
        case PAYLOAD_CRC:
            asio::async_read(port, buffer, asio::transfer_exactly(request_received->length+1),
                std::bind(&Client::onDataCRC, this, std::placeholders::_1, std::placeholders::_2));
            break;
        case PROCESS:
            onProcess();
            break;
        case END:
            parser_state = PREAMBLE;
            break;
        }
        // wait for incomming data
        io.run();
        //io.run_one();
        //io.poll_one();
        io.reset();
        std::cout << "handeled: " << parser_state << std::endl;
    //}jjj
    mutex_buffer.unlock();
}

void Client::start() {
    thread = std::thread([this]{
        parser_state = PREAMBLE;
        running = true;
        while(running) { waitForOneMessage(); }
    });
}

void Client::stop() {
    running = false;
    io.stop();
    thread.join();
}

bool Client::sendData(const uint8_t id, const ByteVector &data) {
    std::lock_guard<std::mutex> lock(mutex_send);
    ByteVector msg;
    msg.push_back('$');                                 // preamble1
    msg.push_back('M');                                 // preamble2
    msg.push_back('<');                                 // direction
    msg.push_back(uint8_t(data.size()));                // data size
    msg.push_back(id);                                  // message_id
    msg.insert(msg.end(), data.begin(), data.end());    // data
    msg.push_back( crc(id, data) );                     // crc

    const std::size_t bytes_written = asio::write(port, asio::buffer(msg.data(), msg.size()));

    return (bytes_written==msg.size());
}

bool Client::request(msp::Request &request, const double timeout) {
    msp::ByteVector data;
    const bool success = request_raw(uint8_t(request.id()), data, timeout);
    if(success) { request.decode(data); }
    return success;
}

bool Client::request_raw(const uint8_t id, ByteVector &data, const double timeout) {
    // send request
    if(!sendRequest(id)) { return false; }

    // wait for thread to received message
    std::unique_lock<std::mutex> lock(mutex_cv_request);
    const auto predicate = [&]{
        mutex_request.lock();
        const bool received = (request_received!=NULL) && (request_received->id==id);
        // unlock to wait for next message
        if(!received) { mutex_request.unlock(); }
        return received;
    };

    if(timeout>0) {
        if(!cv_request.wait_for(lock, std::chrono::milliseconds(uint(timeout*1e3)), predicate))
            return false;
    }
    else {
        cv_request.wait(lock, predicate);
    }

    // check message status and decode
    const bool success = request_received->status==OK;
    if(success) { data = request_received->data; }
    mutex_request.unlock();
    return success;
}

bool Client::respond(const msp::Response &response, const bool wait_ack) {
    return respond_raw(uint8_t(response.id()), response.encode(), wait_ack);
}

bool Client::respond_raw(const uint8_t id, const ByteVector &data, const bool wait_ack) {
    // send response
    if(!sendData(id, data)) { return false; }

    if(!wait_ack)
        return true;

    // wait for thread to received message
    std::unique_lock<std::mutex> lock(mutex_cv_ack);
    cv_ack.wait(lock, [&]{
        mutex_request.lock();
        const bool received = (request_received!=NULL) && (request_received->id==id);
        // unlock to wait for next message
        if(!received) { mutex_request.unlock(); }
        return received;
    });

    // check status, expect ACK without payload
    const bool success = request_received->status==OK;
    mutex_request.unlock();
    return success;
}

uint8_t Client::crc(const uint8_t id, const ByteVector &data) {
    uint8_t crc = uint8_t(data.size())^id;
    for(const uint8_t d : data) { crc = crc^d; }
    return crc;
}

void Client::onPreamble(const asio::error_code& error, const std::size_t bytes_transferred) {
    std::cout << "preamble" << std::endl;
    std::cout << "preamble: " << parser_state << std::endl;
    if(error) {
        //std::cout << "preamble err" << std::endl;
        std::cerr << "onPreamble: " << error.message() << std::endl;
        //std::cerr << error << std::endl;
        return;
    }

    // ignore and remove header bytes
    buffer.consume(bytes_transferred);

    parser_state = HEADER;
    std::cout << "preamble" << std::endl;
}

void Client::onHeader(const asio::error_code& error, const std::size_t bytes_transferred) {
    if(error) {
        std::cerr << "onHeader: " << error.message() << std::endl;
        parser_state = END;
        return;
    }
    //exit(EXIT_FAILURE);

    MessageStatus status = OK;

    // message direction
    const uint8_t dir = uint8_t(buffer.sbumpc());
    const bool ok_id = (dir!='!');

    // payload length
    const uint8_t len = uint8_t(buffer.sbumpc());

    // message ID
    const uint8_t id = uint8_t(buffer.sbumpc());

    if(!ok_id) { status = FAIL_ID; }

    mutex_request.lock();
    request_received.reset(new ReceivedMessage());
    request_received->id = id;
    request_received->length = len;
    request_received->status = status;
    mutex_request.unlock();

    if(print_warnings && !ok_id) {
        std::cerr << "Message with ID " << uint(id) << " is not recognised!" << std::endl;
    }

    parser_state = ok_id ? PAYLOAD_CRC : END;

    ////////

//    // payload
//    std::vector<uint8_t> data;
//    for(uint i(0); i<len; i++) { data.push_back(uint8_t(buffer.sbumpc())); }

//    // CRC
//    const uint8_t rcv_crc = uint8_t(buffer.sbumpc());
//    const uint8_t exp_crc = crc(id,data);
//    const bool ok_crc = (rcv_crc==exp_crc);

//    if(print_warnings && !ok_crc) {
//        std::cerr << "Message with ID " << uint(id) << " has wrong CRC! (expected: " << uint(exp_crc) << ", received: "<< uint(rcv_crc) << ")" << std::endl;
//    }

//    if(!ok_id) { status = FAIL_ID; }
//    else if(!ok_crc) { status = FAIL_CRC; }

//    mutex_request.lock();
//    request_received.reset(new ReceivedMessage());
//    request_received->id = id;
//    request_received->data = data;
//    request_received->status = status;
//    mutex_request.unlock();

//    // notify waiting request methods
//    cv_request.notify_one();
//    // notify waiting respond methods
//    cv_ack.notify_one();

//    // check subscriptions
//    mutex_callbacks.lock();
//    if(status==OK && subscriptions.count(ID(id))) {
//        // fetch message type, decode payload
//        msp::Request *const req = subscribed_requests.at(ID(id));
//        req->decode(data);
//        // call callback
//        subscriptions.at(ID(id))->call(*req);
//    }
//    mutex_callbacks.unlock();
}

void Client::onDataCRC(const asio::error_code& error, const std::size_t bytes_transferred) {
    if(error) {
        parser_state = END;
        return;
    }

    // payload
    std::vector<uint8_t> data;
    for(uint i(0); i<request_received->length; i++) {
        data.push_back(uint8_t(buffer.sbumpc()));
    }

    // CRC
    const uint8_t rcv_crc = uint8_t(buffer.sbumpc());
    const uint8_t exp_crc = crc(request_received->id,data);
    const bool ok_crc = (rcv_crc==exp_crc);

    if(print_warnings && !ok_crc) {
        std::cerr << "Message with ID " << uint(request_received->id) << " has wrong CRC! (expected: " << uint(exp_crc) << ", received: "<< uint(rcv_crc) << ")" << std::endl;
    }

    MessageStatus status = request_received->status;
    if(request_received->status==OK && !ok_crc) {
        status = FAIL_CRC;
    }

//    if(!ok_id) { status = FAIL_ID; }
//    else if(!ok_crc) { status = FAIL_CRC; }

    mutex_request.lock();
    request_received->data = data;
    request_received->status = status;
    mutex_request.unlock();

    parser_state = ok_crc ? PROCESS : END;
}

void Client::onProcess() {
    // notify waiting request methods
    cv_request.notify_one();
    // notify waiting respond methods
    cv_ack.notify_one();

    // check subscriptions
    mutex_callbacks.lock();
    if(request_received->status==OK && subscriptions.count(ID(request_received->id))) {
        // fetch message type, decode payload
        msp::Request *const req = subscribed_requests.at(ID(request_received->id));
        req->decode(request_received->data);
        // call callback
        subscriptions.at(ID(request_received->id))->call(*req);
    }
    mutex_callbacks.unlock();

    parser_state = PREAMBLE;

    exit(EXIT_FAILURE);
}

} // namespace client
} // namespace msp
