// MSP message definitions
// http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol

#ifndef MSP_MSG_HPP
#define MSP_MSG_HPP

#include <vector>

namespace msp {

struct Message {
    const uint8_t id;
    Message(uint8_t id) : id(id) {}
};

// send to FC
struct Request : public Message {
    using Message::Message;
    virtual void decode(const std::vector<uint8_t> &data) = 0;
};

// received from FC
struct Response : public Message {
    using Message::Message;
    virtual std::vector<uint8_t> &encode() = 0;
};

// MSP_IDENT: 100
struct Ident : public Request {
    uint8_t     version;
    uint8_t     type;
    uint8_t     msp_version;
    uint32_t    capability;

    Ident() : Request(100) {}

    void decode(const std::vector<uint8_t> &data) {
        version     = data[0];
        type        = data[1];
        msp_version = data[2];
        capability  = (data[3]<<0) | (data[4]<<8) |
                      (data[5]<<16) | (data[6]<<24);
    }
};

// MSP_STATUS: 101
struct Status : public Request {
    uint16_t    time;   // in us
    uint16_t    i2c_errors_count;
    uint16_t    sensor;
    uint32_t    flag;
    uint8_t     current_setting;

    Status() : Request(101) {}

    void decode(const std::vector<uint8_t> &data) {
        time                = (data[0]<<0) | (data[1]<<8);
        i2c_errors_count    = (data[2]<<0) | (data[3]<<8);
        sensor              = (data[4]<<0) | (data[5]<<8);
        flag                = (data[6]<<0) | (data[7]<<8) |
                              (data[8]<<16) | (data[9]<<24);
        current_setting     = data[10];
    }
};

// MSP_RAW_IMU: 102
struct RawImu : public Request {
    int16_t accx;
    int16_t accy;
    int16_t accz;

    int16_t gyrx;
    int16_t gyry;
    int16_t gyrz;

    int16_t magx;
    int16_t magy;
    int16_t magz;

    RawImu() : Request(102) {}

    void decode(const std::vector<uint8_t> &data) {
        accx = (data[0]<<0) | (data[1]<<8);
        accy = (data[2]<<0) | (data[3]<<8);
        accz = (data[4]<<0) | (data[5]<<8);

        gyrx = (data[6]<<0) | (data[7]<<8);
        gyry = (data[8]<<0) | (data[9]<<8);
        gyrz = (data[10]<<0) | (data[11]<<8);

        magx = (data[12]<<0) | (data[13]<<8);
        magy = (data[14]<<0) | (data[15]<<8);
        magz = (data[16]<<0) | (data[17]<<8);
    }
};

} // namespace msp

#endif // MSP_MSG_HPP