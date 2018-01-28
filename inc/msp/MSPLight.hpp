#ifndef MSPLIGHT_HPP
#define MSPLIGHT_HPP

#include <memory>
#include <vector>

namespace msp {

struct SerialPortImpl;

class MSPLight {
public:
    MSPLight(const std::string &device, const unsigned int baudrate);

    ~MSPLight() {}

    bool send(const uint8_t id, const std::vector<uint8_t> &data = std::vector<uint8_t>());

    std::vector<uint8_t> receive();

private:
    std::vector<uint8_t> readn(std::size_t n_bytes);

    uint8_t crc(const uint8_t id, const std::vector<uint8_t> &data);

    std::shared_ptr<SerialPortImpl> pimpl;
};

}

#endif // MSPLIGHT_HPP
