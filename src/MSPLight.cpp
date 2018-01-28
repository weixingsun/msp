#include <MSPLight.hpp>
#include "SerialPortImpl.cpp"

namespace msp {

MSPLight::MSPLight(const std::string &device, const unsigned int baudrate) : pimpl(new SerialPortImpl) {
    pimpl->port.open(device);
    pimpl->port.set_option(asio::serial_port::baud_rate(baudrate));
    pimpl->port.set_option(asio::serial_port::parity(asio::serial_port::parity::none));
    pimpl->port.set_option(asio::serial_port::character_size(asio::serial_port::character_size(8)));
    pimpl->port.set_option(asio::serial_port::stop_bits(asio::serial_port::stop_bits::one));
}

bool MSPLight::send(const uint8_t id, const std::vector<uint8_t> &data) {
    std::vector<uint8_t> hdr(5);
    hdr[0] = '$';
    hdr[1] = 'M';
    hdr[2] = '<';
    hdr[3] = uint8_t(data.size());
    hdr[4] = id;
    asio::write(pimpl->port, asio::buffer(hdr.data(), hdr.size()));
    asio::write(pimpl->port, asio::buffer(data.data(), data.size()));
    const uint8_t crc1 = crc(id, data);
    asio::write(pimpl->port, asio::buffer(&crc1, 1));
    return true;
}

std::vector<uint8_t> MSPLight::readn(std::size_t n_bytes) {
    std::vector<uint8_t> data(n_bytes);
    asio::read(pimpl->port, asio::buffer(data.data(), data.size()));
    return data;
}

std::vector<uint8_t> MSPLight::receive() {
    while( char(readn(1)[0]) != '$');

    if(char(readn(1)[0]) != 'M')
        return std::vector<uint8_t>();

    if(char(readn(1)[0]) != '>')
        return std::vector<uint8_t>();

    // read data size
    const uint8_t data_size = readn(1)[0];

    // get ID of msg
    const uint8_t id = readn(1)[0];

    // read payload data
    const std::vector<uint8_t> data = readn(data_size);

    // check CRC
    const uint8_t rcv_crc = readn(1)[0];
    const uint8_t exp_crc = crc(id, data);

    if(rcv_crc!=exp_crc)
        return std::vector<uint8_t>();

    return data;
}

uint8_t MSPLight::crc(const uint8_t id, const std::vector<uint8_t> &data) {
    uint8_t crc = uint8_t(data.size())^id;

    for(uint8_t d : data)
        crc = crc^d;

    return crc;
}

}
