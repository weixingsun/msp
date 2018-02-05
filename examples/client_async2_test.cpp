#include <Client.hpp>
#include <msp_msg.hpp>
#include <msg_print.hpp>
#include <iostream>

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    msp::client::Client client;
    client.connect(device, baudrate);

//    client.async_request_raw(uint8_t(msp::ID::MSP_RAW_IMU), msp::ByteVector(), true,
//    [](const msp::ByteVector &payload){
//        std::cout << "got " << payload.size() << " bytes" << std::endl;
//    });

    client.async_request<msp::msg::ImuRaw>([](const msp::msg::ImuRaw &imu){
        std::cout << imu << std::endl;
    });

//    client.async_request<msp::msg::ApiVersion>([](const msp::msg::ApiVersion &imu){
//        std::cout << imu << std::endl;
//    });

    client.run();

    //std::this_thread::sleep_for(std::chrono::seconds(10));
}
