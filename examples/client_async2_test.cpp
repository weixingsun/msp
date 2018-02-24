#include <Client.hpp>
#include <msp_msg.hpp>
#include <msg_print.hpp>
#include <iostream>
#include <boost/timer/timer.hpp>

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    const int dur_wall_s = 5;

    msp::client::Client client;
    client.connect(device, baudrate);
    client.start2();

    const clock_t tstart_cpu = clock();
    const auto start = std::chrono::steady_clock::now();
    boost::timer::auto_cpu_timer t;

    unsigned int n_msg = 0;

    while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count() < dur_wall_s*1000) {
        client.async_request<msp::msg::ImuRaw>([&n_msg](const msp::msg::ImuRaw &imu){
            //std::cout << imu << std::endl;
            n_msg++;
//            std::cout << "in: " << n_msg << std::endl;
        });
//        std::cout << "out: " << n_msg << std::endl;
        //client.run();
//        const size_t hh = client.io().run();
//        if(hh>0)
//            std::cout << hh << std::endl;
        //client.io().poll();
        //std::this_thread::sleep_for(std::chrono::microseconds(1));
        //std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    //std::this_thread::sleep_for(std::chrono::seconds(2));

    const auto dur_cpu_s = float(clock()-tstart_cpu) / CLOCKS_PER_SEC;
    std::cout << "CPU time: " << dur_cpu_s << std::endl;
    std::cout << "load: " << dur_cpu_s / dur_wall_s << std::endl;
    std::cout<<"messages per second: "<<n_msg/float(dur_wall_s)<<" Hz"<<std::endl;


//    client.async_request<msp::msg::ApiVersion>([](const msp::msg::ApiVersion &api){
//        std::cout << api << std::endl;
//    });

//    std::this_thread::sleep_for(std::chrono::seconds(2));

    client.stop2();
}
