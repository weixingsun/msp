#include <Client.hpp>
#include <msp_msg.hpp>

#include <boost/timer/timer.hpp>

#include <iostream>

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    const int dur_wall_s = 5;

    msp::client::Client client;
    client.setPrintWarnings(true);
    client.connect(device, baudrate);
    client.start();

    msp::msg::ImuRaw imu_raw;

    const clock_t tstart_cpu = clock();
    const auto start = std::chrono::steady_clock::now();
    boost::timer::auto_cpu_timer t;

    unsigned int n_msg = 0;

    while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count() < dur_wall_s*1000) {
        if(client.request(imu_raw)==1) {
            n_msg++;
        }
    }

    const auto dur_cpu_s = float(clock()-tstart_cpu) / CLOCKS_PER_SEC;
    std::cout << "CPU time: " << dur_cpu_s << std::endl;
    std::cout << "load: " << dur_cpu_s / dur_wall_s << std::endl;
    std::cout<<"messages per second: "<<n_msg/float(dur_wall_s)<<" Hz"<<std::endl;

    //client.stop();
}
