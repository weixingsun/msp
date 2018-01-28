#include <MSPLight.hpp>

#include <chrono>
#include <ctime>
#include <iostream>
#include <boost/timer/timer.hpp>

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    msp::MSPLight msp(device, baudrate);

    const int dur_wall_s = 10;

    unsigned int n_msg = 0;

    // cpu-time start
    const clock_t tstart_cpu = clock();
    // wall-time start
    const auto start = std::chrono::steady_clock::now();

    boost::timer::auto_cpu_timer t;

    while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count() < dur_wall_s*1000) {
        msp.send(102);

        // wait for response
        std::vector<uint8_t> payload;
        while(payload.size()==0) {
            payload = msp.receive();
        }

        n_msg++;
    }

    const auto dur_cpu_s = float(clock()-tstart_cpu) / CLOCKS_PER_SEC;
    std::cout << "CPU time: " << dur_cpu_s << std::endl;
    std::cout << "load: " << dur_cpu_s / dur_wall_s << std::endl;
    std::cout<<"messages per second: "<<n_msg/float(dur_wall_s)<<" Hz"<<std::endl;
}
