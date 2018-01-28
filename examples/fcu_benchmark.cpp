#include <FlightController.hpp>
#include <msp_msg.hpp>

#include <boost/timer/timer.hpp>

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    const int dur_wall_s = 10;

    fcu::FlightController fcu(device, baudrate);

    fcu.initialise();

    // cpu-time start
    const clock_t tstart_cpu = clock();

    boost::timer::auto_cpu_timer t;

    unsigned int n_msg = 0;

    fcu.subscribe<msp::msg::ImuRaw>([&n_msg](const msp::msg::ImuRaw& imu){
        n_msg++;
    }, 0.0001);

    std::this_thread::sleep_for(std::chrono::milliseconds(dur_wall_s*1000));

    const auto dur_cpu_s = float(clock()-tstart_cpu) / CLOCKS_PER_SEC;
    std::cout << "CPU time: " << dur_cpu_s << std::endl;
    std::cout << "load: " << dur_cpu_s / dur_wall_s << std::endl;
    std::cout<<"messages per second: "<<n_msg/float(dur_wall_s)<<" Hz"<<std::endl;
}
