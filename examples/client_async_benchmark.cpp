#include <Client.hpp>
#include <iostream>

#include <msp_msg.hpp>

#include <boost/timer/timer.hpp>

static bool running = true;

void onExit(int /*signal*/) { running = false; }

int main(int argc, char *argv[]) {
    const std::string device = (argc>1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    const int dur_wall_s = 20;

    msp::client::Client client;
    client.connect(device, baudrate);
    client.start();

    const clock_t tstart_cpu = clock();
    boost::timer::auto_cpu_timer t;
    unsigned int n_msg = 0;

    // using lambda callback with stored lambda object
    const auto imu_cb1 = [&n_msg](const msp::msg::ImuRaw& imu){
        //std::cout<<msp::msg::ImuSI(imu, 512.0, 1.0/4.096, 0.92f/10.0f, 9.80665f);
        n_msg++;
    };
    client.subscribe<msp::msg::ImuRaw>(imu_cb1, 0.0001);

    std::this_thread::sleep_for(std::chrono::milliseconds(dur_wall_s*1000));

    // Ctrl+C to quit
    //std::cin.get();

    const auto dur_cpu_s = float(clock()-tstart_cpu) / CLOCKS_PER_SEC;
    std::cout << "CPU time: " << dur_cpu_s << std::endl;
    std::cout << "load: " << dur_cpu_s / dur_wall_s << std::endl;
    std::cout<<"messages per second: "<<n_msg/float(dur_wall_s)<<" Hz"<<std::endl;

//    client.stop();

    std::cout << "DONE" << std::endl;
}
