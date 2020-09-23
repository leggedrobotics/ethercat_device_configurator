#include "ethercat_device_configurator/EthercatDeviceConfigurator.hpp"

#include <thread>

std::unique_ptr<std::thread> worker_thread;
bool abrt = false;

EthercatDeviceConfigurator::SharedPtr configurator;

void worker()
{
    while(!abrt)
    {
        for(const auto & master: configurator->getMasters() )
        {
            master->update();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int main(int argc, char**argv)
{
    configurator = std::make_shared<EthercatDeviceConfigurator>(argv[1]);


    for(auto & master: configurator->getMasters())
    {
        master->startup();
    }

    worker_thread = std::make_unique<std::thread>(&worker);


    for(auto & slave: configurator->getSlaves())
    {
       std::cout << " " << slave->getName() << ": " << slave->getAddress() << std::endl;
    }
    std::cout << "Startup finished" << std::endl;
    while(true)
    {

    }
}
