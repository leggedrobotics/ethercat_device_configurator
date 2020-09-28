#include "ethercat_device_configurator/EthercatDeviceConfigurator.hpp"

#include <thread>

#include <anydrive/Anydrive.hpp>
#include <rokubi_rsl_ethercat_sdk/Rokubi.hpp>
#include <elmo_ethercat_sdk/Elmo.hpp>
#include <csignal>
std::unique_ptr<std::thread> worker_thread;
bool abrt = false;

EthercatDeviceConfigurator::SharedPtr configurator;

unsigned int counter = 0;
void worker()
{
    while(!abrt)
    {
        for(const auto & master: configurator->getMasters() )
        {
            master->update(ecat_master::UpdateMode::StandaloneEnforceRate);
        }

        for(const auto & slave:configurator->getSlaves())
        {
            if(configurator->getInfoForSlave(slave).type == EthercatDeviceConfigurator::EthercatSlaveType::Anydrive)
            {

                anydrive::AnydriveEthercatSlave::SharedPtr any_slave_ptr = std::dynamic_pointer_cast<anydrive::AnydriveEthercatSlave>(slave);

                if(any_slave_ptr->getActiveStateEnum() == anydrive::fsm::StateEnum::ControlOp)
                {
                    anydrive::Command cmd;
                    cmd.setModeEnum(anydrive::mode::ModeEnum::MotorVelocity);
                    cmd.setMotorVelocity(10);

                    any_slave_ptr->setCommand(cmd);
                }

            }
            else if(configurator->getInfoForSlave(slave).type == EthercatDeviceConfigurator::EthercatSlaveType::Rokubi)
            {
                std::shared_ptr<rokubi::Rokubi> rokubi_slave_ptr = std::dynamic_pointer_cast<rokubi::Rokubi>(slave);
                // Do things with the Rokubi sensors here
            }
            else if(configurator->getInfoForSlave(slave).type == EthercatDeviceConfigurator::EthercatSlaveType::Elmo)
            {
                std::shared_ptr<elmo::Elmo> elmo_slave_ptr = std::dynamic_pointer_cast<elmo::Elmo>(slave);
                // Do things with the Elmo drives here
            }
        }
        counter++;
    }
}

void signal_handler(int sig)
{
    for(const auto & master: configurator->getMasters())
    {
        master->preShutdown();
    }

    abrt = true;
    worker_thread->join();
    for(const auto & master: configurator->getMasters())
    {
        master->shutdown();
    }
    std::cout << "Shutdown" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    exit(0);
}

void anydriveReadingCb(const std::string& name, const anydrive::ReadingExtended& reading)
{
    std::cout << "Reading of anydrive '" << name << "'\n"
              << "Joint velocity: " << reading.getState().getJointVelocity() << "\n\n";
}
void rokubiReadingCb(const std::string& name, const rokubi::Reading& reading)
{
    std::cout << "Reading of rokubi '" << name << "'\n"
              << "Force X: " << reading.getForceX() << "\n\n";
}

int main(int argc, char**argv)
{
    configurator = std::make_shared<EthercatDeviceConfigurator>(argv[1]);

    for(const auto& device : configurator->getSlavesOfType<anydrive::AnydriveEthercatSlave>())
    {
        device->addReadingCb(anydriveReadingCb);
    }
    for(const auto& device : configurator->getSlavesOfType<rokubi::Rokubi>()){
        device->addReadingCb(rokubiReadingCb);
    }

    std::signal(SIGINT, signal_handler);
    for(auto & master: configurator->getMasters())
    {
        master->startup();
    }

    worker_thread = std::make_unique<std::thread>(&worker);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for(auto & slave: configurator->getSlaves())
    {
        std::cout << " " << slave->getName() << ": " << slave->getAddress() << std::endl;
        if(configurator->getInfoForSlave(slave).type == EthercatDeviceConfigurator::EthercatSlaveType::Anydrive)
        {

            anydrive::AnydriveEthercatSlave::SharedPtr any_slave_ptr = std::dynamic_pointer_cast<anydrive::AnydriveEthercatSlave>(slave);

            any_slave_ptr->setFSMGoalState(anydrive::fsm::StateEnum::ControlOp, false,0,0);
            std::cout << "Putting slave into operational mode: " << any_slave_ptr->getName() << " : " << any_slave_ptr->getAddress() << std::endl;

        }
    }


    std::cout << "Startup finished" << std::endl;
    while(true)
    {

    }
}
