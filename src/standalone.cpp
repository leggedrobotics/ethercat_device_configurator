/*
 ** Copyright 2021 Robotic Systems Lab - ETH Zurich:
 ** Lennart Nachtigall, Jonas Junger
 ** Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 **
 ** 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 **
 ** 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 **
 ** 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
** Simple Example executable for the use of the RSL EtherCAT software tools
** ════════════════════════════════════════════════════════════════════════
**
**   To understand the logic it is best to start at the main function and
**   then go through the code step by step. The executable compiled from
**   this source code needs to be executed as root. Simply executing the
**   executable with sudo will not work because the catkin workspace won’t
**   be sourced and the linker cannot find the necessary libraries. Use the
**   following command to start this executable:
**   ┌────
**   │ sudo bash -c 'source /absolute/path/to/your/catkin_ws/devel/setup.bash; path/to/executable path/to/setup.yaml'
**   └────
**
**
** Build errors
** ────────────
**
**   Are you using gcc and g++ both with a version >= 8.4? See the
**   README.md for more details.
 */
#include "ethercat_device_configurator/EthercatDeviceConfigurator.hpp"

#ifdef _ANYDRIVE_FOUND_
#include <anydrive/Anydrive.hpp>
#endif
#ifdef _ELMO_FOUND_
#include <elmo_ethercat_sdk/Elmo.hpp>
#endif
#ifdef _MAXON_FOUND_
#include <maxon_epos_ethercat_sdk/Maxon.hpp>
#endif
#ifdef _ROKUBI_FOUND_
#include <rokubimini_rsl_ethercat/RokubiminiEthercat.hpp>
#endif
#include <thread>
#include <csignal>
// std::unique_ptr<std::thread> worker_thread;
bool abrt = false;
bool abrtMain = false;

EthercatDeviceConfigurator::SharedPtr configurator;
std::unique_ptr<std::thread> worker_thread;

unsigned int counter = 0;
std::string config_file_path = "";

void worker()
{
    
    std::string setup_file_name = "setup_" + std::to_string(0) + ".yaml";
    std::string setup_file = config_file_path + setup_file_name;
    configurator = std::make_shared<EthercatDeviceConfigurator>(setup_file);

    for(const auto & master: configurator->getMasters()) {
        if(!master->startup())
        {
            std::cerr << "Startup not successful." << std::endl;
            abrtMain = true;
            return;
        }
    }

    bool rtSuccess = true;
    for(const auto & master: configurator->getMasters())
    {
        rtSuccess &= master->setRealtimePriority(99);
    }
    std::cout << "Setting RT Priority: " << (rtSuccess? "successful." : "not successful. Check user privileges.") << std::endl;

    /*
    ** The communication update loop.
    ** This loop is supposed to be executed at a constant rate.
    ** The EthercatMaster::update function incorporates a mechanism
    ** to create a constant rate.
     */
    while(!abrt)
    {
        /*
        ** Update each master.
        ** This sends tha last staged commands and reads the latest readings over EtherCAT.
        ** The StandaloneEnforceRate update mode is used.
        ** This means that average update rate will be close to the target rate (if possible).
         */
        for(const auto & master: configurator->getMasters() ) {
            master->update(ecat_master::UpdateMode::StandaloneEnforceStep); // TODO fix the rate compensation (Elmo reliability problem)!!
        }

        for(const auto & slave:configurator->getSlaves())
        {
            // Anydrive
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
        }
    }
}
/*
** Handle the interrupt signal.
** This is the shutdown routine.
** Note: This logic is executed in a thread separated from the communication update!
 */
void signal_handler(int sig)
{
    abrtMain = true;

    // Exit this executable
    std::cout << "Shutdown" << std::endl;
}

#ifdef _ANYDRIVE_FOUND_
// Some dummy callbacks
void anydriveReadingCb(const std::string& name, const anydrive::ReadingExtended& reading)
{
    // std::cout << "Reading of anydrive '" << name << "'\n"
    //           << "Joint velocity: " << reading.getState().getJointVelocity() << "\n\n";
}
#endif



/*
** Program entry.
** Pass the path to the setup.yaml file as first command line argument.
 */
int main(int argc, char**argv)
{
    // Set the abrt_ flag upon receiving an interrupt signal (e.g. Ctrl-c)
    std::signal(SIGINT, signal_handler);

    if(argc < 2)
    {
        std::cerr << "pass path to 'setup.yaml' as command line argument" << std::endl;
        return EXIT_FAILURE;
    }

    config_file_path = argv[1];

    // worker_thread = std::make_unique<std::thread>(&worker);
    // Start the PDO loop in a new thread.
    worker_thread = std::make_unique<std::thread>(&worker);

    /*
    ** Wait for a few PDO cycles to pass.
    ** Set anydrives into to ControlOp state (internal state machine, not EtherCAT states)
     */

    std::cout << "Startup finished" << std::endl;

    while (true) {
        if (abrtMain) {
                for(const auto & master: configurator->getMasters())
                {
                    master->preShutdown();
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            abrt = true;
            // for ( unsigned int i = 0; i < threads.size(); i++) {
            //     threads.at(i).join();
            // }
            worker_thread->join();
                for(const auto & master: configurator->getMasters())
                {
                    master->shutdown();
                }
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "Exit" << std::endl;
    exit(0);

}
