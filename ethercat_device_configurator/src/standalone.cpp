/*
 ** Copyright 2021 Robotic Systems Lab - ETH Zurich:
 ** Lennart Nachtigall, Jonas Junger
 ** Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions
 *are met:
 **
 ** 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 **
 ** 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
 *documentation and/or other materials provided with the distribution.
 **
 ** 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from
 *this software without specific prior written permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "message_logger/message_logger.hpp"

#ifdef _ANYDRIVE_FOUND_
#include <anydrive_rsl/Anydrive.hpp>
#endif
#ifdef _ELMO_FOUND_
#include <elmo_ethercat_sdk/Elmo.hpp>
#endif
#ifdef _MPSDRIVE_FOUND_
#include <mps_ethercat_sdk/MPSDrive.hpp>
#endif
#ifdef _MAXON_FOUND_
#include <maxon_epos_ethercat_sdk/Maxon.hpp>
#endif
#ifdef _ROKUBI_FOUND_
#include <rokubimini_rsl_ethercat_slave/RokubiminiEthercat.hpp>
#endif

#include <csignal>
#include <thread>

namespace example {

// possible dummy slave callbacks:
#ifdef _ANYDRIVE_FOUND_
void anydriveReadingCb(const std::string& name, const anydrive_rsl::ReadingExtended& reading) {
  // note: callbacks are called within the ethercat update loop, they should not block! otherwise you'll see working counter too low errors
  // all the time and your motors will not behave as expected.
  MELO_INFO_THROTTLE_STREAM(5, "[EthercatDeviceConfiguratorExample] Dummy Callback, reading of anydrive '"
                                   << name << "Joint velocity: " << reading.getState().getJointVelocity());
}
#endif
#ifdef _ROKUBI_FOUND_
void rokubiReadingCb(const std::string& name, const rokubimini::Reading& reading) {
  //  //note: callbacks are called within the ethercat update loop, they should not block! otherwise you'll see working counter too low
  //  errors all the time and your motors will not behave as expected.
  MELO_INFO_THROTTLE_STREAM(5, "[EthercatDeviceConfiguratorExample] Dummy Callback, Reading of rokubi: "
                                   << name << " Force X: " << reading.getWrench().wrench_.getForce().toImplementation().x());
}
#endif

class ExampleEcatHardwareInterface {
 public:
  ExampleEcatHardwareInterface() {
    // we could also do the init stuff during construction, but usually you want to have the init somewhere in a hardware interface state
    // machine.
  }

  bool init(char* pathToConfigFile) {
    configurator_ = std::make_shared<EthercatDeviceConfigurator>();
    configurator_->initializeFromFile(pathToConfigFile);

    ecatMaster_ = configurator_->master();  // throws if more than one master.

    // get a list
#ifdef _ELMO_FOUND_
    elmos_ = configurator_->getSlavesOfType<elmo::Elmo>(EthercatDeviceConfigurator::EthercatSlaveType::Elmo);
#endif
#ifdef _MAXON_FOUND_
    maxons_ = configurator_->getSlavesOfType<maxon::Maxon>(EthercatDeviceConfigurator::EthercatSlaveType::Maxon);
#endif
#ifdef _ANYDRIVE_FOUND_
    anydrives_ =
        configurator_->getSlavesOfType<anydrive_rsl::AnydriveEthercatSlave>(EthercatDeviceConfigurator::EthercatSlaveType::Anydrive);
    /*
    ** Add callbacks to the devices that support them.
    ** If you don't want to use callbacks this part can simply be left out.
    ** configurator->getSlavesOfType is another way of extracting only the devices
    ** of a ceratain type.
    */
    for (const auto& anydrive : anydrives_) {
      anydrive->addReadingCb(anydriveReadingCb);
    }
#endif
#ifdef _ROKUBI_FOUND_
    botaSensors_ =
        configurator_->getSlavesOfType<rokubimini::ethercat::RokubiminiEthercat>(EthercatDeviceConfigurator::EthercatSlaveType::Rokubi);
    /*
    ** Add callbacks to the devices that support them.
    ** If you don't want to use callbacks this part can simply be left out.
    ** configurator->getSlavesOfType is another way of extracting only the devices
    ** of a ceratain type.
    */
    for (auto& sensor : botaSensors_) {
      sensor->addReadingCb(rokubiReadingCb);
    }
#endif
#ifdef _MPSDRIVE_FOUND_
    mpsDrives_ = configurator_->getSlavesOfType<mps_ethercat_sdk::MPSDrive>(EthercatDeviceConfigurator::EthercatSlaveType::MPSDrive);
#endif

    // starts up the bus and all slaves, blocks till the bus is in SAFE_OP state therefore cyclic PDO reading thread can be started, and
    // other operations can be performed in between. when the bus is directly put into OP state with startup(true) a watchdog on the slave
    // is started which checks if cyclic PDO is happening, if this communication is not started fast enough the drive goes into an error
    // state.
    if (ecatMaster_->startup(startupAbortFlag_)) {
      MELO_INFO_STREAM("[EthercatDeviceConfiguratorExample] Successfully started Ethercat Master on Network Interface: "
                       << ecatMaster_->getBusPtr()->getName());
    } else {
      MELO_ERROR_STREAM("[EthercatDeviceConfiguratorExample] Could not start the Ethercat Master.")
      return false;
    }

    // slaves are no in SAFE_OP state. SDO communication is available - special SDO config calls should be done in the slaves startup
    // memeber function which is called by the ecatMaster->startup() or here.

    workerThread_ = std::make_unique<std::thread>([this]() -> void {
      if (ecatMaster_->setRealtimePriority(48)) {  // do not set above 48, otherwise starve kernel processes on which soem depends.
        MELO_INFO_STREAM("[EthercatDeviceConfiguratorExample] Set increased thread priority to 48")
      } else {
        MELO_WARN_STREAM("[EthercatDeviceConfiguratorExample] Could not incrase thread priority - check user privileges.")
      }

      // we could also do it outside of the thread after we knew it started looping.
      if (ecatMaster_->activate()) {
        MELO_INFO_STREAM("[EthercatDeviceConfiguratorExample] Activated the Bus: " << ecatMaster_->getBusPtr()->getName())
      }
      // here the watchdog on the slave is activated. therefore don't block/sleep for 100ms..
      while (!abrtFlag_) {
        ecatMaster_->update(ecat_master::UpdateMode::StandaloneEnforceStep);
        // we could have interaction with some slaves here, e.g. getting and setting commands. this is than in sync with the ethercat loop.
        // but we have to be carefully to no block it too long. otherwise problems with certain slaves.
      }
      // make sure that bus is in SAFE_OP state, if preShutdown(true) should already do it, but makes sense to have this call here.
      ecatMaster_->deactivate();
    });

    return true;
  }

  void someUserStartInteraction() {
    // this would be user interaction e.g. publishing, subscribing, writing to shared memory whatever.
    // be aware that this is a concurrent interaction with stageCommand, getReading of the slaves. If you do this to fast you might starve
    // the ethercat cyclic loop which might cause problems with certain slaves, which require a strict timing. e.g. drives in cyclic
    // synchronous modes (csp, csv, cst, csc) with those drives you also have to check that interpolationIndex is set so that it reflects
    // the update time of the ethercat loop.

#ifdef _ANYDRIVE_FOUND_
    for (auto& anydrive : anydrives_) {
      // put into controlOP, in blocking mode.
      anydrive->setFSMGoalState(anydrive_rsl::fsm::StateEnum::ControlOp, true, 1, 10);
    }
#endif
#ifdef _ROKUBI_FOUND_
    // Do things with the Rokubi sensors here, seens a simple sensor no big activation needed.
#endif
#ifdef _ELMO_FOUND_
    for (auto& elmo : elmos_) {
      // Set elmos to operation enabled state, there is a option which is blocking s.t the user thread can pause till all elmos are
      // operational
      elmo->setDriveStateViaPdo(elmo::DriveState::OperationEnabled, true);
      // set commands if we can
    }
#endif

#ifdef _MPSDRIVE_FOUND_
    // Set elmos to operation enabled state, do not block the call!
    for (auto& mpsDrive : mpsDrives_) {
      mpsDrive->setDriveStateViaPdo(mps_ethercat_sdk::DriveState::OperationEnabled, false);
    }

#endif
#ifdef _MAXON_FOUND_
    for (auto& maxon : maxons_) {
      // Set maxons to operation enabled state, do not block the call!
      maxon->setDriveStateViaPdo(maxon::DriveState::OperationEnabled, true);  // todo check
    }
#endif
  }

  void cyclicUserInteraction() {
    userCyclicThread_ = std::make_unique<std::thread>([this]() {
      while (userInteraction_) {
        // this can run fully async, as here! but be aware that we're doing concurrent blocking calls into the time sensitive cyclic PDO
        // loop. there are multiple ways to avoid/improve this e.g. syncing this interaction with the cyclic PDO loop with conditional
        // variables e.g. queue the readings into a (lock-free) fancy producer consumer queue e.g. copy out the readings in a callback. (the
        // call to it is than again synced into cyclic PDO loop)
#ifdef _ANYDRIVE_FOUND_
        for (auto& anydrive : anydrives_) {
          if (anydrive->getActiveStateEnum() == anydrive_rsl::fsm::StateEnum::ControlOp) {
            anydrive_rsl::Command cmd;
            cmd.setModeEnum(anydrive_rsl::mode::ModeEnum::MotorVelocity);
            cmd.setMotorVelocity(1);
            // this is a concurrent call into the ethercat loop.
            anydrive->setCommand(cmd);
          }
        }
#endif
#ifdef _ELMO_FOUND_
        for (auto& elmo : elmos_) {
          if (elmo->lastPdoStateChangeSuccessful() && elmo->getReading().getDriveState() == elmo::DriveState::OperationEnabled) {
            elmo::Command command;
            // we would get a command from somewhere here e.g. a feedback controller, shared memory communication, other thread.
            command.setTargetVelocity(1);

            // this is one concurrent call.
            elmo->stageCommand(command);
          }
          // this is an other concurrent call into the ethercat update loop
          auto reading = elmo->getReading();
          MELO_INFO_STREAM("[EthercatDeviceConfiguratorExample] Elmo: " << elmo->getName() << " velocity: " << reading.getActualVelocity());
        }
#endif
#ifdef _MPSDRIVE_FOUND_
        for (auto& mpsDrive : mpsDrives_) {
          if (mpsDrive->lastPdoStateChangeSuccessful() &&
              mpsDrive->getReading().getDriveState() == mps_ethercat_sdk::DriveState::OperationEnabled) {
            mps_ethercat_sdk::Command command;
            command.setActuatorVelocityDesired(1);
            command.setKp(0.4);
            mpsDrive->stageCommand(command);
            auto reading = mpsDrive->getReading();
            MELO_INFO_STREAM("[EthercatDeviceConfiguratorExample] MPSDrive: " << mpsDrive->getName() << " :\n " << reading);
          } else {
            MELO_WARN_STREAM("[EthercatDeviceConfiguratorExample] MPSDrive not in operational state: "
                             << mpsDrive->getName() << "': " << mpsDrive->getReading().getDriveState());
          }
        }
#endif
#ifdef _MAXON_FOUND_
        for (auto& maxon : maxons_) {
          if (maxon->lastPdoStateChangeSuccessful() && maxon->getReading().getDriveState() == maxon::DriveState::OperationEnabled) {
            maxon::Command command;
            command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);  // todo torque mode with positon command?
            auto reading = maxon->getReading();
            command.setTargetPosition(reading.getActualPosition() + 10);
            maxon->stageCommand(command);
          } else {
            MELO_WARN_STREAM("[EthercatDeviceConfiguratorExample] Maxon not in operationEnabled state: "
                             << maxon->getName() << "': " << maxon->getReading().getDriveState());
          }
        }
#endif

        // this is async. no special timing needed, or your application has to take care for it.. just make sure to not starve the cyclic
        // PDO loop.
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    });
  }

  void shutdown() {
    // stop the user code.
    userInteraction_ = false;
    if (userCyclicThread_) {
      if (userCyclicThread_->joinable()) {
        userCyclicThread_->join();
      }
    }
    MELO_INFO_STREAM("[EthercatDeviceConfiguratorExample] Shutdown user cyclic thread")

    // call preShutdown before terminating the cyclic PDO communication!!
    if (ecatMaster_) {
      ecatMaster_->preShutdown(true);
    }
    // we can do more shutdown stuff here. since bus is in SAFE OP there is no strange PDO timeout which we might trigger.
    MELO_INFO_STREAM("[EthercatDeviceConfiguratorExample] PreShutdown ethercat master and all slaves.")

    abrtFlag_ = true;
    if (workerThread_) {
      if (workerThread_->joinable()) {
        workerThread_->join();
      }
    }
    MELO_INFO_STREAM("[EthercatDeviceConfiguratorExample] Joined the ethercat master.")

    if (ecatMaster_) {
      ecatMaster_->shutdown();
    }
    MELO_INFO_STREAM("[EthercatDeviceConfiguratorExample] Fully shutdown.")
  }

  void abortStartup() { startupAbortFlag_ = true; }

  ~ExampleEcatHardwareInterface() {
    // if no signal handler is used - we can do this in the destructor.
    MELO_INFO_STREAM("[EthercatDeviceConfiguratorExample] Destructor.")
    shutdown();
  }

 private:
  EthercatDeviceConfigurator::SharedPtr configurator_;
  ecat_master::EthercatMaster::SharedPtr ecatMaster_;

// cache the devices on the bus, since topology of the robot does not change dynamically.
// in the more general case you would do this per bus. e.g. you have a robot with two ethercat buses.
// in general use one bus per robot - that's the main motivation of having a bus.
#ifdef _ELMO_FOUND_
  const bool elmoEnabledAfterStartup_ = false;
  std::vector<std::shared_ptr<elmo::Elmo>> elmos_{};
#endif
#ifdef _MAXON_FOUND_
  bool maxonEnabledAfterStartup_ = false;
  std::vector<std::shared_ptr<maxon::Maxon>> maxons_{};
#endif
#ifdef _ANYDRIVE_FOUND_
  std::vector<anydrive_rsl::AnydriveEthercatSlave::SharedPtr> anydrives_{};
#endif
#ifdef _ROKUBI_FOUND_
  std::vector<std::shared_ptr<rokubimini::ethercat::RokubiminiEthercat>> botaSensors_{};
#endif
#ifdef _MPSDRIVE_FOUND_
  std::vector<mps_ethercat_sdk::MPSDrive::SharedPtr> mpsDrives_{};
#endif

  std::unique_ptr<std::thread> workerThread_;
  std::unique_ptr<std::thread> userCyclicThread_;
  std::atomic<bool> startupAbortFlag_{false};
  std::atomic<bool> abrtFlag_{false};
  std::atomic<bool> userInteraction_{true};
};

class SimpleSignalHandler {
 public:
  static void sigIntCall([[maybe_unused]] int signal) {
    MELO_INFO_STREAM(" ** Signal Handling **")
    if (!sigIntCallbacks_.empty()) {
      for (auto& sigIntCallback : sigIntCallbacks_) {
        sigIntCallback(signal);
      }
    }
  }

  static void registerSignalHandler() { std::signal(SIGINT, SimpleSignalHandler::sigIntCall); }

  static void addSigIntCallback(std::function<void(int)> callback) { sigIntCallbacks_.push_back(callback); }

  static std::vector<std::function<void(int)>> sigIntCallbacks_;
};

std::vector<std::function<void(int)>> SimpleSignalHandler::sigIntCallbacks_ = {};
};  // namespace example

/*
** Program entry.
** Pass the path to the setup.yaml file as first command line argument.
*/
int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "pass path to 'setup.yaml' as command line argument" << std::endl;
    return EXIT_FAILURE;
  }

  example::ExampleEcatHardwareInterface exampleEcatHardwareInterface{};
  std::atomic<bool> shutdownFlag{false};

  example::SimpleSignalHandler::addSigIntCallback([&shutdownFlag, &exampleEcatHardwareInterface]([[maybe_unused]] int signal) {
    exampleEcatHardwareInterface.abortStartup();
    shutdownFlag = true;
  });

  example::SimpleSignalHandler::registerSignalHandler();

  if (exampleEcatHardwareInterface.init(argv[1])) {
    MELO_INFO_STREAM("[EthercatExmample] Startup completed.")
    exampleEcatHardwareInterface.someUserStartInteraction();
    exampleEcatHardwareInterface.cyclicUserInteraction();
  }
  // nothing further to do in this thread, so we wait till we receive a shutdown signal s.t later the exmpaleEcatHArdwareInterface can be
  // nicely shutdown the shutdown method should NOT be in the signal handler because the signal handler has to be lock-free, otherwise UB.
  // https://en.cppreference.com/w/cpp/utility/program/signal
  MELO_INFO_STREAM("[Main Thread] started waiting...")
  while (!shutdownFlag) {  // could be replaced with smarter stuff like a conditional variable
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  MELO_INFO_STREAM("[Main Thread] stopped waiting... shut down.. ")
  exampleEcatHardwareInterface.shutdown();
  return 0;
}
