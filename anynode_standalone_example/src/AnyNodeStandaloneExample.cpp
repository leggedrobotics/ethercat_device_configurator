/*!
 * @file     AnyNodeStandaloneExample.cpp
 * @author   Jan Preisig
 * @date     Apr, 2023
 * @brief
 */
#include "anynode_standalone_example/AnyNodeStandaloneExample.hpp"

namespace anynode_standalone_example {

AnyNodeStandaloneExample::AnyNodeStandaloneExample(any_node::Node::NodeHandlePtr nh) : any_node::Node(nh) {}

bool AnyNodeStandaloneExample::init() {
  XmlRpc::XmlRpcValue params;
  try {
    if (!param_io::getParam(getNodeHandle(), "anydrive_setup", params)) {
      MELO_ERROR_STREAM("Container 'anydrive_setup' does not exist.")
      return false;
    }
  } catch (const XmlRpc::XmlRpcException& exception) {
    MELO_ERROR_STREAM("Caught an XmlRpc exception while getting container 'anydrive_setup': " << exception.getMessage() << ".")
    return false;
  }

  configurator_ = std::make_shared<EthercatDeviceConfigurator>();
  configurator_->initializeFromParameters(params);

  any_worker::WorkerOptions startupWorkerOptions;
  startupWorkerOptions.callback_ = std::bind(&AnyNodeStandaloneExample::startupWorker, this, std::placeholders::_1);
  startupWorkerOptions.name_ = "AnyNodeStandaloneExample::startupWorker";
  startupWorkerOptions.timeStep_ = std::numeric_limits<double>::infinity();  // will terminate when startup complete or aborted.
  this->addWorker(startupWorkerOptions);

  any_worker::WorkerOptions ethercatMasterOptions;
  ethercatMasterOptions.callback_ = std::bind(&AnyNodeStandaloneExample::updateEthercatMaster, this, std::placeholders::_1);
  ethercatMasterOptions.defaultPriority_ = 39;
  ethercatMasterOptions.name_ = "AnyNodeStandaloneExample::updateEthercatMaster";
  ethercatMasterOptions.timeStep_ = std::numeric_limits<double>::infinity();
  if (!this->addWorker(ethercatMasterOptions)) {
    MELO_ERROR_STREAM("[AnyNodeStandaloneExample] Worker " << ethercatMasterOptions.name_ << "could not be added!");
    return false;
  }

  return true;
}

void AnyNodeStandaloneExample::preCleanup() {
  MELO_INFO_STREAM(" ");
  abortStartup_ = true;
  for (const auto& master : configurator_->getMasters()) {
    master->preShutdown(true);
  }
  abrt_ = true;
}

void AnyNodeStandaloneExample::cleanup() {
  MELO_INFO_STREAM(" ");
  for (const auto& master : configurator_->getMasters()) {
    master->shutdown();
  }
}

bool AnyNodeStandaloneExample::startupWorker(const any_worker::WorkerEvent& event) {
  for (auto& master : configurator_->getMasters()) {
    if (master->startup(abortStartup_)) {
      startComplete_ = true;
    } else {
      std::cerr << "Startup not successful." << std::endl;
      return false;
    }
  }
  return true;
}

bool AnyNodeStandaloneExample::updateEthercatMaster(const any_worker::WorkerEvent& event) {
  while (!abortStartup_) {  // use a conditional variable or atomic_flag_wait_for or something more smart..
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (startComplete_) {
      break;
    }
  }

  if (startComplete_) {
    for (const auto& master : configurator_->getMasters()) {
      master->activate();
    }

    while (!abrt_) {
      for (const auto& master : configurator_->getMasters()) {
        master->update(ecat_master::UpdateMode::StandaloneEnforceRate);
      }
    }
  }

  return true;
}

} /* namespace anynode_standalone_example */
