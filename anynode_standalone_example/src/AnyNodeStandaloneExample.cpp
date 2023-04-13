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

  for (auto& master : configurator_->getMasters()) {
    if (!master->startup()) {
      std::cerr << "Startup not successful." << std::endl;
      return false;
    }
  }

  any_worker::WorkerOptions ethercatMasterOptions;
  ethercatMasterOptions.callback_ = std::bind(&AnyNodeStandaloneExample::updateEthercatMaster, this, std::placeholders::_1);
  ethercatMasterOptions.defaultPriority_ = 0;  // this has low priority
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
  for (const auto& master : configurator_->getMasters()) {
    master->preShutdown();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  abrt_ = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void AnyNodeStandaloneExample::cleanup() {
  MELO_INFO_STREAM(" ");
  for (const auto& master : configurator_->getMasters()) {
    master->shutdown();
  }
}

bool AnyNodeStandaloneExample::updateEthercatMaster(const any_worker::WorkerEvent& event) {
  MELO_INFO(" ")
  bool rtSuccess = true;
  for (const auto& master : configurator_->getMasters()) {
    rtSuccess &= master->setRealtimePriority(99);
  }
  std::cout << "Setting RT Priority: " << (rtSuccess ? "successful." : "not successful. Check user privileges.") << std::endl;

  while (!abrt_) {
    for (const auto& master : configurator_->getMasters()) {
      master->update(ecat_master::UpdateMode::StandaloneEnforceRate);  // TODO fix the rate compensation (Elmo
                                                                       // reliability problem)!!
    }
  }

  return true;
}

} /* namespace anynode_standalone_example */
