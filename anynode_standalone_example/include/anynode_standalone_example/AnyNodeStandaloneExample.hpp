/*!
 * @file     AnyNodeStandaloneExample.hpp
 * @author   Dario Bellicoso
 * @date     Feb 26, 2018
 * @brief
 */
#pragma once

#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include <any_worker/Worker.hpp>

#include "ethercat_device_configurator/EthercatDeviceConfigurator.hpp"

namespace anynode_standalone_example {

class AnyNodeStandaloneExample : public any_node::Node {
 public:
  explicit AnyNodeStandaloneExample(any_node::Node::NodeHandlePtr nh);
  ~AnyNodeStandaloneExample() override = default;

  bool init() override;
  void preCleanup();
  void cleanup() override;

  bool updateEthercatMaster(const any_worker::WorkerEvent& event);

  bool startupWorker(const any_worker::WorkerEvent& event);

 protected:
  EthercatDeviceConfigurator::SharedPtr configurator_;
  std::atomic<bool> abrt_{false};
  std::atomic<bool> startComplete_{false};
  std::atomic<bool> abortStartup_{false};
};

} /* namespace anynode_standalone_example */
