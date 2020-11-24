# EtherCAT Device Configurator
Configures the EtherCAT communication with the following sdks:

- anydrive_sdk
- rokubi_rsl_ethercat_sdk
- elmo_ethercat_sdk
- maxon_epos_ethercat_sdk

Any single SDK or any combination of SDKs may be used.
The build system automatically builds the EtherCAT device SDKs available in the current catkin workspace.

## Building
__You will need at least gcc version >= 8.4__.
This requires the following for devices running Ubuntu 18.04:

- install gcc-8 and g++-8: `sudo apt install gcc-8 g++-8`
- configure catkin in your workspace: `catkin config --cmake-args -DCMAKE_CXX_COMPILER=/usr/bin/g++-8 -DOTHER_CMAKE_VARIABLES_YOU_WANT_TO_SET_EG_BUILD_TYPE`.

### Dependencies
- anydrive_sdk (refactoring/ethercat_sdk_master)
- rokubi_rsl_ethercat_sdk (refactoring/ethercat_sdk_master)
- elmo_ethercat_sdk (refactoring/ethercat_sdk_master)
- maxon_epos_ethercat_sdk (refactoring/ethercat_sdk_master)
- ethercat_sdk_master (master)
- soem_interface (release)
- message_logger (master)
- any_node (master)
- yaml-cpp (system install)

