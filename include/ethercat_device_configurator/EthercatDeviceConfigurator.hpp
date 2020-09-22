#pragma once

#include <memory>
#include <string>
#include <ethercat_sdk_master/EthercatMaster.hpp>


class EthercatDeviceConfigurator
{
public:
    typedef std::shared_ptr<EthercatDeviceConfigurator> SharedPtr ;

    enum class EthercatSlaveType
    {
        Elmo,
        Anydrive,
        Rokubi
    };

    struct EthercatSlaveEntry
    {
        EthercatSlaveType type;
        std::string name;
        std::string config_file_path;

        uint32_t ethercat_address;
        std::string ethercat_bus;
        std::string ethercat_pdo_type;
    };

    EthercatDeviceConfigurator(std::string path, bool startup = false);

    std::vector<std::shared_ptr<ecat_master::EthercatMaster>> getMasters();
    std::vector<std::shared_ptr<ecat_master::EthercatDrive>> getSlaves();

    std::shared_ptr<ecat_master::EthercatDrive> getSlave(std::string name);

private:
    ecat_master::EthercatMasterConfiguration m_master_configuration;

    std::vector<std::shared_ptr<ecat_master::EthercatMaster>> m_masters;

    std::vector<std::shared_ptr<ecat_master::EthercatDrive>> m_slaves;
    std::vector<EthercatSlaveEntry> m_slave_entries;



    void parseFile(std::string path);
    void setup(bool startup);
};
