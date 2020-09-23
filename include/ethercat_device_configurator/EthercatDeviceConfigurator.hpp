#pragma once

#include <memory>
#include <string>
#include <ethercat_sdk_master/EthercatMaster.hpp>


class EthercatDeviceConfigurator
{
public:
    //Convinience typedef for a shared pointer
    typedef std::shared_ptr<EthercatDeviceConfigurator> SharedPtr ;

    //Type ethercat slave device. If you want to wire in a new slave device type, add an entry to this enum
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
    /**
     * @brief EthercatDeviceConfigurator
     * @param path - path to the setup.yaml
     * @param startup - if true -> calls startup on all masters
     */
    EthercatDeviceConfigurator(std::string path, bool startup = false);
    /**
     * @brief getMasters
     * @return a vector of all masters
     */
    std::vector<std::shared_ptr<ecat_master::EthercatMaster>> getMasters();
    /**
     * @brief getSlaves
     * @return a vector of all slaves
     */
    std::vector<std::shared_ptr<ecat_master::EthercatDrive>> getSlaves();
    /**
     * @brief getSlave - get a certain slave by its name
     * @param name
     * @return
     */
    std::shared_ptr<ecat_master::EthercatDrive> getSlave(std::string name);

    std::shared_ptr<ecat_master::EthercatMaster> master();

private:
    ecat_master::EthercatMasterConfiguration m_master_configuration;

    std::vector<std::shared_ptr<ecat_master::EthercatMaster>> m_masters;

    std::vector<std::shared_ptr<ecat_master::EthercatDrive>> m_slaves;
    std::vector<EthercatSlaveEntry> m_slave_entries;



    void parseFile(std::string path);
    void setup(bool startup);
    std::string handleFilePath(const std::string& path, const std::string &setup_file_path) const;

    std::string m_setup_file_path ="";
};
