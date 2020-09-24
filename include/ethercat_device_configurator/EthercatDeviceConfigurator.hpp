#pragma once

#include <memory>
#include <string>
#include <ethercat_sdk_master/EthercatMaster.hpp>
#include <type_traits>

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
     * @return shared_ptr on slave
     */
    std::shared_ptr<ecat_master::EthercatDrive> getSlave(std::string name);
    /**
     * @brief getInfoForSlave
     * @param slave - shared ptr on slave
     * @return Info entry parsed from setup.yaml
     */
    const EthercatSlaveEntry& getInfoForSlave(const std::shared_ptr<ecat_master::EthercatDrive>& slave);
    /**
     * @brief master
     * @return pointer on master if only a single master is available
     * @throw std::runtime_error if more than one master is configured
     */
    std::shared_ptr<ecat_master::EthercatMaster> master();

    /**
     * @brief getSetupFilePath
     * @return path to setup file
     */
    const std::string& getSetupFilePath();

    /**
     * @brief getSlavesOfType - return all slaves of type T (vector of shared_ptr).
     * @note Warning cache the result if you need them on a regular base. Might have bad performance
     */
    template<typename T, std::enable_if_t<std::is_base_of_v<ecat_master::EthercatDrive, T>>>
    std::vector<std::shared_ptr<T>> getSlavesOfType()
    {

        std::vector<std::shared_ptr<T>> slaves;

        for(auto & slave: m_slaves)
        {
            auto ptr = std::dynamic_pointer_cast<T>(slave);
            if(ptr)
            {
                slaves.push_back(ptr);
            }
        }
        return slaves;
    }

private:
    ecat_master::EthercatMasterConfiguration m_master_configuration;

    std::vector<std::shared_ptr<ecat_master::EthercatMaster>> m_masters;

    std::vector<std::shared_ptr<ecat_master::EthercatDrive>> m_slaves;


    std::vector<EthercatSlaveEntry> m_slave_entries;
    std::map<std::shared_ptr<ecat_master::EthercatDrive>, EthercatSlaveEntry> m_slave_to_entry_map;



    void parseFile(std::string path);
    void setup(bool startup);
    std::string handleFilePath(const std::string& path, const std::string &setup_file_path) const;

    std::string m_setup_file_path ="";
};
