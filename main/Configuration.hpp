#pragma once
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_netif_ip_addr.h"
//#include "lwip/ip4_addr.h"
#include "RotatorHW.h"
#include <string>
#include <mutex>

struct ConfigData {
    // Fourier coefficients
    double C0;
    std::array<double, KMAX+1> A;  // Index 0 will be ignored
    std::array<double, KMAX+1> B;

    // Network configuration
    std::string ipAddress;
    std::string netmask;
    std::array<uint8_t, 6> macAddress;
    
};

class Configuration {
public:
    // Access of the singleton
    static Configuration& getInstance();
    // mount the filesystem - called internally
    bool mountLittleFS();

    // Setter / Getter
    // Fourier-Koeffizienten
    double getC0() const;
    double getA(int k) const;
    double getB(int k) const;
    void  setC0(double v, bool s=true);
    void  setA(int k, double v, bool s=true);
    void  setB(int k, double v, bool s=true);

    // Netzwerk
    std::string getIPAddressString() const;
    uint32_t getIPAddressInt() const;
    std::string getNetmaskString() const;
    uint32_t getNetmaskInt() const;
    std::array<uint8_t,6> getMACAddress() const;
    void setIPAddress(const std::string& ip);
    void setNetmask(const std::string& nm);
    void setMACAddress(const std::array<uint8_t,6>& mac);

private:
    Configuration();              // mountet FS und lädt JSON
    ~Configuration();

    // nicht kopierbar
    Configuration(const Configuration&)=delete;
    Configuration& operator=(const Configuration&)=delete;

    bool load();                  // loads /lfs/config.json
public:    
    bool save() const;            // writes

private:
    mutable std::mutex _mtx;      // for thread safety
    ConfigData _data;
};
