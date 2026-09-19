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

    // Which physical rotation direction counts as "positive" (increasing
    // Alpaca Position) by default, for this specific telescope/mount - see
    // RotatorHW::getDirection(). Application-specific (a property of how the
    // rotator is mounted on a given telescope), not device-specific, so it
    // lives here rather than in NVS alongside the angle-sensor calibration.
    bool nominalClockwise;

    // Full-step-resolution residual correction, layered on top of C0/A/B -
    // see RotatorHW::setFullStepTable()'s comment. All-zero (a no-op) until
    // a camera-referenced fit has been uploaded. Appended last (not grouped
    // with the Fourier coefficients above) so this struct's existing
    // positional aggregate-initializer in Configuration.cpp - {C0, A, B,
    // ipAddress, netmask, macAddress, nominalClockwise} - never has to shift
    // when adding a field; only ever append here.
    std::array<float, N_STEPS> fullStepTable{};
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

    // Nominal rotation direction - see ConfigData::nominalClockwise.
    bool getNominalClockwise() const;
    void setNominalClockwise(bool clockwise);

    // Full-step correction table - see ConfigData::fullStepTable.
    void getFullStepTable(float outTable[N_STEPS]) const;
    void setFullStepTable(const float table[N_STEPS]);

    // Which calibration the stored correction is. Bumped whenever the
    // coefficients actually change, and stamped into the two artefacts
    // derived from them - the full-step table here and the mechanical-zero
    // target in RotatorHW - so that a correction replaced without
    // re-measuring those can be detected instead of silently misapplied.
    // See the comment on AngleCalBlob in Configuration.cpp.
    uint32_t calibrationGeneration() const { return _generation; }

    // True when the stored full-step table belonged to a different
    // calibration and was therefore dropped at load.
    bool fullStepTableStale() const { return _fullStepTableStale; }

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
    // Fourier coefficients live in NVS ("anglecal"/"coeffs"), not config.json -
    // see load()/save(). Called only while _mtx is already held.
    bool loadAngleCalFromNvs();
    bool saveAngleCalToNvs() const;
    // Same idea, for ConfigData::fullStepTable ("anglecal"/"fullsteptable") -
    // no config.json migration path needed, this setting never existed
    // there. Called only while _mtx is already held.
    bool loadFullStepTableFromNvs();
    bool saveFullStepTableToNvs() const;

    mutable std::mutex _mtx;      // for thread safety
    ConfigData _data;
    uint32_t _generation = 0;
    bool _fullStepTableStale = false;
    // Which calibration generation the in-memory full-step table belongs to,
    // carried separately from _generation so an unrelated save() cannot
    // promote a stale table to current.
    uint32_t _fullStepTableGen = 0;
    // Set when load() migrated a pre-header blob, so the constructor can
    // write it back once in the current format.
    bool _migratedAngleCal = false;
};
