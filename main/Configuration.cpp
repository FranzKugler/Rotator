#include "Configuration.hpp"
#include "cJSON.h"
#include "nvs.h"
#include <fstream>

namespace {
constexpr const char *ANGLECAL_NVS_NAMESPACE = "anglecal";
constexpr const char *ANGLECAL_NVS_KEY = "coeffs";
constexpr const char *FULLSTEP_TABLE_NVS_KEY = "fullsteptable";

// Raw NVS blob layout for the Fourier coefficients - fixed size, so a future
// change to KMAX must be treated like any other persistent-format change
// (see loadAngleCalFromNvs()'s size check, which just falls back to defaults
// rather than misreading a differently-shaped blob).
struct AngleCalBlob
{
    double C0;
    double A[KMAX + 1];
    double B[KMAX + 1];
};
} // namespace

extern "C" void ConfigurationSave ()
{
    Configuration::getInstance().save();
}

Configuration &Configuration::getInstance()
{
    static Configuration instance;
    return instance;
}

Configuration::Configuration()
{
    // set default values
    _data = {
        16.99826935,                                           // 20.5921,
        {0, -3.37954371, -4.05827216, -0.5246565, 1.03694412}, //{462.0188, 431.4244, 389.1578, 329.9203},
        {0, -6.68694199, 0.85262175, 0.86551859, 2.80138335},  //{73.7638, 155.3139, 223.4507, 277.9109},
        "192.168.7.1",
        "255.255.255.0",
        {0x02, 0x02, 0x84, 0x6A, 0x96, 0x00},
        true}; // nominalClockwise - matches this firmware's existing, already-calibrated convention

    if (!mountLittleFS())
    {
        ESP_LOGE("cfg", "LittleFS mount failed — using defaults");
    }

    if (!load())
    {
        ESP_LOGW("cfg", "Config load failed — writing defaults");
        save();
    }
}

Configuration::~Configuration()
{
    // evtl. FS unmounten: esp_vfs_littlefs_unregister("littlefs");
}

bool Configuration::mountLittleFS()
{
    esp_vfs_littlefs_conf_t cfg = {
        .base_path = "/lfs",
        .partition_label = "littlefs",
        .format_if_mount_failed = true};
    esp_err_t err = esp_vfs_littlefs_register(&cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE("cfg", "Failed to mount LittleFS (%s)", esp_err_to_name(err));
        return false;
    }
    size_t total = 0, used = 0;
    esp_littlefs_info("littlefs", &total, &used);
    ESP_LOGI("cfg", "LittleFS mounted. total: %d, used: %d", total, used);
    return true;
}

bool Configuration::load()
{
    std::lock_guard<std::mutex> lock(_mtx);

    // Fourier coefficients: NVS is authoritative. A pre-migration device has
    // no "anglecal" namespace yet - in that case fall back to whatever an old
    // config.json still has (parsed below, if present) or the constructor's
    // defaults, then persist that into NVS so this is a one-time migration.
    bool haveAngleCal = loadAngleCalFromNvs();
    // No config.json migration path for this one - it never lived there.
    // Absent (pre-upload device) just leaves the constructor's all-zero
    // default, a no-op correction, in place.
    loadFullStepTableFromNvs();

    FILE *f = fopen("/lfs/config.json", "r");
    if (!f)
    {
        ESP_LOGW("cfg", "No config.json found");
        if (!haveAngleCal)
            saveAngleCalToNvs();
        return haveAngleCal;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::string json(len, '\0');
    fread(&json[0], 1, len, f);
    fclose(f);
    ESP_LOGI("json read", "%s", json.c_str());
    cJSON *root = cJSON_Parse(json.c_str());
    if (!root)
    {
        ESP_LOGE("cfg", "cJSON_Parse error");
        if (!haveAngleCal)
            saveAngleCalToNvs();
        return false;
    }

    // Fourier coefficients - only consulted for migrating an old config.json
    // that still has them; save() no longer writes them here.
    if (!haveAngleCal)
    {
        cJSON *c0Item = cJSON_GetObjectItem(root, "C0");
        cJSON *arrA = cJSON_GetObjectItem(root, "A");
        cJSON *arrB = cJSON_GetObjectItem(root, "B");
        if (c0Item && arrA && arrB)
        {
            _data.C0 = c0Item->valuedouble;
            for (int i = 0; i <= KMAX; ++i)
            {
                _data.A[i] = cJSON_GetArrayItem(arrA, i)->valuedouble;
                _data.B[i] = cJSON_GetArrayItem(arrB, i)->valuedouble;
            }
            ESP_LOGI("cfg", "Migrating Fourier coefficients from config.json into NVS");
        }
        saveAngleCalToNvs();
    }

    // Network
    _data.ipAddress = cJSON_GetObjectItem(root, "ipAddress")->valuestring;
    _data.netmask = cJSON_GetObjectItem(root, "netmask")->valuestring;
    const char *macStr = cJSON_GetObjectItem(root, "macAddress")->valuestring;
    // Absent on any config.json written before this setting existed - default
    // to true (this firmware's existing, already-calibrated convention).
    cJSON *nominalItem = cJSON_GetObjectItem(root, "nominalClockwise");
    _data.nominalClockwise = nominalItem ? cJSON_IsTrue(nominalItem) : true;
    unsigned int tmp[6];
    if (sscanf(macStr, "%02x:%02x:%02x:%02x:%02x:%02x",
               &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5]) == 6)
    {
        for (int i = 0; i < 6; ++i)
            _data.macAddress[i] = (uint8_t)tmp[i];
    }

    cJSON_Delete(root);
    ESP_LOGI("cfg", "Config loaded");
    return true;
}

bool Configuration::loadAngleCalFromNvs()
{
    nvs_handle_t nvs;
    if (nvs_open(ANGLECAL_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK)
        return false;
    AngleCalBlob blob;
    size_t size = sizeof(blob);
    esp_err_t err = nvs_get_blob(nvs, ANGLECAL_NVS_KEY, &blob, &size);
    nvs_close(nvs);
    if (err != ESP_OK || size != sizeof(blob))
        return false;
    _data.C0 = blob.C0;
    for (int i = 0; i <= KMAX; ++i)
    {
        _data.A[i] = blob.A[i];
        _data.B[i] = blob.B[i];
    }
    return true;
}

bool Configuration::saveAngleCalToNvs() const
{
    nvs_handle_t nvs;
    if (nvs_open(ANGLECAL_NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK)
    {
        ESP_LOGE("cfg", "Could not open NVS namespace '%s' to persist angle calibration", ANGLECAL_NVS_NAMESPACE);
        return false;
    }
    AngleCalBlob blob;
    blob.C0 = _data.C0;
    for (int i = 0; i <= KMAX; ++i)
    {
        blob.A[i] = _data.A[i];
        blob.B[i] = _data.B[i];
    }
    esp_err_t err = nvs_set_blob(nvs, ANGLECAL_NVS_KEY, &blob, sizeof(blob));
    if (err == ESP_OK)
        err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err != ESP_OK)
    {
        ESP_LOGE("cfg", "Failed to persist angle calibration to NVS: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool Configuration::loadFullStepTableFromNvs()
{
    nvs_handle_t nvs;
    if (nvs_open(ANGLECAL_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK)
        return false;
    std::array<float, N_STEPS> table;
    size_t size = table.size() * sizeof(float);
    esp_err_t err = nvs_get_blob(nvs, FULLSTEP_TABLE_NVS_KEY, table.data(), &size);
    nvs_close(nvs);
    if (err != ESP_OK || size != table.size() * sizeof(float))
        return false;
    _data.fullStepTable = table;
    return true;
}

bool Configuration::saveFullStepTableToNvs() const
{
    nvs_handle_t nvs;
    if (nvs_open(ANGLECAL_NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK)
    {
        ESP_LOGE("cfg", "Could not open NVS namespace '%s' to persist the full-step table", ANGLECAL_NVS_NAMESPACE);
        return false;
    }
    esp_err_t err = nvs_set_blob(nvs, FULLSTEP_TABLE_NVS_KEY, _data.fullStepTable.data(),
                                  _data.fullStepTable.size() * sizeof(float));
    if (err == ESP_OK)
        err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err != ESP_OK)
    {
        ESP_LOGE("cfg", "Failed to persist the full-step table to NVS: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool Configuration::save() const
{
    std::lock_guard<std::mutex> lock(_mtx);

    // Fourier coefficients live in NVS, not config.json - see load().
    saveAngleCalToNvs();
    saveFullStepTableToNvs();

    cJSON *root = cJSON_CreateObject();

    // Network config
    cJSON_AddStringToObject(root, "ipAddress", _data.ipAddress.c_str());
    cJSON_AddStringToObject(root, "netmask", _data.netmask.c_str());
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            _data.macAddress[0], _data.macAddress[1], _data.macAddress[2],
            _data.macAddress[3], _data.macAddress[4], _data.macAddress[5]);
    cJSON_AddStringToObject(root, "macAddress", macStr);
    cJSON_AddBoolToObject(root, "nominalClockwise", _data.nominalClockwise);

    char *str = cJSON_Print(root);
    ESP_LOGI("cfg", "JSON string = %s", str);
    FILE *f = fopen("/lfs/config.json", "w");
    if (!f)
    {
        ESP_LOGE("cfg", "Failed to open config.json for writing");
        cJSON_free(str);
        cJSON_Delete(root);
        return false;
    }
    // ESP_LOGI("json write", "%s", str);
    fwrite(str, 1, strlen(str), f);
    fclose(f);
    cJSON_free(str);
    cJSON_Delete(root);
    ESP_LOGI("cfg", "Config saved");
    return true;
}

// Getter/Setter:

double Configuration::getC0() const
{
    std::lock_guard<std::mutex> l(_mtx);
    return _data.C0;
}
double Configuration::getA(int k) const
{
    std::lock_guard<std::mutex> l(_mtx);
    return (k >= 0 && k <= KMAX) ? _data.A[k] : 0.0f;
}
double Configuration::getB(int k) const
{
    std::lock_guard<std::mutex> l(_mtx);
    return (k >= 0 && k <= KMAX) ? _data.B[k] : 0.0f;
}
void Configuration::setC0(double v, bool s)
{
    {
        std::lock_guard<std::mutex> l(_mtx);
        _data.C0 = v;
    }
    if (s)
        save();
}
void Configuration::setA(int k, double v, bool s)
{
    {
        std::lock_guard<std::mutex> l(_mtx);
        if (k >= 0 && k <= KMAX)
            _data.A[k] = v;
    }
    if (s)
        save();
}
void Configuration::setB(int k, double v, bool s)
{
    {
        std::lock_guard<std::mutex> l(_mtx);
        if (k >= 0 && k <= KMAX)
            _data.B[k] = v;
    }
    if (s)
        save();
}

std::string Configuration::getIPAddressString() const
{
    std::lock_guard<std::mutex> l(_mtx);
    return _data.ipAddress;
}
uint32_t Configuration::getIPAddressInt() const
{
    std::lock_guard<std::mutex> l(_mtx);
    uint32_t ipAddress;
    ip4addr_aton(_data.ipAddress.c_str(), (ip4_addr *)&ipAddress);
    return ipAddress;
}
std::string Configuration::getNetmaskString() const
{
    std::lock_guard<std::mutex> l(_mtx);
    return _data.netmask;
}
uint32_t Configuration::getNetmaskInt() const
{
    std::lock_guard<std::mutex> l(_mtx);
    uint32_t ipNetmask;
    ip4addr_aton(_data.netmask.c_str(), (ip4_addr *)&ipNetmask);
    return ipNetmask;
}
std::array<uint8_t, 6> Configuration::getMACAddress() const
{
    std::lock_guard<std::mutex> l(_mtx);
    return _data.macAddress;
}
void Configuration::setIPAddress(const std::string &ip)
{
    {
        std::lock_guard<std::mutex> l(_mtx);
        _data.ipAddress = ip;
        ESP_LOGI("configuration", "New IP = %s", _data.ipAddress.c_str());
    }
    save();
}
void Configuration::setNetmask(const std::string &nm)
{
    {
        std::lock_guard<std::mutex> l(_mtx);
        _data.netmask = nm;
    }
    save();
}
void Configuration::setMACAddress(const std::array<uint8_t, 6> &mac)
{
    {
        std::lock_guard<std::mutex> l(_mtx);
        _data.macAddress = mac;
    }
    save();
}

bool Configuration::getNominalClockwise() const
{
    std::lock_guard<std::mutex> l(_mtx);
    return _data.nominalClockwise;
}
void Configuration::setNominalClockwise(bool clockwise)
{
    {
        std::lock_guard<std::mutex> l(_mtx);
        _data.nominalClockwise = clockwise;
    }
    save();
}

void Configuration::getFullStepTable(float outTable[N_STEPS]) const
{
    std::lock_guard<std::mutex> l(_mtx);
    for (int i = 0; i < N_STEPS; ++i)
        outTable[i] = _data.fullStepTable[i];
}
void Configuration::setFullStepTable(const float table[N_STEPS])
{
    {
        std::lock_guard<std::mutex> l(_mtx);
        for (int i = 0; i < N_STEPS; ++i)
            _data.fullStepTable[i] = table[i];
    }
    save();
}
