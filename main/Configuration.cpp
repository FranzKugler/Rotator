#include "Configuration.hpp"
#include "cJSON.h"
#include <fstream>



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
        {0x02, 0x02, 0x84, 0x6A, 0x96, 0x00}};

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
    FILE *f = fopen("/lfs/config.json", "r");
    if (!f)
    {
        ESP_LOGW("cfg", "No config.json found");
        return false;
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
        return false;
    }

    // Fourier coefficients
    _data.C0 = cJSON_GetObjectItem(root, "C0")->valuedouble;
    cJSON *arrA = cJSON_GetObjectItem(root, "A");
    for (int i = 0; i <= KMAX; ++i)
    {
        _data.A[i] = cJSON_GetArrayItem(arrA, i)->valuedouble;
    }
    cJSON *arrB = cJSON_GetObjectItem(root, "B");
    for (int i = 0; i <= KMAX; ++i)
    {
        _data.B[i] = cJSON_GetArrayItem(arrB, i)->valuedouble;
    }

    // Network
    _data.ipAddress = cJSON_GetObjectItem(root, "ipAddress")->valuestring;
    _data.netmask = cJSON_GetObjectItem(root, "netmask")->valuestring;
    const char *macStr = cJSON_GetObjectItem(root, "macAddress")->valuestring;
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

bool Configuration::save() const
{
    std::lock_guard<std::mutex> lock(_mtx);
    cJSON *root = cJSON_CreateObject();

    // Fourier coefficients
    cJSON_AddNumberToObject(root, "C0", _data.C0);
    cJSON *arrA = cJSON_AddArrayToObject(root, "A");
    cJSON *arrB = cJSON_AddArrayToObject(root, "B");
    for (int i = 0; i <= KMAX; ++i)
    {
        cJSON_AddItemToArray(arrA, cJSON_CreateNumber(_data.A[i]));
        cJSON_AddItemToArray(arrB, cJSON_CreateNumber(_data.B[i]));
    }

    // Network config
    cJSON_AddStringToObject(root, "ipAddress", _data.ipAddress.c_str());
    cJSON_AddStringToObject(root, "netmask", _data.netmask.c_str());
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            _data.macAddress[0], _data.macAddress[1], _data.macAddress[2],
            _data.macAddress[3], _data.macAddress[4], _data.macAddress[5]);
    cJSON_AddStringToObject(root, "macAddress", macStr);

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
