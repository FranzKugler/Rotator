#include "Configuration.hpp"
#include "cJSON.h"
#include "nvs.h"
#include <fstream>

namespace {
constexpr const char *ANGLECAL_NVS_NAMESPACE = "anglecal";
constexpr const char *ANGLECAL_NVS_KEY = "coeffs";
constexpr const char *FULLSTEP_TABLE_NVS_KEY = "fullsteptable";
constexpr const char *FULLSTEP_GEN_NVS_KEY = "fullstepgen";

// Everything the angle correction persists carries a generation number, and
// the two artefacts derived from it carry the generation they were made
// with. That is the whole point of the header below.
//
// The three stored pieces - the Fourier coefficients, the mechanical-zero
// target and the full-step residual table - are not independent. The zero is
// stored as a *corrected* sensor value and the table is indexed by one, so
// both are expressed in terms of whatever correction was in force when they
// were measured. Replace the coefficients and they do not become wrong
// noisily; they become wrong silently. Live on 2026-09-19 the stored table
// was found spanning 353 mdeg peak to peak against a correction it no longer
// belonged to, injecting up to 222 mdeg of drift with nothing anywhere
// saying so.
//
// So: writing coefficients that differ from the stored ones bumps
// `generation`, and the table and the zero record the generation they were
// stamped with. A mismatch at load is reported, and the table - whose
// all-zero state is a harmless no-op - is dropped rather than applied.
constexpr uint32_t ANGLECAL_MAGIC = 0x41434C31;   // "ACL1"
constexpr uint16_t ANGLECAL_VERSION = 1;

struct AngleCalBlob
{
    uint32_t magic;
    uint16_t version;
    uint16_t kmax;       // so a KMAX change is detected and migrated, not misread
    uint32_t generation;
    double C0;
    double A[KMAX + 1];
    double B[KMAX + 1];
};

// The layout before the header existed, when KMAX was 4. Recognised by size
// alone, which is all it offers, and migrated rather than discarded: the
// harmonics it holds are still valid, the new top one is simply zero.
struct LegacyAngleCalBlob
{
    double C0;
    double A[5];
    double B[5];
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
    else if (_migratedAngleCal)
    {
        // Write the migrated calibration back in the current format, so the
        // migration happens once rather than on every boot and the
        // generation stops moving under the artefacts stamped against it.
        // Coefficients only. The table keeps whatever stamp it had, so a
        // pre-migration one stays visibly orphaned instead of being adopted.
        saveAngleCalToNvs();
        _migratedAngleCal = false;
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

    size_t size = 0;
    esp_err_t err = nvs_get_blob(nvs, ANGLECAL_NVS_KEY, nullptr, &size);
    if (err != ESP_OK)
    {
        nvs_close(nvs);
        return false;
    }

    if (size == sizeof(AngleCalBlob))
    {
        AngleCalBlob blob;
        size_t got = sizeof(blob);
        err = nvs_get_blob(nvs, ANGLECAL_NVS_KEY, &blob, &got);
        nvs_close(nvs);
        if (err != ESP_OK || blob.magic != ANGLECAL_MAGIC ||
            blob.version != ANGLECAL_VERSION || blob.kmax != KMAX)
        {
            ESP_LOGW("cfg", "Stored angle calibration is not this format "
                            "(magic %08lx version %u kmax %u) - using defaults",
                     (unsigned long)blob.magic, blob.version, blob.kmax);
            return false;
        }
        _data.C0 = blob.C0;
        for (int i = 0; i <= KMAX; ++i)
        {
            _data.A[i] = blob.A[i];
            _data.B[i] = blob.B[i];
        }
        _generation = blob.generation;
        return true;
    }

    if (size == sizeof(LegacyAngleCalBlob) && KMAX >= 4)
    {
        LegacyAngleCalBlob legacy;
        size_t got = sizeof(legacy);
        err = nvs_get_blob(nvs, ANGLECAL_NVS_KEY, &legacy, &got);
        nvs_close(nvs);
        if (err != ESP_OK)
            return false;
        _data.C0 = legacy.C0;
        for (int i = 0; i <= KMAX; ++i)
        {
            _data.A[i] = (i < 5) ? legacy.A[i] : 0.0;
            _data.B[i] = (i < 5) ? legacy.B[i] : 0.0;
        }
        // A migrated calibration keeps its coefficients but gets a fresh
        // generation, because nothing recorded which zero and table went
        // with it - so they are treated as not belonging to it, which is the
        // safe reading rather than the convenient one.
        _generation = 1;
        _migratedAngleCal = true;
        ESP_LOGW("cfg", "Migrated a pre-header angle calibration (KMAX 4 -> %d); "
                        "the stored mechanical zero and full-step table are now "
                        "marked as not belonging to it", KMAX);
        return true;
    }

    nvs_close(nvs);
    ESP_LOGW("cfg", "Stored angle calibration has an unexpected size (%u bytes) - using defaults",
             (unsigned)size);
    return false;
}

bool Configuration::saveAngleCalToNvs() const
{
    nvs_handle_t nvs;
    if (nvs_open(ANGLECAL_NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK)
    {
        ESP_LOGE("cfg", "Could not open NVS namespace '%s' to persist angle calibration", ANGLECAL_NVS_NAMESPACE);
        return false;
    }
    // Only a real change to the coefficients bumps the generation. save() is
    // called for unrelated reasons (a network setting, a direction flag), and
    // bumping on those would invalidate a perfectly good zero and table every
    // time someone changed an IP address.
    AngleCalBlob previous;
    size_t previousSize = sizeof(previous);
    bool havePrevious = nvs_get_blob(nvs, ANGLECAL_NVS_KEY, &previous, &previousSize) == ESP_OK &&
                        previousSize == sizeof(previous) && previous.magic == ANGLECAL_MAGIC;
    bool changed = !havePrevious || previous.C0 != _data.C0;
    for (int i = 0; !changed && i <= KMAX; ++i)
        changed = previous.A[i] != _data.A[i] || previous.B[i] != _data.B[i];

    AngleCalBlob blob;
    blob.magic = ANGLECAL_MAGIC;
    blob.version = ANGLECAL_VERSION;
    blob.kmax = KMAX;
    blob.generation = changed ? (havePrevious ? previous.generation : _generation) + 1
                              : previous.generation;
    blob.C0 = _data.C0;
    for (int i = 0; i <= KMAX; ++i)
    {
        blob.A[i] = _data.A[i];
        blob.B[i] = _data.B[i];
    }
    if (changed)
        ESP_LOGI("cfg", "Angle calibration changed - generation %lu; the mechanical zero "
                        "and full-step table no longer belong to it until re-measured",
                 (unsigned long)blob.generation);
    const_cast<Configuration *>(this)->_generation = blob.generation;
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
    uint32_t stamped = 0;
    if (nvs_get_u32(nvs, FULLSTEP_GEN_NVS_KEY, &stamped) != ESP_OK)
        stamped = 0;
    nvs_close(nvs);
    if (err != ESP_OK || size != table.size() * sizeof(float))
        return false;

    // The table is indexed by the corrected sensor value, so it only means
    // anything alongside the correction it was fitted against. Applying a
    // stale one is worse than applying none: all-zero is a no-op, a
    // mismatched one is a confident wrong answer.
    bool allZero = true;
    for (float v : table)
        if (v != 0.0f) { allZero = false; break; }

    // An all-zero table is a no-op correction and therefore belongs to every
    // calibration - reporting it as orphaned would be a false alarm that
    // never clears.
    if (allZero)
    {
        _fullStepTableStale = false;
        _fullStepTableGen = _generation;
        _data.fullStepTable = table;
        return true;
    }

    if (stamped != _generation)
    {
        _fullStepTableStale = true;
        _data.fullStepTable.fill(0.0f);
        // An all-zero table is a no-op correction, so it is valid against any
        // calibration - it adopts the current generation rather than staying
        // permanently mismatched. The drop itself is reported for this boot
        // and logged once, which is the part that matters.
        _fullStepTableGen = _generation;
        ESP_LOGW("cfg", "Full-step table was fitted against calibration generation %lu "
                        "but the stored correction is generation %lu - dropped. "
                        "Re-measure it against the current correction.",
                 (unsigned long)stamped, (unsigned long)_generation);
        return true;
    }
    _fullStepTableStale = false;
    _fullStepTableGen = stamped;
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
    // The stamp the table actually carries - NOT today's generation. save()
    // is called for unrelated reasons, and writing the current generation
    // here would quietly re-validate a table that nobody re-measured, which
    // is the exact failure this whole mechanism exists to catch.
    if (err == ESP_OK)
        err = nvs_set_u32(nvs, FULLSTEP_GEN_NVS_KEY, _fullStepTableGen);
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
    // Whoever writes a table has just fitted it against the correction in
    // force, so it belongs to that generation from here on.
    _fullStepTableGen = _generation;
    {
        std::lock_guard<std::mutex> l(_mtx);
        for (int i = 0; i < N_STEPS; ++i)
            _data.fullStepTable[i] = table[i];
    }
    save();
}
