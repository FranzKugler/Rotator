#include "esp_log.h"
#include "usb_descconfig.h"
#include "tusb.h"
#include "class/cdc/cdc.h"
#include "class/net/net_device.h" // For ECM/RNDIS

#define ITF_NUM_RNDIS 0
#define ITF_NUM_CDC 2
#define ITF_TOTAL 4

#define EPNUM_NET_NOTIF 0x81
#define EPNUM_NET_DATA 0x02
#define EPNUM_CDC_NOTIF 0x83
#define EPNUM_CDC_DATA 0x04

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_RNDIS_DESC_LEN + TUD_CDC_DESC_LEN)

// Standard Device Descriptor
const tusb_desc_device_t descriptor_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x15AE,  // Kayser-Threde VID
    .idProduct = 0x4002, // PID for CDC+ECM/RNDIS
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01};

// Forward declaration for configuration descriptor
extern const uint8_t descriptor_configuration[];

const char *descriptor_strings[] = {
    (const char[]){0x09, 0x04}, // 0: Supported language (0x0409 = English)
    "AstroGeeks",               // 1: Manufacturer
    "AG2998 Rotator",           // 2: Product
    "123456",                   // 3: Serial Number (could be replaced with chip-specific)
    "RNDIS Interface",          // 4: ECM/RNDIS Interface
    "CDC Interface"             // 5: CDC Interface
};

uint8_t const descriptor_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),
    TUD_RNDIS_DESCRIPTOR(ITF_NUM_RNDIS, 4, EPNUM_NET_NOTIF, 8, EPNUM_NET_DATA, 0x80 | EPNUM_NET_DATA, 64),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 5, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_DATA, 0x80 | EPNUM_CDC_DATA, 64)};

// USB netif glue
// Set your desired IP, mask, gateway
static esp_netif_ip_info_t ip_info = {
    .ip.addr = ESP_IP4TOADDR(192, 168, 7, 1),
    .gw.addr = ESP_IP4TOADDR(192, 168, 7, 1),
    .netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0)};

// --- YOU provide the MAC address, for example:
uint8_t tud_network_mac_address[6] = {0x02, 0x02, 0x84, 0x6A, 0x96, 0x00};

// ---------------------------------------------------
// ------------- TinyUSB RNDIS Callbacks -------------
// ---------------------------------------------------

esp_netif_t *g_usb_netif = NULL;

void usb_free_rx_buffer(void *h, void *buffer)
{
    free(buffer);
}

esp_err_t usb_netif_transmit(void *driver_handle, void *buffer, size_t len)
{
    // If TinyUSB needs to own the buffer, malloc/copy it.
    void *usb_buf = malloc(len);
    if (!usb_buf)
        return ESP_ERR_NO_MEM;
    memcpy(usb_buf, buffer, len);

    if (tud_network_can_xmit(len))
    {
        tud_network_xmit(usb_buf, len);
        return ESP_OK;
    }
    free(usb_buf);
    return ESP_FAIL;
}

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg)
{
    memcpy(dst, ref, arg);
    free(ref); // ONLY IF you malloc'd ref in usb_netif_transmit!
    return arg;
}

bool tud_network_recv_cb(const uint8_t *src, uint16_t size)
{
    // allocate new buffer, copy the data and hand it to esp_netif_receive
    uint8_t *newbuf = malloc(size);
    memcpy(newbuf, src, size);
    esp_netif_receive(g_usb_netif, newbuf, size, NULL);

    // we're ready to receive new packets now
    tud_network_recv_renew();
    return true;
}

// Provide MAC address to TinyUSB
const uint8_t *tud_network_mac_address_cb(void) { return tud_network_mac_address; }

// Called when interface starts
void tud_network_init_cb(void)
{
    ESP_LOGI("RNDIS", "Network interface init callback");
}

// ------------ Setup USB NETIF Glue ------------

void setup_usb_rndis_ip(uint32_t ip, uint32_t gw, uint32_t netmask)
{
    ip_info.ip.addr = ip;
    ip_info.gw.addr = gw;
    ip_info.netmask.addr = netmask;
}

void setup_usb_rndis_mac(uint8_t mac[6])
{
    memcpy(tud_network_mac_address, mac, sizeof(tud_network_mac_address));
}

// ---- THIS IS YOUR MINIMAL USB NETIF CONFIG FOR ESP-IDF 5.4.x ----
void setup_usb_rndis_netif(void)
{
    static esp_netif_inherent_config_t inherent_cfg = {
        .flags = (ESP_NETIF_DHCP_SERVER | ESP_NETIF_FLAG_AUTOUP),
        .mac = {0x02, 0x02, 0x84, 0x6A, 0x96, 0x00}, // just a default placeholder
        .ip_info = &ip_info,
        .get_ip_event = IP_EVENT_ETH_GOT_IP,   // IP_EVENT_STA_GOT_IP,
        .lost_ip_event = IP_EVENT_ETH_LOST_IP, // IP_EVENT_STA_LOST_IP,
        .if_key = "RNDIS",
        .if_desc = "usb-rndis",
        .route_prio = 20,
    };

    // just overwrite with our globally set mac address to be sure we use the same
    memcpy(inherent_cfg.mac, tud_network_mac_address, sizeof(inherent_cfg.mac));

    static esp_netif_driver_ifconfig_t driver_cfg = {
        .handle = (void *)1, //&dummy_usb_driver_ctx,
        .transmit = usb_netif_transmit,
        .driver_free_rx_buffer = usb_free_rx_buffer};

    esp_netif_config_t netif_cfg = {
        .base = &inherent_cfg,
        .driver = &driver_cfg,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    };

    g_usb_netif = esp_netif_new(&netif_cfg);
    assert(g_usb_netif);

    // This triggers "link up"
    esp_netif_action_connected(g_usb_netif, NULL, 0, NULL);

    esp_netif_action_start(g_usb_netif, NULL, 0, NULL);
    esp_netif_dhcps_start(g_usb_netif);

    esp_netif_dhcp_status_t status;
    esp_netif_dhcps_get_status(g_usb_netif, &status);
    ESP_LOGI("RNDIS", "DHCP server status: %d", status);

    if (esp_netif_is_netif_up(g_usb_netif))
    {
        ESP_LOGI("RNDIS", "Netif is UP");
    }
    else
    {
        ESP_LOGI("RNDIS", "Netif is DOWN");
    }
}