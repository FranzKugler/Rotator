#pragma once
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "tusb.h"
//#include "lwip/ip4_addr.h"

// These externs allow main.c to reference the descriptors
extern const tusb_desc_device_t descriptor_device;
extern const uint8_t descriptor_configuration[];
extern const char *descriptor_strings[];

void setup_usb_rndis_ip(uint32_t ip, uint32_t gw, uint32_t netmask);
void setup_usb_rndis_netif(void);
void setup_usb_rndis_mac(uint8_t mac[6]);