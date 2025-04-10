#ifndef WifiLink_h
#define WifiLink_h

#include "crtp.h"
#include "array"
#include "radio.h"

typedef std::array<unsigned char, 6> mac_t; // MAC 地址

void wifi_init();

radio_link_operation_t *WiFiGetLink();

#endif
