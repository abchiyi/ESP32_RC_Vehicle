#ifndef Radio_h
#define Radio_h

#include "crtp.h"
#include "array"

typedef std::array<unsigned char, 6> mac_t; // MAC 地址

typedef struct
{
  size_t size;
  uint8_t data[sizeof(_CRTPPacket::raw) + 1]; // 最后一字节储存校验位
} radio_packet_t;

void wifi_init();

#endif
