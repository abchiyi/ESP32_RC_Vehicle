#ifndef Radio_h
#define Radio_h

#include "functional"

typedef std::array<unsigned char, 6> mac_t; // MAC 地址

// 通讯结构体
typedef enum radio_status
{
  RADIO_BEFORE_WAIT_CONNECTION,
  RADIO_WAIT_CONNECTION,

  RADIO_BEFORE_CONNECTED,
  RADIO_CONNECTED,

  RADIO_BEFORE_DISCONNECT,
  RADIO_DISCONNECT,
} radio_status_t;

void wifi_init();

#endif
