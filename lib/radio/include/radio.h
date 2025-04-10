
#ifndef Radio_h
#define Radio_h

#include "functional"
#include "esp_err.h"
#include "stdlib.h"
#include "crtp.h"

typedef struct
{
    union
    {
        struct
        {
            uint8_t data[sizeof(CRTPPacket::raw)];
            uint8_t checksum;
        };
        uint8_t raw[sizeof(data) + 1]; // +1 for checksum
    };
} __attribute__((packed)) radio_packet_t;

typedef struct
{
    std::function<esp_err_t(radio_packet_t *)> recv; // 接收回调
    std::function<esp_err_t(radio_packet_t *)> send; // 发送回调
    std::function<bool(void)> is_connected;          // 验证连接
    std::function<esp_err_t(void)> start;            // 启动通信链路
    std::function<esp_err_t(void)> rest;             // 重置通讯链路
} radio_link_operation_t;

typedef std::function<void(CRTPPacket *)> CRTPPacketHandler_fn_t;

esp_err_t radio_set_port_callback(CRTPPort port, CRTPPacketHandler_fn_t fn);

bool radio_link_is_connected();

void init_radio(radio_link_operation_t *link);

#endif
