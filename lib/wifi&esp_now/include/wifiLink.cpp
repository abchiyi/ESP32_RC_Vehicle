/**
 * 使用WiFi&ESP-NOW实现无线通讯,通讯协议使用 CRTP 协议。WiFi 允许3个设备连接，
 * ESP-NOW 只允许一个主机进行通讯。peer_info 会记录第一个连接的主机，此后只要该
 * 变量不为 nullptr，ESP_NOW 端口则不会接收来自其他主机的数据。
 * Copyright (c) <abchiyi>
 */

#include "common.h"

#include "wifiLink.h"
#include <crtp.h>

#include <esp_log.h>
#include <esp_err.h>
#include <esp_mac.h>
#include <vector>

// WiFi & esp_now
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "WiFiUdp.h"

// rtos
#include "freertos/queue.h"

#define TAG "Radio"

#define PORT 8080                        // UDP 监听端口
#define STOREG_NAME_SPACE "RADIO_CONFIG" // 储存命名空间
#define STOREG_LAST_DEVICE "HOST_ADDR"   // 最后连接的设备
#define WIFI_MODE WIFI_AP_STA            // WIFI AP&STA 模式
// #define LORA_MODE // 启用以开启 LoRa 模式，默认关闭。启用后使用乐鑫专有协议

static esp_now_peer_info_t *peer_info = nullptr;  // ESP_NOW通讯设备信息
static xQueueHandle crtpPacketDelivery = nullptr; // crtp 发送队列
static xQueueHandle wifiPacketReceive = nullptr;  // wifi 接收队列

int setWiFiLinkEnable(bool enable);
int wifiLinkPacketSend(CRTPPacket *pk);
int wifiLinkPacketRecv(CRTPPacket *pk);
bool isWiFiLinkConnected(void);
int resetWiFiLink(void);

WiFiUDP udp;

static struct crtpLinkOperations wifiLinkInstance = {
    .setEnable = setWiFiLinkEnable,
    .sendPacket = wifiLinkPacketSend,
    .receivePacket = wifiLinkPacketRecv,
    .isConnected = isWiFiLinkConnected,
    // .reset = resetWiFiLink,
};

IRAM_ATTR inline int wifiLinkPacketRecv(CRTPPacket *pk)
{
  if (wifiPacketReceive == nullptr)
    return -1;

  if (xQueueReceive(wifiPacketReceive, pk, 0) == pdTRUE)
    return 0;
  else
    return -1;
}

// TODO 未完成的函数
IRAM_ATTR inline int wifiLinkPacketSend(CRTPPacket *pk)
{
  ESP_LOGE(TAG, "UN SET SEND");
  assert(false);
}

static uint8_t
calculate_cksum(void *data, size_t len)
{
  auto c = (unsigned char *)data;
  unsigned char cksum = 0;
  for (int i = 0; i < len; i++)
    cksum += *(c++);
  return cksum;
}

int setWiFiLinkEnable(bool enable)
{
  return 0;
}

// 检查mac是否有效
bool macOK(const mac_t &arr)
{
  return std::any_of(arr.begin(), arr.end(),
                     [](uint8_t byte)
                     { return byte != 0x00 && byte != 0xFF; });
}

/**
 * @brief 查找最清晰的WiFi信道
 *
 * 该函数扫描1到14信道，统计每个信道上发现的AP数量，并返回AP数量最少的信道编号。
 * 信道编号从1开始。
 *
 * @return uint8_t 最清晰的信道编号
 */
uint8_t find_clear_channel()
{
  uint8_t ap_count[14] = {}; // 保存扫描到的AP数量,信道 1~14

  // 扫描1~13信道，过滤并储存所有查找到的AP信息
  for (size_t channel = 0; channel < sizeof(ap_count); channel++)
  {
    auto scanResults = WiFi.scanNetworks(0, 1, 0, 100, channel + 1);
    ap_count[channel] = scanResults;
    WiFi.scanDelete(); // 清除扫描信息
  }
  // 对 ap_count 进行排序，从小到大
  std::sort(ap_count, ap_count + sizeof(ap_count) / sizeof(ap_count[0]));

  // 找到最清晰的信道（AP数量最少的信道）
  uint8_t clear_channel =
      std::min_element(ap_count, ap_count + sizeof(ap_count) / sizeof(ap_count[0])) - ap_count;
  return clear_channel + 1; // 信道编号从1开始
};

/**
 * @brief 根据 mac 地址配对到指定的设备
 * @param macaddr 数组 mac地址
 * @param channel wifi 频道
 * @param ifidx 要使用的wifi接口用于收发数据
 */
bool add_peer(mac_t macaddr)
{
  auto ifidx = (WIFI_MODE == WIFI_AP_STA)
                   ? WIFI_IF_AP
                   : WIFI_IF_STA;

  esp_now_peer_info_t peer_info = {
      .channel = 0,     // 0 = 当前WiFi所在频道
      .ifidx = ifidx,   // 要使用的接口
      .encrypt = false, // 不加密
  };
  memcpy(peer_info.peer_addr, macaddr.data(), ESP_NOW_ETH_ALEN);

  esp_err_t ret = esp_now_add_peer(&peer_info);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Pair fail, error code: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "Pair success,mac :" MACSTR "", MAC2STR(macaddr));
  return true;
}

// ESP-NOW 接收回调
IRAM_ATTR inline void onRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  static radio_packet_t radio_data_recv; // 接收数据缓存
  if (len > sizeof(radio_packet_t))      // 避免缓冲区溢出
    return;
  memcpy(&radio_data_recv, incomingData, len);

  // 避免阻塞WiFi任务，等待时间0
  xQueueSend(wifiPacketReceive, &radio_data_recv, 0);
}

// 发送回调
IRAM_ATTR inline void onSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  static uint8_t counter_resend = 0;

  // if (status == ESP_NOW_SEND_SUCCESS)
  // {
  //   counter_resend = 0;
  //   if (xQueueSend(Q_ACK, mac_addr, 1) != pdPASS)
  //     ;
  // }
  // else
  // {
  //   ESP_LOGI(TAG, "Send to " MACSTR " FAIl", MAC2STR(mac_addr));
  //   counter_resend++;
  // }

  // if (counter_resend >= radio.resend_count &&
  //     radio.status != RADIO_DISCONNECT &&
  //     radio.status != RADIO_BEFORE_DISCONNECT)
  // {
  //   ESP_LOGI(TAG, "DISCONNECT with timeout");
  //   radio.status = RADIO_BEFORE_DISCONNECT;
  //   counter_resend = 0;
  // }
  // else
  //   ESP_LOGI(TAG, "Send to " MACSTR " SUCCESS", MAC2STR(mac_addr));
}

IRAM_ATTR void wifiUdpTask(void *pvParameters)
{
  // 此任务以200hz的频率运行
  TickType_t xFrequency = HZ2TICKS(200);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true)
  {
    static radio_packet_t rp = {};
    auto packetSize = udp.parsePacket();
    if (packetSize && packetSize < sizeof(rp)) // 包长度有效&避免缓冲区溢出
    {
      udp.read(rp.data, packetSize);
      xQueueSend(wifiPacketReceive, &rp, 0);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
 * @brief 接收校验数据是否有效，有效则发布到队列
 *
 * 该任务持续监听 wifiPacketReceive 队列，接收来自无线模块的数据包。
 * 接收到数据包后，它会验证数据的校验和以确保数据的完整性。
 * 如果数据有效，它会将数据复制到 CRTPPacket 结构体，并将原始数据包发送到 crtpPacketDelivery 队列以供进一步处理。
 *
 * @param pvParameters 任务参数，未在此函数中使用。
 *
 * @note 该函数使用 IRAM_ATTR 属性，表明它应该存储在 IRAM 中，以便在中断上下文中快速访问。
 */
IRAM_ATTR void wifiLinkTask(void *pvParameters)
{
  static radio_packet_t rp = {};
  static CRTPPacket crtp = {};
  while (true)
  {
    memset(&rp, 0, sizeof(radio_packet_t));

    if (xQueueReceive(wifiPacketReceive, &rp, portMAX_DELAY) == pdTRUE)
    {
      // 验证数据是否有效
      auto cksum = rp.data[sizeof(rp.data) - 1];
      if (cksum != calculate_cksum(rp.data, sizeof(rp.data)))
      {
        ESP_LOGE(TAG, "CRC ERROR");
        continue;
      }
      memcpy(&crtp, rp.data, sizeof(crtp.raw));
      xQueueSend(crtpPacketDelivery, &rp, pdMS_TO_TICKS(5));
    }
  }
}

void wifi_init()
{

  // queue setup
  wifiPacketReceive = xQueueCreate(10, sizeof(radio_packet_t));
  assert(wifiPacketReceive);

  crtpPacketDelivery = xQueueCreate(10, sizeof(CRTPPacket));
  assert(crtpPacketDelivery);

  auto wifi_mode = WIFI_MODE == WIFI_STA ? WIFI_IF_STA : WIFI_IF_AP;

  // Set wifi
  esp_wifi_set_storage(WIFI_STORAGE_RAM); // WiFi 配置存储在 RAM 中
#ifdef LORA_MODE
  WiFi.enableLongRange(true); // 启用长距离通信模式
#else
  WiFi.enableLongRange(false); // 启用长距离通信模式
#endif
  assert(WiFi.mode(WIFI_MODE)); // 设置wifi模
  assert(WiFi.setTxPower(WIFI_POWER_19_5dBm));

  if (WIFI_MODE == WIFI_AP_STA)
  {
    mac_t mac;
    WiFi.macAddress(mac.data());
    mac[5]++; // mac末尾地址加一为 AP 的mac地址

    char ssid[33];
    sprintf(ssid, "ESP32-%02X%02X%02X", mac[3], mac[4], mac[5]);

    // TODO 使用nvs储存另设密码
    auto channel = find_clear_channel();             // 查找最清晰的信道
    WiFi.softAP(ssid, "12345678", channel, 0, 3, 1); // 设置AP
    ESP_LOGI(TAG, "AP Started, SSID: %s", ssid);
  }

  // Set UDP
  udp.begin(7890);
  auto ret = xTaskCreate(wifiUdpTask, "wifiUdpTask", 4096, NULL, 10, NULL);

  // Set esp now
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(onRecv)); // 注册接收回调
  ESP_ERROR_CHECK(esp_now_register_send_cb(onSend)); // 注册发送回调
#ifdef LORA_MODE
  ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(
      wifi_mode, WIFI_PHY_RATE_LORA_500K)); // 设置 ESPNOW 通讯速率
#else
  ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(
      wifi_mode, WIFI_PHY_RATE_MCS1_LGI)); // 设置 ESPNOW 通讯速率
#endif

  auto ret = xTaskCreate(wifiLinkTask, "wifiLinkTask", 4096, NULL, 10, NULL);
  assert(ret == pdPASS);
}
