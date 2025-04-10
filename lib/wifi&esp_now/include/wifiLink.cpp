/**
 * 使用WiFi&ESP-NOW实现无线通讯,通讯协议使用 CRTP 协议。WiFi 允许3个设备连接，
 * ESP-NOW 只允许一个主机进行通讯。peer_info 会记录第一个连接的主机，此后只要该
 * 变量不为 nullptr，ESP_NOW 端口则不会接收来自其他主机的数据。
 * Copyright (c) <abchiyi>
 */

#include "wifiLink.h"
#include "common.h"

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

#define TAG "wifiLink"

#define PORT 8080                        // UDP 监听端口
#define STOREG_NAME_SPACE "RADIO_CONFIG" // 储存命名空间
#define STOREG_LAST_DEVICE "HOST_ADDR"   // 最后连接的设备
#define WIFI_MODE WIFI_AP_STA            // WIFI AP&STA 模式
#define TIME_OUT_WiFi_RX_PACKET 100      // WiFi 接收超时时间(ms)

// #define LORA_MODE                        // 启用以开启 LoRa 模式，默认关闭

static esp_now_peer_info_t *peer_info = nullptr;  // ESP_NOW通讯设备信息
static xQueueHandle crtpPacketDelivery = nullptr; // crtp 发送队列
static xQueueHandle wifiPacketReceive = nullptr;  // wifi 接收队列
static volatile TickType_t wifiReceiveInterval;   // WiFi 接收间隔

esp_err_t setWiFiLinkEnable();
esp_err_t wifiLinkPacketSend(radio_packet_t *pk);
int wifiLinkPacketRecv(radio_packet_t *pk);
bool isWiFiLinkConnected(void);
static uint8_t calculate_cksum(void *data, size_t len);
int resetWiFiLink(void);

WiFiUDP udp;

static radio_link_operation_t wifiLinkInstance = {
    .recv = wifiLinkPacketRecv,
    .send = wifiLinkPacketSend,
    .is_connected = isWiFiLinkConnected,
    .start = setWiFiLinkEnable,
    // .reset = resetWiFiLink,
};

IRAM_ATTR inline int wifiLinkPacketRecv(radio_packet_t *rp)
{
  if (wifiPacketReceive == nullptr)
    return -1;

  if (xQueueReceive(wifiPacketReceive, rp, portMAX_DELAY) == pdTRUE)
    return 0;
  else
    return -1;
}

// TODO 未完成的函数
IRAM_ATTR inline int wifiLinkPacketSend(radio_packet_t *rp)
{

  rp->checksum = calculate_cksum(rp->data, sizeof(rp->data) - 1);

  // ESP_NOW
  if (peer_info != nullptr)
  {
    auto ret = esp_now_send(peer_info->peer_addr, (uint8_t *)&rp, sizeof(rp));
    if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "esp_now_send failed: %s", esp_err_to_name(ret));
      return ESP_FAIL;
    }
  }
  // TODO WIFI_UDP

  return ESP_OK;
}

/**
 * @brief 判断 WiFi 链路是否已连接
 * 该函数用于检查 WiFi 链路的连接状态。
 * @return 如果 WiFi 链路已连接，则返回 true；否则返回 false。
 */
bool isWiFiLinkConnected(void)
{
  return wifiReceiveInterval <= TIME_OUT_WiFi_RX_PACKET;
}

radio_link_operation_t *WiFiGetLink()
{
  return &wifiLinkInstance;
}
/*-----------------------------------------------------------------*/

static uint8_t calculate_cksum(void *data, size_t len)
{
  auto c = (unsigned char *)data;
  unsigned char cksum = 0;
  for (int i = 0; i < len; i++)
    cksum += *(c++);
  return cksum;
}

bool identifyRadioPacket(radio_packet_t *rp)
{
  auto cksum = calculate_cksum(rp->data, sizeof(rp->data));
  return rp->checksum == cksum;
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
    auto scanResults = WiFi.scanNetworks(0, 1, 0, 50, channel + 1);
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
 */
bool add_peer(const uint8_t *macaddr)
{
  auto ifidx = (WIFI_MODE == WIFI_AP_STA)
                   ? WIFI_IF_AP
                   : WIFI_IF_STA;

  if (!peer_info)
  {
    peer_info = (esp_now_peer_info_t *)malloc(sizeof(esp_now_peer_info_t));
    if (!peer_info)
    {
      ESP_LOGE(TAG, "Failed to allocate memory for peer_info");
      return false;
    }
  }
  else
    memset(peer_info, 0, sizeof(esp_now_peer_info_t));

  peer_info->channel = WiFi.channel();
  peer_info->encrypt = false;
  peer_info->ifidx = ifidx;

  memcpy(peer_info->peer_addr, macaddr, ESP_NOW_ETH_ALEN);

  esp_err_t ret = esp_now_add_peer(peer_info);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Pair fail, error code: %s", esp_err_to_name(ret));
    free(peer_info);
    peer_info = nullptr;
    return false;
  }
  ESP_LOGI(TAG, "Pair success,mac :" MACSTR "", MAC2STR(macaddr));
  return true;
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

// ESP-NOW 接收回调
IRAM_ATTR inline void onRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  // 接收的数据长度需大于等于 radio_packet_t 的长度,以避免越界访问内存
  if (len < sizeof(radio_packet_t))
    return;

  // 当对等对象未初始化时，在第一次收到数据包进行分配内存并设置为数据源主机
  if (!peer_info && identifyRadioPacket((radio_packet_t *)incomingData))
    add_peer(mac);

  // 过滤非来自目标主机的数据包
  if (peer_info && !memcmp(peer_info->peer_addr, mac, ESP_NOW_ETH_ALEN))
    xQueueSend(wifiPacketReceive, incomingData, len);
}

// WiFi UDP 接收任务
IRAM_ATTR void wifiUdpTaskRecv(void *pvParameters)
{
  // 此任务以200hz的频率运行
  TickType_t xFrequency = HZ2TICKS(200);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    static radio_packet_t rp = {};
    static auto packetSize = udp.parsePacket();

    if (peer_info != nullptr) // esp-now 的数据优先
      continue;

    if (packetSize && packetSize < sizeof(rp)) // 包长度有效&避免缓冲区溢出
    {
      IPAddress remoteIP = udp.remoteIP();
      uint16_t remotePort = udp.remotePort();

      udp.read(rp.data, packetSize);
      xQueueSend(wifiPacketReceive, &rp, 0);
    }
  }
}

#include "ESP32Servo.h"
/**
 * @brief 接收校验数据是否有效，有效则发布到队列
 *
 * 该任务持续监听 wifiPacketReceive 队列，接收来自无线模块的数据包。
 * 接收到数据包后，它会验证数据的校验和以确保数据的完整性。
 * 如果数据有效，它会将数据复制到 CRTPPacket 结构体，并将原始数据包发送到 crtpPacketDelivery 队列以供进一步处理。
 */
IRAM_ATTR void wifiLinkTask(void *pvParameters)
{
  static radio_packet_t rp = {};
  auto xMutex = xSemaphoreCreateMutex();

  while (true)
  {
    memset(&rp, 0, sizeof(radio_packet_t));

    if (xQueueReceive(wifiPacketReceive, &rp, portMAX_DELAY) == pdTRUE)
    {
      // 记录接收时间
      static TickType_t lastRecvTime = 0;
      TickType_t currentTime = xTaskGetTickCount();

      taskENTER_CRITICAL(xMutex);
      if (lastRecvTime != 0)
        wifiReceiveInterval = currentTime - lastRecvTime;
      taskEXIT_CRITICAL(xMutex);

      lastRecvTime = currentTime;

      // TODO 如果数据来自 esp-now 则不校验

      if (!identifyRadioPacket(&rp))
        continue;

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

  // Set wifi
  auto wifi_mode = (WIFI_MODE == WIFI_STA) ? WIFI_IF_STA : WIFI_IF_AP;
  esp_wifi_set_storage(WIFI_STORAGE_RAM); // WiFi 配置存储在 RAM 中
#ifdef LORA_MODE
  WiFi.enableLongRange(true);   // 启用长距离通信模式
  assert(WiFi.mode(WIFI_MODE)); // 设置wifi模
#else
  assert(WiFi.mode(WIFI_MODE)); // 设置wifi模
  auto ret = esp_wifi_set_protocol(wifi_mode, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  ESP_ERROR_CHECK(ret);
#endif
  assert(WiFi.setTxPower(WIFI_POWER_19_5dBm));

  if (WIFI_MODE == WIFI_AP_STA)
  {
    mac_t mac;
    WiFi.macAddress(mac.data());
    mac[5]++; // mac末尾地址加一为 AP 的mac地址

    char ssid[33];
    sprintf(ssid, "ESP32-%02X:%02X:%02X", mac[3], mac[4], mac[5]);

    // TODO 使用nvs储存另设密码
    auto channel = find_clear_channel();             // 查找最清晰的信道
    WiFi.softAP(ssid, "12345678", channel, 0, 1, 1); // 设置AP
    ESP_LOGI(TAG, "AP Started, SSID: %s", ssid);
  }

  // Set UDP
  udp.begin(7890);
  ret = xTaskCreate(wifiUdpTaskRecv, "wifiUdpTask", 4096, NULL, 10, NULL);

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

  ret = xTaskCreate(wifiLinkTask, "wifiLinkTask", 4096, NULL, 10, NULL);
  assert(ret == pdPASS);
}

esp_err_t setWiFiLinkEnable()
{
  wifi_init();
  return ESP_OK;
}
