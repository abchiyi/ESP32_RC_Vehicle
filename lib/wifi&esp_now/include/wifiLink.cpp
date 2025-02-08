#include <esp_log.h>
#include <esp_err.h>
#include <esp_mac.h>
#include "wifiLink.h"

// WiFi & esp_now
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_now.h"

#include "freertos/queue.h"

#include <crtp.h>

#include <vector>

#define TAG "Radio"

uint8_t CHANNEL = 1;     // 通讯频道
uint8_t STATUS_LINK = 0; // 链接状态

/******** 储存配置命名 ********/
#define STOREG_NAME_SPACE "RADIO_CONFIG" // 储存命名空间
#define STOREG_LAST_DEVICE "HOST_ADDR"   // 最后连接的设备

#define WIFI_MODE WIFI_STA

static xQueueHandle crtpPacketDelivery;
static xQueueHandle crtpPacketReceive;

int setWiFiLinkEnable(bool enable);
int sendWifiLinkPacket(CRTPPacket *pk);
int receiveWifiLinkPacket(CRTPPacket *pk);
bool isWiFiLinkConnected(void);
int resetWiFiLink(void);

static struct crtpLinkOperations wifiLinkInstance = {
    .setEnable = setWiFiLinkEnable,
    .sendPacket = sendWifiLinkPacket,
    .receivePacket = receiveWifiLinkPacket,
    .isConnected = isWiFiLinkConnected,
    // .reset = resetWiFiLink,
};

static uint8_t calculate_cksum(void *data, size_t len)
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
 * @brief 根据 mac 地址配对到指定的设备
 * @param macaddr 数组 mac地址
 * @param channel wifi 频道
 * @param ifidx 要使用的wifi接口用于收发数据
 */
bool add_peer(mac_t macaddr, uint8_t channel, wifi_interface_t ifidx = WIFI_IF_STA)
{
  // ESP_LOGI(TAG, "Pair to " MACSTR "", MAC2STR(macaddr));
  esp_now_peer_info_t p;
  auto peer_info = &p;
  memset(peer_info, 0, sizeof(esp_now_peer_info_t)); // 清空对象
  memcpy(peer_info->peer_addr, macaddr.data(), ESP_NOW_ETH_ALEN);
  peer_info->channel = channel;
  peer_info->encrypt = false;
  peer_info->ifidx = ifidx;

  esp_err_t ret = esp_now_add_peer(peer_info);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Pair fail, error code: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGD(TAG, "Pair success, channel: %d", channel);
  return true;
}

template <typename T>
bool send(const T &data)
{
  // ESP_LOGI(TAG, "Send to " MACSTR " - pd of : %d", MAC2STR(this->peer_info.peer_addr), &data);

  // String error_message;
  // switch (esp_now_send(
  //     this->peer_info.peer_addr,
  //     (uint8_t *)&data, sizeof(data)))
  // {
  // case ESP_OK:
  //   return true;
  // case ESP_ERR_ESPNOW_NO_MEM:
  //   error_message = String("out of memory");
  //   break;
  // case ESP_ERR_ESPNOW_NOT_FOUND:
  //   error_message = String("peer is not found");
  //   break;
  // case ESP_ERR_ESPNOW_IF:
  //   error_message = String("current WiFi interface doesn't match that of peer");
  //   break;
  // case ESP_ERR_ESPNOW_NOT_INIT:
  //   error_message = String("ESPNOW is not initialized");
  //   break;
  // case ESP_ERR_ESPNOW_ARG:
  //   error_message = String("invalid argument");
  //   break;
  // case ESP_ERR_ESPNOW_INTERNAL:
  //   error_message = String("internal error");
  //   break;
  // default:
  //   error_message = String("Send fail");
  //   break;
  // }

  // ESP_LOGI(TAG, "Send to " MACSTR " - %s",
  //          MAC2STR(this->peer_info.peer_addr), error_message.c_str());
  return false;
}

// 接收回调
void onRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  // memcpy(&radio_data_recv, incomingData, sizeof(radio_data_recv));
  // memcpy(&radio_data_recv.mac_addr, mac, sizeof(radio_data_recv.mac_addr));
  // if (xQueueSend(Q_RECV_DATA, &radio_data_recv, 1) != pdPASS)
  //   ;
}

// 发送回调
void onSend(const uint8_t *mac_addr, esp_now_send_status_t status)
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

void wifiLinkTask(void *pvParameters)
{
  CRTPPacket *p = nullptr;
  while (true)
  {
    if (xQueueReceive(crtpPacketDelivery, &p, portMAX_DELAY) == pdTRUE)
    {
      xQueueSend(crtpPacketDelivery, &p, pdMS_TO_TICKS(20));
    }
  }
}

#include "sstream"
#include "numeric"

void wifi_init()
{
  std::ostringstream oss;
  auto sta_mac = WiFi.macAddress().c_str();
  oss << "ESP" << "-" << std::accumulate(sta_mac, sta_mac + 6, 0);
  std::string temp_SSID = oss.str();

  auto wifi_mode = WIFI_MODE == WIFI_STA ? WIFI_IF_STA : WIFI_IF_AP;

  // Set wifi
  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84)); // Set TxPower 20db
  esp_wifi_set_storage(WIFI_STORAGE_RAM);         // WiFi 配置存储在 RAM 中
  WiFi.enableLongRange(true);                     // 启用长距离模式
  assert(WiFi.mode(WIFI_MODE));                   // 设置wifi模式为STA

  // Set esp now
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(
      wifi_mode, WIFI_PHY_RATE_LORA_500K));          // 设置 ESPNOW 通讯速率
  ESP_ERROR_CHECK(esp_now_register_recv_cb(onRecv)); // 注册接收回调
  ESP_ERROR_CHECK(esp_now_register_send_cb(onSend)); // 注册发送回调

  auto ret = xTaskCreate(wifiLinkTask, "wifiLinkTask", 4096, NULL, 10, NULL);
  ESP_ERROR_CHECK(ret);
}

// void confgi_save()
// {
//   ESP_LOGI(TAG, "Save confgi");
//   nvs_call(STORGE_NAME_SPACE, [&](Preferences &prefs)
//            { prefs.putBytes(
//                  STORGE_LAST_DEVICE,
//                  &HOST_MAC,
//                  sizeof(mac_t)); });
// };

// void config_clear()
// {
//   ESP_LOGI(TAG, "Clear confgi");
//   nvs_call(STORGE_NAME_SPACE, [](Preferences &prefs)
//            { prefs.clear(); });
// }
