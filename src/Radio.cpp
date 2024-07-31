#include <WiFi.h>
#include <esp_wifi.h>
#include <Radio.h>
#include <esp_now.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_mac.h>
#include <vector>
#include "tool.h"

#define TAG "Radio"

Radio radio;
uint8_t CHANNEL; // 通讯频道
radio_data_t radio_data_recv;
radio_data_t radio_data_send;

/******** 储存配置命名 ********/
#define STORGE_NAME_SPACE "RADIO_CONFIG" // 储存命名空间
#define STORGE_LAST_DEVICE "HOST_ADDR"   // 最后连接的设备

// 接收数据队列
QueueHandle_t Q_RECV_DATA = xQueueCreate(2, sizeof(radio_data_t));

// 向下数据队列，将处理后的数据发布到下级任务
QueueHandle_t Q_DATA_RECV = xQueueCreate(3, sizeof(radio_data_t));
QueueHandle_t Q_DATA_SEND = xQueueCreate(2, sizeof(radio_data_t));

QueueHandle_t Q_ACK = xQueueCreate(2, sizeof(mac_t));

/**
 * @brief 等待主机握手
 * @param waitTick 超时等待
 * @param data 收到响应时接收数据将写入其中
 */
bool wait_response(TickType_t waitTick, radio_data_t *data)
{
  if (xQueueReceive(Q_RECV_DATA, data, waitTick) != pdPASS)
  {
    ESP_LOGI(TAG, "Wait for response timed out");
    return false;
  }
  return true;
};

bool wait_ACK(TickType_t waitTick, mac_t mac)
{
  mac_t temp_mac;
  if (xQueueReceive(Q_ACK, &temp_mac, waitTick) != pdPASS)
  {
    ESP_LOGI(TAG, "Wait for ACK timed out");
    return false;
  }
  if (mac != temp_mac)
  {
    ESP_LOGI(TAG, "temp mac " MACSTR ", MAC " MACSTR "",
             MAC2STR(temp_mac), MAC2STR(mac));
    return wait_ACK(waitTick, mac);
  }
  return true;
};

/**
 * @brief 根据 mac 地址配对到指定的设备
 * @param macaddr 数组 mac地址
 * @param channel wifi 频道
 * @param ifidx 要使用的wifi接口用于收发数据
 */
bool add_peer(
    mac_t macaddr,
    uint8_t channel,
    wifi_interface_t ifidx = WIFI_IF_STA)
{
  // ESP_LOGI(TAG, "Pair to " MACSTR "", MAC2STR(macaddr));
  auto peer_info = &radio.peer_info;
  memset(peer_info, 0, sizeof(esp_now_peer_info_t)); // 清空对象
  memcpy(peer_info->peer_addr, macaddr.data(), ESP_NOW_ETH_ALEN);
  peer_info->channel = channel;
  peer_info->encrypt = false;
  peer_info->ifidx = ifidx;

  esp_err_t addStatus = esp_now_add_peer(peer_info);

  switch (addStatus)
  {
  case ESP_OK:
    // ESP_LOGI(TAG, "Pair success");
    return true;

  case ESP_ERR_ESPNOW_EXIST:
    ESP_LOGE(TAG, "Peer Exists");
    return true;

  case ESP_ERR_ESPNOW_NOT_INIT:
    ESP_LOGE(TAG, "ESPNOW Not Init");
    return false;

  case ESP_ERR_ESPNOW_ARG:
    ESP_LOGE(TAG, "Invalid Argument");
    return false;

  case ESP_ERR_ESPNOW_FULL:
    ESP_LOGE(TAG, "Peer list full");
    return false;

  case ESP_ERR_ESPNOW_NO_MEM:
    ESP_LOGE(TAG, "Out of memory");
    return false;

  default:
    ESP_LOGE(TAG, "Not sure what's going on");
    return false;
  }
}

template <typename T>
bool Radio::send(const T &data)
{
  // ESP_LOGI(TAG, "Send to " MACSTR " - pd of : %d", MAC2STR(this->peer_info.peer_addr), &data);

  String error_message;
  switch (esp_now_send(
      this->peer_info.peer_addr,
      (uint8_t *)&data, sizeof(data)))
  {
  case ESP_OK:
    return true;
  case ESP_ERR_ESPNOW_NO_MEM:
    error_message = String("out of memory");
    break;
  case ESP_ERR_ESPNOW_NOT_FOUND:
    error_message = String("peer is not found");
    break;
  case ESP_ERR_ESPNOW_IF:
    error_message = String("current WiFi interface doesn't match that of peer");
    break;
  case ESP_ERR_ESPNOW_NOT_INIT:
    error_message = String("ESPNOW is not initialized");
    break;
  case ESP_ERR_ESPNOW_ARG:
    error_message = String("invalid argument");
    break;
  case ESP_ERR_ESPNOW_INTERNAL:
    error_message = String("internal error");
    break;
  default:
    error_message = String("Send fail");
    break;
  }

  ESP_LOGI(TAG, "Send to " MACSTR " - %s",
           MAC2STR(this->peer_info.peer_addr), error_message.c_str());
  return false;
}

// 接收回调
void onRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&radio_data_recv, incomingData, sizeof(radio_data_recv));
  memcpy(&radio_data_recv.mac_addr, mac, sizeof(radio_data_recv.mac_addr));
  if (xQueueSend(Q_RECV_DATA, &radio_data_recv, 1) != pdPASS)
    ;
}

// 发送回调
void onSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  static uint8_t counter_resend = 0;

  if (status == ESP_NOW_SEND_SUCCESS)
  {
    counter_resend = 0;
    if (xQueueSend(Q_ACK, mac_addr, 1) != pdPASS)
      ;
  }
  else
  {
    ESP_LOGI(TAG, "Send to " MACSTR " FAIl", MAC2STR(mac_addr));
    counter_resend++;
  }

  if (counter_resend >= radio.resend_count &&
      radio.status != RADIO_DISCONNECT &&
      radio.status != RADIO_BEFORE_DISCONNECT)
  {
    ESP_LOGI(TAG, "DISCONNECT with timeout");
    radio.status = RADIO_BEFORE_DISCONNECT;
    counter_resend = 0;
  }
  // else
  //   ESP_LOGI(TAG, "Send to " MACSTR " SUCCESS", MAC2STR(mac_addr));
}

// 初始化 espNow
void Radio::initRadio()
{
  // wifi set
  WiFi.mode(WIFI_STA);
  WiFi.enableLongRange(true);
  // esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  // esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR);

  // 设置模式 STA
  if (WiFi.mode(WIFI_STA))
    ESP_LOGI(TAG, "WIFI Start in STA,MAC: %s, CHANNEL: %u",
             WiFi.macAddress().c_str(), WiFi.channel());

  // 设置 ESPNOW 通讯速率
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_LORA_500K) == ESP_OK
      ? ESP_LOGI(TAG, "Set ESPNOW WIFI_PHY_RATE_LORA_500K")
      : ESP_LOGI(TAG, "Set ESPNOW RATE FAIL");

  // 设置最大 TX Power 到 20db
  esp_wifi_set_max_tx_power(84) == ESP_OK
      ? ESP_LOGI(TAG, "Set TxPower 20db")
      : ESP_LOGI(TAG, "Set TxPower Fail");

  // 设置 ESPNOW
  if (esp_now_init() != ESP_OK)
    esp_system_abort("ESP NOW Init Fail.");
  else
    ESP_LOGI(TAG, "ESP NOW init success");

  // 注册接收回调
  esp_now_register_recv_cb(onRecv) == ESP_OK
      ? ESP_LOGI(TAG, "Register recv cb success")
      : ESP_LOGE(TAG, "Register recv cb fail");

  // 注册发送回调
  esp_now_register_send_cb(onSend) == ESP_OK
      ? ESP_LOGI(TAG, "Register send cb success")
      : ESP_LOGE(TAG, "Register send cb fail");
}

/**
 * @brief 与指定地址握手
 */
esp_err_t handshake(mac_t *mac_addr)
{
  radio_data_t data;
  ESP_LOGI(TAG, "wait handshake");
  // 2次握手请求，每次重试3次，均无响应握手失败
  for (size_t i = 0; i < 3; i++) // S <- M
  {
    if (wait_response(portMAX_DELAY, &data))
    {
      // 仅在等待连接模式下可以设置主机地址
      if (radio.status == RADIO_WAIT_CONNECTION)
        *mac_addr = data.mac_addr;

      if (data.mac_addr != radio.HOST_MAC)
      {
        ESP_LOGI(TAG,
                 "Get mac : " MACSTR ", HOST_MAC " MACSTR "",
                 MAC2STR(data.mac_addr),
                 MAC2STR(radio.HOST_MAC));
        return ESP_ERR_INVALID_MAC;
      }

      add_peer(data.mac_addr, 1);
      break;
    }
    if (i >= 3)
      return ESP_ERR_TIMEOUT;
  }
  for (size_t i = 0; i < 3; i++) // S -> M
  {
    if (!radio.send(data))
      continue;
    if (wait_ACK(radio.timeout_resend, *mac_addr))
      break;
    if (i >= 3)
      return ESP_ERR_TIMEOUT;
  }

  ESP_LOGI(TAG, "Pair to Host success");
  return ESP_OK;
}

esp_err_t pairNewDevice()
{
  ESP_LOGI(TAG, "Wait connection");
  if (handshake(&radio.HOST_MAC) == ESP_FAIL)
    return ESP_FAIL;
  radio.confgi_save();
  return ESP_OK;
};

void TaskRadioMainLoop(void *pt)
{
  while (true)
  {
    switch (radio.status)
    {
    case RADIO_BEFORE_WAIT_CONNECTION:
      if (!WiFi.softAP(radio.SSID, "", 1, 0, 1, true))
      {
        ESP_LOGE(TAG, "AP Config failed.");
        break;
      }

      ESP_LOGI(TAG, "AP Config Success.SSID: %s , MAC : %s, CHANNEL : %d", radio.SSID, WiFi.softAPmacAddress().c_str(), WiFi.channel());
      radio.status = RADIO_WAIT_CONNECTION; // AP 开启成功则进入等待连接状态
      break;

    case RADIO_WAIT_CONNECTION:
      radio.status = pairNewDevice() == ESP_OK
                         ? RADIO_BEFORE_CONNECTED
                         : RADIO_BEFORE_DISCONNECT;
      break;

    case RADIO_BEFORE_CONNECTED:
      ESP_LOGI(TAG, "Connect Success");
      radio.status = RADIO_CONNECTED;
      break;

    case RADIO_CONNECTED:
      if (radio.status != RADIO_CONNECTED)
        break;

      if (wait_response(radio.timeout_resend, &radio_data_recv))
        ;
      xQueueReceive(Q_DATA_SEND, &radio_data_send, 1);
      if (radio.__onRecv)
        radio.__onRecv(radio_data_recv);
      radio.send(radio_data_send);

      break;

    case RADIO_BEFORE_DISCONNECT:
      if (radio.onDisconnect) // 执行断联回调
        radio.onDisconnect();
      ESP_LOGI(TAG, "RADIO_DISCONNECT");
      radio.status = RADIO_DISCONNECT;
      break;
    case RADIO_DISCONNECT:
      [&]()
      {
        mac_t mac;
        memcpy(&mac, radio.peer_info.peer_addr, ESP_NOW_ETH_ALEN);
        if (handshake(&mac) == ESP_OK)
          radio.status = RADIO_BEFORE_CONNECTED;
      }();
      break;

    default:
      esp_system_abort("Radio status error");
      break;
    }
  }
}

// 启动 esp_now 通讯
void Radio::begin(const char *ssid, uint8_t channel)
{
  channel = channel;
  SSID = ssid;
  radio.status = RADIO_DISCONNECT;

  this->initRadio();
  nvs_call(STORGE_NAME_SPACE,
           [&](Preferences &prefs)
           {
             prefs.getBytes(STORGE_LAST_DEVICE, &HOST_MAC, sizeof(HOST_MAC));
           });
  ESP_LOGI(TAG, "Targe host mac " MACSTR "", MAC2STR(HOST_MAC));

  // 开启AP
  if (!WiFi.softAP(radio.SSID, "", 1, 0, 1, true))
    ESP_LOGE(TAG, "AP Config failed.");

  ESP_LOGI(TAG, "AP Config Success.SSID: %s , MAC : %s, CHANNEL : %d", radio.SSID, WiFi.softAPmacAddress().c_str(), WiFi.channel());

  xTaskCreate(TaskRadioMainLoop, "TaskRadioMainLoop", 4096, NULL, 24, NULL);
}

channel_data_t read_channel_data(radio_data_t radio_data, int channle)
{
  if (channle > RADIO_CHANNEL_MAX - 1) // 超过读取最大通道数量时
    esp_system_abort(" over RADIO_CHANNEL_MAX " + channle);

  channel_data data;

  data.value = radio_data.channel[channle] >> 4;
  data.mode = radio_data.channel[channle] & 0x0F;
  return data;
}

channel_data_int_t read_channel_data(
    radio_data_t radio_data, int channle, bool intValue)
{
  if (channle > RADIO_CHANNEL_MAX - 1) // 超过读取最大通道数量时
    esp_system_abort(" over RADIO_CHANNEL_MAX " + channle);

  channel_data_int data;
  data.value = (int16_t)radio_data.channel[channle] >> 4;
  data.mode = radio_data.channel[channle] & 0x0F;
  return data;
}

uint16_t set_combined_int(uint16_t value1, uint16_t value2)
{
  return (value1 << 4) | value2;
}

void Radio::confgi_save()
{
  ESP_LOGI(TAG, "Save confgi");
  nvs_call(STORGE_NAME_SPACE, [&](Preferences &prefs)
           { prefs.putBytes(
                 STORGE_LAST_DEVICE,
                 &HOST_MAC,
                 sizeof(mac_t)); });
};

void Radio::config_clear()
{
  ESP_LOGI(TAG, "Clear confgi");
  nvs_call(STORGE_NAME_SPACE, [](Preferences &prefs)
           { prefs.clear(); });
}

extern Radio radio;