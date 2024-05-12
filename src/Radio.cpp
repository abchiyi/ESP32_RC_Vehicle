#include <WiFi.h>
#include <esp_wifi.h>
#include <Radio.h>
#include <esp_now.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_mac.h>
#include <vector>

#define TAG "Radio"

#define PASSWORD "--------"
#define PAIR_TIME_OUT 60000 // 配对等待时间 /ms

Radio radio;
uint8_t CHANNEL; // 通讯频道
radio_data_t radio_data_recv;
radio_data_t radio_data_send;

SemaphoreHandle_t radio_data_mutex;  // 资源锁
SemaphoreHandle_t SEND_READY = NULL; // 允许发送

// 接收数据队列
QueueHandle_t Q_RECV_DATA = xQueueCreate(10, sizeof(radio_data_t));

// 向下数据队列，将处理后的数据发布到下级任务
QueueHandle_t Q_DATA_RECV = xQueueCreate(10, sizeof(radio_data_t));
QueueHandle_t Q_DATA_SEND = xQueueCreate(10, sizeof(radio_data_t));

/**
 * 计时器，当连接超时执行
 */
TimerHandle_t ConnectTimeoutTimer;
void IfTimeoutCB(TimerHandle_t xTimer) // 超时回调
{
  ESP_LOGI(TAG, "DISCONNECT with timeout");
  radio.status = RADIO_BEFORE_DISCONNECT;
  // ESP_LOGI(TAG, "ConnectTimeoutTimer STOP");
  // if (xTimerStop(ConnectTimeoutTimer, radio.timeOut) == pdFAIL)
  //   ESP_LOGI(TAG, "");
  // esp_restart();
}

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

/**
 * @brief 等待主机握手
 * @param waitTick 超时等待
 */
bool wait_response(TickType_t waitTick)
{
  if (xSemaphoreTake(SEND_READY, waitTick) != pdTRUE)
  {
    ESP_LOGI(TAG, "Wait for response timed out");
    return false;
  }
  return true;
};

// 对比 mac地址是否一致
bool checkMac(mac_addr_t mac1, mac_addr_t mac2)
{
  if (memcmp(mac1, mac2, sizeof(mac_addr_t)) == 0)
    return true;
  return false;
};

/**
 * @brief 根据 mac 地址配对到指定的设备
 * @param macaddr 数组 mac地址
 * @param channel wifi 频道
 * @param ifidx 要使用的wifi接口用于收发数据
 */
bool add_peer(
    mac_addr_t macaddr,
    uint8_t channel,
    wifi_interface_t ifidx = WIFI_IF_STA)
{
  // ESP_LOGI(TAG, "Pair to " MACSTR "", MAC2STR(macaddr));
  auto peer_info = &radio.peer_info;
  memset(peer_info, 0, sizeof(esp_now_peer_info_t)); // 清空对象
  memcpy(peer_info->peer_addr, macaddr, ESP_NOW_ETH_ALEN);
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
  auto status = esp_now_send(this->peer_info.peer_addr,
                             (uint8_t *)&data, sizeof(data));
  String error_message;

  if (status == ESP_OK)
    return true;

  switch (status)
  {
  case ESP_ERR_ESPNOW_NO_MEM:
    error_message = String("out of memory");
  case ESP_ERR_ESPNOW_NOT_FOUND:
    error_message = String("peer is not found");
  case ESP_ERR_ESPNOW_IF:
    error_message = String("current WiFi interface doesn't match that of peer");
  default:
    error_message = String("Send fail");
  }

  ESP_LOGI(TAG, "Send to " MACSTR " - %s",
           error_message.c_str(), MAC2STR(this->peer_info.peer_addr));
  return false;
}

// 接收回调
void onRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&radio_data_recv, incomingData, sizeof(radio_data_recv));
  memcpy(&radio_data_recv.mac_addr, mac, sizeof(radio_data_recv.mac_addr));
  if (xQueueSend(Q_RECV_DATA, &radio_data_recv, 10) != pdPASS)
    ;

  // ESP_LOGI(TAG, "Queue is full.");
  // else
  //   ESP_LOGI(TAG, "Queue is add.");
}

// 发送回调
void onSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status)
    ESP_LOGI(TAG, "Send to " MACSTR " FAIl", MAC2STR(mac_addr));
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
bool handshake(mac_addr_t mac_addr)
{
  // radio_data_t data;
  ESP_LOGI(TAG, "wait handshake");
  if (!wait_response(radio.timeout_resend, &radio_data_recv))
    return false;

  // TODO 掉电后无法设置正确 mac 地址导致重连失败
  if (checkMac(mac_addr, radio_data_recv.mac_addr))
  {
    memset(&radio_data_recv, 0, sizeof(radio_data_recv));
    radio.send(radio_data_recv);
    return true;
  }

  ESP_LOGI(TAG, "Host :" MACSTR " not paired \n", MAC2STR(radio_data_recv.mac_addr));
  return false;
}

esp_err_t pairNewDevice()
{
  mac_addr_t Host_MAC;

  // 切换 AP 状态
  auto AP_SWITCH = [&](bool ap_switch)
  {
    if (ap_switch)
    {
      ESP_LOGI(TAG, "AP ON");
      WiFi.enableAP(true);
    }
    else
    {
      ESP_LOGI(TAG, "AP OFF");
      WiFi.enableAP(false);
    }
  };

  ESP_LOGI(TAG, "Wait connection");
  if (!wait_response(PAIR_TIME_OUT, &radio_data_recv))
    return ESP_FAIL;

  // 关闭 AP 避免被其他主机扫描到
  AP_SWITCH(false);

  // 接收到主机配对请求后，添加对等并回复主机
  ESP_LOGI(TAG, "Connect Host, Mac :" MACSTR "", MAC2STR(radio_data_recv.mac_addr));
  memcpy(Host_MAC, radio_data_recv.mac_addr, sizeof(Host_MAC)); // 获取主机地址
  WiFi.macAddress((uint8_t *)&radio_data_recv.channel[0]);      // 写入 STA mac
  if (!add_peer(radio_data_recv.mac_addr, CHANNEL, WIFI_IF_STA))
    return ESP_FAIL;
  radio.send(radio_data_recv);

  /**
   * 配对期间有10次接受响应的机会，
   * 当全部不是来自目标主机的响应时判断配对失败
   * 等待目标主机第一次响应,
   */
  ESP_LOGI(TAG, "Wait Host response to STA");
  uint8_t counter = 0;
  // 接收数据响应次数&总超时时间 radio.timeout *  counter_max
  uint8_t counter_max = 10;
  while (true)
  {
    if (wait_response(radio.timeout_resend, &radio_data_recv))
      if (checkMac(Host_MAC, radio_data_recv.mac_addr))
        break;
    counter++;
    if (counter >= counter_max)
      ESP_LOGI(TAG, "Pair time out, no host to pair");
    return ESP_FAIL;
  }
  /**
   * 在配对过程中通道 0 有数据主机则认为从机引导主机配对至STA模式地址，则发送前
   * 需清空通道 0， 清除发送数据内的mac地址
   */
  memset(&radio_data_recv, 0, sizeof(radio_data_recv));
  radio.send(radio_data_recv);
  return ESP_OK;
};

void TaskRadioMainLoop(void *pt)
{
  uint8_t resend_counter = 0;

  while (true)
  {
    switch (radio.status)
    {
    case RADIO_BEFORE_WAIT_CONNECTION:
      if (!WiFi.softAP(radio.SSID, PASSWORD, CHANNEL, 0))
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
      wait_response(radio.timeout_resend, &radio_data_recv)
          ? resend_counter = 0 // 正常发送重置计数器
          : resend_counter++;  // 未收到回复时 计数器+1

      // 校验是否是来自目标主机的数据，非目标主机则不响应
      // if (checkMac(radio.peer_info.peer_addr, radio_data.mac_addr))
      //   break;

      xQueueReceive(Q_DATA_SEND, &radio_data_send, 10);
      radio.send(radio_data_send); // 回传数据

      if (!resend_counter) // 通知其他任务有数据传入
        if (xQueueSend(Q_DATA_RECV, &radio_data_recv, 1) != pdPASS)
          ;

      // 多次重发未收到响应则认为主机已断开连接
      if (resend_counter >= radio.resend_count)
      {
        ESP_LOGI(TAG, "DISCONNECT with timeout");
        radio.status = RADIO_BEFORE_DISCONNECT;
      }
      break;

    case RADIO_BEFORE_DISCONNECT:
      ESP_LOGI(TAG, "RADIO_DISCONNECT");
      xQueueReset(Q_RECV_DATA); // 断开连接清空队列
      radio.status = RADIO_DISCONNECT;
      break;
    case RADIO_DISCONNECT:
      // if (handshake(radio.peer_info.peer_addr))
      //   radio.status = RADIO_BEFORE_CONNECTED;
      vTaskDelay(5);
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
  radio_data_mutex = xSemaphoreCreateMutex(); // 创建资源锁
  SEND_READY = xSemaphoreCreateBinary();

  // 定义连接超时控制器
  ConnectTimeoutTimer = xTimerCreate(
      "Connect time out",       // 定时器任务名称
      radio.timeout_disconnect, // 延迟多少tick后执行回调函数
      pdFAIL,                   // 执行一次,pdTRUE 循 环执行
      0,                        // 任务id
      IfTimeoutCB               // 回调函数
  );

  this->initRadio();
  xTaskCreate(TaskRadioMainLoop, "TaskRadioMainLoop", 4096, NULL, 2, NULL);
}

/**
 * @brief 获取收到的数据
 */
esp_err_t Radio::get_data(radio_data_t *data)
{
  return xQueueReceive(Q_DATA_RECV, data, portMAX_DELAY) != pdPASS
             ? ESP_FAIL
             : ESP_OK;
}

/**
 * @brief 设置要发送的数据
 */
esp_err_t Radio::set_data(radio_data_t *data)
{
  return xQueueSend(Q_DATA_SEND, data, radio.timeout_resend) != pdTRUE
             ? ESP_FAIL
             : ESP_OK;
}