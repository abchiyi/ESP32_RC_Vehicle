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

int CONNECT_TIMEOUT = 500;              // ms // 连接同步等待时间
int ConnectedTimeOut = CONNECT_TIMEOUT; // ms
const int MinSendGapMs = 8;             // 最小发送间隔
int Send_gap_ms = MinSendGapMs;         // 发送间隔

bool SEND_READY = false; // 允许发送数据

esp_now_peer_info Radio::peerInfo;
sendData Radio::SendData;
radio_cb_t Radio::RECVCB;
const char *Radio::SSID;
int Radio::channel;

Radio radio;

// 连接超时控制器
TimerHandle_t ConnectTimeoutTimer;
const int ConnectTimeoutTimerID = 0;
uint8_t CHANNEL; // 通讯频道
uint8_t targetMAC[ESP_NOW_ETH_ALEN];

void IfTimeoutCB(TimerHandle_t xTimer);

template <typename T>
bool compareVectorContents(std::vector<T> &vec1, std::vector<T> &vec2)
{
  if (vec1 == vec2)
    return true;
  else
    return false;
}

/**
 * @brief 根据 mac 地址配对到指定的设备
 * @param macaddr 数组 mac地址
 * @param channel wifi 频道
 * @param ifidx 要使用的wifi接口用于收发数据
 */
bool pairTo(
    uint8_t macaddr[ESP_NOW_ETH_ALEN],
    uint8_t channel,
    wifi_interface_t ifidx)
{

  ESP_LOGI(TAG, "Pair to " MACSTR "", MAC2STR(macaddr));
  esp_now_peer_info_t peer_info;
  memcpy(peer_info.peer_addr, macaddr, ESP_NOW_ETH_ALEN);
  peer_info.channel = channel;
  peer_info.ifidx = ifidx;
  peer_info.encrypt = false;

  switch (esp_now_add_peer(&peer_info))
  {
  case ESP_OK:
    ESP_LOGI(TAG, "Pair success");
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

bool sendTo(const uint8_t *peer_addr, const uint8_t *data, size_t len)
{
  auto status = esp_now_send(peer_addr, data, len);
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
           error_message.c_str(), MAC2STR(peer_addr));
  return false;
};

// 接收回调，在这里执行配对程序&接收数据/发送数据
void onRecvCb(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  radio.RecvData.len = len;
  radio.RecvData.newData = true;
  radio.RecvData.mac = (uint8_t *)mac;
  radio.RecvData.incomingData = (uint8_t *)incomingData;
  SEND_READY = true; // 收到数据后允许发送
}

// 发送回调
void onSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  SEND_READY = false;
  if (status)
    ESP_LOGI(TAG, "Send to " MACSTR " FAIl", MAC2STR(mac_addr));
}

// 初始化 espNow
void Radio::initRadio()
{
  // wifi set
  WiFi.mode(WIFI_STA);
  WiFi.enableLongRange(true);

  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_LORA_500K);
  radio.status = RADIO_BEFORE_WAIT_CONNECTION;
  // esp_now_set
  static int counter = 0;
  esp_now_deinit(); // 重置 esp_now
  if (esp_now_init() == ESP_OK)
  {
    ESP_LOGI(TAG, "ESP NOW init success");
    esp_err_t refStatus = esp_now_register_recv_cb(onRecvCb);
    refStatus == ESP_OK
        ? ESP_LOGI(TAG, "Register recv cb success")
        : ESP_LOGE(TAG, "Register recv cb fail");
    counter = 0;
  }
  else
  {
    counter++;
    counter <= 5
        ? ESP_LOGE(TAG, "ESP NOW init fail, Try again...")
        : ESP_LOGE(TAG, "ESP NOW init fail, Maximum depth, restart...");
    delay(10);
  }

  // 注册发送回调
  esp_now_register_send_cb(onSend) == ESP_OK
      ? ESP_LOGI(TAG, "Register recv cb success")
      : ESP_LOGE(TAG, "Register recv cb fail");
}

// 连接超时控制器回调
void IfTimeoutCB(TimerHandle_t xTimer)
{
  ESP_LOGW(TAG, "Connect time out, reset to pair");
  radio.status = RADIO_BEFORE_DISCONNECT;
  if (xTimerStop(ConnectTimeoutTimer, 100) != pdPASS)
    esp_system_abort("stop timer fial"); // 停止定时器失败
  else
    ESP_LOGI(TAG, "Timer stop");
}

esp_err_t pairNewDevice()
{
  uint16_t pairTimeOut = 60000;           // 配对等待时间 /ms
  uint16_t counter_timeout = pairTimeOut; // 超时计数器
  int timeoutConter = 1000;               // 握手等待时间
  HANDSHAKE_DATA hsd;                     // 握手数据包

  while (pairTimeOut)
  {
    vTaskDelay(1);
    pairTimeOut--;

    if (radio.RecvData.newData)
    {
      memcpy(&hsd, radio.RecvData.get(), sizeof(HANDSHAKE_DATA));
      ESP_LOGI(TAG, "Connect Host, Mac :" MACSTR "", MAC2STR(radio.RecvData.mac));

      // 接收到主机配对请求后，配对并回复握手信息
      // 其中包含 STA 模式下的 mac 地址
      memcpy(targetMAC, radio.RecvData.mac, sizeof(targetMAC));
      if (!pairTo(targetMAC, CHANNEL, WIFI_IF_STA))
        return ESP_FAIL;

      WiFi.macAddress(hsd.mac);
      sendTo(targetMAC, (const uint8_t *)&hsd, sizeof(HANDSHAKE_DATA));

      // 关闭AP 切换到 STA 模式以避免被其他主机扫描到
      ESP_LOGI(TAG, "Close Ap");
      if (!WiFi.enableAP(false))
        return ESP_FAIL;

      while (timeoutConter) // 等待第二次握手
      {
        timeoutConter--;
        vTaskDelay(1);

        if (timeoutConter < 1)
        {
          ESP_LOGI(TAG, "Wait response time out");
          return ESP_FAIL;
        }

        if (!radio.RecvData.newData)
          continue;
        memcpy((void *)&hsd, radio.RecvData.get(), sizeof(HANDSHAKE_DATA));
        // 收到数据校验是否是目标主机的握手请求
        std::vector<uint8_t> va(std::begin(hsd.mac), std::end(hsd.mac));
        std::vector<uint8_t> vb(std::begin(targetMAC), std::end(targetMAC));
        if (!compareVectorContents(va, vb))
          continue;

        if (sendTo(targetMAC, (const uint8_t *)&hsd, sizeof(HANDSHAKE_DATA)))
          return ESP_OK;
        else
          return ESP_FAIL;
      }
    }
  }

  // 关闭AP 切换到 STA 模式以避免被其他主机扫描到
  ESP_LOGI(TAG, "Close Ap");
  ESP_LOGI(TAG, "Pair time out, no host to pair");
  WiFi.enableAP(false);
  return ESP_FAIL;
};

void TaskRadioMainLoop(void *pt)
{
  const uint8_t send_error_max = 3; // 最大发送错误
  uint8_t send_error_counter = 0;   // 发送错误计数器
  uint8_t timeout_counter = 0;      // 连接超时计数器
  int data = 10;

  while (true)
  {
    switch (radio.status)
    {
    case RADIO_BEFORE_WAIT_CONNECTION:
      if (!WiFi.softAP(Radio::SSID, PASSWORD, CHANNEL, 0))
      {
        ESP_LOGE(TAG, "AP Config failed.");
        break;
      }

      ESP_LOGI(TAG, "AP Config Success.SSID: %s , MAC : %s, CHANNEL : %d", Radio::SSID, WiFi.softAPmacAddress().c_str(), WiFi.channel());
      WiFi.setTxPower(WIFI_POWER_19_5dBm);
      radio.status = RADIO_WAIT_CONNECTION; // AP 开启成功则进入等待连接状态
      break;

    case RADIO_WAIT_CONNECTION:
      if (pairNewDevice() == ESP_OK)
        radio.status = RADIO_BEFORE_CONNECTED;
      else
        radio.status = RADIO_BEFORE_DISCONNECT;
      break;

    case RADIO_BEFORE_CONNECTED:
      ESP_LOGI(TAG, "Connect Success");
      radio.status = RADIO_CONNECTED;
      SEND_READY = true;
      break;

    case RADIO_CONNECTED:
      SEND_READY ? timeout_counter = 0 : timeout_counter++;
      if (timeout_counter >= radio.timeOut)
      {
        ESP_LOGI(TAG, "DISCONNECT with timeout");
        radio.status = RADIO_BEFORE_DISCONNECT;
      }

      // 连续发送失败次数达到设置的目标时视为连接断开
      sendTo(targetMAC, (uint8_t *)&radio.SendData, sizeof(radio.SendData));
      // ESP_LOGI(TAG, "send");
      vTaskDelay(10);
      break;

    case RADIO_BEFORE_DISCONNECT:
      ESP_LOGI(TAG, "Host lost / Pair Fail");
      radio.status = RADIO_DISCONNECT;
      // if (xTimerStop(ConnectTimeoutTimer, 100) != pdPASS)
      //   esp_system_abort("stop timer fial"); // 停止定时器失败
      break;
    case RADIO_DISCONNECT:
      vTaskDelay(5);
      // ESP_LOGI(TAG, "RADIO_DISCONNECT");
      break;

    default:
      vTaskDelay(5);
      break;
    }
  }
}
// 启动 esp_now 通讯
void Radio::begin(const char *ssid, uint8_t channel, radio_cb_t recvCB)
{
  channel = channel;
  RECVCB = recvCB;
  SSID = ssid;

  // 定义连接超时控制器
  ConnectTimeoutTimer = xTimerCreate(
      "Connect time out",             // 定时器任务名称
      500,                            // 延迟多少tick后执行回调函数
      pdTRUE,                         // 执行一次,pdTRUE 循环执行
      (void *)&ConnectTimeoutTimerID, // 任务id
      IfTimeoutCB                     // 回调函数
  );

  this->initRadio();
  xTaskCreate(TaskRadioMainLoop, "TaskRadioMainLoop", 4096, NULL, 2, NULL);
}
