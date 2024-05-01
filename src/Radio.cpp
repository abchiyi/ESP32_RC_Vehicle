#include <WiFi.h>
#include <esp_wifi.h>
#include <Radio.h>
#include <esp_now.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_mac.h>

#define TAG "Radio"

#define PASSWORD "--------"

int CONNECT_TIMEOUT = 500;              // ms // 连接同步等待时间
int ConnectedTimeOut = CONNECT_TIMEOUT; // ms
const int MinSendGapMs = 8;             // 最小发送间隔
int Send_gap_ms = MinSendGapMs;         // 发送间隔

esp_now_peer_info Radio::peerInfo;
sendData Radio::SendData;
radio_cb_t Radio::RECVCB;
const char *Radio::SSID;
int Radio::channel;

// 接收到的数据
struct Data
{
  int len;
  bool newData = false;
  uint8_t *mac;
  uint8_t *incomingData;
  uint8_t *get()
  {
    newData = false;
    return incomingData;
  }
} RecvData;

Radio radio;

// 连接超时控制器
TimerHandle_t ConnectTimeoutTimer;
const int ConnectTimeoutTimerID = 0;
uint8_t CHANNEL; // 通讯频道

void IfTimeoutCB(TimerHandle_t xTimer);

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
  ;
  switch (esp_now_send(peer_addr, data, len))
  {
  case ESP_OK:
    return true;

  default:
    return false;
  }
};

// 接收回调，在这里执行配对程序&接收数据/发送数据
void onRecvCb(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  RecvData.len = len;
  RecvData.newData = true;
  RecvData.mac = (uint8_t *)mac;
  RecvData.incomingData = (uint8_t *)incomingData;
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
  HANDSHAKE_DATA hsd; // 握手数据包
  uint8_t timeoutConter = 50;
  if (RecvData.newData)
  {
    memcpy((void *)&hsd, RecvData.get(), sizeof(HANDSHAKE_DATA));
    ESP_LOGI(TAG, "Connect Host, Mac :" MACSTR "", MAC2STR(RecvData.mac));

    if (pairTo(RecvData.mac, CHANNEL, WIFI_IF_AP))
      radio.status = RADIO_BEFORE_DISCONNECT;

    // 回复配对信息
    WiFi.macAddress(hsd.mac);
    if (!sendTo(hsd.mac, (const uint8_t *)&hsd, sizeof(HANDSHAKE_DATA)))
      radio.status = RADIO_BEFORE_DISCONNECT;

    ESP_LOGI(TAG, "Close Ap");
    if (!WiFi.enableAP(false))
      radio.status = RADIO_BEFORE_DISCONNECT;

    while (timeoutConter) // 等待第二次握手
    {
      if (timeoutConter <= 1)
        ESP_LOGI(TAG, "Timeout Connect fail");
      if (RecvData.newData)
      {
        // 收到数据校验是否是目标主机的握手请求
        memcpy((void *)&hsd, RecvData.get(), sizeof(HANDSHAKE_DATA));
        if (hsd.mac == radio.peerInfo.peer_addr)
          if (!sendTo(hsd.mac, (const uint8_t *)&hsd, sizeof(HANDSHAKE_DATA))) // 回复配对信息
            radio.status = RADIO_BEFORE_DISCONNECT;
      }

      timeoutConter--;
      vTaskDelay(1);
    }
  }
};

void TaskRadioMainLoop(void *pt)
{

  while (true)
  {
    switch (radio.status)
    {
    case RADIO_BEFORE_WAIT_CONNECTION:
      if (WiFi.softAP(Radio::SSID, PASSWORD, CHANNEL, 0))
      {
        ESP_LOGI(TAG, "AP Config Success.SSID: %s , MAC : %s, CHANNEL : %d", Radio::SSID, WiFi.softAPmacAddress().c_str(), WiFi.channel());
        radio.status = RADIO_WAIT_CONNECTION; // AP 开启成功则进入等待连接状态
        break;
      }
      WiFi.setTxPower(WIFI_POWER_19_5dBm);
      ESP_LOGE(TAG, "AP Config failed.");
      break;

    case RADIO_WAIT_CONNECTION:
      if (pairNewDevice() == ESP_OK)
        radio.status = RADIO_BEFORE_CONNECTED;
      else
      {
        radio.status = RADIO_BEFORE_DISCONNECT;
        ESP_LOGI(TAG, "Connect Success");
      }

      vTaskDelay(5);
      break;

    case RADIO_BEFORE_CONNECTED:

      radio.status = RADIO_CONNECTED;
      // 启动定时器，数据传输超时触发
      if (xTimerStart(ConnectTimeoutTimer, 100) != pdPASS)
        esp_system_abort("start timer fial"); // 启动定时器失败
      ESP_LOGI(TAG, "COMP");
      break;

    case RADIO_CONNECTED:
      // 执行接收回调
      if (RecvData.newData)
      {
        // ESP_LOGI(TAG, "Connected");
        RecvData.newData = false;
        radio.RECVCB(RecvData.get());
        // 利用主机发送间隔向主机返回数据
        esp_now_send(Radio::peerInfo.peer_addr, (uint8_t *)&Radio::SendData, sizeof(Radio::SendData));

        // 重置定时器，数据传输超时触发
        if (xTimerStart(ConnectTimeoutTimer, 100) != pdPASS)
          esp_system_abort("start timer fial"); // 重置定时器失败
      }
      vTaskDelay(Send_gap_ms);
      break;

    case RADIO_BEFORE_DISCONNECT:
      // 停止定时器
      ESP_LOGI(TAG, "Host lost ... rest to wait connection");
      radio.status = RADIO_BEFORE_WAIT_CONNECTION;
      // if (xTimerStop(ConnectTimeoutTimer, 100) != pdPASS)
      //   esp_system_abort("stop timer fial"); // 停止定时器失败
      break;
    case RADIO_DISCONNECT:
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
