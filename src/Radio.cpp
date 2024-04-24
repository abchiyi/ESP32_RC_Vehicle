#include <WiFi.h>
#include <esp_wifi.h>
#include <Radio.h>
#include <esp_now.h>
#include <esp_log.h>
#include <esp_err.h>

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

struct Data
{
  int len;
  bool newData;
  uint8_t *mac;
  uint8_t *incomingData;
  Data() : newData(false){};
} RecvDATA;

Radio radio;

// 连接超时控制器
TimerHandle_t ConnectTimeoutTimer;
const int ConnectTimeoutTimerID = 0;
uint8_t CHANNEL; // 通讯频道

void IfTimeoutCB(TimerHandle_t xTimer);
// void EspNowInit();

// 返回mac地址字符串
String parseMac(const uint8_t *mac)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

// 接收回调，在这里执行配对程序&接收数据/发送数据
void onRecvCb(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  // TODO 使用互斥锁保证数据同步
  RecvDATA.len = len;
  RecvDATA.newData = true;
  RecvDATA.mac = (uint8_t *)mac;
  RecvDATA.incomingData = (uint8_t *)incomingData;
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
    // counter <= 5 ? EspNowInit() : ESP.restart();
  }
}

// 连接超时控制器回调
void IfTimeoutCB(TimerHandle_t xTimer)
{
  ESP_LOGW(TAG, "Connect time out, reset to pair");
  radio.status = RADIO_BEFORE_DISCONNECT;
  if (xTimerStop(ConnectTimeoutTimer, 100) != pdPASS)
    esp_system_abort("stop timer fial"); // 停止定时器失败
}

void TaskRadioMainLoop(void *pt)
{
  char macStr[18];

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
      // ESP_LOGI(TAG, "wifi mode is %d", WiFi.getMode());
      if (!RecvDATA.newData)
        ESP_LOGI(TAG, "Wait connection...");
      else
      {
        RecvDATA.newData = false;
        ESP_LOGI(TAG, "Connect To Host...");
        memset(&Radio::peerInfo, 0, sizeof(Radio::peerInfo)); // 清空对象
        memcpy(Radio::peerInfo.peer_addr, RecvDATA.mac, ESP_NOW_ETH_ALEN);
        snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
                 RecvDATA.mac[0], RecvDATA.mac[1], RecvDATA.mac[2], RecvDATA.mac[3], RecvDATA.mac[4], RecvDATA.mac[5]);
        ESP_LOGI(TAG, "Host Mac is :%s", macStr);
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_now_add_peer(&radio.peerInfo));
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_now_send(
            RecvDATA.mac,
            (const uint8_t *)WiFi.softAPSSID().c_str(),
            sizeof((const uint8_t *)WiFi.softAPSSID().c_str())));

        ESP_LOGI(TAG, "Connect Success");
        radio.status = RADIO_BEFORE_CONNECTED;
        // 启动定时器，数据传输超时触发
        if (xTimerStart(ConnectTimeoutTimer, 100) != pdPASS)
          esp_system_abort("start timer fial"); // 启动定时器失败
        ESP_LOGI(TAG, "COMP");
      }

      vTaskDelay(5);
      break;

    case RADIO_BEFORE_CONNECTED:
      ESP_LOGI(TAG, "Close Ap");
      WiFi.enableAP(false);
      radio.status = RADIO_CONNECTED;
      break;

    case RADIO_CONNECTED:
      // 执行接收回调
      if (RecvDATA.newData)
      {
        RecvDATA.newData = false;
        radio.RECVCB(RecvDATA.incomingData);
        vTaskDelay(Send_gap_ms);
        // 利用主机发送间隔向主机返回数据
        esp_now_send(Radio::peerInfo.peer_addr, (uint8_t *)&Radio::SendData, sizeof(Radio::SendData));

        // 重置定时器，数据传输超时触发
        if (xTimerReset(ConnectTimeoutTimer, 100) != pdPASS)
          esp_system_abort("start timer fial"); // 重置定时器失败
      }
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
