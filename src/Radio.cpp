#include <WiFi.h>
#include <esp_wifi.h>
#include <Radio.h>
#include <esp_now.h>
#include <esp_log.h>

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
bool Radio::connected;
int Radio::channel;

// 连接超时控制器
TimerHandle_t ConnectTimeoutTimer;
const int ConnectTimeoutTimerID = 0;
uint8_t CHANNEL; // 通讯频道

void IfTimeoutCB(TimerHandle_t xTimer);
void EspNowInit();

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
  if (Radio::connected)
  {
    // 执行接收回调
    Radio::RECVCB(incomingData);

    // 利用主机发送间隔向主机返回数据
    esp_err_t a = esp_now_send(Radio::peerInfo.peer_addr, (uint8_t *)&Radio::SendData, sizeof(Radio::SendData));
  }
  else // if not pair
  {
    ESP_LOGI(TAG, "to pair");

    memset(&Radio::peerInfo, 0, sizeof(Radio::peerInfo)); // 清空对象
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    memcpy(Radio::peerInfo.peer_addr, mac, 6);

    if (esp_now_add_peer(&Radio::peerInfo) == ESP_OK) // 配对
    {
      ESP_LOGI(TAG, "pairing...");
      // 记录主机mac地址后向主机发送配对信息
      WiFi.mode(WIFI_STA);
      esp_err_t result = esp_now_send(mac,
                                      (const uint8_t *)WiFi.softAPSSID().c_str(), sizeof((const uint8_t *)WiFi.softAPSSID().c_str()));

      if (result == ESP_OK)
      {
        ESP_LOGI(TAG, "Pair Success");
        Radio::connected = true;
      }
      else
      {
        ESP_LOGI(TAG, "Send fail");
        ESP_LOGE(TAG, "Pair fail, rest radio");
        EspNowInit();
      }
    }
    else
    {
      ESP_LOGE(TAG, "Pair fail, rest radio");
      EspNowInit();
    }
  }

  // 启动定时器，在计时器结束前接收到返回信号则立即重置定时器
  if (xTimerStart(ConnectTimeoutTimer, 100) != pdPASS)
  {
    ESP_LOGE(TAG, "start/reset timer fial");
  }
  // else
  // {
  //   ESP_LOGI(TAG, "start/reset timer success");
  // }
}

// 初始化 espNow
void EspNowInit()
{

  Radio::connected = false; // re set flage

  // 定义连接超时控制器
  ConnectTimeoutTimer = xTimerCreate(
      "Connect time out",             // 定时器任务名称
      500,                            // 延迟多少tick后执行回调函数
      pdFALSE,                        // 执行一次,pdTRUE 循环执行
      (void *)&ConnectTimeoutTimerID, // 任务id
      IfTimeoutCB                     // 回调函数
  );

  // wifi set
  delay(100);
  WiFi.mode(WIFI_AP);
  // esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (WiFi.softAP(Radio::SSID, PASSWORD, CHANNEL, 0))
  {
    ESP_LOGI(TAG, "AP Config Success. Broadcasting with AP: %s", String(Radio::SSID).c_str());
    ESP_LOGI(TAG, "AP soft mac : %s, Channel : %u", WiFi.softAPmacAddress().c_str(), WiFi.channel());
  }
  else
  {
    ESP_LOGE(TAG, "AP Config failed.");
  }

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
    counter <= 5 ? EspNowInit() : ESP.restart();
  }
}

// 连接超时控制器回调
void IfTimeoutCB(TimerHandle_t xTimer)
{
  ESP_LOGW(TAG, "Connect time out, reset to pair");
  EspNowInit();
}

void TaskRadioMainLoop(void *pt)
{
  while (true)
  {
    vTaskDelay(1);
  }
}

// 启动 esp_now 通讯
void Radio::begin(const char *ssid, uint8_t channel, radio_cb_t recvCB)
{
  connected = false;
  channel = channel;
  RECVCB = recvCB;
  SSID = ssid;

  // 定义连接超时控制器
  ConnectTimeoutTimer = xTimerCreate(
      "Connect time out",             // 定时器任务名称
      500,                            // 延迟多少tick后执行回调函数
      pdFALSE,                        // 执行一次,pdTRUE 循环执行
      (void *)&ConnectTimeoutTimerID, // 任务id
      IfTimeoutCB                     // 回调函数
  );

  EspNowInit();

  xTaskCreate(TaskRadioMainLoop, "TaskRadioMainLoop", 4096, NULL, 2, NULL);
}
