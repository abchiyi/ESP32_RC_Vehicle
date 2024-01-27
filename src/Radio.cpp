#include <WiFi.h>
#include <esp_wifi.h>
#include <Radio.h>
#include <esp_now.h>
#include <esp_log.h>

#define TAG "Radio"
// XXX 固定的配对设置
#define SSID "Slave_2"
#define PASSWORD "Slave_1_Password"

esp_now_peer_info peerInfo;
// esp_now_recv_cb_t *onRecvCb;

int CONNECT_TIMEOUT = 500;              // ms // 连接同步等待时间
int ConnectedTimeOut = CONNECT_TIMEOUT; // ms
const int MinSendGapMs = 8;
bool PairRuning = false;
bool IsPaired = false;
int Send_gap_ms = 0;
void *RecvData;

// 返回mac地址字符串
String parseMac(const uint8_t *mac)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

void onRecvCb(const uint8_t *mac, const uint8_t *incomingData, int len)
{

  esp_err_t a = esp_now_send(peerInfo.peer_addr, incomingData, len);
  ESP_LOGI("Radio", "Controller mac : %s, Send data %s",
           parseMac(mac), a == ESP_OK ? "success" : " fail");
  // if not pair

  if (esp_now_add_peer(&peerInfo) == ESP_OK) // 配对
  {
    // 记录主机mac地址后向主机发送配对信息
    WiFi.mode(WIFI_STA);
    esp_err_t result = esp_now_send(mac,
                                    (const uint8_t *)WiFi.softAPSSID().c_str(), sizeof((const uint8_t *)WiFi.softAPSSID().c_str()));
    Serial.print("Send Status: ");
    if (result == ESP_OK)
    {
      Serial.println("send ok");
      esp_now_unregister_recv_cb() == ESP_OK ? Serial.println("unreg peer ok") : Serial.println("unreg peer fail");
    }
    else
    {
      Serial.println("Send fail");
    }
  }
  else
  {
    Serial.println("Peer fail");
  }
}

/**
 * @brief 启动 esp_now 通讯
 */
void Radio::begin(const char *ssid, uint8_t channel)
{
  controller = &peerInfo; // 设置配对对象

  WiFi.mode(WIFI_AP);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE); // 设置通讯频道

  esp_now_init(); // 初始化 esp_now
  if (WiFi.softAP(SSID, PASSWORD, channel, 0))
  {
    ESP_LOGI(TAG, "AP Config Success. Broadcasting with AP %s", String(SSID).c_str());
    ESP_LOGI(TAG, "AP soft mac : %s, Channel : %u", WiFi.softAPmacAddress(), WiFi.channel());
    esp_now_register_recv_cb(onRecvCb);
  }
  else
  {
    Serial.println("AP Config failed.");
  }
}
