#include <WiFi.h>
#include <esp_wifi.h>
#include <Radio.h>
#include <esp_now.h>

esp_now_peer_info peerInfo;
esp_now_recv_cb_t *onRecvCb;

void ON_RECV_CB(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  memcpy(peerInfo.peer_addr, mac, 6);
  Serial.print("Controller mac :");
  Serial.println(macStr);
}

// XXX 固定的配对设置
#define SSID "Slave_2"
#define PASSWORD "Slave_1_Password"

// 配对回调
void PeerOnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  memcpy(peerInfo.peer_addr, mac, 6);

  Serial.print("This soft mac :");
  Serial.println(WiFi.softAPmacAddress());
  Serial.print("This mac :");
  Serial.println(WiFi.macAddress());
  Serial.print("Controll mac :");
  Serial.println(macStr);
  Serial.print("Data :");
  Serial.println(*incomingData);

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
      // 发送成功后注册接收函数
      esp_now_register_recv_cb(ON_RECV_CB);
      // esp_now_register_recv_cb(*onRecvCb);
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
void Radio::begin()
{
  WiFi.mode(WIFI_STA);
  esp_now_init();                                       // 初始化 esp_now
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE); // 设置通讯频道
  controller = &peerInfo;                               // 设置配对对象
}

/**
 * @brief 进入配对模式
 **/
void Radio::startPairing()
{
  // set AP
  WiFi.mode(WIFI_AP);

  if (WiFi.softAP(SSID, PASSWORD, channel, 0))
  {

    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP soft mac :");
    Serial.println(WiFi.softAPmacAddress());
    Serial.print("AP CHANNEL ");
    Serial.println(WiFi.channel());
    delay(100);
    // xTaskCreate(taskPeer, "taskPeer", 1024, NULL, 3, NULL); // 设置配对任务
    esp_now_register_recv_cb(PeerOnDataRecv); // 注册配对函数
  }
  else
  {
    Serial.println("AP Config failed.");
  }
}
