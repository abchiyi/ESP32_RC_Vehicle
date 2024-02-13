#include <esp_now.h>

typedef void (*radio_cb_t)(const uint8_t *incomingData);

struct sendData
{
  float volts;
  int gear;
  int ang;
};

/**
 * @brief 被控设备无线通讯
 */
class Radio
{
private:
public:
  static esp_now_peer_info peerInfo; // 无线控制器的配对信息
  static bool connected;             // 配对状态
  static int channel;                // 通讯频道 0 ~ 14
  static radio_cb_t RECVCB;          // 接收数据处理回调
  static const char *SSID;           // 设备名称
  static sendData SendData;          // 待发送数据

  void begin(const char *ssid, uint8_t channel, radio_cb_t recvCB);
};
