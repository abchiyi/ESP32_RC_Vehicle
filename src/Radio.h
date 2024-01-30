#include <esp_now.h>

typedef void (*recv_cb_t)(const uint8_t *incomingData);

/**
 * @brief 被控设备无线通讯
 */
class Radio
{
private:
  /* data */
public:
  esp_now_peer_info *master; // 无线控制器的配对信息
  uint8_t *Channel;          // 通讯频道
  void begin(const char *ssid, uint8_t channel, recv_cb_t recvCB);
};
