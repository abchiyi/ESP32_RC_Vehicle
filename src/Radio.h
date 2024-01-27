#include <esp_now.h>

/**
 * @brief 被控设备无线通讯
 */
class Radio
{
private:
  /* data */
public:
  esp_now_peer_info *controller; // 无线控制器的配对信息
  int Channel = 1;               // 通讯频道
  void begin(const char *ssid, uint8_t channel);
};
