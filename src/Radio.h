#include <esp_now.h>

struct ControllerStatus
{
  bool btnA, btnB, btnX, btnY;
  bool btnShare, btnStart, btnSelect, btnXbox;
  bool btnLB, btnRB;
  bool btnLS, btnRS;
  bool btnDirUp, btnDirLeft, btnDirRight, btnDirDown;
  int16_t joyLHori;
  int16_t joyLVert;
  int16_t joyRHori;
  int16_t joyRVert;
  int16_t trigLT, trigRT;
};

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
  void begin(const char *ssid, uint8_t channel);

  // 读取接收到的xbox控制器状态
  ControllerStatus controller();
};
