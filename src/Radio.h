#include <esp_now.h>

typedef void (*radio_cb_t)(uint8_t *incomingData);

typedef enum radio_status
{
  RADIO_BEFORE_WAIT_CONNECTION,
  RADIO_WAIT_CONNECTION,

  RADIO_BEFORE_CONNECTED,
  RADIO_CONNECTED,

  RADIO_BEFORE_DISCONNECT,
  RADIO_DISCONNECT,
} radio_status_t;

struct sendData
{
  float volts;
  int gear;
  int ang;
};

// 常规数据处理结构
struct Data
{
  int len;
  bool newData;
  uint8_t *mac;
  uint8_t *incomingData;
  Data() : newData(false){};

  uint8_t *get()
  {
    newData = false;
    return incomingData;
  }
};

// 握手数据结构
struct HANDSHAKE_DATA
{
  uint8_t mac[ESP_NOW_ETH_ALEN];
  uint32_t code = rand();
};

/**
 * @brief 被控设备无线通讯
 */
class Radio
{
private:
public:
  static esp_now_peer_info peerInfo; // 无线控制器的配对信息
  static int channel;                // 通讯频道 0 ~ 14
  static radio_cb_t RECVCB;          // 接收数据处理回调
  static const char *SSID;           // 设备名称
  static sendData SendData;          // 待发送数据

  radio_status_t status;
  void begin(const char *ssid, uint8_t channel, radio_cb_t recvCB);
  void initRadio();     // 初始化无线
  uint8_t timeOut = 50; // 通讯超时
  Data RecvData;        // 接收到的数据
};

extern Radio radio;