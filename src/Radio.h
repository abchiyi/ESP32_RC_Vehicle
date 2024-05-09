#include <esp_now.h>

#define MAX_CHANNEL 8 // 最大控制通道数量

typedef esp_err_t (*send_cb_t)(uint8_t *);
typedef uint8_t mac_addr_t[ESP_NOW_ETH_ALEN];

// 通讯结构体
typedef struct
{
  mac_addr_t mac_addr;           // 发送者地址
  uint16_t channel[MAX_CHANNEL]; // 通道信息
} radio_data_t;

typedef enum radio_status
{
  RADIO_BEFORE_WAIT_CONNECTION,
  RADIO_WAIT_CONNECTION,

  RADIO_BEFORE_CONNECTED,
  RADIO_CONNECTED,

  RADIO_BEFORE_DISCONNECT,
  RADIO_DISCONNECT,
} radio_status_t;

/**
 * @brief 无线通讯
 */
class Radio
{
private:
public:
  radio_data_t dataToSent;     // 待发送数据
  esp_now_peer_info peer_info; // 配对信息
  const char *SSID;            // 设备名称
  radio_status_t status;       // 无线状态
  int channel;                 // 通讯频道 0 ~ 14

  template <typename T>
  bool send(const T &data);

  void begin(const char *ssid, uint8_t channel);
  void initRadio(); // 初始化无线

  uint8_t timeOut = 100; // 通讯超时
  uint8_t sendGap = 5;   // 发送间隔
};

extern Radio radio;