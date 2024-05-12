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
  radio_data_t data;           // 待发送数据
  esp_now_peer_info peer_info; // 配对信息
  const char *SSID;            // 设备名称
  radio_status_t status;       // 无线状态
  int channel;                 // 通讯频道 0 ~ 14

  template <typename T>
  bool send(const T &data);                      // 发送数据
  void initRadio();                              // 初始化无线
  void begin(const char *ssid, uint8_t channel); // 启动无线

  esp_err_t get_data(radio_data_t *data); // 读取收到的数据
  esp_err_t set_data(radio_data_t *data); // 设置要发送的数据

  /**
   * freeRTOS 在esp32 一个 tick 为 1ms
   * 以下定义的超时值单位为 1ms
   */
  uint8_t timeout_disconnect = 120; // 超时断开连接
  uint8_t timeout_resend = 80;      // 超时重发
  uint8_t resend_count = 2;         // 超时重发次数
};

extern Radio radio;