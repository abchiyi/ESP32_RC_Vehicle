#include <esp_now.h>

#define RADIO_CHANNEL_MAX 8 // 最大控制通道数量

typedef esp_err_t (*send_cb_t)(uint8_t *);

typedef uint8_t mac_addr_t[ESP_NOW_ETH_ALEN];

// 通讯结构体
typedef struct
{
  mac_addr_t mac_addr;                 // 发送者地址
  uint16_t channel[RADIO_CHANNEL_MAX]; // 通道信息
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
  uint8_t timeout_resend = 50;      // 超时重发
  uint8_t resend_count = 5;         // 超时重发次数
  uint8_t timeout_disconnect = 250; // 超时断开连接
};

typedef struct channel_data
{
  uint16_t value;
  uint8_t mode;
} channel_data_t;

typedef struct channel_data_int
{
  int16_t value;
  uint8_t mode;
} channel_data_int_t;

/**
 * @brief 读取组合通道数据
 * @param radio_data 收到的数据
 * @param channle 要读取的通道 最大通道号参考0 ~ RADIO_CHANNEL_MAX - 1
 */
channel_data_t read_channel_data(radio_data_t radio_data, int channle);

/**
 * @brief 读取组合通道数据
 * @param radio_data 收到的数据
 * @param channle 要读取的通道 最大通道号参考0 ~ RADIO_CHANNEL_MAX - 1
 * @param intValue 指定读取 高12bit为有符号整数
 */
channel_data_int_t read_channel_data(radio_data_t radio_data, int channle, bool intValue);

/**
 * @brief 输出一个高低位合并的数值
 * @param value1 12bit特数值
 * @param value2 4 bit数值
 */
uint16_t set_combined_int(uint16_t value1, uint16_t value2);

extern Radio radio;