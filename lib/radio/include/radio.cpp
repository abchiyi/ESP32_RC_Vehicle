#include "radio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"

#include "esp_attr.h"

#define TAG "Radio"

static radio_link_operation_t noLink{
    .recv = [](radio_packet_t *rp)
    { return ESP_FAIL; },

    .send = [](radio_packet_t *rp)
    { return ESP_FAIL; },

    .is_connected = []()
    { return false; },

    .start = []()
    { return ESP_FAIL; },

    .rest = []()
    { return ESP_FAIL; },
};

static radio_link_operation_t *RLOP = &noLink;

static QueueHandle_t Q_SEND_PACK = xQueueCreate(10, sizeof(radio_packet_t));
CRTPPacketHandler_fn_t CRTPPacketHandler[16] = {nullptr}; // CRTP Packet Handler

IRAM_ATTR void task_radio_recv(void *pvParameters)
{
    static radio_packet_t rp;

    while (true)
    {
        if (RLOP->recv(&rp) == ESP_OK)
        {
            auto fn = CRTPPacketHandler[((CRTPPacket *)rp.data)->port];
            if (fn != nullptr)
                fn((CRTPPacket *)rp.data);
        }
    }
}

IRAM_ATTR void task_radio_send(void *pvParameters)
{
    static radio_packet_t rp;
    while (true)
    {
        if (xQueueReceive(Q_SEND_PACK, &rp, portMAX_DELAY) == pdTRUE)
            RLOP->send(&rp);
    }
}

void init_radio(radio_link_operation_t *link)
{
    esp_err_t ret;

    if (link == nullptr)
        esp_system_abort("link is null");

    RLOP = link;

    ret = xTaskCreate(task_radio_recv, "radio_recv", 4096, NULL, 10, NULL);
    ESP_ERROR_CHECK(ret);

    ret = xTaskCreate(task_radio_send, "radio_send", 4096, NULL, 10, NULL);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Radio init done :)");
}

bool radio_link_is_connected()
{
    return RLOP->is_connected();
}

esp_err_t radio_send_packet(radio_packet_t *rp)
{
    return RLOP->send(rp);
}

/**
 * @brief 设置指定端口的回调函数。
 *
 * 此函数将回调函数 `fn` 绑定到指定的 `CRTPPort` 端口。如果该端口已经有回调函数，则返回失败。
 *
 * @param port 要设置回调的端口。
 * @param fn 要绑定的回调函数。
 * @return esp_err_t 返回操作结果，成功时返回 ESP_OK，失败时返回 ESP_FAIL。
 */
esp_err_t radio_set_port_callback(CRTPPort port, CRTPPacketHandler_fn_t fn)
{
    if (!CRTPPacketHandler[port])
    {
        CRTPPacketHandler[port] = fn;
        return ESP_OK;
    }
    return ESP_FAIL;
}
