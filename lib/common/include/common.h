#ifndef COMMON_H
#define COMMON_H
#include "freertos/FreeRTOS.h"

/**
 * @brief 将频率（Hz）转换为滴答数（Ticks）。
 *
 * 该函数用于将给定的频率值（以赫兹为单位）转换为对应的滴答数。
 * 在 ESP32 ARDUINO 中，1 Tick 等于 1 毫秒。
 *
 * @param hz 频率值，单位为赫兹（Hz）。
 * @return 返回转换后的滴答数（Ticks）。
 */
#define HZ2TICKS(hz) ((TickType_t)(1000.00f / (hz)))

#endif
