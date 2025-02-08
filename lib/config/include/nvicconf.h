/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * 本程序是免费软件：您可以在GNU通用公共许可证版本3下重新分发和/或修改它。
 *
 * 本程序按“原样”分发，不提供任何形式的保证；不提供适销性或特定用途适用性的默示保证。有关更多详细信息，请参阅GNU通用公共许可证。
 *
 * nvicconf.c - 中断优先级配置
 *
 * STM32有16个优先级可供选择，其中0是最高优先级。现在它们配置为不使用组。
 *
 * 调用FreeRTOS FromISR函数的中断函数
 * 必须具有10及以上的中断编号，该编号目前由configMAX_SYSCALL_INTERRUPT_PRIORITY设置。 */
#ifndef NVIC_CONF_H_
#define NVIC_CONF_H_

/*
Crazyflie中的中断优先级组织：

在Cortex-M中，低优先级数字是高优先级。因此，优先级0是最高优先级中断，优先级15是最低优先级（STM32实现了4个优先级位）

低于MAX_SYSCALL_INTERRUPT_PRIORITY的中断不能调用任何RTOS函数！它们应该像某种软设备一样处理，运行在操作系统之上。

定义了4个中断级别
 - NVIC_LOW_PRI
 - NVIC_MID_PRI
 - NVIC_HIGH_PRI
 - NVIC_VERY_HIGH_PRI
目标是简化中断处理并记录为什么需要任何特殊情况。

15 -
14 -
13 - NVIC_LOW_PRI
12 - NVIC_ADC_PRI
11 - NVIC_RADIO_PRI
10 - NVIC_MID_PRI
 9 -
 8 -
 7 - NVIC_HIGH_PRI
 6 - NVIC_VERY_HIGH_PRI
 5 -                                     <-- MAX_SYSCALL_INTERRUPT_PRIORITY
 4 ! NVIC_I2C_PRI_LOW NVIC_TRACE_TIM_PRI --- 不调用任何RTOS函数
 3 ! NVIC_I2C_PRI_HIGH
 2 !
 1 !
 0 !
*/

// Standard interrupt levels
#define NVIC_LOW_PRI 13
#define NVIC_MID_PRI 10
#define NVIC_HIGH_PRI 7
#define NVIC_VERY_HIGH_PRI 7

// Priorities used for Crazyflie
#define NVIC_I2C_HIGH_PRI 3
#define NVIC_I2C_LOW_PRI 4
#define NVIC_TRACE_TIM_PRI 4
#define NVIC_UART_PRI 6

// Priorities for Crazyflie 2.0
#define NVIC_RADIO_PRI 11
#define NVIC_ADC_PRI 12
#define NVIC_CPPM_PRI 14
#define NVIC_SYSLINK_PRI 5

// Priorities for external interrupts
#define EXTI0_PRI NVIC_LOW_PRI
#define EXTI1_PRI NVIC_LOW_PRI
#define EXTI2_PRI NVIC_LOW_PRI
#define EXTI3_PRI NVIC_LOW_PRI
#define EXTI4_PRI NVIC_SYSLINK_PRI // this serves the syslink UART
#define EXTI9_5_PRI NVIC_LOW_PRI
#define EXTI15_10_PRI NVIC_MID_PRI // this serves the decks and sensors

#endif /* NVIC_CONF_H_ */
