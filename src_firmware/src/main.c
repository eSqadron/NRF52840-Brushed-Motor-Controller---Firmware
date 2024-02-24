// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2023 Maciej Baczmanski, Michal Kawiak, Jakub Mazur
// Copyright (c) 2016 Intel Corporation

#include <zephyr/kernel.h>

#include <zephyr/usb/usb_device.h>

#if defined(CONFIG_BT_SUPPORT)
#include "ble_gatt_service.h"
#endif

#include <string.h>

#include "driver.h"

#if defined(CONFIG_UART_SHELL_SUPPORT)
#include <zephyr/drivers/uart.h>
#endif

int main(void)
{
	#if defined(CONFIG_BT_SUPPORT)
	init_bt();
	#endif

	init_pwm_motor_driver();
}
