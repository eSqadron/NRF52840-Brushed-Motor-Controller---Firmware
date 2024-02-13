// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2023 Maciej Baczmanski, Michal Kawiak, Jakub Mazur
// Copyright (c) 2016 Intel Corporation

#include <zephyr/kernel.h>

#include <zephyr/usb/usb_device.h>

#if CONFIG_IS_BT_SUPPORTED==y
#include "ble_gatt_service.h"
#endif

#include <string.h>

#include "driver.h"

#if CONFIG_IS_UART_SHELL_SUPPORTED==y
#include <zephyr/drivers/uart.h>
#endif

int main(void)
{
	#if CONFIG_IS_BT_SUPPORTED==y
	init_bt();
	#endif

	init_pwm_motor_driver();
}
