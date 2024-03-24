// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024 Maciej Baczmanski, Jakub Mazur

#if defined(CONFIG_UART_SHELL_SUPPORT)

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include "uart_shell/shell_utils.h"


enum ChannelNumber relevant_channel = CH0;

static int cmd_channel(const struct shell *shell, size_t argc, char *argv[])
{
	if (argc == 1) {
		shell_fprintf(shell, SHELL_NORMAL, "relevant channel: %d\n", relevant_channel);
	} else if (argc == 2) {
		enum ChannelNumber new_channel = (enum ChannelNumber)strtol(argv[1], NULL, 10);

		if (new_channel >= CONFIG_SUPPORTED_CHANNEL_NUMBER) {
			shell_fprintf(shell, SHELL_NORMAL,
			"New channel %d higher than number of supported channels %d\n",
			new_channel, CONFIG_SUPPORTED_CHANNEL_NUMBER);
			return 0;
		}
		relevant_channel = new_channel;
		shell_fprintf(shell, SHELL_NORMAL, "channel set to: %d\n", relevant_channel);
	}
	return 0;
}

enum ChannelNumber get_relevant_channel(void)
{
	return relevant_channel;
}

static int cmd_drv_version(const struct shell *shell, size_t argc, char *argv[])
{
	const struct DriverVersion tmp_ver = get_driver_version();

	shell_fprintf(shell, SHELL_ERROR,
		      "Software version: %d.%d\n", tmp_ver.major, tmp_ver.minor);

	return 0;
}

#if defined(CONFIG_BOARD_NRF52840DONGLE_NRF52840)
static int cmd_boot(const struct shell *shell, size_t argc, char *argv[])
{
	enter_boot();
	return 0;
}
#endif

static int cmd_debug(const struct shell *shell, size_t argc, char *argv[])
{
	shell_fprintf(shell, SHELL_INFO, "Current cycles count: %" PRIu64  "\n"
					 "Time cycles count: %" PRIu64  "\n"
					 "Ret: %d\n"
					 "CalcSpeed: %u\n"
					 "pos target: %u\n"
					 "pos delta %d",
	get_cycles_count_DEBUG(),
	get_time_cycles_count_DEBUG(),
	get_ret_DEBUG(),
	get_calc_speed_DEBUG(),
	get_target_pos_DEBUG(),
	get_pos_d_DEBUG()
	);
	return 0;
}

SHELL_CMD_ARG_REGISTER(channel, NULL, "Get/set current relevant channel", cmd_channel, 1, 1);
#if defined(CONFIG_BOARD_NRF52840DONGLE_NRF52840)
SHELL_CMD_REGISTER(boot, NULL,
	"Enter bootloader mode, in order to flash new software via nRF connect programmer",
	cmd_boot);
#endif
SHELL_CMD_REGISTER(drv_version, NULL, "get motor driver version", cmd_drv_version);
SHELL_CMD_REGISTER(debug, NULL, "get debug info", cmd_debug);

#endif
