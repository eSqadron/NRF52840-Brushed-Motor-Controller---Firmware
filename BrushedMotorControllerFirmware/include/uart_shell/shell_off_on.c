// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024 Maciej Baczmanski, Jakub Mazur

#if defined(CONFIG_UART_SHELL_SUPPORT)

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include "driver.h"
#include "uart_shell/shell_utils.h"

static int cmd_off_on_get(const struct shell *shell, size_t argc, char *argv[])
{
	if (get_motor_off_on(get_relevant_channel())) {
		shell_fprintf(shell, SHELL_NORMAL,
				"Motor channel %d is turned on!\n", get_relevant_channel());

	} else {
		shell_fprintf(shell, SHELL_NORMAL,
				"Motor channel %d is turned off!\n", get_relevant_channel());
	}
	return 0;
}

static int cmd_off_on_stop(const struct shell *shell, size_t argc, char *argv[])
{
	return_codes_t ret = motor_off(get_relevant_channel());

	if (ret != SUCCESS) {
		shell_fprintf(shell, SHELL_ERROR,
			"Couldn't change motor state! Error %d\n", ret);

	} else {
		shell_fprintf(shell, SHELL_NORMAL,
				"Motor stopped successfully on channel %d!\n",
				get_relevant_channel());
	}
	return 0;
}

static int cmd_off_on_start_forward(const struct shell *shell, size_t argc, char *argv[])
{
	return_codes_t ret = motor_on(FORWARD, get_relevant_channel());

	if (ret != SUCCESS) {
		shell_fprintf(shell, SHELL_ERROR,
			"Couldn't change motor state! Error %d\n", ret);

	} else {
		shell_fprintf(shell, SHELL_NORMAL,
				"Motor started forward successfully on channel %d!\n",
				get_relevant_channel());
	}
	return 0;
}

static int cmd_off_on_start_backward(const struct shell *shell, size_t argc, char *argv[])
{
	return_codes_t ret = motor_on(BACKWARD, get_relevant_channel());

	if (ret != SUCCESS) {
		shell_fprintf(shell, SHELL_ERROR,
			"Couldn't change motor state! Error %d\n", ret);

	} else {
		shell_fprintf(shell, SHELL_NORMAL,
				"Motor started backward successfully on channel %d!\n",
				get_relevant_channel());
	}
	return 0;
}

static const char start_f_des[] = "Start motor in direction forward";
static const char start_b_des[] = "Start motor in direction backward";

SHELL_STATIC_SUBCMD_SET_CREATE(offon_tree_start,
SHELL_CMD(f,       NULL, start_f_des, cmd_off_on_start_forward),
SHELL_CMD(forward, NULL, start_f_des, cmd_off_on_start_forward),
SHELL_CMD(fwd,     NULL, start_f_des, cmd_off_on_start_forward),
SHELL_CMD(b,       NULL, start_b_des, cmd_off_on_start_backward),
SHELL_CMD(bck,     NULL, start_b_des, cmd_off_on_start_backward),
SHELL_CMD(bacward, NULL, start_b_des, cmd_off_on_start_backward),
SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(offon_tree,
SHELL_CMD(start, &offon_tree_start, "Start motor", cmd_off_on_start_forward),
SHELL_CMD(stop, NULL, "Stop motor", cmd_off_on_stop),
SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(motor, &offon_tree,
		   "Get motor off/on state",
		   cmd_off_on_get);

#endif
