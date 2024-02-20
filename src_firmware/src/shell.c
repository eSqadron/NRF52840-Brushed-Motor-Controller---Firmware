// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2023 Maciej Baczmanski, Michal Kawiak, Jakub Mazur
#if (CONFIG_IS_UART_SHELL_SUPPORTED==y)

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include "driver.h"

enum ChannelNumber relevant_channel = CH0;

static int cmd_channel(const struct shell *shell, size_t argc, char *argv[]){
    if(argc == 1){
        shell_fprintf(shell, SHELL_NORMAL, "relevant channel: %d\n", relevant_channel);
    } else if(argc == 2){
        enum ChannelNumber new_channel = (enum ChannelNumber)strtol(argv[1], NULL, 10);

		if(new_channel >= CONFIG_SUPPORTED_CHANNEL_NUMBER){
			shell_fprintf(shell, SHELL_NORMAL, "New channel %d higher than no of supported channels %d\n", new_channel, CONFIG_SUPPORTED_CHANNEL_NUMBER);
			return 0;
		}
		relevant_channel = new_channel;
        shell_fprintf(shell, SHELL_NORMAL, "channel set to: %d\n", relevant_channel);
    }
    return 0;
}

static int cmd_speed(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;
	uint32_t speed_mrpm;

	if (argc == 1) {
		ret = speed_get(relevant_channel, &speed_mrpm);
		if (ret == SUCCESS) {
			shell_fprintf(shell, SHELL_NORMAL, "speed: %d\n", speed_mrpm);

		} else if (ret == NOT_INITIALISED) {
			shell_fprintf(shell, SHELL_ERROR,
				"driver not initialised, could't get speed!\n");

		} else {
			shell_fprintf(shell, SHELL_ERROR, "other error, code: %d\n", ret);
		}

		return 0;
	} else if (argc == 2) {
		speed_mrpm = (uint32_t)strtol(argv[1], NULL, 10);
		ret = target_speed_set(speed_mrpm, relevant_channel);

		if (ret == SUCCESS) {
			shell_fprintf(shell, SHELL_NORMAL, "speed set to: %d\n", speed_mrpm);

		} else if (ret == NOT_INITIALISED) {
			shell_fprintf(shell, SHELL_ERROR,
				"driver not initialised, could't set speed!\n");

		} else if (ret == DESIRED_VALUE_TO_HIGH) {
			shell_fprintf(shell, SHELL_ERROR,
				"Desired speed to high! Desired speed: %d; Max speed: %d\n",
				speed_mrpm, get_current_max_speed());

		} else if (ret == UNSUPPORTED_FUNCTION_IN_CURRENT_MODE) {
			shell_fprintf(shell, SHELL_ERROR,
				"Function unsupported in current mode!\n");

		} else {
			shell_fprintf(shell, SHELL_ERROR, "other error, code: %d\n", ret);
		}
	}
	return 0;
}

static int cmd_off_on(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;

	if (argc == 1) {
		if (get_motor_off_on(relevant_channel)) {
			shell_fprintf(shell, SHELL_NORMAL, "Motor channel %d is turned on!\n", relevant_channel);

		} else {
			shell_fprintf(shell, SHELL_NORMAL, "Motor channel %d is turned off!\n", relevant_channel);
		}

	} else if (argc == 2) {
		if (strcmp(argv[1], "start") == 0) {
			ret = motor_on(FORWARD, relevant_channel);

		} else if (strcmp(argv[1], "stop") == 0) {
			ret = motor_off(relevant_channel);

		} else {
			shell_fprintf(shell, SHELL_ERROR, "Unknown subcommand!\n");
			return 0;
		}

		if (ret != 0) {
			shell_fprintf(shell, SHELL_ERROR,
				"Couldn't change motor state! Error %d\n", ret);

		} else {
			shell_fprintf(shell, SHELL_NORMAL, "Operation executed on channel %d!\n", relevant_channel);
		}

	} else if (argc == 3) {
		if (strcmp(argv[1], "start") == 0) {
			if (strcmp(argv[2], "fwd") == 0 ||
			    strcmp(argv[2], "forward") == 0 ||
			    strcmp(argv[2], "f") == 0) {
				ret = motor_on(FORWARD, relevant_channel);

			} else if (strcmp(argv[2], "bck") == 0 ||
				   strcmp(argv[2], "backward") == 0 ||
				   strcmp(argv[2], "b") == 0) {
				ret = motor_on(BACKWARD, relevant_channel);

			} else {
				shell_fprintf(shell, SHELL_ERROR,
					"Unknown subcommand!, %s\n", argv[2]);
				return 0;
			}

			if (ret != 0) {
				shell_fprintf(shell, SHELL_ERROR,
					"Couldn't change motor state! Error %d\n\n", ret);

			} else {
				shell_fprintf(shell, SHELL_NORMAL, "Operation executed!\n");
			}

		} else {
			shell_fprintf(shell, SHELL_ERROR, "Unknown subcommand!, %s\n", argv[1]);
		}
	}

	return 0;
}

static int cmd_position(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;
	uint32_t position;

	if (argc == 1) {
		ret = position_get(&position, relevant_channel);
		if (ret == SUCCESS) {
			shell_fprintf(shell, SHELL_NORMAL, "Position: %d\n", position);

		} else if (ret == NOT_INITIALISED) {
			shell_fprintf(shell, SHELL_ERROR,
				"Driver not initialised, could't get position!\n");

		} else {
			shell_fprintf(shell, SHELL_ERROR, "Other error, code: %d\n", ret);
		}

		return 0;
	} else if (argc == 2) {
		position = (uint32_t)strtol(argv[1], NULL, 10);
		ret = target_position_set(position, relevant_channel);
		if (ret == SUCCESS) {
			shell_fprintf(shell, SHELL_NORMAL, "Position set to: %d\n", position);

		} else if (ret == NOT_INITIALISED) {
			shell_fprintf(shell, SHELL_ERROR,
				"Driver not initialised, could't set position!\n");

		} else if (ret == DESIRED_VALUE_TO_HIGH) {
			shell_fprintf(shell, SHELL_ERROR,
				"Desired position to high! Desired: %d. Max: 360\n", position);

		} else if (ret == UNSUPPORTED_FUNCTION_IN_CURRENT_MODE) {
			shell_fprintf(shell, SHELL_ERROR,
			"Function unsupported in current mode!\n");

		} else {
			shell_fprintf(shell, SHELL_ERROR, "Other error, code: %d\n", ret);
		}
	}
	return 0;
}

static int cmd_mode(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;
	enum ControlModes mode;

	if (argc == 1) {
		ret = mode_get(&mode);
		if (ret == SUCCESS) {
			char *mode_as_str = "";
			int r = get_control_mode_as_string(mode, &mode_as_str);

			if (r == SUCCESS) {
				shell_fprintf(shell, SHELL_NORMAL, "mode: %s\n", mode_as_str);

			} else {
				shell_fprintf(shell, SHELL_ERROR,
				"Conversion error, code: %d\n", ret);

			}
		} else if (ret == NOT_INITIALISED) {
			shell_fprintf(shell, SHELL_ERROR,
				"driver not initialised, could't get mode!\n");

		} else {
			shell_fprintf(shell, SHELL_ERROR, "other error, code: %d\n", ret);
		}

		return 0;
	} else if (argc == 2) {
		ret = get_control_mode_from_string(argv[1], &mode);
		if (ret != SUCCESS) {
			shell_fprintf(shell, SHELL_ERROR,
				"Setting Control Mode crashed at conversion, errc: %d\n", ret);
		}

		ret = mode_set(mode);
		if (ret == SUCCESS) {
			shell_fprintf(shell, SHELL_NORMAL, "mode set to: %s\n", argv[1]);

		} else if (ret == NOT_INITIALISED) {
			shell_fprintf(shell, SHELL_ERROR,
				"driver not initialised, could't set mode!\n");

		} else {
			shell_fprintf(shell, SHELL_ERROR, "other error, code: %d\n", ret);
		}
	}
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

static int cmd_drv_version(const struct shell *shell, size_t argc, char *argv[])
{
	const struct DriverVersion tmp_ver = get_driver_version();

	shell_fprintf(shell, SHELL_ERROR,
		      "Software version: %d.%d\n", tmp_ver.major, tmp_ver.minor);
	return 0;
}

SHELL_CMD_ARG_REGISTER(channel, NULL, "Get/set current relevant channel", cmd_channel, 1, 1);

SHELL_CMD_ARG_REGISTER(speed, NULL,
	"speed in milli RPM (one thousands of RPM)\nspeed to get speed\n speed <val> to set speed",
	cmd_speed, 1, 1);

SHELL_CMD_ARG_REGISTER(motor, NULL, "Start or stop motor\nstart <f b>\noff", cmd_off_on, 1, 2);
SHELL_CMD_ARG_REGISTER(pos, NULL, "----------------------", cmd_position, 1, 1);
SHELL_CMD_ARG_REGISTER(mode, NULL, "----------------------", cmd_mode, 1, 1);
SHELL_CMD_REGISTER(debug, NULL, "get debug info", cmd_debug);
SHELL_CMD_REGISTER(drv_version, NULL, "get motor driver version", cmd_drv_version);


#if defined(CONFIG_BOARD_NRF52840DONGLE_NRF52840)
SHELL_CMD_REGISTER(boot, NULL,
	"Enter bootloader mode, in order to flash new software via nRF connect programmer",
	cmd_boot);
#endif


#endif