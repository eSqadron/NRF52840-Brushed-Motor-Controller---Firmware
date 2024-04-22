// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024 Maciej Baczmanski, Michal Kawiak, Jakub Mazur

#if defined(CONFIG_UART_SHELL_SUPPORT)

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include "driver.h"
#include "storage.h"
#include "uart_shell/shell_utils.h"

// TODO - worth to create a log_error() function which would map common errors and print them to
// avoid lots of if else statements. Can be done later

static int cmd_speed(const struct shell *shell, size_t argc, char *argv[])
{
	// TODO - add reset current template in this function
	return_codes_t ret;
	uint32_t speed_mrpm;

	if (argc == 1) {
		ret = speed_get(get_relevant_channel(), &speed_mrpm);
		if (ret == SUCCESS) {
			shell_fprintf(shell, SHELL_NORMAL, "speed: %d\n", speed_mrpm);

		} else {
			shell_fprintf(shell, SHELL_ERROR, "Error, code: %d!\n", ret);
		}

		return 0;
	} else if (argc == 2) {
		speed_mrpm = (uint32_t)strtol(argv[1], NULL, 10);
		ret = target_speed_set(speed_mrpm, get_relevant_channel());

		switch (ret) {
		case (SUCCESS):
			shell_fprintf(shell, SHELL_NORMAL, "speed set to: %d\n", speed_mrpm);
		break;
		case (ERR_DESIRED_VALUE_TO_HIGH):
			shell_fprintf(shell, SHELL_ERROR,
				"Desired speed to high! Desired speed: %d; Max speed: %d\n",
				speed_mrpm, get_current_max_speed());
		break;
		case(ERR_UNSUPPORTED_FUNCTION_IN_CURRENT_MODE):
			shell_fprintf(shell, SHELL_ERROR,
				"Function unsupported in current mode!\n");

		break;
		default:
			shell_fprintf(shell, SHELL_ERROR, "Other error, code: %d\n", ret);
		break;
		}
	}
	return 0;
}

static int cmd_position(const struct shell *shell, size_t argc, char *argv[])
{
	return_codes_t ret;
	uint32_t position;

	if (argc == 1) {
		ret = position_get(&position, get_relevant_channel());
		if (ret == SUCCESS) {
			shell_fprintf(shell, SHELL_NORMAL, "Position: %d\n", position);

		} else {
			shell_fprintf(shell, SHELL_ERROR, "Error, code: %d!\n", ret);
		}

		return 0;
	} else if (argc == 2) {
		position = (uint32_t)strtol(argv[1], NULL, 10);
		ret = target_position_set(position, get_relevant_channel());
		switch (ret) {
		case (SUCCESS):
			shell_fprintf(shell, SHELL_NORMAL, "Position set to: %d\n", position);
		break;
		case (ERR_DESIRED_VALUE_TO_HIGH):
			shell_fprintf(shell, SHELL_ERROR,
				"Desired position to high! Desired: %d. Max: 360\n", position);
		break;
		case (ERR_UNSUPPORTED_FUNCTION_IN_CURRENT_MODE):
			shell_fprintf(shell, SHELL_ERROR,
			"Function unsupported in current mode!\n");
		break;
		default:
			shell_fprintf(shell, SHELL_ERROR, "Other error, code: %d\n", ret);
		break;
		}
	}
	return 0;
}

static int cmd_position_zero(const struct shell *shell, size_t argc, char *argv[])
{
	return_codes_t ret;

	ret = position_reset_zero(get_relevant_channel());

	if (ret == SUCCESS) {
		shell_fprintf(shell, SHELL_NORMAL, "Position reset succesfulll\n");

	} else {
		shell_fprintf(shell, SHELL_ERROR, "Error, code: %d!\n", ret);
	}

	return 0;
}

static int cmd_mode(const struct shell *shell, size_t argc, char *argv[])
{
	return_codes_t ret;
	enum ControlModes mode;

	if (argc == 1) {
		ret = mode_get(&mode);
		switch (ret) {
		case (SUCCESS):
			char *mode_as_str = "";
			int r = get_control_mode_as_string(mode, &mode_as_str);

			if (r == SUCCESS) {
				shell_fprintf(shell, SHELL_NORMAL, "mode: %s\n", mode_as_str);

			} else {
				shell_fprintf(shell, SHELL_ERROR,
				"Conversion error, code: %d\n", ret);

			}
		break;
		case (ERR_NOT_INITIALISED):
			shell_fprintf(shell, SHELL_ERROR,
				"driver not initialised, could't get mode!\n");

		break;
		default:
			shell_fprintf(shell, SHELL_ERROR, "other error, code: %d\n", ret);
		break;
		}

		return 0;
	} else if (argc == 2) {
		ret = get_control_mode_from_string(argv[1], &mode);
		if (ret != SUCCESS) {
			shell_fprintf(shell, SHELL_ERROR,
				"Conversion error, errc: %d\n", ret);
			return 0;
		}

		ret = mode_set(mode);
		if (ret == SUCCESS) {
			shell_fprintf(shell, SHELL_NORMAL, "mode set to: %s\n", argv[1]);

		} else {
			shell_fprintf(shell, SHELL_ERROR, "Error, code: %d\n", ret);
		}
	}
	return 0;
}

static int cmd_actual_direction(const struct shell *shell, size_t argc, char *argv[])
{
	enum MotorDirection dir;
	int ret = get_motor_actual_direction(get_relevant_channel(),  &dir);

	if (ret != SUCCESS) {
		shell_fprintf(shell, SHELL_ERROR,
			      "Error while reading channel direction: %d\n", ret);
		return 0;
	}

	switch (dir) {
	case (FORWARD):
		shell_fprintf(shell, SHELL_INFO, "Motor is spinning forward!\n");
	break;
	case (BACKWARD):
		shell_fprintf(shell, SHELL_INFO, "Motor is spinning backward!\n");
	break;
	case (STOPPED):
		shell_fprintf(shell, SHELL_INFO, "Motor is stationary!\n");
	break;
	default:
		shell_fprintf(shell, SHELL_ERROR, "Unknown direction value! - %d\n", dir);
	break;
	}
	return 0;
}

SHELL_CMD_ARG_REGISTER(speed, NULL,
	"speed in milli RPM (one thousands of RPM)\nspeed to get speed\n speed <val> to set speed",
	cmd_speed, 1, 1);

SHELL_STATIC_SUBCMD_SET_CREATE(position_tree,
SHELL_CMD(zero,       NULL, "Zero position to current position", cmd_position_zero),
SHELL_SUBCMD_SET_END
);

SHELL_CMD_ARG_REGISTER(pos, &position_tree, "get/set position in deca-degree", cmd_position, 1, 1);
SHELL_CMD_ARG_REGISTER(mode, NULL, "choose mode, pos/speed control. Default speed", cmd_mode, 1, 1);

SHELL_CMD_REGISTER(actual_direction, NULL, "get motor actual spinning direction",
		   cmd_actual_direction);

#endif
