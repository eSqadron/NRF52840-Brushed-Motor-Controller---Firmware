// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024 Maciej Baczmanski, Jakub Mazur

#if defined(CONFIG_UART_SHELL_SUPPORT)

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include "driver.h"
#include "storage.h"
#include "uart_shell/shell_utils.h"

static int cmd_template_active(const struct shell *shell, size_t argc, char *argv[])
{
	struct Template current;
	return_codes_t ret = get_current_template(&current);

	switch (ret) {
	case (SUCCESS):
		shell_fprintf(shell, SHELL_INFO,
			      "Name: %-9s speed %d\n", current.name, current.speed);
	break;
	case (ERR_ERROR_CODE_FROM_ERRNO):
		shell_fprintf(shell, SHELL_ERROR,
			      "Error code from errno.h, code: %d in line %d",
			      get_errno_error_code(), get_errno_error_line());
	break;
	default:
		shell_fprintf(shell, SHELL_ERROR,
			      "Error while checking active template: %d!\n", ret);
	break;
	}

	return 0;
}

static int cmd_template_get(const struct shell *shell, size_t argc, char *argv[])
{
	uint8_t size = get_template_size();
	return_codes_t ret;

	if (argc == 1) {
		struct Template *tmp = (struct Template *)malloc(size * sizeof(struct Template));

		ret = get_templates(tmp);

		switch (ret) {
		case (SUCCESS):
		break;
		case (ERR_EMPTY_TEMPLATE_LIST):
			shell_fprintf(shell, SHELL_WARNING, "Template list is empty!\n");
		return 0;
		case (ERR_ERROR_CODE_FROM_ERRNO):
			shell_fprintf(shell, SHELL_ERROR,
				"Error code from errno.h, code: %d in line %d",
				get_errno_error_code(), get_errno_error_line());
		return 0;
		default:
			shell_fprintf(shell, SHELL_WARNING,
				      "Error while getting template list: %d!\n", ret);
		return 0;
		}

		for (int i = 0; i < size; i++) {
			shell_fprintf(shell, SHELL_INFO,
				      "Name: %-9s speed %d\n", tmp[i].name, tmp[i].speed);
		}

		free(tmp);
	} else {
		struct Template res;
		uint8_t template_id;

		return_codes_t ret = get_template_and_id_by_name(argv[1], &res, &template_id);
		// TODO -NULL?

		switch (ret) {
		case(SUCCESS):
			shell_fprintf(shell, SHELL_INFO, "speed %d\n", res.speed);
		break;
		case (ERR_COULDNT_FIND_TEMPLATE):
			shell_fprintf(shell, SHELL_WARNING, "Couldn't find this template!\n");
		break;
		case (ERR_EMPTY_TEMPLATE_LIST):
			shell_fprintf(shell, SHELL_WARNING, "Template list is empty!\n");
		break;
		case (ERR_ERROR_CODE_FROM_ERRNO):
			shell_fprintf(shell, SHELL_ERROR,
				"Error code from errno.h, code: %d in line %d",
				get_errno_error_code(), get_errno_error_line());
		break;
		default:
			shell_fprintf(shell, SHELL_ERROR,
				      "Other error during template get! Error Code: %d\n", ret);
		break;
		}
	}

	return 0;
}

// TODO - add template name as a dynamic subcomand
static int cmd_template_apply(const struct shell *shell, size_t argc, char *argv[])
{
	struct Template res;
	uint8_t template_id;
	int ret;

	ret = get_template_and_id_by_name(argv[1], &res, &template_id);
	switch (ret) {
	case (ERR_COULDNT_FIND_TEMPLATE):
		shell_fprintf(shell, SHELL_WARNING, "Couldn't find template!\n");
	return 0;
	case (ERR_ERROR_CODE_FROM_ERRNO):
		shell_fprintf(shell, SHELL_ERROR,
			      "Error code from errno.h, code: %d in line %d",
			      get_errno_error_code(), get_errno_error_line());
	return 0;
	case (SUCCESS):
	break;
	default:
		shell_fprintf(shell, SHELL_ERROR,
			      "Other error while searching for saved template: %d!\n", ret);
	return 0;
	}

	ret = target_speed_set(res.speed, get_relevant_channel());
	switch (ret) {
	case (SUCCESS):
		shell_fprintf(shell, SHELL_NORMAL,
			      "speed set to: %d on channel %d\n",
			      res.speed,
			      get_relevant_channel());
	break;
	case (ERR_ERROR_CODE_FROM_ERRNO):
		shell_fprintf(shell, SHELL_ERROR,
			      "Error code from errno.h, code: %d in line %d",
			      get_errno_error_code(), get_errno_error_line());
	return 0;
	case (ERR_DESIRED_VALUE_TO_HIGH):
		shell_fprintf(shell, SHELL_ERROR,
			"Desired template speed too high! Desired speed: %d; Max speed: %d\n",
			res.speed, get_current_max_speed());
	return 0;
	case (ERR_UNSUPPORTED_FUNCTION_IN_CURRENT_MODE):
		shell_fprintf(shell, SHELL_ERROR,
			"Function unsupported in current mode!\n");
	return 0;
	default:
		shell_fprintf(shell, SHELL_ERROR, "other error, code: %d\n", ret);
	return 0;
	}

	ret = set_current_template(res.name);
	if (ret != SUCCESS) {
		shell_fprintf(shell, SHELL_ERROR,
			      "But couldn't update current template value! error %d\n", ret);
	}
	return 0;
}

static int cmd_template_set(const struct shell *shell, size_t argc, char *argv[])
{
	struct Template new_template;
	return_codes_t ret;

	strcpy(new_template.name, argv[1]);
	new_template.speed = (uint32_t)strtol(argv[2], NULL, 10);
	ret = set_template(new_template);
	if (ret == SUCCESS) {
		shell_fprintf(shell, SHELL_INFO, "New template created\n");
	} else {
		shell_fprintf(shell, SHELL_ERROR,
			      "Error while setting new template, code: %d\n", ret);
	}

	return 0;
}

static int cmd_template_clear(const struct shell *shell, size_t argc, char *argv[])
{
	if (argc == 1) {
		return_codes_t ret = factory_reset();

		switch (ret) {
		case (SUCCESS):
			shell_fprintf(shell, SHELL_NORMAL, "Templates cleared\n");
		break;
		case (ERR_ERROR_CODE_FROM_ERRNO):
			shell_fprintf(shell, SHELL_ERROR,
				"Error code from errno.h, code: %d in line %d",
				get_errno_error_code(), get_errno_error_line());
		break;
		default:
			shell_fprintf(shell, SHELL_ERROR, "other error, code: %d\n", ret);
		break;
		}
	} else {
		remove_template_by_name(argv[1]);
	}
	return 0;
}

static int cmd_template_size(const struct shell *shell, size_t argc, char *argv[])
{
	int size = get_template_size();

	shell_fprintf(shell, SHELL_ERROR, "Current size of template buffer: %d\n", size);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_template,
	SHELL_CMD_ARG(active, NULL, "Active template", cmd_template_active, 1, 0),
	SHELL_CMD_ARG(get, NULL, "get <optional name>", cmd_template_get, 1, 1),
	SHELL_CMD_ARG(apply,   NULL, "apply name", cmd_template_apply, 2, 0),
	SHELL_CMD_ARG(set,   NULL, "set name speed", cmd_template_set, 3, 0),
	SHELL_CMD_ARG(clear,   NULL, "clear template by name. If none given clears all templates.",
				  cmd_template_clear, 1, 1),
	SHELL_CMD_ARG(size,   NULL, "get amount of templates", cmd_template_size, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(template, &sub_template, "get/apply/create speed template", NULL);

#endif
