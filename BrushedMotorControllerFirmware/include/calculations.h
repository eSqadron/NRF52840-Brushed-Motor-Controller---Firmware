/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 Maciej Baczmanski, Jakub Mazur
 */

#define CALC_SPEED_CONTROL(target_speed, actual_speed) \
	((int32_t)(CONFIG_KP_NUMERATOR_FOR_SPEED * (int32_t)(target_speed - actual_speed) / \
		   CONFIG_KP_DENOMINATOR_FOR_SPEED))


#define CALC_SPEED_CONTROL_FOR_POS(target_speed, actual_speed) \
	((int32_t)(CONFIG_KP_NUMERATOR_FOR_SPEED_IN_POS * (int32_t)(target_speed - actual_speed) / \
		   CONFIG_KP_DENOMINATOR_FOR_SPEED_IN_POS))

// Difference from target position and actual position
#define CALC_POSITION_DELTA(chnl) \
	drv_chnls[chnl].position_delta = drv_chnls[chnl].target_position - \
					 drv_chnls[chnl].curr_pos; \
	bool is_target_behind = false; \
	if (drv_chnls[chnl].position_delta < 0) { \
		drv_chnls[chnl].position_delta = \
		-drv_chnls[chnl].position_delta; \
		is_target_behind = true; \
	} \
// Check if motor should spin backward to get to the target
// (if target is smalller number than current position)
