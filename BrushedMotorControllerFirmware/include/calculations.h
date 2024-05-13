
#define CALC_SPEED_CONTROL(target_speed, actual_speed) \
        ((int32_t)( CONFIG_KP_NUMERATOR_FOR_SPEED * (int32_t)(target_speed - actual_speed) / \
                    CONFIG_KP_DENOMINATOR_FOR_SPEED))


#define CALC_SPEED_CONTROL_FOR_POS(target_speed, actual_speed) \
        ((int32_t)( CONFIG_KP_NUMERATOR_FOR_SPEED_IN_POS * (int32_t)(target_speed - actual_speed) / \
                    CONFIG_KP_DENOMINATOR_FOR_SPEED_IN_POS))
