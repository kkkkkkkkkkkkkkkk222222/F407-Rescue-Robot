#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Optional S1 wheel-speed bench test. Disabled while the autonomous task runs. */
#define APP_ENABLE_AUTOMATIC_MOTOR_TEST  0
#define APP_WHEEL_DIAMETER_MM            70U
#define APP_MOTOR_GEAR_RATIO             34U
#define APP_ENCODER_LINES_PER_MOTOR_REV  13U
#define APP_ENCODER_QUADRATURE_FACTOR    4U
#define APP_ENCODER_COUNTS_PER_WHEEL_REV (APP_MOTOR_GEAR_RATIO * \
                                          APP_ENCODER_LINES_PER_MOTOR_REV * \
                                          APP_ENCODER_QUADRATURE_FACTOR)
#define APP_MOTOR_SPEED_TEST_TARGET_MM_S 110.0f
#define APP_ENABLE_SERVO_SWEEP_TEST      0

/* LED_3-derived autonomous rescue flow; it takes priority over bench tests. */
#define APP_ENABLE_TASK                  1

/*
 * The robot uses the three LED_3 channels: M1/TIM5, M2/TIM9 and M3/TIM2.
 * Change signs after checking the real installation.
 */
#define APP_OMNI_M1_MOTOR_SIGN           1
#define APP_OMNI_M2_MOTOR_SIGN           1
#define APP_OMNI_M3_MOTOR_SIGN           1
#define APP_OMNI_M1_ENCODER_SIGN        -1
#define APP_OMNI_M2_ENCODER_SIGN        -1
#define APP_OMNI_M3_ENCODER_SIGN        -1

/* Three-motor speed loop, updated every 10 ms. */
#define APP_MOTOR_MAX_COUNT_10MS         60
#define APP_MOTOR_BASE_PWM               450
#define APP_MOTOR_SPEED_KP               6.0f
#define APP_MOTOR_SPEED_KI               0.15f
#define APP_MOTOR_SPEED_KD               0.0f
#define APP_MOTOR_PID_LIMIT              450.0f
#define APP_MOTOR_INTEGRAL_LIMIT         2500.0f
#define APP_MOTOR_DIRECTION_FAULT_COUNT  1
#define APP_MOTOR_DIRECTION_FAULT_CYCLES 6U
#define APP_MOTOR_DIRECTION_GRACE_CYCLES 20U
#define APP_MOTOR_STALL_MIN_PWM          500
#define APP_MOTOR_STALL_MAX_COUNT_10MS   0
#define APP_MOTOR_STALL_CYCLES           50U
#define APP_MOTOR_STALL_GRACE_CYCLES     50U
#define APP_MOTOR_BRAKE_CYCLES           6U

/* Go_distance() accepts metres; internal speed, slowdown and tolerance use millimetres. */
#define APP_GO_DISTANCE_SPEED_MM_S       300.0f
#define APP_GO_DISTANCE_SLOWDOWN_MM      100.0f
#define APP_GO_DISTANCE_TOLERANCE_MM     3.0f
#define APP_GO_DISTANCE_PROGRESS_MM      0.25f
#define APP_GO_DISTANCE_NO_PROGRESS_CYCLES 100U

/* LED_3 used servo 3 for the camera and servo 4 for the claw. */
#define APP_CAMERA_SERVO_ID              3U
#define APP_CLAW_SERVO_ID                4U
#define APP_CAMERA_MIN_ANGLE             0U
#define APP_CAMERA_MAX_ANGLE             170U
#define APP_CAMERA_WIDE_ANGLE            90U
#define APP_CAMERA_CHECK_ANGLE           0U
#define APP_CLAW_OPEN_ANGLE              90U
#define APP_CLAW_CLOSE_ANGLE             30U

/* Motor feedback runs at 10 ms; the higher-level task remains at 20 ms. */
#define APP_MOTOR_CONTROL_PERIOD_MS      10U
#define APP_TASK_PERIOD_MS               20U
#define APP_MATCH_TIME_S                 180U
#define APP_MATCH_TIME_MS                (APP_MATCH_TIME_S * 1000U)
#define APP_INITIAL_FORWARD_DISTANCE_M   0.7f
#define APP_TARGET_WAIT_MS               700U
#define APP_VISION_TIMEOUT_MS            250U
#define APP_CONFIG_CONFIRM_FRAMES        3U
#define APP_CARGO_CONFIRM_FRAMES         3U
#define APP_NAV_CONFIRM_FRAMES           3U
#define APP_DROP_CONFIRM_FRAMES          3U
#define APP_VISION_TARGET_X              320U
#define APP_VISION_TARGET_Y              240U
#define APP_VISION_MAX_X                 639U
#define APP_VISION_MAX_Y                 479U
#define APP_VISION_MIN_DISTANCE_MM       1U
#define APP_SEARCH_ROTATE_SPEED          220
#define APP_APPROACH_SPEED               780
#define APP_CRAB_MID_SPEED               450
#define APP_CRAB_SLOW_SPEED              220
#define APP_CRAB_ADJUST_SPEED            180
#define APP_CRAB_MID_DISTANCE_MM         500U
#define APP_CRAB_SLOW_DISTANCE_MM        250U
#define APP_CRAB_STOP_DISTANCE_MM        120U
#define APP_CRAB_CLOSE_WAIT_MS           500U
#define APP_CRAB_BACK_MS                 250U
#define APP_STEERING_DEAD_ZONE           4
#define APP_STEERING_DIRECTION           1.0f
#define APP_CAMERA_DIRECTION             1.0f

/* Upper-computer-guided return and local delivery sequence. */
#define APP_NAV_TIMEOUT_MS               200U
#define APP_RETURN_FORWARD_SPEED         520
#define APP_RETURN_BACKWARD_SPEED        300
#define APP_RETURN_TURN_SPEED            240
#define APP_DROP_FORWARD_DISTANCE_M      0.20f
#define APP_DROP_BACK_DISTANCE_M         0.50f
#define APP_DROP_RELEASE_WAIT_MS         600U
#define APP_CAMERA_SETTLE_MS             400U

/* This 1.8-inch 128x160 ST7735 panel exposes GRAM origin (0, 0). */
#define APP_LCD_WIDTH       128U
#define APP_LCD_HEIGHT      160U
#define APP_LCD_X_OFFSET    0U
#define APP_LCD_Y_OFFSET    0U

#define APP_MOTOR_KEY_DEBOUNCE_MS        30U

#endif
