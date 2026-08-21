#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* LCD + PID wheel-speed test for the three chassis motors. Keep wheels lifted. */
#define APP_ENABLE_AUTOMATIC_MOTOR_TEST  1
#define APP_WHEEL_DIAMETER_MM            70U
#define APP_MOTOR_GEAR_RATIO             34U
#define APP_ENCODER_LINES_PER_MOTOR_REV  13U
#define APP_ENCODER_QUADRATURE_FACTOR    4U
#define APP_ENCODER_COUNTS_PER_WHEEL_REV (APP_MOTOR_GEAR_RATIO * \
                                          APP_ENCODER_LINES_PER_MOTOR_REV * \
                                          APP_ENCODER_QUADRATURE_FACTOR)
#define APP_MOTOR_SPEED_TEST_TARGET_MM_S 110.0f
#define APP_ENABLE_SERVO_SWEEP_TEST      0

/*
 * LED_3 control-layer port. Keep disabled until wheel positions/directions,
 * camera/claw channels and the vision protocol have been checked on hardware.
 * When enabled it takes priority over the automatic reversing bench test.
 */
#define APP_ENABLE_TASK                  0

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
#define APP_GO_DISTANCE_SPEED_MM_S       110.0f
#define APP_GO_DISTANCE_SLOWDOWN_MM      100.0f
#define APP_GO_DISTANCE_TOLERANCE_MM     3.0f
#define APP_GO_DISTANCE_PROGRESS_MM      0.25f
#define APP_GO_DISTANCE_NO_PROGRESS_CYCLES 100U

/* LED_3 used servo 3 for the camera and servo 4 for the claw. */
#define APP_CAMERA_SERVO_ID              3U
#define APP_CLAW_SERVO_ID                4U
#define APP_CAMERA_MIN_ANGLE             10U
#define APP_CAMERA_MAX_ANGLE             170U
#define APP_CAMERA_WIDE_ANGLE            90U

/* Motor feedback runs at 10 ms; the higher-level task remains at 20 ms. */
#define APP_MOTOR_CONTROL_PERIOD_MS      10U
#define APP_TASK_PERIOD_MS               20U
#define APP_CAMERA_SETTLE_MS             400U
#define APP_INITIAL_SEARCH_MS            700U
#define APP_CLEAR_BUMP_MS                900U
#define APP_VISION_TIMEOUT_MS            250U
#define APP_VISION_TARGET_X              320U
#define APP_VISION_TARGET_Y              240U
#define APP_VISION_MAX_X                 639U
#define APP_VISION_MAX_Y                 479U
#define APP_CLEAR_BUMP_SPEED             350
#define APP_SEARCH_ROTATE_SPEED          220
#define APP_APPROACH_SPEED               780
#define APP_STEERING_DEAD_ZONE           4
#define APP_STEERING_DIRECTION           1.0f
#define APP_CAMERA_DIRECTION             1.0f

/* This 1.8-inch 128x160 ST7735 panel exposes GRAM origin (0, 0). */
#define APP_LCD_WIDTH       128U
#define APP_LCD_HEIGHT      160U
#define APP_LCD_X_OFFSET    0U
#define APP_LCD_Y_OFFSET    0U

#define APP_MOTOR_KEY_DEBOUNCE_MS        30U

#endif
