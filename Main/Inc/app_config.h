#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Automatic bench test: keep the wheels lifted and all mechanisms clear. */
#define APP_ENABLE_AUTOMATIC_MOTOR_TEST  1
#define APP_MOTOR_TEST_MIN_COMMAND       500U
#define APP_MOTOR_TEST_DEFAULT_COMMAND   APP_MOTOR_TEST_MIN_COMMAND
#define APP_MOTOR_TEST_COMMAND_STEP      100U
#define APP_MOTOR_TEST_DIRECTION_MS      500U
#define APP_MOTOR_TEST_REVERSAL_PAUSE_MS 100U
#define APP_MOTOR_KEY_DEBOUNCE_MS        30U
#define APP_ENABLE_SERVO_SWEEP_TEST      0
#define APP_LCD_STARTUP_COLOR_TEST       1

/*
 * LED_3 control-layer port. Keep disabled until wheel positions/directions,
 * camera/claw channels and the vision protocol have been checked on hardware.
 * When enabled it takes priority over the automatic reversing bench test.
 */
#define APP_ENABLE_TASK                  0

/*
 * The robot uses the three LED_3 channels: M1/TIM5, M2/TIM9 and M3/TIM2.
 * M4/TIM10+TIM11 remains available for board tests and is stopped by the
 * autonomous controller. Change signs after checking the real installation.
 */
#define APP_OMNI_M1_MOTOR_SIGN           1
#define APP_OMNI_M2_MOTOR_SIGN           1
#define APP_OMNI_M3_MOTOR_SIGN           1
#define APP_OMNI_M1_ENCODER_SIGN         1
#define APP_OMNI_M2_ENCODER_SIGN         1
#define APP_OMNI_M3_ENCODER_SIGN         1

/* LED_3 three-motor speed loop, updated every 20 ms. */
#define APP_MOTOR_MAX_COUNT_20MS         120
#define APP_MOTOR_FEED_FORWARD_PERCENT   65
#define APP_MOTOR_SPEED_KP               3.0f
#define APP_MOTOR_SPEED_KI               0.15f
#define APP_MOTOR_SPEED_KD               0.0f
#define APP_MOTOR_PID_LIMIT              450.0f
#define APP_MOTOR_INTEGRAL_LIMIT         2500.0f

/* LED_3 used servo 3 for the camera and servo 4 for the claw. */
#define APP_CAMERA_SERVO_ID              3U
#define APP_CLAW_SERVO_ID                4U
#define APP_CAMERA_MIN_ANGLE             10U
#define APP_CAMERA_MAX_ANGLE             170U
#define APP_CAMERA_WIDE_ANGLE            90U
#define APP_CLAW_OPEN_ANGLE              30U
#define APP_CLAW_CLOSE_ANGLE             100U

/* Vision/task parameters; the fixed-period PID gains come from LED_3. */
#define APP_TASK_PERIOD_MS               20U
#define APP_CAMERA_SETTLE_MS             400U
#define APP_INITIAL_SEARCH_MS            700U
#define APP_CLEAR_BUMP_MS                900U
#define APP_VISION_TIMEOUT_MS            250U
#define APP_VISION_TARGET_X              320U
#define APP_VISION_TARGET_Y              240U
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

#endif
