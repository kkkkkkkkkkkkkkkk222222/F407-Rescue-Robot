#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Standalone test modes. Keep only one mode enabled at a time. */
#define APP_ENABLE_MOTION_TEST           1
#define APP_ENABLE_AUTOMATIC_MOTOR_TEST  0
#define APP_ENABLE_SERVO_SWEEP_TEST      0

/* Non-blocking IMU angle turn used by Motor_TurnAngle(). */
#define APP_MOTOR_TURN_TOLERANCE_MDEG    1000L
#define APP_MOTOR_TURN_SLOWDOWN_MDEG     30000L
#define APP_MOTOR_TURN_FAST_MM_S         250.0f
#define APP_MOTOR_TURN_SLOW_MM_S         120.0f
#define APP_MOTOR_TURN_TIMEOUT_MS        10000U
#define APP_MOTOR_TURN_MAX_DEG           360.0f
#define APP_MOTION_TEST_SPEED_MM_S       750.0f
#define APP_MOTION_TEST_MOVE_ANGLE_DEG   180.0f
#define APP_MOTION_TEST_MOVE_TIME_MS     3000U
#define APP_MOTION_TEST_BRAKE_TIME_MS    100U
#define APP_MOTION_TEST_TURN_DEG         180.0f
#define APP_MOTION_TEST_REPEAT_COUNT     2U
#define APP_MOTION_TEST_TRANSITION_TIMEOUT_MS 2000U

#define APP_WHEEL_DIAMETER_MM            70U
#define APP_MOTOR_GEAR_RATIO             34U
#define APP_ENCODER_LINES_PER_MOTOR_REV  13U
#define APP_ENCODER_QUADRATURE_FACTOR    4U
#define APP_ENCODER_COUNTS_PER_WHEEL_REV (APP_MOTOR_GEAR_RATIO * \
                                          APP_ENCODER_LINES_PER_MOTOR_REV * \
                                          APP_ENCODER_QUADRATURE_FACTOR)
#define APP_MOTOR_SPEED_TEST_TARGET_MM_S 110.0f

/* The rescue task returns automatically when the motion test is disabled. */
#define APP_ENABLE_TASK                  (!APP_ENABLE_MOTION_TEST)

#if APP_ENABLE_MOTION_TEST && \
    (APP_ENABLE_AUTOMATIC_MOTOR_TEST || APP_ENABLE_SERVO_SWEEP_TEST)
#error "Motion test must be the only enabled application mode"
#endif

/*
 * Physical layout from the 2026-08-25 test photo:
 * M1 is the lower-left wheel toward the servos, M2 is upper-left, M3 is right.
 * Confirmed chassis axis: forward makes M1 negative, M2 stop and M3 positive.
 * Backward reverses M1/M3 while M2 remains stopped.
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

/* IMU heading hold used only by Motor_MoveAngle(). */
#define APP_MOTOR_HEADING_KP              4.0f
#define APP_MOTOR_HEADING_KI              0.02f
#define APP_MOTOR_HEADING_KD              0.0f
#define APP_MOTOR_HEADING_LIMIT_MM_S      150.0f
#define APP_MOTOR_HEADING_INTEGRAL_LIMIT  1500.0f
/* The installed chassis rotates opposite to the IMU positive Z direction. */
#define APP_MOTOR_HEADING_OUTPUT_SIGN    -1.0f

/* Non-blocking vector ramp used by Motor_MoveAngle(). */
#define APP_OMNI_ACCEL_MM_S2              2500.0f
#define APP_OMNI_DECEL_MM_S2              3000.0f
#define APP_OMNI_STOP_ANGLE_DEG           120.0f
#define APP_OMNI_ZERO_SPEED_MM_S          40
#define APP_OMNI_ZERO_CONFIRM_CYCLES      3U
#define APP_OMNI_ZERO_TIMEOUT_CYCLES      40U

/* Go_distance() accepts metres; internal speed, slowdown and tolerance use millimetres. */
#define APP_GO_DISTANCE_SPEED_MM_S       300.0f
#define APP_GO_DISTANCE_MIN_SPEED_MM_S   120.0f
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

/* IMU polls at 1 kHz; motor feedback runs at 10 ms; task remains at 20 ms. */
#define APP_IMU_UPDATE_PERIOD_MS         1U
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
#define APP_SEARCH_ROTATE_SPEED_MM_S     164.0f
#define APP_APPROACH_SPEED_MM_S          582.0f
#define APP_CRAB_MID_SPEED_MM_S          336.0f
#define APP_CRAB_SLOW_SPEED_MM_S         164.0f
#define APP_CRAB_ADJUST_SPEED_MM_S       134.0f
#define APP_CRAB_MID_DISTANCE_MM         500U
#define APP_CRAB_SLOW_DISTANCE_MM        250U
#define APP_CRAB_STOP_DISTANCE_MM        120U
#define APP_CRAB_CLOSE_WAIT_MS           500U
#define APP_CRAB_BACK_MS                 250U
#define APP_STEERING_DEAD_ZONE           4
#define APP_STEERING_KP_MM_S             2.24f
#define APP_STEERING_KD_MM_S             1.49f
#define APP_STEERING_LIMIT_MM_S          239.0f
#define APP_STEERING_DIRECTION           1.0f
#define APP_CAMERA_DIRECTION             1.0f

/* Upper-computer-guided return and local delivery sequence. */
#define APP_NAV_TIMEOUT_MS               200U
#define APP_RETURN_FORWARD_SPEED_MM_S    388.0f
#define APP_RETURN_BACKWARD_SPEED_MM_S   224.0f
#define APP_RETURN_TURN_SPEED_MM_S       179.0f
#define APP_DROP_FORWARD_DISTANCE_M      0.20f
#define APP_DROP_BACK_DISTANCE_M         0.50f
#define APP_DROP_RELEASE_WAIT_MS         600U
#define APP_CAMERA_SETTLE_MS             400U
#define APP_DROP_VERIFY_TIMEOUT_MS       3000U
#define APP_DROP_VERIFY_RETRIES          2U

/* This 1.8-inch 128x160 ST7735 panel exposes GRAM origin (0, 0). */
#define APP_LCD_WIDTH       128U
#define APP_LCD_HEIGHT      160U
#define APP_LCD_X_OFFSET    0U
#define APP_LCD_Y_OFFSET    0U

#define APP_MOTOR_KEY_DEBOUNCE_MS        30U

#endif
