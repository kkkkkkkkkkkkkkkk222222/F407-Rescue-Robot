#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Standalone test modes. Keep only one mode enabled at a time. */
#define APP_ENABLE_MOTION_TEST           0
#define APP_ENABLE_LOCATION_DEMO         0
#define APP_ENABLE_MOVE_SPIN_TEST        0
#define APP_ENABLE_AUTOMATIC_MOTOR_TEST  0
#define APP_ENABLE_SERVO_SWEEP_TEST      0
#define APP_ENABLE_TASK                  1

/* Non-blocking IMU angle turn used by Motor_TurnAngle(). */
#define APP_MOTOR_TURN_TOLERANCE_MDEG    1000L
#define APP_MOTOR_TURN_SLOWDOWN_MDEG     30000L
#define APP_MOTOR_TURN_FAST_MM_S         500.0f
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

/* Fixed-floor-direction translation plus simultaneous high-speed rotation. */
#define APP_MOVE_SPIN_TEST_SPEED_MM_S     350.0f
#define APP_MOVE_SPIN_TEST_ANGLE_DEG       90.0f
#define APP_MOVE_SPIN_TEST_YAW_MM_S       380.0f
#define APP_MOVE_SPIN_TEST_TIME_MS       5000U
#define APP_MOVE_SPIN_TEST_CONTROL_MS      10U
/* Location-assisted cross-track controller used by Motor_MoveSpin(). */
#define APP_MOVE_SPIN_LINE_KP               1.8f
#define APP_MOVE_SPIN_LINE_MAX_MM_S       120.0f
#define APP_MOVE_SPIN_ACCEL_MM_S2        1800.0f
#define APP_MOVE_SPIN_YAW_ACCEL_MM_S2    1950.0f
#define APP_MOVE_SPIN_WHEEL_MARGIN_MM_S    10.0f
#define APP_MOVE_SPIN_MAX_UPDATE_MS         50U
/* A spinning chassis changes all three wheel targets rapidly. Use a target-
 * proportional feed-forward instead of the normal fixed 450-PWM floor, then
 * phase-lead the field transform to cover the geared motor response delay. */
#define APP_MOVE_SPIN_FF_STATIC_PWM         220.0f
#define APP_MOVE_SPIN_FF_MAX_PWM            900.0f
#define APP_MOVE_SPIN_PID_BOOST                1.8f
#define APP_MOVE_SPIN_OUTPUT_LIMIT_PWM      1000
#define APP_MOVE_SPIN_HEADING_LEAD_MS         55.0f

#if (APP_ENABLE_MOTION_TEST || APP_ENABLE_LOCATION_DEMO || \
     APP_ENABLE_MOVE_SPIN_TEST) && \
    (APP_ENABLE_AUTOMATIC_MOTOR_TEST || APP_ENABLE_SERVO_SWEEP_TEST)
#error "Standalone test must be the only enabled application mode"
#endif

#if (APP_ENABLE_MOTION_TEST + APP_ENABLE_LOCATION_DEMO + \
     APP_ENABLE_MOVE_SPIN_TEST) > 1
#error "Only one motion/location standalone test can run"
#endif

/* Coarse field localization and start-zone-4 demonstration. */
#define APP_LOCATION_FIELD_HALF_MM       1500.0f
#define APP_LOCATION_START_CENTER_MM     1350.0f
/* Installed IMU Z yaw is negative for a positive (counter-clockwise) field
 * heading change, so localization must invert the measured yaw increment. */
#define APP_LOCATION_IMU_YAW_SIGN       -1.0f
#define APP_LOCATION_MAX_YAW_STEP_MDEG   30000LL
#define APP_LOCATION_DEMO_START_ZONE     4U
#define APP_LOCATION_DEMO_SPEED_MM_S     750.0f
#define APP_LOCATION_DEMO_TIME_MS        1500U
#define APP_LOCATION_DEMO_TIMEOUT_MS     2000U
/* Location-only route test: leave zone 4, visit the field centre, wait,
 * then drive into the blue material-placement half of the lower safe zone. */
#define APP_LOCATION_DEMO_MATERIAL_X_MM  0.0f
#define APP_LOCATION_DEMO_MATERIAL_Y_MM  0.0f
#define APP_LOCATION_DEMO_DROP_X_MM      150.0f
#define APP_LOCATION_DEMO_DROP_Y_MM     -1350.0f
#define APP_LOCATION_DEMO_TRAVEL_MAX_MM_S 750.0f
#define APP_LOCATION_DEMO_RETURN_MAX_MM_S 900.0f
#define APP_LOCATION_DEMO_TRAVEL_MIN_MM_S 140.0f
#define APP_LOCATION_DEMO_SLOWDOWN_MM    300.0f
#define APP_LOCATION_DEMO_TOLERANCE_MM   40.0f
#define APP_LOCATION_DEMO_CONFIRM_CYCLES 5U
#define APP_LOCATION_DEMO_BRAKE_WAIT_MS  150U
#define APP_LOCATION_DEMO_MOVE_TIMEOUT_MS 20000U
#define APP_LOCATION_DEMO_CONTROL_MS     20U

/* The upper computer requests recovery with EVENT=0x02. */
#define APP_RESCUE_REVERSE_SPEED_MM_S    850.0f
#define APP_RESCUE_REVERSE_TIME_MS       1000U
#define APP_RESCUE_COMMAND_TIMEOUT_MS     500U

/*
 * Physical layout from the 2026-08-25 test photo:
 * Current motor-port layout: M1 is the right wheel, M2 is the left wheel,
 * and M3 is the rear wheel. Forward makes M1 positive, M2 negative and M3
 * stationary; backward reverses M1/M2 while M3 remains stationary.
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

/* Camera tracking limits; fixed servo IDs and poses stay at their call sites. */
#define APP_CAMERA_MIN_ANGLE             0U
#define APP_CAMERA_MAX_ANGLE             170U

/* IMU polls at 1 kHz; motor feedback runs at 10 ms; task remains at 20 ms. */
#define APP_IMU_UPDATE_PERIOD_MS         1U
#define APP_MOTOR_CONTROL_PERIOD_MS      10U
#define APP_TASK_PERIOD_MS               20U
#define APP_TASK_STATUS_PERIOD_MS       200U
#define APP_MATCH_TIME_S                 180U
#define APP_MATCH_TIME_MS                (APP_MATCH_TIME_S * 1000U)
#define APP_START_REVERSE_SPEED_MM_S    850.0f
#define APP_START_REVERSE_TIME_MS      1000U
#define APP_START_BRAKE_WAIT_MS          150U
#define APP_START_TURN_DEG               180.0f
#define APP_START_SCAN_WAIT_MS          5000U
#define APP_TARGET_WAIT_MS               700U
#define APP_START_TIMEOUT_MS           20000U
#define APP_SEARCH_FULL_TURN_MDEG     360000U
#define APP_SEARCH_ADVANCE_DISTANCE_M     0.8f
#define APP_SEARCH_ADVANCE_SPEED_MM_S    750.0f
#define APP_GRAB_APPROACH_TIMEOUT_MS    8000U
#define APP_GRAB_MECHANISM_TIMEOUT_MS   3000U
#define APP_GRAB_TARGET_LOSS_GRACE_MS    250U
#define APP_RETURN_TIMEOUT_MS          30000U
#define APP_DROP_TOTAL_TIMEOUT_MS      20000U
#define APP_TASK_RESCUE_MAX_RETRIES       2U
#define APP_VISION_TIMEOUT_MS            250U
#define APP_CONFIG_CONFIRM_FRAMES        3U
#define APP_CARGO_CONFIRM_FRAMES         3U
#define APP_NAV_CONFIRM_FRAMES           3U
#define APP_DROP_CONFIRM_FRAMES          3U
#define APP_VISION_TARGET_X              640U
#define APP_VISION_TARGET_Y              512U
#define APP_VISION_MAX_X                1279U
#define APP_VISION_MAX_Y                1023U
#define APP_VISION_MIN_DISTANCE_MM       1U
#define APP_SEARCH_ROTATE_SPEED_MM_S     120.0f
#define APP_APPROACH_SPEED_MM_S          750.0f
#define APP_GRAB_MID_SPEED_MM_S          450.0f
#define APP_GRAB_SLOW_SPEED_MM_S         250.0f
#define APP_GRAB_RECOVERY_SPEED_MM_S     200.0f
#define APP_GRAB_MID_DISTANCE_MM         500U
#define APP_GRAB_SLOW_DISTANCE_MM        250U
#define APP_GRAB_CONFIRM_WAIT_MS          500U
#define APP_GRAB_RECOVERY_TIME_MS         250U
#define APP_STEERING_EXIT_DEAD_ZONE       8
#define APP_STEERING_ENTER_DEAD_ZONE     16
#define APP_STEERING_KP_MM_S             1.12f
/* Time-normalized equivalent of the former 0.745-per-frame derivative at
 * the nominal 40 ms vision period. */
#define APP_STEERING_KD_MM                0.0298f
#define APP_STEERING_MIN_MM_S             80.0f
#define APP_STEERING_LIMIT_MM_S          239.0f
#define APP_STEERING_DIRECTION           1.0f
#define APP_CAMERA_DEAD_ZONE              12
#define APP_CAMERA_KP_DEG_PER_PX          0.0175f
#define APP_CAMERA_KI_DEG_PER_PX_S        0.00625f
#define APP_CAMERA_KD_DEG_S_PER_PX        0.0012f
#define APP_CAMERA_INTEGRAL_LIMIT_PX_S  240.0f
#define APP_VISION_PID_DEFAULT_DT_S        0.040f
#define APP_VISION_PID_MIN_DT_S            0.020f
#define APP_VISION_PID_MAX_DT_S            0.100f
/* Upper-computer-guided return and local delivery sequence. */
#define APP_NAV_TIMEOUT_MS               200U
#define APP_RETURN_FORWARD_SPEED_MM_S    776.0f
#define APP_RETURN_BACKWARD_SPEED_MM_S   448.0f
#define APP_RETURN_TURN_SPEED_MM_S       358.0f
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
