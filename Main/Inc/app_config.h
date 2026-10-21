#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* The ordinary Debug build is temporarily the mapping firmware requested for
 * field tests.  NormalRun explicitly overrides this back to the rescue Task. */
#define APP_MODE_RESCUE_TASK 3
#define APP_MODE_MOTION_DEBUG_TASK 2
#define APP_MODE_CENTERING_TASK 1
#define APP_MODE_GAMEPAD_TASK 4
#ifndef APP_ACTIVE_MODE

#define APP_ACTIVE_MODE APP_MODE_RESCUE_TASK
//正常跑图

//#define APP_ACTIVE_MODE APP_MODE_MOTION_DEBUG_TASK
// 扫图

#endif
#if APP_ACTIVE_MODE < 1 || APP_ACTIVE_MODE > 4
#error "Invalid APP_ACTIVE_MODE"
#endif
#define APP_ENABLE_MOTION_DEBUG_TASK (APP_ACTIVE_MODE == APP_MODE_MOTION_DEBUG_TASK)
#define APP_ENABLE_GAMEPAD_TASK (APP_ACTIVE_MODE == APP_MODE_GAMEPAD_TASK)
#define APP_OMNI_LATERAL_API_SIGN -1.0f
#define APP_MOTION_DEBUG_STATUS_PERIOD_MS 50U
#define APP_MOTION_DEBUG_MAX_DISTANCE_MM 10000U
#define APP_MOTION_DEBUG_MIN_SPEED_MM_S 50.0f
#define APP_MOTION_DEBUG_MAX_SPEED_MM_S 700.0f
#define APP_MOTION_DEBUG_STRAIGHT_SPEED_MM_S 500.0f
#define APP_MOTION_DEBUG_MAX_TURN_RATE_DDEG_S 3600U
#define APP_MOTION_DEBUG_TURN_RADIUS_MM 130.0f
#define APP_MOTION_DEBUG_DISTANCE_TOLERANCE_MM 10.0f
#define APP_MOTION_DEBUG_SLOWDOWN_MM 150.0f
#define APP_MOTION_DEBUG_MIN_SLOW_SPEED_MM_S 80.0f
#define APP_MOTION_DEBUG_MOVE_TIMEOUT_MIN_MS 5000U
#define APP_MOTION_DEBUG_MOVE_TIMEOUT_MAX_MS 120000U
#define APP_MOTION_DEBUG_MAX_TURN_DDEG 32767
#define APP_GAMEPAD_COMMAND_TIMEOUT_MS 150U
#define APP_GAMEPAD_STATUS_PERIOD_MS 50U
#define APP_GAMEPAD_MAX_LINEAR_MM_S 700.0f
#define APP_GAMEPAD_MAX_YAW_MM_S 300.0f
#define APP_GAMEPAD_AXIS_DEAD_ZONE 5
#define APP_GAMEPAD_CAMERA_RATE_DEG_S 60.0f
#define APP_GAMEPAD_CAMERA_MIN_ANGLE 0.0f
#define APP_GAMEPAD_CAMERA_MAX_ANGLE 165.0f
#define APP_GAMEPAD_LIFT_RATE_DEG_S 45.0f
#define APP_GAMEPAD_LIFT_MIN_ANGLE 45.0f
#define APP_GAMEPAD_LIFT_MAX_ANGLE 120.0f
#define APP_GAMEPAD_CLAW_RATE_DEG_S 35.0f
#define APP_GAMEPAD_CLAW_LEFT_CLOSED_ANGLE ((float)APP_CLAW_LEFT_TOUCH_ANGLE)
#define APP_GAMEPAD_CLAW_RIGHT_CLOSED_ANGLE ((float)APP_CLAW_RIGHT_TOUCH_ANGLE)
#define APP_GAMEPAD_CLAW_LEFT_OPEN_ANGLE 108.0f
#define APP_GAMEPAD_CLAW_RIGHT_OPEN_ANGLE 72.0f
#define APP_ENABLE_MOTION_TEST           0
#define APP_ENABLE_LOCATION_DEMO         0
#define APP_ENABLE_MOVE_SPIN_TEST        0
#define APP_ENABLE_AUTOMATIC_MOTOR_TEST  0
#define APP_ENABLE_SERVO_SWEEP_TEST      0
#define APP_ENABLE_TASK (APP_ACTIVE_MODE == APP_MODE_RESCUE_TASK)
#define APP_ENABLE_CENTERING_TASK (APP_ACTIVE_MODE == APP_MODE_CENTERING_TASK)
/* Local stall/direction/no-progress interlocks are disabled by user request. */
#define APP_ENABLE_LOCAL_MOTOR_FAULT_STOP 0
/* USART1 (PCB serial port 2) text console. It can suspend TASK and position
 * one servo at a time without changing the USART3 RDK protocol. */
#define APP_ENABLE_RUNTIME_SERVO_DEBUG \
  (!APP_ENABLE_MOTION_DEBUG_TASK && !APP_ENABLE_GAMEPAD_TASK)

/* Non-blocking IMU angle turn used by Motor_TurnAngle(). */
#define APP_MOTOR_TURN_TOLERANCE_MDEG    1000L
#define APP_MOTOR_TURN_SLOWDOWN_MDEG     30000L
#define APP_MOTOR_TURN_FAST_MM_S         250.0f
#define APP_MOTOR_TURN_SLOW_MM_S         120.0f
#define APP_MOTOR_TURN_TIMEOUT_MS        10000U
#define APP_MOTOR_TURN_MAX_DEG           360.0f
#define APP_MOTOR_TURN_WRONG_WAY_MDEG    5000L
/* Trial angle controller: MotionDebug only until validated on the chassis. */
#define APP_DEBUG_TURN_POSITION_KP       2.5f
#define APP_DEBUG_TURN_RATE_KP           0.5f
#define APP_DEBUG_TURN_ACCEL_DPS2        120.0f
#define APP_DEBUG_TURN_DECEL_DPS2        120.0f
#define APP_DEBUG_TURN_MIN_RATE_DPS      6.0f
#define APP_DEBUG_TURN_TRIM_RATE_DPS     12.0f
#define APP_DEBUG_TURN_STATIC_PWM        70.0f
#define APP_DEBUG_TURN_STOP_RATE_MDPS    3000L
#define APP_DEBUG_TURN_SETTLE_MS         150U
#define APP_DEBUG_TURN_IMU_STALE_MS      100U
#define APP_DEBUG_TURN_OVERRUN_MDEG      8000L
#define APP_DEBUG_TURN_WRONG_WAY_MDEG    APP_MOTOR_TURN_WRONG_WAY_MDEG
#define APP_DEBUG_TURN_MAX_CORRECTIONS   2U
#define APP_DEBUG_TURN_ENTER_MDEG        300L
#define APP_DEBUG_TURN_EXIT_MDEG         500L
#define APP_DEBUG_TURN_BRAKE_DELAY_S     0.08f
#define APP_DEBUG_TURN_BRAKE_MARGIN_DPS  3.0f
#define APP_DEBUG_TURN_BRAKE_RELEASE     0.75f
#define APP_DEBUG_TURN_BOOST_WINDOW_MDEG 8000L
#define APP_DEBUG_TURN_BOOST_WAIT_MS     200U
#define APP_DEBUG_TURN_BOOST_PWM         130
#define APP_DEBUG_TURN_BOOST_MAX_MS      60U
#define APP_DEBUG_TURN_BOOST_TRAVEL_MDEG 150L
#define APP_DEBUG_TURN_BOOST_COOLDOWN_MS 200U
#define APP_DEBUG_TURN_BOOST_MAX_COUNT   3U
#define APP_DEBUG_CIRCLES_MAX_COUNT      10
#define APP_DEBUG_CIRCLES_MIN_RATE_DPS   60
#define APP_DEBUG_CIRCLES_MAX_RATE_DPS   120
#define APP_DEBUG_CIRCLES_DWELL_MS       1000U
#define APP_MOTION_TEST_SPEED_MM_S       750.0f
#define APP_MOTION_TEST_MOVE_ANGLE_DEG   180.0f
#define APP_MOTION_TEST_MOVE_TIME_MS     3000U
#define APP_MOTION_TEST_BRAKE_TIME_MS    100U
#define APP_MOTION_TEST_TURN_DEG         180.0f
#define APP_MOTION_TEST_REPEAT_COUNT     2U
#define APP_MOTION_TEST_TRANSITION_TIMEOUT_MS 2000U
#define APP_WHEEL_DIAMETER_MM            70U
#define APP_MOTOR_GEAR_RATIO             20.409f
#define APP_ENCODER_LINES_PER_MOTOR_REV  13U
#define APP_ENCODER_QUADRATURE_FACTOR    4U
#define APP_ENCODER_COUNTS_PER_WHEEL_REV (APP_MOTOR_GEAR_RATIO * \
                                          APP_ENCODER_LINES_PER_MOTOR_REV * \
                                          APP_ENCODER_QUADRATURE_FACTOR)
/* MG310 Hall, 70 mm wheels, battery-fed rail modeled as 12 V (not regulated). */
#define APP_MOTOR_RATED_VOLTAGE_MV        7400U
#define APP_MOTOR_SUPPLY_VOLTAGE_MV      12000U
#define APP_MOTOR_NO_LOAD_RPM             500.0f
#define APP_MOTOR_MAX_WHEEL_RPM           300.0f
/* Actual drive fraction; slow decay holds one input high and complements the other. */
#define APP_MOTOR_PWM_MAX_PERMILLE        600
#define APP_MOTOR_VOLTAGE_PWM_LIMIT \
  (APP_MOTOR_RATED_VOLTAGE_MV * 1000U / APP_MOTOR_SUPPLY_VOLTAGE_MV)
#if APP_MOTOR_SUPPLY_VOLTAGE_MV == 0U || APP_MOTOR_PWM_MAX_PERMILLE <= 0 || \
    APP_MOTOR_PWM_MAX_PERMILLE > 1000
#error "Invalid motor supply or PWM limit"
#endif
#if APP_MOTOR_PWM_MAX_PERMILLE > APP_MOTOR_VOLTAGE_PWM_LIMIT
#error "Motor PWM limit exceeds the configured rated/supply voltage ratio"
#endif
#define APP_MOTOR_SPEED_TEST_TARGET_MM_S 110.0f

/* Fixed-floor-direction translation plus simultaneous high-speed rotation. */
#define APP_MOVE_SPIN_TEST_SPEED_MM_S     350.0f
#define APP_MOVE_SPIN_TEST_ANGLE_DEG       90.0f
#define APP_MOVE_SPIN_TEST_TURN_DEG      180.0f
#define APP_MOVE_SPIN_TEST_RATE_DPS       60.0f
#define APP_MOVE_SPIN_TEST_TIME_MS      10000U
#define APP_MOVE_SPIN_TEST_CONTROL_MS      10U
/* Location-assisted cross-track controller used by Motor_MoveSpin(). */
#define APP_MOVE_SPIN_LINE_KP               1.0f
#define APP_MOVE_SPIN_LINE_MAX_MM_S        60.0f
#define APP_MOVE_SPIN_ACCEL_MM_S2         900.0f
#define APP_MOVE_SPIN_YAW_ACCEL_MM_S2     270.0f
#define APP_MOVE_SPIN_WHEEL_MARGIN_MM_S    10.0f
#define APP_MOVE_SPIN_MAX_UPDATE_MS         50U
/* Both paths use voltage-scaled feed-forward and the same hard PWM cap. */
#define APP_MOVE_SPIN_FF_STATIC_PWM          APP_MOTOR_BASE_PWM
/* MG310 uses the same speed-loop correction scale in combined motion. */
#define APP_MOVE_SPIN_PID_BOOST               1.0f
#define APP_MOVE_SPIN_OUTPUT_LIMIT_PWM APP_MOTOR_PWM_MAX_PERMILLE
#define APP_MOVE_SPIN_HEADING_LEAD_MS          0.0f
#define APP_MOVE_SPIN_MAX_LINEAR_MM_S       500.0f
#define APP_MOVE_SPIN_MAX_RATE_DPS           90.0f
#define APP_MOVE_SPIN_ANGLE_KP                2.5f
#define APP_MOVE_SPIN_DECEL_DPS2            120.0f
#define APP_MOVE_SPIN_ANGLE_TOL_DEG           1.0f
#define APP_MOVE_SPIN_SETTLE_MS              150U

#if (APP_ENABLE_MOTION_TEST + APP_ENABLE_LOCATION_DEMO + \
     APP_ENABLE_MOVE_SPIN_TEST + APP_ENABLE_AUTOMATIC_MOTOR_TEST + \
     APP_ENABLE_SERVO_SWEEP_TEST + APP_ENABLE_TASK + \
     APP_ENABLE_CENTERING_TASK + APP_ENABLE_MOTION_DEBUG_TASK) > 1
#error "Only one application mode can run"
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
 * and M3 is the rear wheel. Gripper-forward makes M1 negative, M2 positive and M3
 * stationary; backward reverses M1/M2 while M3 remains stationary.
 * Change signs after checking the real installation.
 */
/* Scheme A: retain per-wheel drive/feedback polarity. Calibrate the forward
 * axis and raw-IMU heading hold separately; do not apply Scheme B sign flips. */
#define APP_OMNI_M1_MOTOR_SIGN           1
#define APP_OMNI_M2_MOTOR_SIGN           1
#define APP_OMNI_M3_MOTOR_SIGN           1
#define APP_OMNI_M1_ENCODER_SIGN        -1
#define APP_OMNI_M2_ENCODER_SIGN        -1
#define APP_OMNI_M3_ENCODER_SIGN        -1
/* Physical gripper-forward was measured opposite to the base wheel X axis.
 * Apply the same polarity in the mixer, distance progress and odometry. */
#define APP_OMNI_FORWARD_SIGN           -1
#if APP_OMNI_FORWARD_SIGN != 1 && APP_OMNI_FORWARD_SIGN != -1
#error "Forward calibration sign must be +1 or -1"
#endif

/* Three-motor speed loop, updated every 10 ms. */
#define APP_MOTOR_ZERO_ENTER_MM_S          3.0f
#define APP_MOTOR_ZERO_EXIT_MM_S           6.0f
#define APP_MOTOR_MAX_COUNT_10MS \
  (APP_ENCODER_COUNTS_PER_WHEEL_REV * APP_MOTOR_MAX_WHEEL_RPM / 60.0f * \
   APP_MOTOR_CONTROL_PERIOD_MS / 1000.0f)
/* Static friction compensation, not a fixed PWM floor at every speed. */
#define APP_MOTOR_BASE_PWM               100
#define APP_MOTOR_SPEED_KP               5.0f
#define APP_MOTOR_SPEED_KI               0.12f
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

/* IMU heading hold shared by Motor_MoveAngle() and Motor_MoveDistance(). */
/* Heading-hold trial is separate from the shared wheel-speed PI tuning. */
#define APP_MOTOR_HEADING_KP              2.2f
#define APP_MOTOR_HEADING_KI              0.005f
#define APP_MOTOR_HEADING_KD              0.0f
#define APP_MOTOR_HEADING_LIMIT_MM_S      150.0f
#define APP_MOTOR_HEADING_INTEGRAL_LIMIT  1500.0f
/* Measured: positive wheel yaw increases raw IMU yaw. With reference-current
 * error, positive PID polarity is required for negative feedback. */
#define APP_MOTOR_HEADING_OUTPUT_SIGN     1.0f
/* Location/teleop yaw is positive counter-clockwise; wheel yaw follows raw
 * IMU polarity. Convert at mode boundaries, not again inside the mixer. */
#define APP_OMNI_FIELD_YAW_TO_DRIVE_SIGN \
  (APP_LOCATION_IMU_YAW_SIGN * APP_MOTOR_HEADING_OUTPUT_SIGN)

/* Non-blocking vector ramp used by Motor_MoveAngle(). */
#define APP_OMNI_ACCEL_MM_S2              2500.0f
#define APP_OMNI_DECEL_MM_S2              3000.0f
#define APP_OMNI_STOP_ANGLE_DEG           120.0f
#define APP_OMNI_ZERO_SPEED_MM_S          40
#define APP_OMNI_ZERO_CONFIRM_CYCLES      3U
#define APP_OMNI_ZERO_TIMEOUT_CYCLES      40U

/* Motor_MoveDistance() accepts metres, holds its starting IMU yaw, and uses
 * millimetres internally for speed, slowdown and tolerance. */
#define APP_GO_DISTANCE_SPEED_MM_S       300.0f
#define APP_GO_DISTANCE_MIN_SPEED_MM_S   120.0f
#define APP_GO_DISTANCE_ACCEL_MM_S2     3000.0f
#define APP_GO_DISTANCE_DECEL_MM_S2     3500.0f
#define APP_GO_DISTANCE_SLOWDOWN_MM      100.0f
#define APP_GO_DISTANCE_TOLERANCE_MM     3.0f
#define APP_GO_DISTANCE_PROGRESS_MM      0.25f
#define APP_GO_DISTANCE_NO_PROGRESS_CYCLES 100U

/* Camera tracking limits; fixed servo IDs and poses stay at their call sites. */
#define APP_CAMERA_MIN_ANGLE             0U
#define APP_CAMERA_MAX_ANGLE             170U

/* IMU polls at 1 kHz; motor feedback runs at 10 ms; task remains at 20 ms. */
#define APP_IMU_UPDATE_PERIOD_MS         1U
/* UART single-axis module on USART2 PD5(TX)/PD6(RX). Set 0 for IMU660RC SPI. */
#ifndef APP_IMU_USE_UART
#define APP_IMU_USE_UART                 1
#endif
#ifndef APP_IMU_UART_YAW_SIGN
#define APP_IMU_UART_YAW_SIGN            1
#endif
#if APP_IMU_UART_YAW_SIGN != 1 && APP_IMU_UART_YAW_SIGN != -1
#error "UART IMU yaw sign must be +1 or -1 (applied to angle AND rate)"
#endif
#define APP_IMU_UART_BAUD                115200U
#define APP_IMU_UART_START_TIMEOUT_MS    2000U
#define APP_IMU_UART_STALE_MS            150U
/* Below half a turn even at 500 dps (module rated range: +/-400 dps). */
#define APP_IMU_UART_UNWRAP_MAX_GAP_MS   300U
#define APP_IMU_UART_RECOVERY_SAMPLES    3U
#define APP_IMU_UART_AUTO_RATE           1
/* Identity until measured against a known external rotation, not TURN output. */
#define APP_IMU_GYRO_Z_SCALE_PPM         1000000L
#if APP_IMU_GYRO_Z_SCALE_PPM < 900000L || APP_IMU_GYRO_Z_SCALE_PPM > 1100000L
#error "IMU gyro scale must be within +/-10 percent"
#endif
#define APP_MOTOR_CONTROL_PERIOD_MS      10U
#define APP_TASK_PERIOD_MS               20U
#define APP_COMMAND_RX_TIMEOUT_MS         500U
#define APP_VISUAL_UPDATE_TIMEOUT_MS      500U
#define APP_TASK_STATUS_PERIOD_MS        50U
#define APP_MATCH_TIME_S                 180U
#define APP_MATCH_TIME_MS                (APP_MATCH_TIME_S * 1000U)
#define APP_START_FORWARD_DISTANCE_M    1.10f
#define APP_START_CLEARANCE_DISTANCE_M   0.60f
#define APP_START_CLEARANCE_SPEED_MM_S  600.0f
#define APP_START_FORWARD_SPEED_MM_S    800.0f
#define APP_START_TURN_SPEED_MM_S       250.0f
#define APP_START_SETTLE_MS              250U
#define APP_START_TURN_DEG               180.0f
#define APP_START_TIMEOUT_MS           30000U
#define APP_SEARCH_LOW_CAMERA_ANGLE      105U
#define APP_SEARCH_HIGH_CAMERA_ANGLE     120U
#define APP_SEARCH_FULL_TURN_MDEG     360000U
#define APP_CAMERA_SCAN_STEP_DEG            1U
#define APP_CAMERA_SCAN_STEP_MS            20U
#define APP_CAMERA_SCAN_ENDPOINT_HOLD_MS  300U
#define APP_VISION_TIMEOUT_MS            250U
#define APP_CONFIG_CONFIRM_FRAMES        1U
#define APP_VISION_TARGET_X              640U
#define APP_VISION_TARGET_Y              512U
#define APP_VISION_MAX_X                1279U
#define APP_VISION_MAX_Y                1023U
#define APP_SEARCH_ROTATE_SPEED_MM_S     200.0f
#define APP_SEARCH_WAIT_RETURN_SPEED_MM_S 200.0f
#define APP_RETURN_TURN_SPEED_MM_S       200.0f
#define APP_RETURN_CENTER_ARRIVAL_MM      80U
#define APP_APPROACH_SPEED_MM_S          350.0f
#define APP_GRAB_MID_SPEED_MM_S          350.0f
#define APP_GRAB_SLOW_SPEED_MM_S         125.0f
#define APP_GRAB_MID_DISTANCE_MM         500U
#define APP_GRAB_SLOW_DISTANCE_MM        250U
#define APP_GRAB_ALIGN_CAMERA_ANGLE      125U
#define APP_GRAB_ALIGN_X_ERROR_PX         24.0f
#define APP_GRAB_CAMERA_SETTLE_MS          250U
#define APP_GRAB_WATCH_CRAWL_SPEED_MM_S  200.0f
#define APP_CAPTURE_FORWARD_BUDGET_MM    200U
#define APP_APPROACH_ALIGN_TIMEOUT_MS   6000U
#define APP_APPROACH_RANGE_WAIT_MS      5000U
#define APP_APPROACH_RANGE_TIMEOUT_MS  12000U
#define APP_APPROACH_RANGE_MAX_MM       2500U
#define APP_APPROACH_RANGE_SPEED_MM_S   500.0f
#define APP_APPROACH_RANGE_END_MM_S     200.0f
#define APP_APPROACH_RANGE_SLOW_MM      250.0f
#define APP_APPROACH_LINK_RECOVER_MS    2000U
#define APP_AUDIT_ACCUMULATE_WINDOW_MS   500U
#define APP_CAPTURE_DECISION_WAIT_MS    4000U
#define APP_CAPTURE_ESCAPE_DISTANCE_MM  300U
#define APP_CAPTURE_ESCAPE_SPEED_MM_S   300.0f
#define APP_CAPTURE_ESCAPE_TURN_DEG      90.0f
#define APP_SEPARATION_MAX_ATTEMPTS        2U
#define APP_SEPARATION_RELAX_STEP_DEG      0U
#define APP_CORE_GRAB_ADVANCE_DISTANCE_MM  50U
#define APP_CORE_GRAB_ADVANCE_SPEED_MM_S 180.0f
#define APP_POST_GRAB_AUDIT_WINDOW_MS     3500U
#define APP_POST_GRAB_CAMERA_NUDGE_MS      400U
#define APP_POST_GRAB_DECISION_WAIT_MS    4000U
#define APP_POST_GRAB_CAMERA_NUDGE_DEG        2U
#define APP_INJURY_CLAW_RELAX_DEG             5U
#define APP_INJURY_CLAW_SETTLE_MS           600U
#define APP_INJURY_BACKOFF_TIME_MS         2000U
#define APP_INJURY_BACKOFF_SPEED_MM_S      150.0f
#define APP_AUDIT_SAMPLE_INTERVAL_MS       100U
#define APP_AUDIT_REQUIRED_SAMPLES           2U
#define APP_AUDIT_EVIDENCE_MAX_GAP_MS       500U
#define APP_AUDIT_STATE_TIMEOUT_MS       15000U
#define APP_DISPERSE_APPROACH_ANGLE       130U
#define APP_DISPERSE_APPROACH_SPEED_MM_S 300.0f
#define APP_DISPERSE_ALIGN_X_ERROR_PX      40.0f
#define APP_DISPERSE_CAPTURE_ANGLE         140U
#define APP_DISPERSE_CAPTURE_SPEED_MM_S   200.0f
#define APP_APPROACH_LOSS_HOLD_MS         500U
#define APP_STEERING_EXIT_DEAD_ZONE       8
#define APP_STEERING_ENTER_DEAD_ZONE     16
#define APP_STEERING_KP_MM_S             0.18f
/* Derivative uses seconds, not frames; softened together with visual P. */
#define APP_STEERING_KD_MM                0.0025f
#define APP_STEERING_MIN_MM_S             10.0f
#define APP_STEERING_LIMIT_MM_S           90.0f
#define APP_STEERING_RATE_MM_S2          400.0f
#define APP_STEERING_DIRECTION           1.0f
#define APP_CAMERA_DEAD_ZONE              12
#define APP_CAMERA_KP_DEG_PER_PX          0.55f
#define APP_CAMERA_KI_DEG_PER_PX_S        0.00875f
#define APP_CAMERA_KD_DEG_S_PER_PX        0.00168f
#define APP_CAMERA_INTEGRAL_LIMIT_PX_S  240.0f
#define APP_VISION_COORD_FILTER_ALPHA      0.55f
#define APP_APPROACH_FULL_SPEED_ERROR_PX    48.0f
#define APP_APPROACH_SLOW_SPEED_ERROR_PX   160.0f
#define APP_APPROACH_ALIGN_MIN_SPEED_MM_S  150.0f
#define APP_CAMERA_TRACK_MAX_RATE_DEG_S   45.0f
#define APP_VISION_PID_DEFAULT_DT_S        0.040f
#define APP_VISION_PID_MIN_DT_S            0.020f
#define APP_VISION_PID_MAX_DT_S            0.300f
#define APP_CENTERING_CAMERA_MIN_ANGLE       0U
#define APP_CENTERING_CAMERA_MAX_ANGLE     165U
#define APP_CENTERING_CAMERA_START_ANGLE    90U
#define APP_CENTERING_CAMERA_STEP_LIMIT_DEG  6.0f

/* Native-resolution mission flow shared with shijue_fangan/mission_test. */
#define APP_GRAB_VIEW_ANGLE              140U
#define APP_GRAB_INITIAL_OBSERVE_MS      500U
#define APP_GRAB_CAMERA_RAISE_STEP_DEG    10U
#define APP_GRAB_CAMERA_MIN_ANGLE         90U
#define APP_GRAB_RAISE_OBSERVE_MS       1000U
#define APP_GRAB_SCAN_ROTATE_MM_S        100.0f
#define APP_NAV_FAST_SPEED_MM_S          800.0f
#define APP_NAV_FINAL_APPROACH_DISTANCE_MM 300U
#define APP_NAV_FINAL_APPROACH_SPEED_MM_S 400.0f
#define APP_NAV_LINEAR_SLOWDOWN_MM       300.0f
#define APP_NAV_END_SPEED_RATIO            0.2f
#define APP_NAV_MIN_END_SPEED_MM_S        120.0f
#define APP_NAV_REMOTE_STOP_DISTANCE_MM      0U
#define APP_NAV_REMOTE_RESUME_DISTANCE_MM   50U
#define APP_NAV_TERMINAL_WINDOW_MM           20U
#define APP_NAV_TERMINAL_STABLE_MS          300U
#define APP_NAV_TERMINAL_TIMEOUT_MS        1500U
#define APP_NAV_TERMINAL_RELEASE_MM          50U
#define APP_NAV_STAGE_NEAR_DISTANCE_MM        30U
#define APP_NAV_STAGE_OVERRUN_MARGIN_MM      100U
#define APP_NAV_HEADING_LOCK_DISTANCE_MM    50U
#define APP_NAV_HEADING_UNLOCK_DISTANCE_MM 100U
#define APP_SAFE_ZONE_RED_HEADING_DEG       90.0f
#define APP_SAFE_ZONE_BLUE_HEADING_DEG     270.0f
#define APP_NAV_REMOTE_MAX_DISTANCE_MM     5000U
#define APP_NAV_PROGRESS_DISTANCE_MM           2
#define APP_NAV_PROGRESS_PATH_MM               5U
#define APP_NAV_PROGRESS_HEADING_MDEG        1000
#define APP_NAV_PROGRESS_TIMEOUT_MS          500U
#define APP_NAV_HEADING_KP_MM_S_PER_DEG     20.0f
#define APP_NAV_HEADING_MAX_MM_S            300.0f
#define APP_NAV_SPEED_ACCEL_MM_S2           1800.0f
#define APP_NAV_SPEED_DECEL_MM_S2           3000.0f
#define APP_NAV_YAW_ACCEL_MM_S2             1800.0f
#define APP_NAV_HEADING_TOLERANCE_DEG      1.0f
#define APP_NAV_FINAL_TURN_TOLERANCE_DEG   1.5f
#define APP_NAV_FINAL_PUSH_SPEED_MM_S    200.0f
#define APP_NAV_FINAL_PUSH_MIN_TIME_MS   1500U
#define APP_NAV_FINAL_PUSH_TIME_MS       2000U
#define APP_NAV_FINAL_PUSH_MAX_DISTANCE_MM 440U
#define APP_NAV_REALIGN_DEG                6.0f
#define APP_NAV_FINAL_REALIGN_DEG          3.0f
#define APP_NAV_FINAL_REALIGN_HOLD_MS      150U
#define APP_NAV_TURN_SETTLE_MS            100U
/* Updated safe-zone flow: stop at the 0.60 m staging point, align once from
 * pose and once from a frozen image, then approach the fence while holding
 * the latched heading. The upper computer sends signed pixel error; the
 * equivalent focal length and sign are kept here for field calibration. */
#define APP_SAFE_ALIGN_TOLERANCE_DEG        3.0f
#define APP_SAFE_ALIGN_HOLD_TOLERANCE_DEG   6.0f
#define APP_SAFE_ALIGN_CAMERA_ANGLE           120U
#define APP_SAFE_ALIGN_CAMERA_SETTLE_MS       300U
#define APP_SAFE_VISUAL_FOCAL_LENGTH_PX   640.0f
#define APP_SAFE_VISUAL_YAW_SIGN           -1.0f
#define APP_SAFE_VISUAL_MAX_CORRECTION_DEG 15.0f
#define APP_SAFE_ENTER_CRUISE_SPEED_MM_S   400.0f
#define APP_SAFE_ENTER_MIN_SPEED_MM_S      200.0f
#define APP_SAFE_ENTER_SLOWDOWN_MM         300.0f
#define APP_SAFE_ENTER_TOTAL_DISTANCE_MM       650U
#define APP_SAFE_ENTER_CONTACT_DISTANCE_MM 113U
#define APP_SAFE_FINAL_PUSH_DISTANCE_MM    250U
#define APP_SAFE_FINAL_PUSH_SPEED_MM_S     330.0f
#define APP_SAFE_FINAL_PUSH_TIMEOUT_MS     1500U
/* Visual CLEAR: P2/P3=0, +/-200 means a 90-deg turn and forward placement.
 * Only legacy fixed-distance CLEAR retains the +/-150 lateral stroke. */
#define APP_SAFE_SWEEP_LATERAL_MM            150
#define APP_SAFE_SWEEP_PLACEMENT_MM          200
#define APP_SAFE_SWEEP_RETRIEVE_BUDGET_MM     200U
#define APP_SAFE_SWEEP_RETRIEVE_SEARCH_MM_S    80.0f
#define APP_SAFE_SWEEP_RETRIEVE_TIMEOUT_MS  15000U
#define APP_SAFE_SWEEP_OBSERVE_TIMEOUT_MS    3500U
#define APP_SAFE_SWEEP_TRACE_CAPACITY         256U
#define APP_SAFE_SWEEP_TRACE_STEP_MM          10.0f
#define APP_SAFE_SWEEP_SEGMENT_MERGE_MM         8.0f
#define APP_SAFE_SWEEP_SEGMENT_MERGE_DEG        8.0f
#define APP_SAFE_SWEEP_SEGMENT_END_MM          10.0f
#define APP_SAFE_SWEEP_SEGMENT_CROSS_MM        20.0f
#define APP_SAFE_SWEEP_SEGMENT_ALIGN_DEG        5.0f
#define APP_SAFE_SWEEP_SEGMENT_LOOKAHEAD_MM   120.0f
#define APP_SAFE_SWEEP_MIN_FORWARD_MM          80
#define APP_SAFE_SWEEP_MAX_FORWARD_MM         600
#define APP_SAFE_SWEEP_LATERAL_SPEED_MM_S   350.0f
#define APP_SAFE_SWEEP_FORWARD_SPEED_MM_S   300.0f
#define APP_SAFE_SWEEP_RETURN_SPEED_MM_S    350.0f
#define APP_SAFE_SWEEP_MODE39_HOLD_MS         500U
#define APP_SAFE_SWEEP_SEARCH_SPEED_MM_S      160.0f
#define APP_SAFE_SWEEP_APPROACH_SPEED_MM_S    250.0f
#define APP_SAFE_SWEEP_RETURN_TOLERANCE_MM     30.0f
#define APP_SAFE_SWEEP_RETURN_REALIGN_DEG       8.0f
#define APP_SAFE_SWEEP_AUDIT_TIMEOUT_MS       3500U
/* Normal autonomous motion turns back at 30 cm from the 3 m x 3 m map edge.
 * Accepted fixed-distance ENTER alone is exempt from map-edge recovery. */
#define APP_FIELD_EDGE_ABORT_MARGIN_MM        50.0f
#define APP_FIELD_EDGE_REARM_MARGIN_MM       400.0f
#define APP_BOUNDARY_TURN_TOLERANCE_DEG        5.0f
#define APP_BOUNDARY_RECOVERY_SPEED_MM_S     300.0f
#define APP_POSE_WAIT_TIMEOUT_MS          1500U
#define APP_LIFT_START_ANGLE              55U
#define APP_LIFT_TRAVEL_ANGLE             85U
#define APP_DELIVERY_CAMERA_ANGLE         120U
#define APP_DELIVERY_CAMERA_SETTLE_MS     300U
#define APP_DELIVERY_VERIFY_WAIT_MS       1000U
#define APP_DELIVERY_EXIT_DISTANCE_M       0.30f
#define APP_DELIVERY_EXIT_SPEED_MM_S      400.0f
#define APP_DELIVERY_EXIT_LIFT_SETTLE_MS   1000U
#define APP_RETURN_CENTER_SPEED_MM_S       800.0f
#define APP_RETURN_CENTER_HEADING_MAX_MM_S 100.0f
#define APP_RETURN_CENTER_TURN_TOLERANCE_DEG 3.0f
#define APP_RETURN_CENTER_REALIGN_DEG       8.0f
/* After RETURN reaches D=0, briefly ACK repeated RETURN frames in SEARCH.
 * This lets an older upper computer observe a post-return ACK even when its
 * 8-bit sequence has wrapped back to the value saved at RETURN entry. */
#define APP_RETURN_SEARCH_ACK_GRACE_MS    1500U
#define APP_STASH_RETURN_BACKOFF_M           0.35f
#define APP_STASH_RETURN_BACKOFF_SPEED_MM_S 650.0f
#define APP_REMOTE_YIELD_SPEED_MM_S          750.0f
#define APP_CLAW_LEFT_TOUCH_ANGLE                76U
#define APP_CLAW_RIGHT_TOUCH_ANGLE              104U
#define APP_CLAW_SEPARATE_SINGLE_CLOSE_DEG        10U
#define APP_CLAW_SEPARATE_MULTI_CLOSE_DEG         10U
#define APP_CARGO_SEPARATE_ARC_BACK_MM_S        300.0f
#define APP_CARGO_SEPARATE_ARC_SIDE_MM_S          0.0f
#define APP_CARGO_SEPARATE_ARC_YAW_MM_S         150.0f
#define APP_CARGO_SEPARATE_ARC_MIN_MM             60U
#define APP_CARGO_SEPARATE_ARC_MAX_MM            250U
#define APP_CARGO_SEPARATE_ARC_RETRY_STEP_MM      50U
#define APP_CARGO_SEPARATE_SIDE_BUDGET_MM       120.0f
#define APP_CARGO_SEPARATE_PREPULL_MM             60U
#define APP_CARGO_SEPARATE_TAIL_MM                70U
#define APP_CARGO_SEPARATE_NO_PROGRESS_MS       1500U
#define APP_CARGO_SEPARATE_TOTAL_TIMEOUT_MS     8000U
#define APP_CARGO_SEPARATE_SIDE_BRAKE_MARGIN_MM   5.0f
#define APP_CARGO_SEPARATE_TURN_FIRST_DEG        30.0f
#define APP_CARGO_SEPARATE_TURN_STEP_DEG         15.0f
#define APP_CARGO_SEPARATE_TURN_LIMIT_DEG        47.0f
#define APP_CARGO_SEPARATE_BACK_SPEED_MM_S      500.0f
#define APP_SELECTIVE_DISPERSE_CURVE_MM          430U
#define APP_CARGO_RECHECK_SETTLE_MS             300U
#define APP_REMOTE_LANE_SPEED_MM_S           850.0f
#define APP_REMOTE_DISPERSE_TIMEOUT_MS          18000U
#define APP_REMOTE_DISPERSE_HOLD_CANCEL_MS       1000U
#define APP_DISPERSE_OBSERVE_TURN_DEG              20.0f
#define APP_FIRST_GREEN_BUMP_BACKOFF_M              0.10f
#define APP_FIRST_GREEN_BUMP_FORWARD_M              0.20f
#define APP_FIRST_GREEN_BUMP_SPEED_MM_S           500.0f
#define APP_REMOTE_ACTION_TIMEOUT_MS          7000U
#define APP_STASH_ROUTE_HOLD_ACCEPT_MM          100U

/* This 1.8-inch 128x160 ST7735 panel exposes GRAM origin (0, 0). */
#define APP_LCD_WIDTH       128U
#define APP_LCD_HEIGHT      160U
#define APP_LCD_X_OFFSET    0U
#define APP_LCD_Y_OFFSET    0U
#define APP_MOTOR_KEY_DEBOUNCE_MS        30U

#endif
