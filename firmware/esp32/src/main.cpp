#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/string.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <I2Cdev.h>
#include <MPU6050.h>

// ─── HARDWARE CONSTANTS ───
#define WHEEL_RADIUS      0.051f
#define WHEEL_BASE        0.3f
#define TICKS_PER_REV     1664
#define METERS_PER_TICK   (2.0f * M_PI * WHEEL_RADIUS / TICKS_PER_REV)
#define RADS_PER_TICK     (2.0f * M_PI / TICKS_PER_REV)

// ─── PID CONSTANTS (starter values — tune on the bench) ───
// Kp: PWM per (rad/s) error.  Too low → sluggish; too high → oscillation.
// Ki: integrates out steady-state drag/friction offset.
// Kd: set to 0 first; only raise if Kp+Ki loop is stable and you need faster settling.
// INT_LIMIT: caps |Ki·integral| at ±100 PWM so the integrator can't saturate the output.
// TASK_DT:   must match the vTaskDelay period at the bottom of motor_task (10 ms).
#define PID_KP          50.0f
#define PID_KI          20.0f
#define PID_KD           0.5f
#define PID_INT_LIMIT  100.0f
#define MOTOR_TASK_DT    0.01f

// Defined here (before any function signatures) so Arduino auto-prototyping
// doesn't generate a forward declaration for pid_compute before the type exists.
struct PIDState { float integral; float prev_error; };

// ─── PIN DEFINITIONS ───
#define MOTOR_L_PWM   25
#define MOTOR_L_DIR   26
#define MOTOR_R_PWM   27
#define MOTOR_R_DIR   14
#define ENC_L_A       4
#define ENC_L_B       13
#define ENC_R_A       32
#define ENC_R_B       33

// HC-SR04 ultrasonic — front-facing object detection.
// NOTE: GPIO34/35 are input-only on the ESP32 (no output driver), so TRIG
// cannot physically be wired to 35 as originally planned. TRIG moved to the
// free GPIO23; ECHO stays on 34 since input-only is fine for an input pin.
#define ULTRASONIC_TRIG_PIN   23
#define ULTRASONIC_ECHO_PIN   34

// ─── ARM CONSTANTS — placeholders, all TODOs below need real hardware values ───
// Mechanism: one linear actuator lifts the scoop, one servo rotates at the
// top to dump the load backward, then both retract/re-home.
// TODO(partner): confirm actuator driver type. Assumed here: a PWM+DIR
// H-bridge, same pattern as the drive motors (Cytron-style). If it's a
// relay-reversing or L298N-style driver the DIR polarity below may need to
// flip, and if it has built-in limit switches wire them to ARM_ACT_LIMIT_*.
#define ARM_ACT_PWM           16   // linear actuator drive PWM
#define ARM_ACT_DIR           17   // linear actuator direction (HIGH = extend/lift, TODO confirm)
#define ARM_ACT_LIMIT_TOP     18   // optional top limit switch, INPUT_PULLUP, active LOW
#define ARM_ACT_LIMIT_BOTTOM   5   // optional bottom limit switch, INPUT_PULLUP, active LOW

// One shared I2C bus carries both the dump-gate servo driver (PCA9685) and
// the MPU-6050 IMU — this board wires SDA to GPIO21 and SCL to GPIO22.
#define I2C_SDA_PIN               21
#define I2C_SCL_PIN               22

// Dump-gate servo is on a PCA9685 PWM driver board, not a direct GPIO — leaves
// GPIO pins free and gives the servo its own dedicated ~50Hz PWM generator
// instead of sharing an ESP32 LEDC channel with other PWM users.
#define PCA9685_I2C_ADDR       0x40
#define PCA9685_PWM_FREQ_HZ      50   // hobby servos expect ~50Hz
#define ARM_SERVO_CHANNEL         0   // PCA9685 output channel driving the dump-gate servo

// MPU-6050 terrain-safety IMU — see terrain_monitor.py (STOP_IMU channel).
// Scale factors are for the library's power-on defaults: ±2g accel, ±250°/s gyro.
#define MPU6050_I2C_ADDR       0x68   // AD0 tied LOW; 0x69 if tied HIGH
#define IMU_ACCEL_LSB_PER_G  16384.0f
#define IMU_GYRO_LSB_PER_DPS   131.0f
#define GRAVITY_MSS              9.81f
#define IMU_PUBLISH_INTERVAL_MS   10   // 100Hz, matches terrain_monitor's calibration window

#define ARM_ACT_SPEED        200   // 0-255 PWM — placeholder, tune once actuator specs are known
// TODO(partner): these two are a time-based fallback for actuators with no
// limit switches/position feedback wired yet. Once ARM_ACT_LIMIT_TOP/BOTTOM
// are physically wired, they end the stroke early and these just become a
// timeout safety net — measure real travel time and set them a bit above it.
#define ARM_LIFT_TIME_MS     3000
#define ARM_LOWER_TIME_MS    3000

#define ARM_SERVO_HOME_DEG      0   // TODO(partner): calibrate against the real dump-gate geometry
#define ARM_SERVO_DUMP_DEG    120
#define ARM_SERVO_PULSE_MIN_US 500  // pulse width at 0 degrees — TODO(partner): tune to the servo's real range
#define ARM_SERVO_PULSE_MAX_US 2400 // pulse width at 180 degrees
#define ARM_SERVO_MOVE_MS     600   // time to let the servo finish travelling before continuing
#define ARM_SERVO_HOLD_MS     500   // how long to hold at the dump angle before returning home

// ─── ULTRASONIC CONSTANTS ───
// HC-SR04 datasheet range is 2cm-4m with a ~15° detection cone.
#define ULTRASONIC_TIMEOUT_US       30000   // ~5m round trip @343m/s; abandon the read if no echo
#define ULTRASONIC_MIN_RANGE_M         0.02f
#define ULTRASONIC_MAX_RANGE_M         4.0f
#define ULTRASONIC_FIELD_OF_VIEW_RAD   0.26f   // ~15 degrees
#define ULTRASONIC_DETECT_THRESHOLD_CM 30.0f   // "object right in front" cutoff — tune on the bench
#define ULTRASONIC_READ_INTERVAL_MS   100      // 10Hz

// ─── micro-ROS objects ───
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t cmd_vel_sub;
rcl_subscription_t estop_sub;
rcl_subscription_t arm_cmd_sub;
rcl_publisher_t odom_pub;
rcl_publisher_t motor_events_pub;
rcl_publisher_t arm_events_pub;
rcl_publisher_t imu_pub;
rcl_publisher_t ultrasonic_pub;
rcl_publisher_t obstacle_events_pub;

geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__String estop_msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__String motor_event_msg;
std_msgs__msg__String arm_cmd_msg;
std_msgs__msg__String arm_event_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Range range_msg;
std_msgs__msg__String obstacle_event_msg;

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA9685_I2C_ADDR);
MPU6050 imu(MPU6050_I2C_ADDR);

// ─── Shared state with mutex ───
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
float target_left  = 0.0f;
float target_right = 0.0f;
bool  estop        = false;
volatile long enc_left  = 0;
volatile long enc_right = 0;
volatile unsigned long last_cmd_vel_ms = 0;
volatile bool arm_trigger_pickup = false;
volatile bool object_detected = false;   // true when something is within ULTRASONIC_DETECT_THRESHOLD_CM

// ─── Odometry state ───
float odom_x     = 0.0f;
float odom_y     = 0.0f;
float odom_theta = 0.0f;

// ─── Encoder ISRs ───
void IRAM_ATTR enc_left_isr() {
    if (digitalRead(ENC_L_B) == LOW)
        enc_left++;
    else
        enc_left--;
}

void IRAM_ATTR enc_right_isr() {
    if (digitalRead(ENC_R_B) == LOW)
        enc_right++;
    else
        enc_right--;
}

// ─── PID ───
static float pid_compute(PIDState& s, float target, float actual) {
    float error  = target - actual;
    float i_term = s.integral + error * MOTOR_TASK_DT;
    // Anti-windup: clamp the integral so Ki·integral stays within ±INT_LIMIT PWM
    s.integral   = constrain(i_term, -PID_INT_LIMIT / PID_KI, PID_INT_LIMIT / PID_KI);
    float d_term = (error - s.prev_error) / MOTOR_TASK_DT;
    s.prev_error = error;
    return PID_KP * error + PID_KI * s.integral + PID_KD * d_term;
}

// ─── cmd_vel callback ───
void cmd_vel_callback(const void* msg) {
    const geometry_msgs__msg__Twist* twist =
        (const geometry_msgs__msg__Twist*)msg;

    float linear  = twist->linear.x;
    float angular = twist->angular.z;

    portENTER_CRITICAL(&mux);
    target_left  = (linear - angular * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
    target_right = (linear + angular * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
    last_cmd_vel_ms = millis();
    portEXIT_CRITICAL(&mux);

    motor_event_msg.data.data = (char*)"CMD_VEL received";
    motor_event_msg.data.size = 16;
    rcl_publish(&motor_events_pub, &motor_event_msg, NULL);
}

// ─── estop callback ───
void estop_callback(const void* msg) {
    const std_msgs__msg__String* s = (const std_msgs__msg__String*)msg;
    // Coordinator sends "1" to cut motors, "0" to re-enable.
    bool active = (s->data.size > 0 && s->data.data[0] == '1');
    portENTER_CRITICAL(&mux);
    estop = active;
    if (active) {
        target_left  = 0.0f;
        target_right = 0.0f;
    }
    portEXIT_CRITICAL(&mux);
    if (active) {
        analogWrite(MOTOR_L_PWM, 0);
        analogWrite(MOTOR_R_PWM, 0);
    }
}

// ─── arm_cmd callback ───
// Coordinator/mission_fsm sends "PICKUP" to run the lift-dump-lower sequence.
void arm_cmd_callback(const void* msg) {
    const std_msgs__msg__String* s = (const std_msgs__msg__String*)msg;
    if (s->data.size >= 6 && strncmp(s->data.data, "PICKUP", 6) == 0) {
        portENTER_CRITICAL(&mux);
        arm_trigger_pickup = true;
        portEXIT_CRITICAL(&mux);
    }
}

// ─── PCA9685 servo helper ───
static void arm_servo_write(int deg) {
    int pulse_us = map(deg, 0, 180, ARM_SERVO_PULSE_MIN_US, ARM_SERVO_PULSE_MAX_US);
    pca.writeMicroseconds(ARM_SERVO_CHANNEL, pulse_us);
}

static void publish_arm_event(const char* msg) {
    arm_event_msg.data.data = (char*)msg;
    arm_event_msg.data.size = strlen(msg);
    arm_event_msg.data.capacity = strlen(msg) + 1;
    rcl_publish(&arm_events_pub, &arm_event_msg, NULL);
}

static void publish_obstacle_event(const char* msg) {
    obstacle_event_msg.data.data = (char*)msg;
    obstacle_event_msg.data.size = strlen(msg);
    obstacle_event_msg.data.capacity = strlen(msg) + 1;
    rcl_publish(&obstacle_events_pub, &obstacle_event_msg, NULL);
}

// ─── Ultrasonic task — Core 0 ───
// Polls the HC-SR04 at ULTRASONIC_READ_INTERVAL_MS. pulseIn() blocks the
// calling task for up to ULTRASONIC_TIMEOUT_US, so this runs in its own
// low-priority task rather than inline in loop().
void ultrasonic_task(void* pvParameters) {
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    bool was_detected = false;

    while (true) {
        // 10us trigger pulse per HC-SR04 datasheet
        digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

        unsigned long echo_us = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);

        // echo_us == 0 means pulseIn timed out — no echo (out of range / no object)
        bool in_range = echo_us > 0;
        float distance_cm = echo_us / 58.0f;   // round-trip time -> cm

        bool detected = in_range && distance_cm <= ULTRASONIC_DETECT_THRESHOLD_CM;

        portENTER_CRITICAL(&mux);
        object_detected = detected;
        portEXIT_CRITICAL(&mux);

        if (detected != was_detected) {
            publish_obstacle_event(detected ? "OBSTACLE_DETECTED" : "OBSTACLE_CLEAR");
            was_detected = detected;
        }

        if (in_range) {
            range_msg.range = distance_cm / 100.0f;   // publish in meters
            rcl_publish(&ultrasonic_pub, &range_msg, NULL);
        }

        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_READ_INTERVAL_MS));
    }
}

// ─── Arm task — Core 0 ───
// Non-PID: the actuator sequence is open-loop (time or limit-switch
// bounded), so it doesn't need the tight 100Hz loop the drivetrain does.
void arm_task(void* pvParameters) {
    arm_servo_write(ARM_SERVO_HOME_DEG);

    pinMode(ARM_ACT_PWM, OUTPUT);
    pinMode(ARM_ACT_DIR, OUTPUT);
    pinMode(ARM_ACT_LIMIT_TOP, INPUT_PULLUP);
    pinMode(ARM_ACT_LIMIT_BOTTOM, INPUT_PULLUP);
    analogWrite(ARM_ACT_PWM, 0);

    while (true) {
        bool trigger;
        portENTER_CRITICAL(&mux);
        trigger = arm_trigger_pickup;
        arm_trigger_pickup = false;
        portEXIT_CRITICAL(&mux);

        if (trigger) {
            // 1. Lift
            digitalWrite(ARM_ACT_DIR, HIGH);
            analogWrite(ARM_ACT_PWM, ARM_ACT_SPEED);
            unsigned long start = millis();
            bool aborted = false;
            while (millis() - start < ARM_LIFT_TIME_MS) {
                if (digitalRead(ARM_ACT_LIMIT_TOP) == LOW) break;
                portENTER_CRITICAL(&mux);
                aborted = estop;
                portEXIT_CRITICAL(&mux);
                if (aborted) break;
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            analogWrite(ARM_ACT_PWM, 0);

            if (aborted) {
                publish_arm_event("ARM_ABORTED");
            } else {
                // 2. Dump
                arm_servo_write(ARM_SERVO_DUMP_DEG);
                vTaskDelay(pdMS_TO_TICKS(ARM_SERVO_MOVE_MS));
                vTaskDelay(pdMS_TO_TICKS(ARM_SERVO_HOLD_MS));
                arm_servo_write(ARM_SERVO_HOME_DEG);
                vTaskDelay(pdMS_TO_TICKS(ARM_SERVO_MOVE_MS));

                // 3. Lower back down
                digitalWrite(ARM_ACT_DIR, LOW);
                analogWrite(ARM_ACT_PWM, ARM_ACT_SPEED);
                start = millis();
                while (millis() - start < ARM_LOWER_TIME_MS) {
                    if (digitalRead(ARM_ACT_LIMIT_BOTTOM) == LOW) break;
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                analogWrite(ARM_ACT_PWM, 0);

                publish_arm_event("ARM_DONE");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ─── Motor task — Core 1 ───
void motor_task(void* pvParameters) {
    PIDState pid_l = {}, pid_r = {};
    long prev_enc_l = 0, prev_enc_r = 0;

    while (true) {
        float t_left, t_right;
        bool  e_stop;
        unsigned long last_cmd;

        portENTER_CRITICAL(&mux);
        t_left   = target_left;
        t_right  = target_right;
        e_stop   = estop;
        last_cmd = last_cmd_vel_ms;
        portEXIT_CRITICAL(&mux);

        // Measure actual wheel speed (rad/s) from encoder delta
        long curr_l = enc_left;
        long curr_r = enc_right;
        float actual_l = (curr_l - prev_enc_l) * RADS_PER_TICK / MOTOR_TASK_DT;
        float actual_r = (curr_r - prev_enc_r) * RADS_PER_TICK / MOTOR_TASK_DT;
        prev_enc_l = curr_l;
        prev_enc_r = curr_r;

        // Watchdog / e-stop: cut power and reset PID state
        if (millis() - last_cmd > 1000 || e_stop) {
            analogWrite(MOTOR_L_PWM, 0);
            analogWrite(MOTOR_R_PWM, 0);
            pid_l = {};
            pid_r = {};
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Left motor
        if (fabsf(t_left) < 0.01f) {
            analogWrite(MOTOR_L_PWM, 0);
            pid_l = {};
        } else {
            float out_l  = pid_compute(pid_l, t_left, actual_l);
            // Direction from target sign; PID magnitude drives PWM.
            // Negative out (overshoot) clamps to 0 (coast) rather than reversing.
            float drive_l = out_l * (t_left >= 0.0f ? 1.0f : -1.0f);
            digitalWrite(MOTOR_L_DIR, t_left >= 0.0f ? HIGH : LOW);
            analogWrite(MOTOR_L_PWM, (int)constrain(drive_l, 0.0f, 255.0f));
        }

        // Right motor
        if (fabsf(t_right) < 0.01f) {
            analogWrite(MOTOR_R_PWM, 0);
            pid_r = {};
        } else {
            float out_r   = pid_compute(pid_r, t_right, actual_r);
            float drive_r = out_r * (t_right >= 0.0f ? 1.0f : -1.0f);
            digitalWrite(MOTOR_R_DIR, t_right >= 0.0f ? LOW : HIGH);
            analogWrite(MOTOR_R_PWM, (int)constrain(drive_r, 0.0f, 255.0f));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─── setup ───
void setup() {
    Serial.begin(115200);
    set_microros_transports();

    // Motor pins
    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_DIR, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_DIR, OUTPUT);

    // Start motors off
    analogWrite(MOTOR_L_PWM, 0);
    analogWrite(MOTOR_R_PWM, 0);

    // Encoders
    pinMode(ENC_L_A, INPUT);
    pinMode(ENC_L_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), enc_left_isr, RISING);
    pinMode(ENC_R_A, INPUT);
    pinMode(ENC_R_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), enc_right_isr, RISING);

    // Shared I2C bus — PCA9685 (arm dump-gate servo) + MPU-6050 (terrain IMU)
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    pca.begin();
    pca.setPWMFreq(PCA9685_PWM_FREQ_HZ);

    imu.initialize();
    if (!imu.testConnection()) {
        Serial.println("MPU6050 init failed — check wiring/address");
    }
    // Fields that never change: no orientation estimate (raw accel/gyro only),
    // so mark those covariances unknown per REP-145.
    imu_msg.header.frame_id.data     = (char*)"imu";
    imu_msg.header.frame_id.size     = 3;
    imu_msg.header.frame_id.capacity = 4;
    imu_msg.orientation.w                    = 1.0;
    imu_msg.orientation_covariance[0]        = -1.0;
    imu_msg.angular_velocity_covariance[0]   = -1.0;
    imu_msg.linear_acceleration_covariance[0] = -1.0;

    // Fields that never change across reads
    range_msg.header.frame_id.data     = (char*)"ultrasonic";
    range_msg.header.frame_id.size     = 10;
    range_msg.header.frame_id.capacity = 11;
    range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    range_msg.field_of_view  = ULTRASONIC_FIELD_OF_VIEW_RAD;
    range_msg.min_range      = ULTRASONIC_MIN_RANGE_M;
    range_msg.max_range      = ULTRASONIC_MAX_RANGE_M;

    // micro-ROS init
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_controller", "", &support);

    // Subscriptions
    rclc_subscription_init_default(&cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");
    rclc_subscription_init_default(&estop_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/e_stop");
    rclc_subscription_init_default(&arm_cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/arm_cmd");

    // Publishers
    rclc_publisher_init_default(&odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/wheel/odometry");
    rclc_publisher_init_default(&motor_events_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/motor_events");
    rclc_publisher_init_default(&arm_events_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/arm_events");
    rclc_publisher_init_default(&imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data");
    rclc_publisher_init_default(&ultrasonic_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/ultrasonic/range");
    rclc_publisher_init_default(&obstacle_events_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/obstacle_events");

    // Executor
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &estop_sub, &estop_msg,
        &estop_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &arm_cmd_sub, &arm_cmd_msg,
        &arm_cmd_callback, ON_NEW_DATA);

    // Pin motor task to Core 1, arm + ultrasonic tasks to Core 0 (mostly idle in vTaskDelay)
    xTaskCreatePinnedToCore(motor_task, "MOTOR", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(arm_task, "ARM", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(ultrasonic_task, "ULTRASONIC", 2048, NULL, 1, NULL, 0);
}

// ─── loop — Core 0 ───
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    static unsigned long last_imu = 0;
    if (millis() - last_imu > IMU_PUBLISH_INTERVAL_MS) {
        last_imu = millis();

        int16_t ax, ay, az, gx, gy, gz;
        imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // TODO(partner): mounting orientation isn't calibrated yet — flat
        // ground should read az ≈ +GRAVITY_MSS (terrain_monitor's atan2(ax, az)
        // pitch convention assumes this). Re-check axis signs once mounted.
        imu_msg.linear_acceleration.x = (ax / IMU_ACCEL_LSB_PER_G) * GRAVITY_MSS;
        imu_msg.linear_acceleration.y = (ay / IMU_ACCEL_LSB_PER_G) * GRAVITY_MSS;
        imu_msg.linear_acceleration.z = (az / IMU_ACCEL_LSB_PER_G) * GRAVITY_MSS;
        imu_msg.angular_velocity.x = (gx / IMU_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
        imu_msg.angular_velocity.y = (gy / IMU_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
        imu_msg.angular_velocity.z = (gz / IMU_GYRO_LSB_PER_DPS) * DEG_TO_RAD;

        rcl_publish(&imu_pub, &imu_msg, NULL);
    }

    static unsigned long last_odom = 0;
    if (millis() - last_odom > 100) {
        last_odom = millis();

        static long last_enc_l = 0;
        static long last_enc_r = 0;
        long curr_l = enc_left;
        long curr_r = enc_right;
        float dl = (curr_l - last_enc_l) * METERS_PER_TICK;
        float dr = (curr_r - last_enc_r) * METERS_PER_TICK;
        last_enc_l = curr_l;
        last_enc_r = curr_r;

        float ds     = (dl + dr) / 2.0f;
        float dtheta = (dr - dl) / WHEEL_BASE;
        odom_theta  += dtheta;
        odom_x      += ds * cosf(odom_theta);
        odom_y      += ds * sinf(odom_theta);

        odom_msg.pose.pose.position.x    = odom_x;
        odom_msg.pose.pose.position.y    = odom_y;
        odom_msg.pose.pose.orientation.w = cosf(odom_theta / 2.0f);
        odom_msg.pose.pose.orientation.x = 0.0f;
        odom_msg.pose.pose.orientation.y = 0.0f;
        odom_msg.pose.pose.orientation.z = sinf(odom_theta / 2.0f);

        rcl_publish(&odom_pub, &odom_msg, NULL);
    }
}