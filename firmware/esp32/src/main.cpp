#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/string.h>
#include <ESP32Servo.h>

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
#define ARM_SERVO_PIN         19   // hobby servo signal (dump gate)

#define ARM_ACT_SPEED        200   // 0-255 PWM — placeholder, tune once actuator specs are known
// TODO(partner): these two are a time-based fallback for actuators with no
// limit switches/position feedback wired yet. Once ARM_ACT_LIMIT_TOP/BOTTOM
// are physically wired, they end the stroke early and these just become a
// timeout safety net — measure real travel time and set them a bit above it.
#define ARM_LIFT_TIME_MS     3000
#define ARM_LOWER_TIME_MS    3000

#define ARM_SERVO_HOME_DEG      0   // TODO(partner): calibrate against the real dump-gate geometry
#define ARM_SERVO_DUMP_DEG    120
#define ARM_SERVO_MOVE_MS     600   // time to let the servo finish travelling before continuing
#define ARM_SERVO_HOLD_MS     500   // how long to hold at the dump angle before returning home

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

geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__String estop_msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__String motor_event_msg;
std_msgs__msg__String arm_cmd_msg;
std_msgs__msg__String arm_event_msg;

Servo arm_servo;

// ─── Shared state with mutex ───
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
float target_left  = 0.0f;
float target_right = 0.0f;
bool  estop        = false;
volatile long enc_left  = 0;
volatile long enc_right = 0;
volatile unsigned long last_cmd_vel_ms = 0;
volatile bool arm_trigger_pickup = false;

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

static void publish_arm_event(const char* msg) {
    arm_event_msg.data.data = (char*)msg;
    arm_event_msg.data.size = strlen(msg);
    arm_event_msg.data.capacity = strlen(msg) + 1;
    rcl_publish(&arm_events_pub, &arm_event_msg, NULL);
}

// ─── Arm task — Core 0 ───
// Non-PID: the actuator sequence is open-loop (time or limit-switch
// bounded), so it doesn't need the tight 100Hz loop the drivetrain does.
void arm_task(void* pvParameters) {
    arm_servo.setPeriodHertz(50);
    arm_servo.attach(ARM_SERVO_PIN, 500, 2400);
    arm_servo.write(ARM_SERVO_HOME_DEG);

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
                arm_servo.write(ARM_SERVO_DUMP_DEG);
                vTaskDelay(pdMS_TO_TICKS(ARM_SERVO_MOVE_MS));
                vTaskDelay(pdMS_TO_TICKS(ARM_SERVO_HOLD_MS));
                arm_servo.write(ARM_SERVO_HOME_DEG);
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

    // Executor
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &estop_sub, &estop_msg,
        &estop_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &arm_cmd_sub, &arm_cmd_msg,
        &arm_cmd_callback, ON_NEW_DATA);

    // Pin motor task to Core 1, arm task to Core 0 (mostly idle in vTaskDelay)
    xTaskCreatePinnedToCore(motor_task, "MOTOR", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(arm_task, "ARM", 4096, NULL, 1, NULL, 0);
}

// ─── loop — Core 0 ───
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

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