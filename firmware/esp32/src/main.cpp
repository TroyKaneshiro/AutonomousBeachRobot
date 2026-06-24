#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

// ─── HARDWARE CONSTANTS (fill these in once you have motor specs) ───
#define WHEEL_BASE        0.0f   // meters, center-to-center left/right wheels
#define WHEEL_RADIUS      0.0f   // meters
#define TICKS_PER_REV     0      // encoder ticks per wheel revolution (after gearbox)
#define METERS_PER_TICK   (2.0f * M_PI * WHEEL_RADIUS / TICKS_PER_REV)

// ─── PIN DEFINITIONS (assign once you decide wiring) ───
#define MOTOR_L_PWM   0    // GPIO for left motor PWM
#define MOTOR_L_DIR   0    // GPIO for left motor direction
#define MOTOR_R_PWM   0    // GPIO for right motor PWM
#define MOTOR_R_DIR   0    // GPIO for right motor direction
#define ENC_L_A       0    // left encoder channel A
#define ENC_L_B       0    // left encoder channel B
#define ENC_R_A       0    // right encoder channel A
#define ENC_R_B       0    // right encoder channel B
#define BATTERY_PIN   34   // ADC pin for battery voltage divider
#define CURRENT_L_PIN 35   // ACS712 left motor
#define CURRENT_R_PIN 32   // ACS712 right motor

// ─── micro-ROS objects ───
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t cmd_vel_sub;
rcl_subscription_t estop_sub;
rcl_publisher_t odom_pub;
rcl_publisher_t battery_pub;
rcl_publisher_t motor_events_pub;

geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__String estop_msg;

// ─── Shared state between cores ───
volatile float target_left  = 0.0f;   // rad/s
volatile float target_right = 0.0f;   // rad/s
volatile bool  estop        = false;

volatile long enc_left  = 0;
volatile long enc_right = 0;

// ─── Odometry state ───
float odom_x     = 0.0f;
float odom_y     = 0.0f;
float odom_theta = 0.0f;

// ─── PID state ───
struct PID {
    float kp, ki, kd;
    float integral  = 0.0f;
    float prev_error = 0.0f;

    float compute(float setpoint, float measured, float dt) {
        float error    = setpoint - measured;
        integral      += error * dt;
        float deriv    = (error - prev_error) / dt;
        prev_error     = error;
        return kp * error + ki * integral + kd * deriv;
    }
};

PID pid_left  = {1.0f, 0.1f, 0.01f};
PID pid_right = {1.0f, 0.1f, 0.01f};

// ─── Encoder ISRs ───
void IRAM_ATTR enc_left_isr()  { enc_left++;  }
void IRAM_ATTR enc_right_isr() { enc_right++; }

// ─── cmd_vel callback ───
void cmd_vel_callback(const void* msg) {
    const geometry_msgs__msg__Twist* twist =
        (const geometry_msgs__msg__Twist*)msg;

    float linear  = twist->linear.x;
    float angular = twist->angular.z;

    target_left  = (linear - angular * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
    target_right = (linear + angular * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
}

// ─── estop callback ───
void estop_callback(const void* msg) {
    estop = true;
    target_left  = 0.0f;
    target_right = 0.0f;
    // cut motor PWM immediately
    analogWrite(MOTOR_L_PWM, 0);
    analogWrite(MOTOR_R_PWM, 0);
}

// ─── PID task — Core 1 ───
void pid_task(void* pvParameters) {
    const float dt = 0.01f;  // 100Hz
    long prev_enc_left  = 0;
    long prev_enc_right = 0;

    while (true) {
        if (estop) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Measure current velocity
        long curr_left  = enc_left;
        long curr_right = enc_right;

        float vel_left  = ((curr_left  - prev_enc_left)  * METERS_PER_TICK) / dt / WHEEL_RADIUS;
        float vel_right = ((curr_right - prev_enc_right) * METERS_PER_TICK) / dt / WHEEL_RADIUS;

        prev_enc_left  = curr_left;
        prev_enc_right = curr_right;

        // PID
        float out_left  = pid_left.compute(target_left,  vel_left,  dt);
        float out_right = pid_right.compute(target_right, vel_right, dt);

        // Write to Cytron MDD10A
        // DIR pin: HIGH = forward, LOW = reverse
        // PWM pin: 0-255 duty cycle
        digitalWrite(MOTOR_L_DIR, out_left  >= 0 ? HIGH : LOW);
        digitalWrite(MOTOR_R_DIR, out_right >= 0 ? HIGH : LOW);
        analogWrite(MOTOR_L_PWM, constrain((int)abs(out_left  * 255), 0, 255));
        analogWrite(MOTOR_R_PWM, constrain((int)abs(out_right * 255), 0, 255));

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─── setup ───
void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // Pin modes
    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_DIR, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_DIR, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENC_L_A), enc_left_isr,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), enc_right_isr, RISING);

    // micro-ROS init
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_controller", "", &support);

    // Subscriptions
    rclc_subscription_init_default(&cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
    rclc_subscription_init_default(&estop_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/e_stop");

    // Publishers
    rclc_publisher_init_default(&odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/wheel/odometry");
    rclc_publisher_init_default(&battery_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/battery_voltage");
    rclc_publisher_init_default(&motor_events_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/motor_events");

    // Executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &estop_sub, &estop_msg,
        &estop_callback, ON_NEW_DATA);

    // Pin PID to Core 1
    xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 1, NULL, 1);
}

// ─── loop — Core 0 (micro-ROS + publishing) ───
void loop() {
    // Spin micro-ROS executor
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    // Publish odometry at 10Hz
    static unsigned long last_odom = 0;
    if (millis() - last_odom > 100) {
        last_odom = millis();
        // TODO: fill odometry message from enc_left/enc_right and publish
    }

    // Publish battery at 1Hz
    static unsigned long last_batt = 0;
    if (millis() - last_batt > 1000) {
        last_batt = millis();
        // ADC read → voltage divider math → publish
        // float voltage = analogRead(BATTERY_PIN) * (3.3f / 4095.0f) * DIVIDER_RATIO;
    }

    // Stall detection at 10Hz
    static unsigned long last_current = 0;
    if (millis() - last_current > 100) {
        last_current = millis();
        // ACS712: Vout = 2.5V + (66mV/A * current)
        // float current_l = (analogRead(CURRENT_L_PIN) * 3.3f/4095.0f - 2.5f) / 0.066f;
    }
}