#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/string.h>

// ─── HARDWARE CONSTANTS ───
#define WHEEL_RADIUS      0.051f
#define WHEEL_BASE        0.3f
#define TICKS_PER_REV     1664
#define METERS_PER_TICK   (2.0f * M_PI * WHEEL_RADIUS / TICKS_PER_REV)

// ─── PIN DEFINITIONS ───
#define MOTOR_L_PWM   25
#define MOTOR_L_DIR   26
#define MOTOR_R_PWM   27
#define MOTOR_R_DIR   14
#define ENC_L_A       34
#define ENC_L_B       35

// ─── micro-ROS objects ───
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t cmd_vel_sub;
rcl_subscription_t estop_sub;
rcl_publisher_t odom_pub;
rcl_publisher_t motor_events_pub;

geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__String estop_msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__String motor_event_msg;

// ─── Shared state with mutex ───
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
float target_left  = 0.0f;
float target_right = 0.0f;
bool  estop        = false;
volatile long enc_left = 0;

// ─── Odometry state ───
float odom_x     = 0.0f;
float odom_y     = 0.0f;
float odom_theta = 0.0f;

// ─── Encoder ISR ───
void IRAM_ATTR enc_left_isr() {
    if (digitalRead(ENC_L_B) == LOW)
        enc_left++;
    else
        enc_left--;
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
    portEXIT_CRITICAL(&mux);

    motor_event_msg.data.data = (char*)"CMD_VEL received";
    motor_event_msg.data.size = 16;
    rcl_publish(&motor_events_pub, &motor_event_msg, NULL);
}

// ─── estop callback ───
void estop_callback(const void* msg) {
    portENTER_CRITICAL(&mux);
    estop        = true;
    target_left  = 0.0f;
    target_right = 0.0f;
    portEXIT_CRITICAL(&mux);
    analogWrite(MOTOR_L_PWM, 0);
    analogWrite(MOTOR_R_PWM, 0);
}

// ─── Motor task — Core 1 ───
void motor_task(void* pvParameters) {
    while (true) {
        float t_left, t_right;
        bool e_stop;

        portENTER_CRITICAL(&mux);
        t_left  = target_left;
        t_right = target_right;
        e_stop  = estop;
        portEXIT_CRITICAL(&mux);

        if (e_stop) {
            analogWrite(MOTOR_L_PWM, 0);
            analogWrite(MOTOR_R_PWM, 0);
        } else {
            if (abs(t_left) > 0.01f) {
                digitalWrite(MOTOR_L_DIR, t_left >= 0 ? HIGH : LOW);
                analogWrite(MOTOR_L_PWM, 150);
            } else {
                analogWrite(MOTOR_L_PWM, 0);
            }

            if (abs(t_right) > 0.01f) {
                digitalWrite(MOTOR_R_DIR, t_right >= 0 ? HIGH : LOW);
                analogWrite(MOTOR_R_PWM, 150);
            } else {
                analogWrite(MOTOR_R_PWM, 0);
            }
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

    // Encoder
    pinMode(ENC_L_A, INPUT);
    pinMode(ENC_L_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), enc_left_isr, RISING);
    


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

    // Publishers
    rclc_publisher_init_default(&odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/wheel/odometry");
    rclc_publisher_init_default(&motor_events_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/motor_events");

    // Executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &estop_sub, &estop_msg,
        &estop_callback, ON_NEW_DATA);

    // Pin motor task to Core 1
    xTaskCreatePinnedToCore(motor_task, "MOTOR", 4096, NULL, 1, NULL, 1);



}

// ─── loop — Core 0 ───
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    static unsigned long last_odom = 0;
    if (millis() - last_odom > 100) {
        last_odom = millis();

        static long last_enc = 0;
        long curr_enc = enc_left;
        float dl = (curr_enc - last_enc) * METERS_PER_TICK;
        last_enc = curr_enc;

        odom_x += dl * cos(odom_theta);
        odom_y += dl * sin(odom_theta);

        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;

        rcl_publish(&odom_pub, &odom_msg, NULL);
    }
}