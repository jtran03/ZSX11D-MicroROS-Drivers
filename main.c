#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include <geometry_msgs/msg/twist.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include "esp32_serial_transport.h"
#include "ZSX11D_drivers.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define WHEEL_BASE_M 0.2 
#define WHEEL_RADIUS_M 0.165

#define LEFT_MOTOR_PWM_OUT_PIN GPIO_NUM_26
#define LEFT_MOTOR_DIRECTION_PIN GPIO_NUM_13
#define LEFT_MOTOR_HALL_SENSOR_PIN GPIO_NUM_4

#define LEFT_MOTOR_LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEFT_MOTOR_PWM_RES LEDC_TIMER_10_BIT  
#define LEFT_MOTOR_TIMER LEDC_TIMER_0
#define LEFT_MOTOR_PWM_FREQ (4000) 
#define LEFT_MOTOR_LEDC_CHANNEL LEDC_CHANNEL_0
#define LEFT_MOTOR_MIN_PWM (0)
#define LEFT_MOTOR_MAX_PWM (1023)
#define LEFT_MOTOR_FORWARD (1)
#define LEFT_MOTOR_BACKWARD (-1)
#define LEFT_MOTOR_MIN_RPM (30.0)

#define LEFT_MOTOR_PCNT_HIGH_LIMIT (10)
#define LEFT_MOTOR_PCNT_LOW_LIMIT (-10)
#define LEFT_MOTOR_PCNT_GLITCH_NS (1000)
#define LEFT_MOTOR_SAMPLE_PERIOD_S (0.2)
#define LEFT_MOTOR_PPR (90)
#define LEFT_MOTOR_TIMEOUT_PERIOD (1000000)

#define LEFT_MOTOR_KP (1.2)
#define LEFT_MOTOR_KI (0.5)
#define LEFT_MOTOR_KD (0)  
#define LEFT_MOTOR_PID_SAMPLE_PERIOD_MS (50)

#define RIGHT_MOTOR_PWM_OUT_PIN GPIO_NUM_25
#define RIGHT_MOTOR_DIRECTION_PIN GPIO_NUM_16
#define RIGHT_MOTOR_HALL_SENSOR_PIN GPIO_NUM_14

#define RIGHT_MOTOR_LEDC_MODE LEDC_LOW_SPEED_MODE
#define RIGHT_MOTOR_PWM_RES LEDC_TIMER_10_BIT  
#define RIGHT_MOTOR_TIMER LEDC_TIMER_1
#define RIGHT_MOTOR_PWM_FREQ (4000) 
#define RIGHT_MOTOR_LEDC_CHANNEL LEDC_CHANNEL_1
#define RIGHT_MOTOR_MIN_PWM (0)
#define RIGHT_MOTOR_MAX_PWM (1023)
#define RIGHT_MOTOR_FORWARD (1)
#define RIGHT_MOTOR_BACKWARD (-1)
#define RIGHT_MOTOR_MIN_RPM (30.0)

#define RIGHT_MOTOR_PCNT_HIGH_LIMIT (10)
#define RIGHT_MOTOR_PCNT_LOW_LIMIT (-10)
#define RIGHT_MOTOR_PCNT_GLITCH_NS (1000)
#define RIGHT_MOTOR_SAMPLE_PERIOD_S (0.2)
#define RIGHT_MOTOR_PPR (90)
#define RIGHT_MOTOR_TIMEOUT_PERIOD (1000000)

#define RIGHT_MOTOR_KP (1.2)
#define RIGHT_MOTOR_KI (0.5)
#define RIGHT_MOTOR_KD (0)  
#define RIGHT_MOTOR_PID_SAMPLE_PERIOD_MS (50)

zsx11d_driver_t left_motor;
zsx11d_driver_t right_motor; 

rcl_publisher_t left_rpm_publisher; 
std_msgs__msg__Float32 left_rpm_msg; 

rcl_publisher_t left_rpm_setpoint_publisher; 
std_msgs__msg__Float32 left_rpm_setpoint_msg; 

rcl_publisher_t right_rpm_publisher; 
std_msgs__msg__Float32 right_rpm_msg; 

rcl_publisher_t right_rpm_setpoint_publisher; 
std_msgs__msg__Float32 right_rpm_setpoint_msg; 

rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_recv_msg;

void cmd_vel_cb(const void *msgin) {

    const geometry_msgs__msg__Twist *cmd_vel_recv_msg = (const geometry_msgs__msg__Twist *)msgin;

    // Extract linear and angular velocities
    float linear_velocity = cmd_vel_recv_msg->linear.x; 
    float angular_velocity = cmd_vel_recv_msg->angular.z; 

    // Precompute constants
    static const float wheel_base_half = WHEEL_BASE_M / 2.0;
    static const float inv_circumference = 1.0 / (2 * M_PI * WHEEL_RADIUS_M); // Precompute 1 / circumference

    // Compute wheel velocities
    float v_left = linear_velocity - (angular_velocity * wheel_base_half);
    float v_right = linear_velocity + (angular_velocity * wheel_base_half);

    // Convert to RPM
    float left_rpm = v_left * inv_circumference * 60.0;
    float right_rpm = v_right * inv_circumference * 60.0;

    // If both linear and angular velocities are zero, set RPMs to zero
    if (linear_velocity == 0.0 && angular_velocity == 0.0) {
        left_rpm = 0.0;
        right_rpm = 0.0;
    } else {
        // Apply minimum RPM constraints if necessary
        left_rpm = (left_rpm > 0) ? fmax(left_rpm, LEFT_MOTOR_MIN_RPM) : fmin(left_rpm, -LEFT_MOTOR_MIN_RPM);
        right_rpm = (right_rpm > 0) ? fmax(right_rpm, RIGHT_MOTOR_MIN_RPM) : fmin(right_rpm, -RIGHT_MOTOR_MIN_RPM);
    }

    // Set motor RPMs
    zsx11d_driver_set_rpm(&left_motor, left_rpm);
    zsx11d_driver_set_rpm(&right_motor, right_rpm);
}


void publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {

        left_rpm_msg.data = zsx11d_driver_get_rpm(&left_motor); 
        RCSOFTCHECK(rcl_publish(&left_rpm_publisher, &left_rpm_msg, NULL));

        right_rpm_msg.data = zsx11d_driver_get_rpm(&right_motor); 
        RCSOFTCHECK(rcl_publish(&right_rpm_publisher, &right_rpm_msg, NULL));
    }   
}

void micro_ros_task(void * arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "hub_motor_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &left_rpm_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "left_rpm"));

    RCCHECK(rclc_publisher_init_default(
        &right_rpm_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "right_rpm"));


    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(50),
        publisher_timer_callback));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // 1 Timer + 1 Subscriber
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_recv_msg, &cmd_vel_cb, ON_NEW_DATA));

    left_rpm_msg.data = 0;     
    right_rpm_msg.data = 0;

    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&left_rpm_publisher, &node));
    RCCHECK(rcl_publisher_fini(&right_rpm_publisher, &node));
    RCCHECK(rcl_subscription_fini(&cmd_vel_subscriber, &node));
    RCCHECK(rcl_timer_fini(&timer));
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

static size_t uart_port = UART_NUM_0;

void app_main(void) {

    esp_rom_gpio_pad_select_gpio(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

    zsx11d_driver_config_t left_motor_config = {
        .pwm_pin = LEFT_MOTOR_PWM_OUT_PIN,
        .direction_pin = LEFT_MOTOR_DIRECTION_PIN,
        .hall_sensor_pin = LEFT_MOTOR_HALL_SENSOR_PIN,
        .ledc_mode = LEFT_MOTOR_LEDC_MODE,
        .pwm_res = LEFT_MOTOR_PWM_RES,
        .timer = LEFT_MOTOR_TIMER,
        .pwm_freq = LEFT_MOTOR_PWM_FREQ,
        .ledc_channel = LEFT_MOTOR_LEDC_CHANNEL,
        .pcnt_high_limit = LEFT_MOTOR_PCNT_HIGH_LIMIT,
        .pcnt_low_limit = LEFT_MOTOR_PCNT_LOW_LIMIT,
        .pcnt_glitch_ns = LEFT_MOTOR_PCNT_GLITCH_NS,
        .motor_ppr = LEFT_MOTOR_PPR,
        .kp = LEFT_MOTOR_KP,
        .ki = LEFT_MOTOR_KI,
        .kd = LEFT_MOTOR_KD,
        .max_pwm = LEFT_MOTOR_MAX_PWM,
        .min_pwm = LEFT_MOTOR_MIN_PWM,
        .sample_period_s = LEFT_MOTOR_SAMPLE_PERIOD_S,
        .motor_timeout_period = LEFT_MOTOR_TIMEOUT_PERIOD,
        .pid_sample_period_ms = LEFT_MOTOR_PID_SAMPLE_PERIOD_MS, 
    };

    zsx11d_driver_config_t right_motor_config = {
        .pwm_pin = RIGHT_MOTOR_PWM_OUT_PIN,
        .direction_pin = RIGHT_MOTOR_DIRECTION_PIN,
        .hall_sensor_pin = RIGHT_MOTOR_HALL_SENSOR_PIN,
        .ledc_mode = RIGHT_MOTOR_LEDC_MODE,
        .pwm_res = RIGHT_MOTOR_PWM_RES,
        .timer = RIGHT_MOTOR_TIMER,
        .pwm_freq = RIGHT_MOTOR_PWM_FREQ,
        .ledc_channel = RIGHT_MOTOR_LEDC_CHANNEL,
        .pcnt_high_limit = RIGHT_MOTOR_PCNT_HIGH_LIMIT,
        .pcnt_low_limit = RIGHT_MOTOR_PCNT_LOW_LIMIT,
        .pcnt_glitch_ns = RIGHT_MOTOR_PCNT_GLITCH_NS,
        .motor_ppr = RIGHT_MOTOR_PPR,
        .kp = RIGHT_MOTOR_KP,
        .ki = RIGHT_MOTOR_KI,
        .kd = RIGHT_MOTOR_KD,
        .max_pwm = RIGHT_MOTOR_MAX_PWM,
        .min_pwm = RIGHT_MOTOR_MIN_PWM,
        .sample_period_s = RIGHT_MOTOR_SAMPLE_PERIOD_S,
        .motor_timeout_period = RIGHT_MOTOR_TIMEOUT_PERIOD,
        .pid_sample_period_ms = RIGHT_MOTOR_PID_SAMPLE_PERIOD_MS, 
    };
    
    zsx11d_driver_init(&left_motor, &left_motor_config);
    zsx11d_driver_init(&right_motor, &right_motor_config);

    xTaskCreate(zsx11d_driver_pid_loop,
        "LeftPIDTask",
        2048,
        &left_motor,
        5,
        NULL);
    
    xTaskCreate(zsx11d_driver_pid_loop,
        "RightPIDTask",
        2048,
        &right_motor,
        5,
        NULL);

    #if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(
        true,
        (void *) &uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read
    );
    #else
    #error micro-ROS transports misconfigured
    #endif  // RMW_UXRCE_TRANSPORT_CUSTOM

    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);

    // TickType_t xLastWakeTime = xTaskGetTickCount();
    // const TickType_t xPrintFrequency = pdMS_TO_TICKS(200); // 1000 ms (1 second)

    // while (1) {
    //     // zsx11d_driver_set_rpm(&left_motor, 50);  // Set desired RPM
    //     // zsx11d_driver_set_rpm(&right_motor, 50);  // Set desired RPM
    //     // printf("Setpoint: %f || Error: %d || PWM: %f || RPM: %f\n ", zsx11d_driver_get_rpm_setpoint(&right_motor), zsx11d_driver_get_error(&right_motor), zsx11d_driver_get_duty_cycle(&right_motor), zsx11d_driver_get_rpm(&right_motor));
    //     printf("LRPM: %f || RRPM: %f\n ", zsx11d_driver_get_rpm(&left_motor), zsx11d_driver_get_rpm(&right_motor));
    //     vTaskDelayUntil(&xLastWakeTime, xPrintFrequency);
    // }
}