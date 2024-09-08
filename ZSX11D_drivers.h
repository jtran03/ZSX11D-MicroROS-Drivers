#ifndef ZSX11D_DRIVER_H
#define ZSX11D_DRIVER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

#define FORWARD 1.0
#define REVERSE -1.0
#define STOP 0

// ZSX11D driver configuration structure
typedef struct {
    gpio_num_t pwm_pin;
    gpio_num_t direction_pin;
    gpio_num_t hall_sensor_pin;
    ledc_mode_t ledc_mode;
    ledc_timer_bit_t pwm_res;
    ledc_timer_t timer;
    uint32_t pwm_freq;
    ledc_channel_t ledc_channel;
    int32_t pcnt_high_limit;
    int32_t pcnt_low_limit;
    uint32_t pcnt_glitch_ns;
    float motor_ppr; // Pulses per revolution
    float kp, ki, kd; // PID coefficients
    uint32_t max_pwm; // Max PWM value
    uint32_t min_pwm; // Min PWM value
    float sample_period_s; // Sample period in seconds
    int64_t motor_timeout_period; 
    int pid_sample_period_ms; 
} zsx11d_driver_config_t;

typedef struct {
    zsx11d_driver_config_t config;
    pcnt_unit_handle_t pcnt_unit;
    pcnt_channel_handle_t pcnt_channel;
    QueueHandle_t pcnt_queue;
    float latest_rpm;
    float latest_pwm_duty_cycle;
    float motor_direction;
    int64_t previous_time;
    int64_t current_time_period; 
    float integral_PID;
    float previous_error_PID;
    float setpoint_PID;
    int error_PID; 
} zsx11d_driver_t;

esp_err_t zsx11d_driver_init(zsx11d_driver_t *motor, const zsx11d_driver_config_t *config);
void zsx11d_driver_set_pwm(zsx11d_driver_t *motor, uint32_t duty);
void zsx11d_driver_set_rpm(zsx11d_driver_t *motor, float rpm);
float zsx11d_driver_get_rpm(zsx11d_driver_t *motor);
float zsx11d_driver_get_rpm_setpoint(zsx11d_driver_t *motor);
float zsx11d_driver_get_duty_cycle(zsx11d_driver_t *motor);
int zsx11d_driver_get_error(zsx11d_driver_t *motor); 
void zsx11d_driver_pid_loop(void *pvParameters);
bool zsx11d_driver_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx); 

#endif // ZSX11D_DRIVER_H
