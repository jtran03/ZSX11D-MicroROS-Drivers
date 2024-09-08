#include "ZSX11D_drivers.h"

esp_err_t zsx11d_driver_init(zsx11d_driver_t *motor, const zsx11d_driver_config_t *config) {
    motor->config = *config;
    motor->previous_time = esp_timer_get_time(); 
    
    ledc_timer_config_t motor_pwm_timer = {
        .speed_mode =  config->ledc_mode, 
        .duty_resolution = config->pwm_res, 
        .timer_num = config->timer, 
        .freq_hz = config->pwm_freq, 
        .clk_cfg = LEDC_USE_APB_CLK
    }; 
    ESP_ERROR_CHECK(ledc_timer_config(&motor_pwm_timer));

    ledc_channel_config_t motor_pwm_channel = {
        .gpio_num = config->pwm_pin, 
        .speed_mode = config->ledc_mode, 
        .channel = config->ledc_channel, 
        .timer_sel = config->timer,
        .intr_type = LEDC_INTR_DISABLE, 
        .duty = 0, 
        .hpoint = 0
    }; 
    ESP_ERROR_CHECK(ledc_channel_config(&motor_pwm_channel));

    // Configure direction pin
    gpio_config_t motor_direction_pin_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << config->direction_pin), 
        .pull_down_en = 0, 
        .pull_up_en = 0
    }; 
    ESP_ERROR_CHECK(gpio_config(&motor_direction_pin_config)); 

    pcnt_unit_config_t motor_pcnt_unit_config = {
        .high_limit = config->pcnt_high_limit,
        .low_limit = config->pcnt_low_limit, 
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&motor_pcnt_unit_config, &motor->pcnt_unit));

    pcnt_glitch_filter_config_t motor_filter_config = {
        .max_glitch_ns = config->pcnt_glitch_ns,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(motor->pcnt_unit, &motor_filter_config));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(motor->pcnt_unit, config->pcnt_high_limit));
    motor->pcnt_queue = xQueueCreate(10, sizeof(uint32_t)); 
    pcnt_event_callbacks_t cbs = {
        .on_reach = zsx11d_driver_pcnt_on_reach,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(motor->pcnt_unit, &cbs, motor));

    pcnt_chan_config_t motor_chan_config = {
        .edge_gpio_num = config->hall_sensor_pin,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(motor->pcnt_unit, &motor_chan_config, &motor->pcnt_channel));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(motor->pcnt_channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_unit_enable(motor->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(motor->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(motor->pcnt_unit));

    motor->integral_PID = 0;
    motor->previous_error_PID = 0;
    motor->setpoint_PID = 0;
    motor->current_time_period = 0; 
    motor->motor_direction = 0; 

    return ESP_OK;
}

void zsx11d_driver_set_pwm(zsx11d_driver_t *motor, uint32_t duty) {
    motor->latest_pwm_duty_cycle = duty; 
    ESP_ERROR_CHECK(ledc_set_duty(motor->config.ledc_mode, motor->config.ledc_channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(motor->config.ledc_mode, motor->config.ledc_channel));
}

void zsx11d_driver_set_rpm(zsx11d_driver_t *motor, float rpm) {
    motor->motor_direction = (rpm < 0) ? REVERSE : FORWARD; 
    gpio_set_level(motor->config.direction_pin, (rpm < 0) ? 0 : 1);
    motor->setpoint_PID = fabs(rpm);
}

float zsx11d_driver_get_rpm(zsx11d_driver_t *motor) {
    float rpm = 0;
    if (esp_timer_get_time() - motor->previous_time > motor->config.motor_timeout_period){
        rpm = 0; 
    } else if (motor->current_time_period > 0) {
        float period_s = motor->current_time_period / 1000000.0; 
        float frequency_hz = 1.0 / period_s;  
        rpm = ((frequency_hz * 60) / motor->config.motor_ppr) * motor->config.pcnt_high_limit; // Since we are measuring 10 pulses, multiply by 10
    }
    motor->latest_rpm = rpm * motor->motor_direction;
    return motor->latest_rpm; 
}

float zsx11d_driver_get_duty_cycle(zsx11d_driver_t *motor){
    return motor->latest_pwm_duty_cycle; 
}

float zsx11d_driver_get_rpm_setpoint(zsx11d_driver_t *motor) {
    return motor->setpoint_PID * motor->motor_direction; 
}

int zsx11d_driver_get_error(zsx11d_driver_t *motor){
    return motor->error_PID; 
}

bool zsx11d_driver_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_wakeup = pdFALSE;
    zsx11d_driver_t *motor = (zsx11d_driver_t *)user_ctx;

    int64_t current_time = esp_timer_get_time();
    motor->current_time_period = current_time - motor->previous_time;
    motor->previous_time = current_time; 

    return (high_task_wakeup == pdTRUE);
}

void zsx11d_driver_pid_loop(void *pvParameters) {
    zsx11d_driver_t *motor = (zsx11d_driver_t *)pvParameters;
    const TickType_t xDelay = pdMS_TO_TICKS(motor->config.pid_sample_period_ms);

    while (1) {
        // If the setpoint is 0, turn off the PID loop
        if (motor->setpoint_PID == 0) {
            zsx11d_driver_set_pwm(motor, 0);  // Set PWM to 0
            motor->integral_PID = 0;          // Reset integral term
            motor->previous_error_PID = 0;    // Reset previous error
        } else {
            // Perform PID calculations
            float error = motor->setpoint_PID - fabs(zsx11d_driver_get_rpm(motor));
            motor->error_PID = error;
            motor->integral_PID += error * motor->config.sample_period_s;
            float derivative = (error - motor->previous_error_PID) / motor->config.sample_period_s;
            float output = (motor->config.kp * error) + (motor->config.ki * motor->integral_PID) + (motor->config.kd * derivative);
            motor->previous_error_PID = error;
            if (output > motor->config.max_pwm) output = motor->config.max_pwm;
            if (output < motor->config.min_pwm) output = motor->config.min_pwm;
            zsx11d_driver_set_pwm(motor, output);
        }

        // Delay for the next loop iteration
        vTaskDelay(xDelay);
    }
}
