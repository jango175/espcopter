#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_madgwick.h"
#include "tiny_bldc.h"
#include "elrs_crsf.h"

#define I2C_NUM          I2C_NUM_0
#define SDA_PIN          GPIO_NUM_3
#define SCL_PIN          GPIO_NUM_9
#define I2C_FREQ         400000

#define UART_NUM         UART_NUM_1
#define TX_PIN           GPIO_NUM_42
#define RX_PIN           GPIO_NUM_41

#define SIGNAL_PIN_0     GPIO_NUM_7
#define SIGNAL_PIN_1     GPIO_NUM_16
#define SIGNAL_PIN_2     GPIO_NUM_17
#define SIGNAL_PIN_3     GPIO_NUM_15

#define BLDC_NUM         4

#define PID_MAX_INTEGRAL 100.0f

// #define TEST_LOOP_SPEED 0 // uncomment to measure performance

static const char* TAG = "main";


/**
 * @brief map function
 * 
 * @param x value to map
 * @param in_min minimum input value
 * @param in_max maximum input value
 * @param out_min minimum output value
 * @param out_max maximum output value
 * 
 * @return mapped value
 */
static float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
}


/**
 * @brief task for stabilised flight
 * 
 * @param arg pointer to the channels struct
 */
static IRAM_ATTR void stable_flight_task(void* arg)
{
    tiny_bldc_conf_t* bldc_conf = (tiny_bldc_conf_t*)arg;

    crsf_channels_t channels;
    bool armed = false;

    float des_roll = 0.0f;
    float des_pitch = 0.0f;

    float cur_roll = 0.0f;
    float cur_pitch = 0.0f;
    float cur_yaw = 0.0f;

    float roll_error = 0.0f;
    float pitch_error = 0.0f;
    float yaw_error = 0.0f;

    float roll_pid = 0.0f;
    float pitch_pid = 0.0f;
    float yaw_pid = 0.0f;

    float prev_roll_error = 0.0f;
    float prev_pitch_error = 0.0f;
    float prev_yaw_error = 0.0f;

    float roll_integral = 0.0f;
    float pitch_integral = 0.0f;
    float yaw_integral = 0.0f;

    // PID gains
    float roll_kp = 10.0f;
    float roll_ki = 0.0f;
    float roll_kd = 0.5f;

    float pitch_kp = 10.0f;
    float pitch_ki = 0.0f;
    float pitch_kd = 0.5f;

    float yaw_kp = 1.0f;
    float yaw_ki = 0.0f;
    float yaw_kd = 0.0f;

    uint32_t dt_ms = 3;
    float dt = dt_ms / 1000.0f;

    int32_t speed[BLDC_NUM];
    for (uint32_t i = 0; i < BLDC_NUM; i++)
        speed[i] = BLDC_MIN_ROT_SPEED;

    while (1)
    {

        // check for arm
        get_channels(&channels);

        if (channels.channel_5 < 2000 && channels.channel_5 > 1500 && channels.channel_3 < 200 && channels.channel_3 > 0)
            armed = true;
        else
            armed = false;

        if (armed)
        {
            // start bldc motors
            for (uint32_t i = BLDC_MIN_SPEED; i < BLDC_MIN_ROT_SPEED; i++)
            {
                for (uint32_t j = 0; j < BLDC_NUM; j++)
                    tiny_bldc_set_speed(&bldc_conf[j], i);

                vTaskDelay(2/portTICK_PERIOD_MS);
            }

            while (channels.channel_5 > 1500 && channels.channel_5 < 2000)
            {
#ifdef TEST_LOOP_SPEED
                int64_t loop_start = esp_timer_get_time();
#endif

                TickType_t xLastWakeTime = xTaskGetTickCount();

                // get channels
                get_channels(&channels);

                // map throttle
                int32_t throttle = (int32_t)map(channels.channel_3, 0, 2000, BLDC_MIN_ROT_SPEED, BLDC_MAX_SPEED);
                for (uint32_t i = 0; i < BLDC_NUM; i++)
                    speed[i] = throttle;

                // get current attitude
                esp_madgwick_get_attitude(&cur_roll, &cur_pitch, &cur_yaw);

                // roll pid
                // des_roll = map(channels.channel_1, 0, 2000, -30.0f, 30.0f);
                roll_error = des_roll - cur_roll;
                roll_integral += roll_error*dt;
                if (roll_integral > PID_MAX_INTEGRAL)
                    roll_integral = PID_MAX_INTEGRAL;
                else if (roll_integral < -PID_MAX_INTEGRAL)
                    roll_integral = -PID_MAX_INTEGRAL;
                roll_pid = roll_kp*roll_error + roll_ki*roll_integral + roll_kd*(roll_error - prev_roll_error)/dt;
                prev_roll_error = roll_error;

                // pitch pid
                // des_pitch = map(channels.channel_2, 0, 2000, -30.0f, 30.0f);
                pitch_error = des_pitch - cur_pitch;
                pitch_integral += pitch_error*dt;
                if (pitch_integral > PID_MAX_INTEGRAL)
                    pitch_integral = PID_MAX_INTEGRAL;
                else if (pitch_integral < -PID_MAX_INTEGRAL)
                    pitch_integral = -PID_MAX_INTEGRAL;
                pitch_pid = pitch_kp*pitch_error + pitch_ki*pitch_integral + pitch_kd*(pitch_error - prev_pitch_error)/dt;
                prev_pitch_error = pitch_error;

                // yaw pid
                // yaw_error = cur_yaw + map(channels.channel_4, 0, 2000, -0.5f, 0.5f);
                yaw_integral += yaw_error*dt;
                if (yaw_integral > PID_MAX_INTEGRAL)
                    yaw_integral = PID_MAX_INTEGRAL;
                else if (yaw_integral < -PID_MAX_INTEGRAL)
                    yaw_integral = -PID_MAX_INTEGRAL;
                yaw_pid = yaw_kp*yaw_error + yaw_ki*yaw_integral + yaw_kd*(yaw_error - prev_yaw_error)/dt;
                prev_yaw_error = yaw_error;

                // mixer
                speed[0] += (int32_t)(-pitch_pid - roll_pid + yaw_pid);
                speed[1] += (int32_t)(-pitch_pid + roll_pid - yaw_pid);
                speed[2] += (int32_t)(pitch_pid + roll_pid + yaw_pid);
                speed[3] += (int32_t)(pitch_pid - roll_pid - yaw_pid);

                // limit speeds
                for (uint32_t i = 0; i < BLDC_NUM; i++)
                {
                    if (speed[i] < BLDC_MIN_ROT_SPEED)
                        speed[i] = BLDC_MIN_ROT_SPEED;
                    else if (speed[i] > BLDC_MAX_SPEED)
                        speed[i] = BLDC_MAX_SPEED;
                }

                // set new throttle
                for (uint32_t i = 0; i < BLDC_NUM; i++)
                    tiny_bldc_set_speed(bldc_conf + i, (uint32_t)speed[i]);
                // ESP_LOGI(TAG, "%ld\t%ld\t%ld\t%ld", speed[0], speed[1], speed[2], speed[3]);

#ifdef TEST_LOOP_SPEED
                uint64_t loop_end = esp_timer_get_time() - loop_start;
                ESP_LOGI(TAG, "%lld us", loop_end);
#endif
                vTaskDelayUntil(&xLastWakeTime, dt_ms/portTICK_PERIOD_MS);
            }
        }

        // stop motors
        for (uint32_t i = 0; i < BLDC_NUM; i++)
            tiny_bldc_set_speed(bldc_conf + i, BLDC_MIN_SPEED);

        vTaskDelay(1);
    }

    vTaskDelete(NULL);
}


/**
 * @brief main function
 */
void app_main(void)
{
    // init Madgwick filter
    esp_madgwick_conf_t conf = {
        .i2c_port = I2C_NUM,
        .sda_pin = SDA_PIN,
        .scl_pin = SCL_PIN,
        .i2c_freq = I2C_FREQ
    };
    esp_madgwick_init(&conf);

    // init CRSF receiver
    crsf_conf_t crsf = {
        .uart_num = UART_NUM,
        .tx_pin = TX_PIN,
        .rx_pin = RX_PIN
    };
    crsf_init(crsf);

    // init bldc motors
    tiny_bldc_conf_t bldc_conf[BLDC_NUM];
    bldc_conf[0].pwm_pin = SIGNAL_PIN_0;
    bldc_conf[1].pwm_pin = SIGNAL_PIN_1;
    bldc_conf[2].pwm_pin = SIGNAL_PIN_2;
    bldc_conf[3].pwm_pin = SIGNAL_PIN_3;
    bldc_conf[0].group_id = 0;
    bldc_conf[1].group_id = 0;
    bldc_conf[2].group_id = 1;
    bldc_conf[3].group_id = 1;
    for (uint32_t i = 0; i < BLDC_NUM; i++)
    {
        bldc_conf[i].timer = NULL;
        bldc_conf[i].operator = NULL;
        bldc_conf[i].comparator = NULL;
        bldc_conf[i].generator = NULL;

        tiny_bldc_init(&bldc_conf[i]);
    }
    tiny_bldc_arm(bldc_conf, BLDC_NUM);

    xTaskCreate(stable_flight_task, "stable_flight_task", 4096, bldc_conf, 12, NULL);

    while (1)
    {
        vTaskDelay(portMAX_DELAY);
    }
}