/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/twai.h"

static const char *TAG = "example";

float pi = 3.14159f;

typedef struct queue_element_t
{
    uint16_t header;
    char buffer[32];
} queue_element_t;

enum errors
{
    CRITICAL_ERRORS_GREATER
}

enum headers
{
    ADC_UPDATE_HEADER,
    STEERING_QUEUE_HEADER,
    ROT_QUEUE_HEADER,
    ACCEL_QUEUE_HEADER,
    SPEED_UPDATE_HEADER,
    MC_TEMP_UPDATE_HEADER,
    MC_TORQUE_UPDATE_HEADER,
};

void write_err(uint32_t err)
{
    twai_message_t message = {
        .identifier = 0x140,
        .data_length_code = 4,
        .data = {
            (uint8_t)(err << 0),
            (uint8_t)(err << 8),
            (uint8_t)(err << 16),
            (uint8_t)(err << 24),
        },
    };

    // Queue message for transmission
    if (twai_transmit(&message, 10) != ESP_OK)
    {
        esp_restart();
    }
    if (err > CRITICAL_ERRORS_GREATER)
    {
        esp_restart();
    }
}

uint16_t id_array[4] = {CONFIG_PREVCU_FL_ADDR, CONFIG_PREVCU_FR_ADDR, CONFIG_PREVCU_BL_ADDR, CONFIG_PREVCU_BR_ADDR};

typedef struct our_handles_t
{
    spi_device_handle_t adc;
    uart_port_t sas;
    spi_device_handle_t accelerometer;
    spi_device_handle_t gyroscope;
    QueueHandle_t queue_handle;
    TaskHandle_t twai_receive_th;
    TaskHandle_t send_torques_th;
    TaskHandle_t read_adc_th;
    TaskHandle_t read_accel_th;
    TaskHandle_t read_speeds_th;
    TaskHandle_t read_torques_th;
    TaskHandle_t read_gyro_th;
    TaskHandle_t read_mc_temps_th;
    TaskHandle_t read_sas_th;
    volatile uint32_t calculated_torques; // 32 bits have no tearing
    volatile uint32_t lifted_throttle;    // 32 bits have no tearing.
} our_handles_t;

static void IRAM_ATTR wake_task_isr(void *args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TaskHandle_t xHandlingTask = (TaskHandle_t)args;
    xTaskNotifyIndexedFromISR(xHandlingTask, 1, 0, eNoAction, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

esp_err_t write_imu(spi_device_handle_t imu, uint8_t addr, uint8_t data)
{
    spi_transaction_t t = {
        .length = 8,
        .cmd = 0x0,
        .addr = addr,
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {data},
    };
    return spi_device_transmit(imu, &t);
}

void wait_for_start()
{
    xTaskNotifyWaitIndexed(0, 0, 0, NULL, portMAX_DELAY);
}

float accel_reading_conversion(char *data)
{
    uint16_t ucounts = data[1];
    ucounts <<= 8;
    ucounts |= data[0];
    int16_t counts = ucounts;
    return counts / 32768.f * 1000.f * 1.5f * (1 << CONFIG_PREVCU_ACCEL_FULL_SCALE);
}

typedef struct accel_update_t
{
    TickType_t time;
    float x;
    float y;
    float z;
} accel_update_t;
void read_accel_task(void *handle_void)
{
    wait_for_start();
    our_handles_t *handles = (our_handles_t *)handle_void;
    for (;;)
    {

        char data[13];
        // basically, keep reading through the reserved stuff until you hit the interupt reset in burst mode. Faster than stopping then restarting the transmission
        spi_transaction_t t = {
            .length = 104,
            .cmd = 0x1,
            .addr = 0x12,
            .rx_buffer = data,
        };
        ESP_ERROR_CHECK(spi_device_transmit(handles->accelerometer, &t));
        accel_update_t accel_data;
        accel_data.x = accel_reading_conversion(data + 1);
        accel_data.y = accel_reading_conversion(data + 3);
        accel_data.z = accel_reading_conversion(data + 5);
        queue_element_t queue_element;
        queue_element.header = ACCEL_QUEUE_HEADER;
        accel_data.time = xTaskGetTickCount();
        memcpy(queue_element.buffer, &accel_data, sizeof(accel_data));

        xQueueSendToBack(handles->queue_handle, &queue_element, portMAX_DELAY);
        // just block me until the ISR tells me to go again
        xTaskNotifyWaitIndexed(1, 0, 0, NULL, portMAX_DELAY);
    }
}

float gyro_reading_conversion(char *data)
{
    uint16_t ucounts = data[1];
    ucounts <<= 8;
    ucounts |= data[0];
    int16_t counts = ucounts;
    return counts / 32767.f * pi / 180.f * 2000.f / (1 << CONFIG_PREVCU_GYRO_FULL_SCALE);
}

typedef struct rot_update_t
{
    TickType_t time;
    float x;
    float y;
    float z;
} gyro_update_t;

void read_gyro_task(void *handle_void)
{
    wait_for_start();
    our_handles_t *handles = (our_handles_t *)handle_void;
    for (;;)
    {
        char data[8];
        // basically, keep reading through the reserved stuff until you hit the interupt reset in burst mode. Faster than stopping then restarting the transmission
        spi_transaction_t t = {
            .length = 72,
            .cmd = 0x1,
            .addr = 0x02,
            .rx_buffer = data,
        };
        ESP_ERROR_CHECK(spi_device_transmit(handles->gyroscope, &t));
        gyro_update_t gyro_data;
        gyro_data.x = accel_reading_conversion(data + 0);
        gyro_data.y = accel_reading_conversion(data + 2);
        gyro_data.z = accel_reading_conversion(data + 4);
        queue_element_t queue_element;
        queue_element.header = ROT_QUEUE_HEADER;
        gyro_data.time = xTaskGetTickCount();

        memcpy(queue_element.buffer, &gyro_data, sizeof(gyro_data));
        xQueueSendToBack(handles->queue_handle, &queue_element, portMAX_DELAY);
        // just block me until the ISR tells me to go again
        xTaskNotifyWaitIndexed(1, 0, 0, NULL, portMAX_DELAY);
    }
}

typedef struct steering_update_t
{
    TickType_t time;
    float left_wheel_radian;
    float right_wheel_radian;
} steering_update_t;

void read_sas_task(void *handle_void)
{
    wait_for_start();
    our_handles_t *handles = (our_handles_t *)handle_void;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        char addr = 0x54;
        if (unlikely(uart_write_bytes(handles->sas, &addr, 1) != 1))
        {
            abort();
        }
        ESP_ERROR_CHECK(uart_wait_tx_done(handles->sas, portMAX_DELAY));
        char data[2];
        if (unlikely(uart_read_bytes(handles->sas, data, 2, portMAX_DELAY) != 2))
        {
            abort();
        }
        uint16_t ucounts = data[1];
        ucounts <<= 8;
        ucounts |= data[0];
        int16_t counts = ucounts;
        counts -= CONFIG_PREVCU_SAS_FORWARD;
        if (counts < 0)
        {
            counts += 4096;
        }
        counts -= 2048;
        float rack_angle = counts / 2.f / pi;
        steering_update_t steering_data = {
            .left_wheel_radian = rack_angle,
            .right_wheel_radian = rack_angle,
        };
        queue_element_t queue_element;
        queue_element.header = STEERING_QUEUE_HEADER;
        steering_data.time = xTaskGetTickCount();

        memcpy(queue_element.buffer, &steering_data, sizeof(steering_data));
        xQueueSendToBack(handles->queue_handle, &queue_element, portMAX_DELAY);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONFIG_PREVCU_READ_CONTROLLER_TEMPS_PERIOD));
    }
}

// TODO: Switch to pulldown
float adc_to_temp(uint16_t adc_val, float pulldown)
{
    float ret;
    float A = 0.0039083f;
    float B = -0.0000005775f;
    ret = 1.f * adc_val / 2048.f;
    ret = 1.f - (0.001f * pulldown * (1.f - ret)) / ret;
    ret = A * A - 4 * B * ret;
    ret = (-A - sqrtf(ret)) / 2.f / B;
    return ret;
}

typedef struct adc_update_t
{
    TickType_t time;
    float throttle_percent;
    uint8_t temps[4];
    bool includes_temp;
} adc_update_t;

void read_adc_task(void *handle_void)
{
    wait_for_start();
    our_handles_t *handles = (our_handles_t *)handle_void;
    spi_transaction_t throttle_transmission = {
        .length = 11,
        .addr = CONFIG_PREVCU_THROTTLE_ADC_NUM,
        .cmd = 0x1,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    struct
    {
        spi_transaction_t fl, fr, bl, br;
    } transmissions = {
        .fl = throttle_transmission,
        .fr = throttle_transmission,
        .bl = throttle_transmission,
        .br = throttle_transmission,
    };
    transmissions.fl.addr = CONFIG_PREVCU_FL_ADC_NUM;
    transmissions.fr.addr = CONFIG_PREVCU_FR_ADC_NUM;
    transmissions.bl.addr = CONFIG_PREVCU_BL_ADC_NUM;
    transmissions.br.addr = CONFIG_PREVCU_BR_ADC_NUM;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint16_t divisor = (CONFIG_PREVCU_READ_CONTROLLER_TEMPS_PERIOD + CONFIG_PREVCU_READ_THROTTLE_PERIOD - 1) / CONFIG_PREVCU_READ_THROTTLE_PERIOD;
    uint16_t current_count = 0;
    for (;;)
    {
        adc_update_t update;
        uint8_t read_temps = 0;
        if (current_count == 0)
        {
            update.includes_temp = true;
            read_temps = 4;
            current_count = divisor;
        }
        else
        {
            update.includes_temp = false;
            --current_count;
        }
        // queue them all at once so that we have to switch context less (hopefully)
        ESP_ERROR_CHECK(spi_device_queue_trans(handles->adc, &throttle_transmission, portMAX_DELAY));
        if (read_temps)
        {
            ESP_ERROR_CHECK(spi_device_queue_trans(handles->adc, &transmissions.fl, portMAX_DELAY));
            ESP_ERROR_CHECK(spi_device_queue_trans(handles->adc, &transmissions.fr, portMAX_DELAY));
            ESP_ERROR_CHECK(spi_device_queue_trans(handles->adc, &transmissions.bl, portMAX_DELAY));
            ESP_ERROR_CHECK(spi_device_queue_trans(handles->adc, &transmissions.br, portMAX_DELAY));
        }
        for (int i = 0; i < 1 + read_temps; ++i)
        {
            spi_transaction_t *completed_transmission; // will point one of the positions in the array above
            ESP_ERROR_CHECK(spi_device_get_trans_result(handles->adc, &completed_transmission, portMAX_DELAY));
            uint16_t adc_result = completed_transmission->rx_data[0] & 0x3;
            adc_result <<= 8;
            adc_result |= completed_transmission->rx_data[1];
            if (completed_transmission == &throttle_transmission)
            {
                update.throttle_percent = adc_result / 2048.f * 100.f;
                if (update.throttle_percent > CONFIG_PREVCU_ERR_MAX_THROTTLE_PERCENT)
                {
                    // error
                }
                if (update.throttle_percent < CONFIG_PREVCU_ERR_MIN_THROTTLE_PERCENT)
                {
                    // error
                }
                update.throttle_percent = (update.throttle_percent - CONFIG_PREVCU_MIN_THROTTLE_PERCENT) / (CONFIG_PREVCU_MAX_THROTTLE_PERCENT - CONFIG_PREVCU_MIN_THROTTLE_PERCENT);
                if (update.throttle_percent < 0)
                {
                    update.throttle_percent = 0;
                    handles->lifted_throttle = 1;
                }
                else
                {
                    handles->lifted_throttle = 0;
                }
                if (update.throttle_percent > 1)
                {
                    update.throttle_percent = 1;
                }
            }
            else if (read_temps)
            {
                if (completed_transmission == &transmissions.fl)
                {
                    update.temps[0] = adc_to_temp(adc_result, CONFIG_PREVCU_FL_PULLDOWN);
                }
                else if (completed_transmission == &transmissions.fr)
                {
                    update.temps[1] = adc_to_temp(adc_result, CONFIG_PREVCU_FR_PULLDOWN);
                }
                else if (completed_transmission == &transmissions.bl)
                {
                    update.temps[2] = adc_to_temp(adc_result, CONFIG_PREVCU_BL_PULLDOWN);
                }
                else if (completed_transmission == &transmissions.br)
                {
                    update.temps[3] = adc_to_temp(adc_result, CONFIG_PREVCU_BR_PULLDOWN);
                }
                else
                {
                    // Error
                }
            }
            else
            {
                // error
            }
        }
        update.time = xTaskGetTickCount();

        queue_element_t queue_element;
        queue_element.header = ADC_UPDATE_HEADER;
        memcpy(queue_element.buffer, &update, sizeof(update));
        xQueueSendToBack(handles->queue_handle, &queue_element, portMAX_DELAY);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONFIG_PREVCU_READ_THROTTLE_PERIOD));
    }
}

void read_data(unsigned int address, unsigned int sub_index)
{
    uint8_t b1 = (address >> 8);
    uint8_t b2 = (address >> 0);
    for (int i = 0; i < 4; i++)
    {
        twai_message_t message = {
            .identifier = 0x600 + id_array[i],
            .data_length_code = 8,
            .data = {
                0x40,
                // info address little endian
                b2,
                b1,
                sub_index,
                // zeroes for padding
                0x00,
                0x00,
                0x00,
                0x00,
            },
        };

        // Queue message for transmission
        if (twai_transmit(&message, pdMS_TO_TICKS(CONFIG_PREVCU_CAN_TIMEOUT)) != ESP_OK)
        {
            // TODO we weren't able to send our message in a reasonable amount of time. We should probably tell someone and reboot
        }
    }
}

void read_speeds_task(void *handle_void)
{
    wait_for_start();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        read_data(0x606C, 0x00);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONFIG_PREVCU_READ_SPEEDS_PERIOD));
    }
}

void read_mc_temps_task(void *handle_void)
{
    wait_for_start();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        read_data(0x2026, 0x00);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONFIG_PREVCU_READ_CONTROLLER_TEMPS_PERIOD));
    }
}

void read_real_torque_task(void *handle_void)
{
    wait_for_start();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        read_data(0x6077, 0x00);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONFIG_PREVCU_READ_MOTOR_TORQUES_PERIOD));
    }
}

void send_torques_task(void *handle_void)
{
    wait_for_start();
    our_handles_t *handles = (our_handles_t *)handle_void;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        if (handles->lifted_throttle == 2)
        {
            // we didn't recieve an update. Abort immediately
        }
        handles->lifted_throttle = 2;
        uint8_t torques[4];
        if (handles->lifted_throttle)
        {
            // The driver has lifted their foot off the pedal. No matter what, we need to stop
            memset(torques, 0, 4);
        }
        else
        {
            uint32_t calculated_torques_long = handles->calculated_torques;
            torques[0] = calculated_torques_long;       // fl
            torques[1] = calculated_torques_long >> 8;  // fr
            torques[2] = calculated_torques_long >> 16; // bl
            torques[3] = calculated_torques_long >> 24; // br
        }

        for (int i = 0; i < 4; ++i)
        {
            twai_message_t message = {
                .identifier = 0x600 + id_array[i],
                .data_length_code = 8,
                .data = {
                    0x2B, // magic numbers for CANopen
                    0x71,
                    0x60, // 0x6071 is the specifier
                    0x00,
                    torques[i], // 0-255
                    0x0,
                    0x0, // CANopen padding
                    0x0,
                },
            };

            // Queue message for transmission
            if (twai_transmit(&message, pdMS_TO_TICKS(CONFIG_PREVCU_CAN_TIMEOUT)) != ESP_OK)
            {
                // TODO we weren't able to send our message in a reasonable amount of time. We should probably tell someone and reboot
            }
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONFIG_PREVCU_SEND_TORQUES_PERIOD));
    }
}
typedef struct speed_update_t
{
    TickType_t time;
    int32_t speeds[4];
} speed_update_t;

typedef struct mc_temp_update_t
{
    TickType_t time;
    uint8_t temps[4];
} mc_temp_update_t;

typedef struct mc_torque_update_t
{
    TickType_t time;
    uint16_t torques[4];
} mc_torque_update_t;

// takes care of all of the receiving
void twai_receive_task(void *handle_void)
{
    wait_for_start();
    our_handles_t *handles = (our_handles_t *)handle_void;
    TickType_t current_time = xTaskGetTickCount();
    TickType_t last_heartbeat_message[4] = {current_time, current_time, current_time, current_time};

    uint32_t latest_speeds[4];
    uint8_t waiting_speeds = 0;

    uint8_t latest_mc_temps[4];
    uint8_t waiting_mc_temps = 0;

    uint16_t latest_torques[4];
    uint8_t waiting_torques = 0;
    for (;;)
    {
        TickType_t ticks_till_timeout;
        TickType_t oldest_heartbeat = last_heartbeat_message[0];
        if (oldest_heartbeat > last_heartbeat_message[1])
        {
            oldest_heartbeat = last_heartbeat_message[1];
        }
        if (oldest_heartbeat > last_heartbeat_message[2])
        {
            oldest_heartbeat = last_heartbeat_message[2];
        }
        if (oldest_heartbeat > last_heartbeat_message[3])
        {
            oldest_heartbeat = last_heartbeat_message[3];
        }
        TickType_t max_time_diff = xTaskGetTickCount() - oldest_heartbeat;
        if (max_time_diff >= pdMS_TO_TICKS(500))
        {
            ticks_till_timeout = 0;
        }
        else
        {
            ticks_till_timeout = pdMS_TO_TICKS(500) - max_time_diff;
        }
        twai_message_t message;

        if (twai_receive(&message, ticks_till_timeout) != ESP_OK)
        {
            // report heartbeat timeout error and shutdown
        }
        uint16_t command = message.identifier & 0x780;
        uint16_t node_id = message.identifier - 0x7F;

        uint8_t index;
        switch (node_id)
        {
        case CONFIG_PREVCU_FL_ADDR:
            index = 0;
            break;
        case CONFIG_PREVCU_FR_ADDR:
            index = 1;
            break;
        case CONFIG_PREVCU_BL_ADDR:
            index = 2;
            break;
        case CONFIG_PREVCU_BR_ADDR:
            index = 3;
            break;
        default:
            abort();
            // we got a message we didn't expect.
            break;
        }
        if (command == 0x700)
        {
            last_heartbeat_message[index] = xTaskGetTickCount();
            return;
        }
        else if (command == 0x580)
        {
            uint32_t index_and_subindex = message.data[1];
            index_and_subindex <<= 8;
            index_and_subindex |= message.data[2];
            index_and_subindex <<= 8;
            index_and_subindex |= message.data[3];
            switch (index_and_subindex)
            {
            case 0x606C00:
                // speed
                latest_speeds[index] = message.data[4];
                latest_speeds[index] <<= 8;
                latest_speeds[index] |= message.data[5];
                latest_speeds[index] <<= 8;
                latest_speeds[index] |= message.data[6];
                latest_speeds[index] <<= +8;
                latest_speeds[index] |= message.data[7];
                waiting_speeds |= 1 << index;
                if (waiting_speeds == 0xF)
                {
                    waiting_speeds = 0;
                    queue_element_t queue_element;
                    queue_element.header = SPEED_UPDATE_HEADER;
                    speed_update_t update;
                    update.time = xTaskGetTickCount();
                    memcpy(update.speeds, latest_speeds, sizeof(latest_speeds));
                    memcpy(queue_element.buffer, &update, sizeof(update));
                    xQueueSendToBack(handles->queue_handle, &queue_element, portMAX_DELAY);
                }
                break;
            case 0x202600:
                latest_mc_temps[index] = message.data[4];
                waiting_mc_temps |= 1 << index;
                if (waiting_mc_temps == 0xF)
                {
                    waiting_mc_temps = 0;
                    queue_element_t queue_element;
                    queue_element.header = MC_TEMP_UPDATE_HEADER;
                    mc_temp_update_t update;
                    update.time = xTaskGetTickCount();
                    memcpy(update.temps, latest_mc_temps, sizeof(latest_mc_temps));
                    memcpy(queue_element.buffer, &update, sizeof(update));
                    xQueueSendToBack(handles->queue_handle, &queue_element, portMAX_DELAY);
                }
                break;
            case 0x607700:
                latest_torques[index] = message.data[4];
                latest_torques[index] <<= 8;
                latest_torques[index] |= message.data[5];
                waiting_torques |= 1 << index;
                if (waiting_torques == 0xF)
                {
                    waiting_torques = 0;
                    queue_element_t queue_element;
                    queue_element.header = MC_TORQUE_UPDATE_HEADER;
                    mc_torque_update_t update;
                    update.time = xTaskGetTickCount();
                    memcpy(update.torques, latest_torques, sizeof(latest_torques));
                    memcpy(queue_element.buffer, &update, sizeof(update));
                    xQueueSendToBack(handles->queue_handle, &queue_element, portMAX_DELAY);
                }
                break;
            default:
                // we got a message we didnt expect
                break;
            }
        }
    }
}

uint8_t find_max_temp(uint8_t temps[4])
{
    uint8_t ret = temps[0];
    if (ret < temps[1])
    {
        ret = temps[1];
    }
    if (ret < temps[2])
    {
        ret = temps[2];
    }
    if (ret < temps[3])
    {
        ret = temps[3];
    }
    return ret;
}

void proccessing_task(void *handle_void)
{
    wait_for_start();
    our_handles_t *handles = (our_handles_t *)handle_void;
    float throttle = 0;
    float speed_estimates[4] = {0, 0, 0, 0};
    float speed_l_offset = 0;
    float wheel_speeds[4] = {0, 0, 0, 0};
    float motor_temp_ramp = 1;
    float controller_temp_ramp = 1;
    uint8_t motor_temps[4];
    uint8_t controller_temps[4];
    float accumulated_errors[4];
    TickType_t last_accel_tick = 0;
    for (;;)
    {
        queue_element_t generic_element;
        ESP_ERROR_CHECK(xQueueReceive(handles->queue_handle, &generic_element, portMAX_DELAY));
        switch (generic_element.header)
        {
        case ADC_UPDATE_HEADER:
        {
            adc_update_t *update = (adc_update_t *)generic_element.buffer;
            throttle = update->throttle_percent;
            if (update->includes_temp)
            {
                memcpy(motor_temps, update->temps, sizeof(update->temps));
                uint8_t max_temp = find_max_temp(update->temps);
                if (max_temp < 100)
                {
                    motor_temp_ramp = 1.f;
                }
                else if (max_temp > 150)
                {
                    motor_temp_ramp = 0.f;
                }
                else
                {
                    motor_temp_ramp = 1.f - ((max_temp - 100.f) / 50.f);
                }
            }
        }
        break;
        case STEERING_QUEUE_HEADER:
        {
            steering_update_t *update = (steering_update_t *)generic_element.buffer;
        }
        break;
        case ROT_QUEUE_HEADER:
        {
            gyro_update_t *update = (gyro_update_t *)generic_element.buffer;
            speed_l_offset = -update->z * CONFIG_PREVCU_HALF_WHEELBASE_MM / 1000.f;
        }
        break;
        case ACCEL_QUEUE_HEADER:
        {
            accel_update_t *update = (accel_update_t *)generic_element.buffer;
            if (throttle != 0 && last_accel_tick)
            {
                for (int i = 0; i < 4; i++)
                {
                    speed_estimates[i] += update->x * (update->time - last_accel_tick) * portTICK_PERIOD_MS / 1000;
                }
            }
            last_accel_tick = update->time;
        }
        break;
        case SPEED_UPDATE_HEADER:
        {
            speed_update_t *update = (speed_update_t *)generic_element.buffer;
            if (throttle == 0)
            {
                for (int i = 0; i < 4; i++)
                {
                    speed_estimates[i] = update->speeds[i] * CONFIG_PREVCU_WHEEL_RADIUS_MM / 1000.f;
                }
            }
            for (int i = 0; i < 4; i++)
            {
                wheel_speeds[i] = update->speeds[i] * CONFIG_PREVCU_WHEEL_RADIUS_MM / 1000.f;
            }
        }
        break;
        case MC_TEMP_UPDATE_HEADER:
        {
            mc_temp_update_t *update = (mc_temp_update_t *)generic_element.buffer;
            memcpy(controller_temps, update->temps, sizeof(update->temps));
            uint8_t max_temp = find_max_temp(update->temps);
            if (max_temp < 75)
            {
                controller_temp_ramp = 1.f;
            }
            else if (max_temp > 100)
            {
                controller_temp_ramp = 0.f;
            }
            else
            {
                controller_temp_ramp = 1.f - ((max_temp - 75.f) / 25.f);
            }
        }
        break;
        case MC_TORQUE_UPDATE_HEADER:
        {
            mc_torque_update_t *update = (mc_torque_update_t *)generic_element.buffer;
        }
        break;
        default:
            // todo: unexpected header
            break;
        }
        bool offset_l = true;
        uint8_t requested_torques[4];
        uint8_t max_requested_torque = 0;
        for (int i = 0; i < 4; i++)
        {
            float offset = offset_l ? speed_l_offset : -speed_l_offset;
            float error = 0.19 * throttle - (wheel_speeds[i] - speed_estimates[i] - offset) / speed_estimates[i];
            requested_torques[i] = error * 500 + accumulated_errors[i] * 10;
            if (requested_torques[i] > max_requested_torque)
            {
                max_requested_torque = requested_torques[i];
            }
            accumulated_errors[i] += error;
            offset_l = !offset_l;
        }
        float ramp = fmin(controller_temp_ramp, motor_temp_ramp);
        if (ramp < 1 && max_requested_torque > ramp * 255)
        {
            float eff_ramp = ramp * 255 / max_requested_torque;
            for (int i = 0; i < 4; ++i)
            {
                requested_torques[i] = floor(eff_ramp * requested_torques[i]);
            }
        }
        handles->calculated_torques = *(uint32_t *)requested_torques;
    }
}

void setup()
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = CONFIG_PREVCU_GPIO_MOSI,
        .miso_io_num = CONFIG_PREVCU_GPIO_MISO,
        .sclk_io_num = CONFIG_PREVCU_GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_host_device_t spi_host = SPI2_HOST;

    if (spi_bus_initialize(spi_host, &buscfg, SPI_DMA_DISABLED) != ESP_OK)
    {
        write_err(FAILED_SPI_SETUP);
    }

    // this ADC is garbage, but it's what PCB team has given us.
    spi_device_interface_config_t adc_device_config = {
        .flags = SPI_DEVICE_HALFDUPLEX,
        .command_bits = 8,
        .address_bits = 4,
        .dummy_bits = 0,
        .clock_speed_hz = 500000, // fast enough that the capacitor doesn't lose charge. Slow enough it can build it up
        .duty_cycle_pos = 128,    // 50% duty cycle
        .mode = 0,
        .spics_io_num = CONFIG_PREVCU_GPIO_CS_ADC,
        .cs_ena_pretrans = 3,
        .cs_ena_posttrans = 3, // Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size = 8,
    };
    our_handles_t *handles = malloc(sizeof(our_handles_t));
    if (spi_bus_add_device(spi_host, &adc_device_config, &handles->adc) != ESP_OK)
    {
        write_err(FAILED_TO_SETUP_ADC);
    }
    /*
    uart_config_t uart_config = {
        .baud_rate = 2000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    handles->sas = UART_NUM_0;
    ESP_ERROR_CHECK(uart_driver_install(handles->sas, 32, 32, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(handles->sas, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(handles->sas, CONFIG_PREVCU_GPIO_SAS_TX, CONFIG_PREVCU_GPIO_SAS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(handles->sas, UART_MODE_RS485_HALF_DUPLEX));
    ESP_ERROR_CHECK(uart_set_rx_timeout(handles->sas, 10));
    */

    spi_device_interface_config_t accelerometer_device_config = {
        .flags = SPI_DEVICE_HALFDUPLEX,
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0, // dummy bits depend on if its a read or a write
        .clock_speed_hz = 1000000,
        .duty_cycle_pos = 128, // 50% duty cycle
        .mode = 0,
        .spics_io_num = CONFIG_PREVCU_GPIO_CS_ACCEL,
        .cs_ena_pretrans = 3,
        .cs_ena_posttrans = 3, // Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size = 3,
    };
    if (spi_bus_add_device(spi_host, &accelerometer_device_config, &handles->accelerometer) != ESP_OK)
    {
        write_err(FAILED_TO_WRITE_ACCEL);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
    // dummy accelerometer spi read to activate spi
    spi_transaction_t t = {
        .rxlength = 16,
        .flags = SPI_TRANS_USE_RXDATA,
        .cmd = 0x1,
        .addr = 0x0,
    };
    if ((spi_device_transmit(handles->accelerometer, &t)) != ESP_OK)
    {
        write_err(FAILED_TO_WRITE_ACCEL);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    if (write_imu(handles->accelerometer, 0x7D, 0x4) != ESP_OK)
    {
        write_err(FAILED_TO_WRITE_ACCEL);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    write_imu(handles->accelerometer, 0x52 + CONFIG_PREVCU_ACCEL_INT, 0xC + 0x4 * CONFIG_PREVCU_ACCEL_INT_TYPE);
    if (CONFIG_PREVCU_ACCEL_INT == 1)
    {
        if (write_imu(handles->accelerometer, 0x58, 0x4) != ESP_OK)
        {
            write_err(FAILED_TO_WRITE_ACCEL);
        }
    }
    else
    {
        if (write_imu(handles->accelerometer, 0x58, 0x40) != ESP_OK)
        {
            write_err(FAILED_TO_WRITE_ACCEL);
        }
    }
    if (write_imu(handles->accelerometer, 0x41, CONFIG_PREVCU_ACCEL_FULL_SCALE) != ESP_OK)
    {
        write_err(FAILED_TO_WRITE_ACCEL);
    }
    if (write_imu(handles->accelerometer, 0x40, CONFIG_PREVCU_ACCEL_OVERSAMPLING << 4 | CONFIG_PREVCU_ACCEL_ODR) != ESP_OK)
    {
        write_err(FAILED_TO_WRITE_ACCEL);
    }

    spi_device_interface_config_t gyroscope_device_config = {
        .flags = SPI_DEVICE_HALFDUPLEX,
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
        .clock_speed_hz = 1000000,
        .duty_cycle_pos = 128, // 50% duty cycle
        .mode = 0,
        .spics_io_num = CONFIG_PREVCU_GPIO_CS_GYRO,
        .cs_ena_pretrans = 3,
        .cs_ena_posttrans = 3, // Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size = 3,
    };
    if (spi_bus_add_device(spi_host, &gyroscope_device_config, &handles->gyroscope) != ESP_OK)
    {
        write_err(FAILED_TO_WRITE_GYRO);
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    if (write_imu(handles->gyroscope, 0x15, 0x80) != ESP_OK)
    {
        write_err(FAILED_TO_WRITE_GYRO);
    }
    if (CONFIG_PREVCU_GYRO_INT == 3)
    {
        if (write_imu(handles->gyroscope, 0x16, CONFIG_PREVCU_GYRO_INT_TYPE << 1) != ESP_OK)
        {
            write_err(FAILED_TO_WRITE_GYRO);
        }
        if (write_imu(handles->gyroscope, 0x18, 0x01) != ESP_OK)
        {
            write_err(FAILED_TO_WRITE_GYRO);
        }
    }
    else
    {
        if (write_imu(handles->gyroscope, 0x16, CONFIG_PREVCU_GYRO_INT_TYPE << 3) != ESP_OK)
        {
            write_err(FAILED_TO_WRITE_GYRO);
        }
        if (write_imu(handles->gyroscope, 0x18, 0x80) != ESP_OK)
        {
            write_err(FAILED_TO_WRITE_GYRO);
        }
    }
    if (write_imu(handles->gyroscope, 0xF, CONFIG_PREVCU_GYRO_FULL_SCALE) != ESP_OK)
    {
        write_err(FAILED_TO_WRITE_GYRO);
    }
    if (write_imu(handles->gyroscope, 0x10, CONFIG_PREVCU_GYRO_ODR) != ESP_OK)
    {
        write_err(FAILED_TO_WRITE_GYRO);
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CONFIG_PREVCU_CANTX_GPIO, CONFIG_PREVCU_CANRX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install CAN driver
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    {
        write_err(FAILED_TO_SETUP_CAN);
    }
    if (twai_start() != ESP_OK)
    {
        write_err(FAILED_TO_SETUP_CAN);
    }

    handles->queue_handle = xQueueCreate(10, sizeof(queue_element_t));

    gpio_set_direction(CONFIG_PREVCU_ACCEL_INT_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(CONFIG_PREVCU_ACCEL_INT_GPIO, GPIO_FLOATING);
    gpio_set_intr_type(CONFIG_PREVCU_ACCEL_INT_GPIO, GPIO_INTR_POSEDGE);

    gpio_set_direction(CONFIG_PREVCU_GYRO_INT_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(CONFIG_PREVCU_GYRO_INT_GPIO, GPIO_FLOATING);
    gpio_set_intr_type(CONFIG_PREVCU_GYRO_INT_GPIO, GPIO_INTR_POSEDGE);

    xTaskCreate(twai_receive_task, "Can Receive", configMINIMAL_STACK_SIZE + 500, (void *)handles, CONFIG_PREVCU_RECEIVE_CAN_PRIORITY, &handles->twai_receive_th);
    xTaskCreate(send_torques_task, "Send Torque", configMINIMAL_STACK_SIZE + 500, (void *)handles, CONFIG_PREVCU_SEND_TORQUES_PRIORITY, &handles->send_torques_th);
    xTaskCreate(read_adc_task, "Adc Update", configMINIMAL_STACK_SIZE + 500, (void *)handles, CONFIG_PREVCU_READ_THROTTLE_PRIORITY, &handles->read_adc_th);
    xTaskCreate(read_accel_task, "Read Accel", configMINIMAL_STACK_SIZE + 500, (void *)handles, CONFIG_PREVCU_READ_ACCEL_PRIORITY, &handles->read_accel_th);
    xTaskCreate(read_speeds_task, "Read Speeds", configMINIMAL_STACK_SIZE + 500, (void *)handles, CONFIG_PREVCU_READ_SPEEDS_PRIORITY, &handles->read_speeds_th);
    xTaskCreate(read_gyro_task, "Read Gyro", configMINIMAL_STACK_SIZE + 500, (void *)handles, CONFIG_PREVCU_READ_GYRO_PRIORITY, &handles->read_gyro_th);
    xTaskCreate(read_mc_temps_task, "Read MC Temps", configMINIMAL_STACK_SIZE + 500, (void *)handles, CONFIG_PREVCU_READ_CONTROLLER_TEMPS_PRIORITY, &handles->read_mc_temps_th);
    // xTaskCreate(read_sas_task, "Read SAS", configMINIMAL_STACK_SIZE + 500, (void *)handles, CONFIG_PREVCU_READ_SAS_PRIORITY, &handles->read_sas_th);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(CONFIG_PREVCU_ACCEL_INT_GPIO, wake_task_isr, (void *)handles->read_accel_th);
    gpio_isr_handler_add(CONFIG_PREVCU_GYRO_INT_GPIO, wake_task_isr, (void *)handles->read_gyro_th);

    // MAYBE WAIT FOR SOME SIGNAL

    // release all the tasks whose periodicity isn't controlled by us
    xTaskNotifyIndexed(handles->twai_receive_th, 0, 0, eNoAction);
    xTaskNotifyIndexed(handles->read_accel_th, 0, 0, eNoAction);
    xTaskNotifyIndexed(handles->read_gyro_th, 0, 0, eNoAction);
    // offset the periodic ones by 7ms
    xTaskNotifyIndexed(handles->read_adc_th, 0, 0, eNoAction);
    vTaskDelay(pdMS_TO_TICKS(2 * CONFIG_PREVCU_READ_THROTTLE_PERIOD + 7));
    xTaskNotifyIndexed(handles->send_torques_th, 0, 0, eNoAction);
    vTaskDelay(pdMS_TO_TICKS(7));
    xTaskNotifyIndexed(handles->read_speeds_th, 0, 0, eNoAction);
    vTaskDelay(pdMS_TO_TICKS(7));
    xTaskNotifyIndexed(handles->read_torques_th, 0, 0, eNoAction);
    vTaskDelay(pdMS_TO_TICKS(7));
    xTaskNotifyIndexed(handles->read_mc_temps_th, 0, 0, eNoAction);
    vTaskDelay(pdMS_TO_TICKS(7));
    xTaskNotifyIndexed(handles->read_sas_th, 0, 0, eNoAction);
}

void app_main(void)
{
    setup();
}
