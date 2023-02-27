/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include "driver/can.h"

static const char *TAG = "example";

struct
{
    uint16_t fl, fr, bl, br;
} mc_ids = {
    .fl = PREVCU_FL_ADDR,
    .fr = PREVCU_FR_ADDR,
    .bl = PREVCU_BL_ADDR,
    .br = PREVCU_BR_ADDR,
};

typedef struct queue_element_t
{
    uint16_t queue_header;
    char[64] buffer;
} queue_element_t;

typedef struct our_handles_t
{
    spi_handle_t adc;
    uart_port_t sas;
    spi_handle_t accelerometer;
    spi_handle_t gyroscope;
} our_handles_t;

typedef struct adc_update_t
{
    float throttle_percent, temp_fl, temp_fr, temp_bl, temp_br;
    bool includes_temp;
} adc_update_t;

our_handles_t setup()
{
    esp_err_t err;
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_DISABLED));

    // this ADC is garbage, but it's what PCB team has given us.
    spi_device_interface_config_t adc_device_config = {
        .flags = SPI_DEVICE_HALFDUPLEX,
        .command_bits = 8,
        .address_bits = 4,
        .dummy_bits = 0,
        .clock_speed_hz = 500000, // fast enough that the capacitor doesn't lose charge. Slow enough it can build it up
        .duty_cycle_pos = 128,    // 50% duty cycle
        .mode = 0,
        .spics_io_num = GPIO_CS_ADC,
        .cs_ena_pretrans = 3,
        .cs_ena_posttrans = 3, // Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size = 8,
    };
    our_handles_t handles;
    ESP_ERROR_CHECK(spi_bus_add_device(SENDER_HOST, &adc_device_config, &handles.adc));
    uart_config_t uart_config = {
        .baud_rate = 2000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    handles.sas = UART_NUM_0;
    ESP_ERROR_CHECK(uart_driver_install(handles.sas, 32, 32, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, GPIO_SAS_TX, GPIO_SAS_RX, SAS_RTS_PIN, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, 10));

    spi_device_interface_config_t accelerometer_device_config = {
        .flags = SPI_DEVICE_HALFDUPLEX,
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0, // dummy bits depend on if its a read or a write
        .clock_speed_hz = 1000000,
        .duty_cycle_pos = 128, // 50% duty cycle
        .mode = 0,
        .spics_io_num = GPIO_CS_ACCEL,
        .cs_ena_pretrans = 3,
        .cs_ena_posttrans = 3, // Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size = 3,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SENDER_HOST, &accelerometer_device_config, &handles.accelerometer));

    // TODO: wait some time

    // dummy accelerometer spi read to activate spi
    read_imu_addr(handles.acceleromter, 0x00);
    // turn accelerometer on
    write_imu_addr(handles.accelerometer, 0x7D, 0x4);

    // TODO: set odr and osr, and range

    spi_device_interface_config_t gyroscope_device_config = {
        .flags = SPI_DEVICE_HALFDUPLEX,
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
        .clock_speed_hz = 1000000,
        .duty_cycle_pos = 128, // 50% duty cycle
        .mode = 0,
        .spics_io_num = GPIO_CS_GYRO,
        .cs_ena_pretrans = 3,
        .cs_ena_posttrans = 3, // Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size = 3,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SENDER_HOST, &gyroscope_device_config, &handles.gyroscope));
    // TODO: wait some time

    // TODO: set odr and osr, and range

    can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(PREVCU_CANTX_GPIO, PREVCU_CANRX_GPIO, CAN_MODE_NORMAL);
    can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
    can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

    // Install CAN driver
    ESP_ERROR_CHECK(can_driver_install(&g_config, &t_config, &f_config))
    ESP_ERROR_CHECK(can_start());

    QueueHandle_t xQueueCreate(10, sizeof(queue_element_t));

    return handles;
}

float accel_reading_conversion(char *data)
{
    uint16_t ucounts = data[1];
    ucounts <<= 8;
    ucounts |= data[0];
    int16_t counts = ucounts;
}

typedef struct accel_data_t
{
    float x;
    float y;
    float z;
} accel_data_t;
void read_accel()
{
    spi_device_handle_t accel_handle;

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
        ESP_ERROR_CHECK(spi_device_transmit(accel_handle, &t));
        acccel_data_t accel_data;
        accel_data.x = accel_reading_conversion(data + 1);
        accel_data.y = accel_reading_conversion(data + 3);
        accel_data.z = accel_reading_conversion(data + 5);
        queue_element_t queue_element;
        queue_element.header = ACCEL_QUEUE_HEADER;
        memcpy(&queue_element.buffer, &accel_data, sizeof(accel_data));

        xQueueSendToBack(queue_handle, &queue_element, portMAX_DELAY);
        // TODO Block on waiting for interupt line to go high (level trigger) again
    }
}

float rot_reading_conversion(char *data)
{
    uint16_t ucounts = data[1];
    ucounts <<= 8;
    ucounts |= data[0];
    int16_t counts = ucounts;
}

typedef struct rot_data_t
{
    float x;
    float y;
    float z;
} rot_data_t;

void read_rot()
{
    spi_device_handle_t rot_handle;
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
        ESP_ERROR_CHECK(spi_device_transmit(rot_handle, &t));
        rot_data_t rot_data;
        rot_data.x = accel_reading_conversion(data + 0);
        rot_data.y = accel_reading_conversion(data + 2);
        rot_data.z = accel_reading_conversion(data + 4);
        queue_element_t queue_element;
        queue_element.header = ROT_QUEUE_HEADER;
        memcpy(&queue_element.buffer, &rot_data, sizeof(rot_data));
        xQueueSendToBack(queue_handle, &queue_element, portMAX_DELAY);
        // TODO Block on waiting for interupt line to go high (level trigger) again
    }
}

typedef struct steering_data_t
{
    float left_wheel_radian;
    float right_wheel_radian;
} steering_data_t;

void read_sas()
{
    uart_port_t sas_handle;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        char addr = 0x54;
        if (unlikely(uart_write_bytes(sas_handle, &addr, 1) != 1))
        {
            abort();
        }
        ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, portMAX_DELAY));
        char data[2];
        if (unlikely(uart_read_bytes(sas_handle, data, 2, portMAX_DELAY) != 2))
        {
            abort();
        }
        uint16_t ucounts = data[1];
        ucounts <<= 8;
        ucounts |= data[0];
        int16_t counts = ucounts;
        counts -= PREVCU_SAS_FORWARD;
        if (counts < 0)
        {
            counts += 4096
        }
        counts -= 2048;
        float rack_angle = counts / 2.f / 3.14159f;
        steering_data_t steering_data = {
            .left_wheel_radian = rack_angle,
            .right_wheel_radian = rack_angle,
        };
        queue_element_t queue_element;
        queue_element.header = STEERING_QUEUE_HEADER;
        memcpy(&queue_element.buffer, &steering_data, sizeof(steering_data));
        xQueueSendToBack(queue_handle, &queue_element, portMAX_DELAY);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PREVCU_READ_CONTROLLER_TEMP_PERIOD));
    }
}

// TODO: Switch to pulldown
float adc_to_temp(uint16_t adc_val, float pullup)
{
    float ret;
    float A = 0.0039083f;
    float B = -0.0000005775f;
    ret = 1.f * adc_val / 2048.f;
    ret = (1.f - 0.001f * pullup * ret) / (1.f - ret);
    ret = A * A - 4 * B * ret * ret;
    ret = (-A - sqrtf(ret)) / 2.f / B;
}

void read_adc_task()
{
    spi_device_handle_t adc_handle;
    spi_transaction_t throttle_transmission = {
        .length = 11,
        .addr = PREVCU_THROTTLE_ADC_NUM,
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
    transmission.fl.addr = PREVCU_FL_ADC_NUM;
    transmission.fr.addr = PREVCU_FR_ADC_NUM;
    transmission.bl.addr = PREVCU_BL_ADC_NUM;
    transmission.br.addr = PREVCU_BR_ADC_NUM;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint16_t divisor = (PREVCU_READ_CONTROLLER_TEMP_PERIOD + PREVCU_READ_THROTTLE_PERIOD - 1) / PREVCU_READ_THROTTLE_PERIOD;
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
        ESP_ERROR_CHECK(spi_device_queue_trans(adc_handle, &throttle_transmission, portMAX_DELAY));
        if (read_temps)
        {
            ESP_ERROR_CHECK(spi_device_queue_trans(adc_handle, &transmissions.fl, portMAX_DELAY));
            ESP_ERROR_CHECK(spi_device_queue_trans(adc_handle, &transmissions.fr, portMAX_DELAY));
            ESP_ERROR_CHECK(spi_device_queue_trans(adc_handle, &transmissions.bl, portMAX_DELAY));
            ESP_ERROR_CHECK(spi_device_queue_trans(adc_handle, &transmissions.br, portMAX_DELAY));
        }
        for (int i = 0; i < 1 + read_temps; ++i)
        {
            spi_transaction_t *completed_transmission; // will point one of the positions in the array above
            ESP_ERROR_CHECK(spi_device_get_trans_result(adc_handle, &completed_transmission, portMAX_DELAY));
            uint16_t adc_result = completed_transmission->rx_data[0] & 0x3;
            uint16_t adc_result <<= 8;
            uint16_t adc_result |= completed_transmission->rx_data[1];
            if (completed_transmission == &throttle_transmission)
            {
                update.throttle_percent = adc_result / 2048.f * 100.f;
                if (update.throttle_percent > PREVCU_ERR_MAX_THROTTLE_PERCENT)
                {
                    // error
                }
                if (update.throttle_percent < PREVCU_ERR_MIN_THROTTLE_PERCENT)
                {
                    // error
                }
                update.throttle_percent = (update.throttle_percent - PREVCU_MIN_THROTTLE_PERCENT) / (PREVCU_MAX_THROTTLE_PERCENT - PREVCU_MIN_THROTTLE_PERCENT);
            }
            else if (read_temps)
            {
                if (completed_transmission == &transmissions.fl)
                {
                    update.temp_fl = adc_to_temp(adc_result, PREVCU_FL_PULLUP);
                }
                else if (completed_transmission == &transmissions.fr)
                {
                    update.temp_fr = adc_to_temp(adc_result, PREVCU_FR_PULLUP);
                }
                else if (completed_transmission == &transmissions.bl)
                {
                    update.temp_bl = adc_to_temp(adc_result, PREVCU_BL_PULLUP);
                }
                else if (completed_transmission == &transmissions.br)
                {
                    update.temp_br = adc_to_temp(adc_result, PREVCU_BR_PULLUP);
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
        queue_element_t queue_element;
        queue_element.header = ADC_UPDATE_HEADER;
        memcpy(&queue_element.buffer, &update, sizeof(update));
        xQueueSendToBack(queue_handle, &queue_element, portMAX_DELAY);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PREVCU_READ_THROTTLE_PERIOD));
    }
}

void read_data(unsigned int address, unsigned int sub_index)
{
    uint8_t b1 = (address >> 8);
    uint8_t b2 = (address >> 0);
    can_message_t message;
    uint16_t id_array = {mc_ids.fl, mc_ids.fr, mc_ids.bl, mc_ids.br};
    for (i = 0; i < 4; i++)
    {
        message.identifier = 0x600 + id_array[i];
        message.data_length_code = 8;
        message.data = {
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
        }
        // Queue message for transmission
        if (can_transmit(&message, pdMS_TO_TICKS(PREVCU_CAN_TIMEOUT)) != ESP_OK)
        {
            // TODO we weren't able to send our message in a reasonable amount of time. We should probably tell someone and reboot
        }
    }
}

void read_speeds()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        read_data(0x606C, 0x00);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PREVCU_READ_SPEEDS_PERIOD));
    }
}

void read_mc_temps()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        read_data(0x2026, 0x00);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PREVCU_READ_CONTROLLER_TEMP_PERIOD));
    }
}

void read_real_torque()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        read_data(0x6077, 0x00);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PREVCU_READ_MOTOR_TORQUE_PERIOD));
    }
}

void send_torque()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        can_message_t message;
        message.identifier = 0x600 + mc_id;
        message.data_length_code = 8;
        message.data = {
            0x2B, // magic numbers for CANopen
            0x71,
            0x60, // 0x6071 is the specifier
            0x00,
            torque, // 0-256
            0x0,
            0x0, // CANopen padding
            0x0,
        };

        // Queue message for transmission
        if (can_transmit(&message, pdMS_TO_TICKS(PREVCU_CAN_TIMEOUT)) != ESP_OK)
        {
            // TODO we weren't able to send our message in a reasonable amount of time. We should probably tell someone and reboot
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PREVCU_SEND_TORQUE_PERIOD));
    }
}

// takes care of all of the receiving
void can_receive()
{
    TickType_t current_time = xTaskGetTickCount();
    TickType_t last_heartbeat_message[4] = {current_time, current_time, current_time, current_time};
    for (;;)
    {
        TickType_t ticks_till_timeout;
        current_time = xTaskGetTickCount();
        // TODO: Replace subtraction with something overflow aware
        max_time_diff = max(
            max(
                max(current_time - last_heartbeat_message[0],
                    current_time - last_heartbeat_message[1]),
                current_time - last_heartbeat_message[2]),
            current_time - last_heartbeat_message[3]);
        if (max_time_diff >= pdMS_TO_TICKS(500))
        {
            ticks_till_timeout = 0
        }
        else
        {
            ticks_till_timeout = pdMS_TO_TICKS(500) - max_time_diff;
        }
        can_message_t message;

        if (can_receive(&message, ticks_till_timeout) != ESP_OK)
        {
            // report heartbeat timeout error and shutdown
        }
        // process the message:
        // message.identifier
        // message.data_length_code
        // message.data
    }
}

void app_main(void)
{

    while (1)
    {
    }
}
