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

typedef struct our_handles_t
{
    spi_handle_t adc;
    uart_port_t sas;
    spi_handle_t accelerometer;
    spi_handle_t gyroscope;
} our_handles_t;

typedef struct motor_temps_t
{
   float[4] temps;
} motor_temps_t;

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

    return handles;
}

int16_t accel_reading_to_count(char *data)
{
    uint16_t ucounts = data[1];
    ucounts <<= 8;
    ucounts |= data[0];
    int16_t counts = ucounts;
}

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
        int16_t x = accel_reading_to_count(data + 1);
        int16_t y = accel_reading_to_count(data + 3);
        int16_t z = accel_reading_to_count(data + 5);
        // TODO add xyz to queue
        // TODO Block on waiting for interupt line to go high (level trigger) again
    }
}

int16_t rot_reading_to_count(char *data)
{
    uint16_t ucounts = data[1];
    ucounts <<= 8;
    ucounts |= data[0];
    int16_t counts = ucounts;
}

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
        int16_t x = rot_reading_to_count(data + 0);
        int16_t y = rot_reading_to_count(data + 2);
        int16_t z = rot_reading_to_count(data + 4);
        // TODO add xyz to queue
        // TODO Block on waiting for interupt line to go high (level trigger) again
    }
}

float read_sas(uart_port_t sas_handle)
{
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
        counts -= 2048;
        // TODO: Add data to queue
        vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS); // 10 Hz
    }
}

motor_temps_t read_adc_task()
{
    spi_device_handle_t adc_handle;
    spi_transaction_t transmissions[5];
    TickType_t xLastWakeTime;
    uint8_t k;
    uint8_t addresses[4];
    float pullups[4];
    uint16_t temp;
    float A = 0.0039083.f;
    float B = -0.0000005775.f;
    float midval;
#ifdef MOTOR_TEMPERATURE_ADC_0
    addresses[k] = 0;
    pullups[k++] = (float) MOTOR_TEMP_PULLUP_0
#endif
#ifdef MOTOR_TEMPERATURE_ADC_1
    addresses[k++] = 1;
    pullups[k++] = (float) MOTOR_TEMP_PULLUP_1
#endif
#ifdef MOTOR_TEMPERATURE_ADC_2
    addresses[k++] = 2;
    pullups[k++] = (float) MOTOR_TEMP_PULLUP_2
#endif
#ifdef MOTOR_TEMPERATURE_ADC_3
    addresses[k++] = 3;
    pullups[k++] = (float) MOTOR_TEMP_PULLUP_3
#endif
#ifdef MOTOR_TEMPERATURE_ADC_4
    addresses[k++] = 4;
    pullups[k++] = (float) MOTOR_TEMP_PULLUP_4
#endif
#ifdef MOTOR_TEMPERATURE_ADC_5
    addresses[k++] = 5;
    pullups[k++] = (float) MOTOR_TEMP_PULLUP_5
#endif
#ifdef MOTOR_TEMPERATURE_ADC_6
    addresses[k++] = 6;
    pullups[k++] = (float) MOTOR_TEMP_PULLUP_6
#endif
#ifdef MOTOR_TEMPERATURE_ADC_7
    addresses[k++] = 7;
    pullups[k++] = (float) MOTOR_TEMP_PULLUP_7
#endif
    for (int adc_num = 0; adc_num < 4; ++adc_num) {
       transmissions[adc_num] = {
          .length = 11,
          .cmd = 0x1,
          .addr = addresses[adc_num],
          .flags = SPI_TRANS_USE_RXDATA,
       };
    }
    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        uint16_t data[4];
            // queue them all at once so that we have to switch context less (hopefully)
        }
        for (int adc_num = 0; adc_num < 4; ++adc_num) {
            ESP_ERROR_CHECK(spi_device_queue_trans(adc_handle, transmissions + adc_num, portMAX_DELAY));
        }
        for (int i = 0; i < 4; ++i)
        {
            spi_transaction_t *completed_transmission; // will point one of the positions in the array above
            ESP_ERROR_CHECK(spi_device_get_trans_result(adc_handle, &completed_transmission, portMAX_DELAY));
            if (unlikely(completed_transmission - transmissions > 4))
            {
                abort();
            }
            int adc_num = completed_transmission->addr;
            data[adc_num] = t.rx_data[0] & 0x3;
            data[adc_num] <<= 8;
            data[adc_num] |= t.rx_data[1];
        }
        motor_temps_t motortemps;
        // TODO: add data to queue
        for (int i = 0; i < 4; ++i) {
           midval = 1.f*data[i]/2048.f;
           midval = (1.f-0.001f*pullups*midval)/(1.f-midval);
           midval = A * A - 4 * B * midval * midval;
           midval = (-A-sqrtf(midval))/2.f/B;
           motortemps.temps[i] = midval;
        }
        vTaskDelayUntil(&xLastWakeTime, 4 / portTICK_PERIOD_MS); // 250 Hz
    }
return motortemps;
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
        if (can_transmit(&message, pdMS_TO_TICKS(PREVCU_SEND_TORQUE_PERIOD / 2)) != ESP_OK)
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
            current_time - current_time - last_heartbeat_message[3]);
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
