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

typedef struct queue_element_t
{
    uint16_t queue_header;
    char[32] buffer;
} queue_element_t;

uint16_t id_array = {PREVCU_FL_ADDR, PREVCU_FR_ADDR, PREVCU_BL_ADDR, PREVCU_BR_ADDR};

typedef struct our_handles_t
{
    spi_handle_t adc;
    uart_port_t sas;
    spi_handle_t accelerometer;
    spi_handle_t gyroscope;
    QueueHandle_t queue_handle;
    TaskHandle_t can_receive_th;
    TaskHandle_t send_torque_th;
    TaskHandle_t adc_update_th;
    TaskHandle_t read_accel_th;
    TaskHandle_t read_speeds_th;
    TaskHandle_t read_torque_th;
    TaskHandle_t read_gyro_th;
    TaskHandle_t read_mc_temps_th;
    TaskHandle_t read_sas_th;
} our_handles_t;

static void IRAM_ATTR wake_task_isr(void *args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TaskHandle_t = (TaskHandle_t)args;
    xTaskNotifyIndexedFromISR(xHandlingTask, 1, 0, eNoAction, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void setup()
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
    our_handles_t *handles = malloc(sizeof(our_handles_t));
    ESP_ERROR_CHECK(spi_bus_add_device(SENDER_HOST, &adc_device_config, &handles->adc));
    uart_config_t uart_config = {
        .baud_rate = 2000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    handles->sas = UART_NUM_0;
    ESP_ERROR_CHECK(uart_driver_install(handles->sas, 32, 32, 0, NULL, 0));
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
    ESP_ERROR_CHECK(spi_bus_add_device(SENDER_HOST, &accelerometer_device_config, &handles->accelerometer));

    // TODO: wait some time

    // dummy accelerometer spi read to activate spi
    read_imu_addr(handles->acceleromter, 0x00);
    // turn accelerometer on
    write_imu_addr(handles->accelerometer, 0x7D, 0x4);

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
    ESP_ERROR_CHECK(spi_bus_add_device(SENDER_HOST, &gyroscope_device_config, &handles->gyroscope));
    // TODO: wait some time

    // TODO: set odr and osr, and range

    can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(PREVCU_CANTX_GPIO, PREVCU_CANRX_GPIO, CAN_MODE_NORMAL);
    can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
    can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

    // Install CAN driver
    ESP_ERROR_CHECK(can_driver_install(&g_config, &t_config, &f_config))
    ESP_ERROR_CHECK(can_start());

    handles->queue_handle = xQueueCreate(10, sizeof(queue_element_t));

    gpio_pad_select_gpio(PREVCU_ACCEL_INT_GPIO);
    gpio_pad_select_gpio(PREVCU_GYRO_INT_GPIO);
    gpio_set_direction(PREVCU_ACCEL_INT_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(PREVCU_GYRO_INT_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PREVCU_ACCEL_INT_GPIO, GPIO_FLOATING);
    gpio_set_pull_mode(PREVCU_GYRO_INT_GPIO, GPIO_FLOATING);
    gpio_set_intr_type(PREVCU_ACCEL_INT_GPIO, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(PREVCU_GYRO_INT_GPIO, GPIO_INTR_POSEDGE);

    xTaskCreate(can_receive_task, "Can Receive", configMINIMAL_SECURE_STACK_SIZE + 100, (void *)handles, PREVCU_RECEIVE_CAN_PRIORITY, &handles->can_receive_th);
    xTaskCreate(send_torque_task, "Send Torque", configMINIMAL_SECURE_STACK_SIZE + 100, (void *)handles, PREVCU_SEND_TORQUE_PRIORITY, &handles->send_torque_th);
    xTaskCreate(adc_update_task, "Adc Update", configMINIMAL_SECURE_STACK_SIZE + 100, (void *)handles, PREVCU_READ_THROTTLE_PRIORITY, &handles->read_adc_update_th);
    xTaskCreate(read_accel_task, "Read Accel", configMINIMAL_SECURE_STACK_SIZE + 100, (void *)handles, PREVCU_READ_ACCEL_PRIORITY, &handles->read_accel_th);
    xTaskCreate(read_speeds_task, "Read Speeds", configMINIMAL_SECURE_STACK_SIZE + 100, (void *)handles, PREVCU_READ_SPEED_PRIORITY, &handles->read_speeds_th);
    xTaskCreate(read_torque_task, "Read Torque", configMINIMAL_SECURE_STACK_SIZE + 100, (void *)handles, PREVCU_READ_TORQUE_PRIORITY, &handles->read_torque_th);
    xTaskCreate(read_gyro_task, "Read Gyro", configMINIMAL_SECURE_STACK_SIZE + 100, (void *)handles, PREVCU_READ_GYRO_PRIORITY, &handles->read_gyro_th);
    xTaskCreate(read_mc_temps_task, "Read MC Temps", configMINIMAL_SECURE_STACK_SIZE + 100, (void *)handles, PREVCU_READ_CONTROLLER_TEMPS_PRIORITY, &handles->read_mc_temps_th);
    xTaskCreate(read_sas_task, "Read SAS", configMINIMAL_SECURE_STACK_SIZE + 100, (void *)handles, PREVCU_READ_SAS_PRIORITY, &handles->read_sas_th);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PREVCU_ACCEL_INT_GPIO, wake_task_isr, (void *)handles->read_accel_th);
    gpio_isr_handler_add(PREVCU_GYRO_INT_GPIO, wake_task_isr, (void *)handles->read_gyro_th);

    // MAYBE WAIT FOR SOME SIGNAL

    // release all the tasks whose periodicity isn't controlled by us
    xTaskNotifyIndexed(handles->can_receive_th, 0, 0, eNoAction);
    xTaskNotifyIndexed(handles->read_accel_th, 0, 0, eNoAction);
    xTaskNotifyIndexed(handles->read_gyro_th, 0, 0, eNoAction);
    // offset the periodic ones by 7ms
    xTaskNotifyIndexed(handles->adc_update_th, 0, 0, eNoAction);
    vTaskDelay(pdMS_TO_TICKS(2 * PREVCU_READ_THROTTLE_PERIOD + 7));
    xTaskNotifyIndexed(handles->send_torque_th, 0, 0, eNoAction);
    vTaskDelay(pdMS_TO_TICKS(7));
    xTaskNotifyIndexed(handles->read_speeds_th, 0, 0, eNoAction);
    vTaskDelay(pdMS_TO_TICKS(7));
    xTaskNotifyIndexed(handles->read_torque_th, 0, 0, eNoAction);
    vTaskDelay(pdMS_TO_TICKS(7));
    xTaskNotifyIndexed(handles->read_mc_temps_th, 0, 0, eNoAction);
    vTaskDelay(pdMS_TO_TICKS(7));
    xTaskNotifyIndexed(handles->read_sas_th, 0, 0, eNoAction);
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
}

typedef struct accel_update_t
{
    TickType_t time;
    float x;
    float y;
    float z;
} accel_data_t;
void read_accel_task(void *handle_void)
{
    wait_to_start();
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
        memcpy(&queue_element.buffer, &accel_data, sizeof(accel_data));

        xQueueSendToBack(handles->queue_handle, &queue_element, portMAX_DELAY);
        // just block me until the ISR tells me to go again
        xTaskNotifyWaitIndexed(1, 0, 0, NULL, portMAX_DELAY);
    }
}

float rot_reading_conversion(char *data)
{
    uint16_t ucounts = data[1];
    ucounts <<= 8;
    ucounts |= data[0];
    int16_t counts = ucounts;
}

typedef struct rot_update_t
{
    TickType_t time;
    float x;
    float y;
    float z;
} rot_data_t;

void read_rot_task(void *handle_void)
{
    wait_to_start();
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
        rot_update_t rot_data;
        rot_data.x = accel_reading_conversion(data + 0);
        rot_data.y = accel_reading_conversion(data + 2);
        rot_data.z = accel_reading_conversion(data + 4);
        queue_element_t queue_element;
        queue_element.header = ROT_QUEUE_HEADER;
        rot_data.time = xTaskGetTickCount();

        memcpy(&queue_element.buffer, &rot_data, sizeof(rot_data));
        xQueueSendToBack(handles->queue_handle, &queue_element, portMAX_DELAY);
        // just block me until the ISR tells me to go again
        xTaskNotifyWaitIndexed(1, 0, 0, NULL, portMAX_DELAY);
    }
}

typedef struct steering_data_t
{
    TickType_t time;
    float left_wheel_radian;
    float right_wheel_radian;
} steering_data_t;

void read_sas_task(void *handle_void)
{
    wait_to_start();
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
        counts -= PREVCU_SAS_FORWARD;
        if (counts < 0)
        {
            counts += 4096
        }
        counts -= 2048;
        float rack_angle = counts / 2.f / 3.14159f;
        steering_update_t steering_data = {
            .left_wheel_radian = rack_angle,
            .right_wheel_radian = rack_angle,
        };
        queue_element_t queue_element;
        queue_element.header = STEERING_QUEUE_HEADER;
        steering_data.time = xTaskGetTickCount();

        memcpy(&queue_element.buffer, &steering_data, sizeof(steering_data));
        xQueueSendToBack(handles->queue_handle, &queue_element, portMAX_DELAY);
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

typedef struct adc_update_t
{
    TickType_t time;
    float throttle_percent, temp_fl, temp_fr, temp_bl, temp_br;
    bool includes_temp;
} adc_update_t;

void read_adc_task(void *void_handles)
{
    wait_to_start();
    our_handles_t *handles = (our_handles_t *)handle_void;
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
                if (throttle_percent < 0)
                {
                    throttle_percent = 0;
                }
                if (throttle_percent > 1)
                {
                    throttle_percent = 1
                }
                xTaskNotifyIndexed(handles->send_torque_th, 1, floor(throttle_percent * 255), eSetValueWithOverwrite);
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
        update.time = xTaskGetTickCount();

        queue_element_t queue_element;
        queue_element.header = ADC_UPDATE_HEADER;
        memcpy(&queue_element.buffer, &update, sizeof(update));
        xQueueSendToBack(handles->queue_handle, &queue_element, portMAX_DELAY);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PREVCU_READ_THROTTLE_PERIOD));
    }
}

void read_data_task(unsigned int address, unsigned int sub_index)
{
    uint8_t b1 = (address >> 8);
    uint8_t b2 = (address >> 0);
    can_message_t message;
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

void read_speeds_task(void *)
{
    wait_for_start();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        read_data(0x606C, 0x00);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PREVCU_READ_SPEEDS_PERIOD));
    }
}

void read_mc_temps_task(void *)
{
    wait_to_start();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        read_data(0x2026, 0x00);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PREVCU_READ_CONTROLLER_TEMP_PERIOD));
    }
}

void read_real_torque_task(void *)
{
    wait_to_start();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        read_data(0x6077, 0x00);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PREVCU_READ_MOTOR_TORQUE_PERIOD));
    }
}

void send_torque_task(void *handle_void)
{
    wait_to_start();
    our_handles_t *handles = (our_handles_t *)handle_void;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t direct_torque_long;
    uint32_t calculated_torque_long;
    uint8_t torques[4];
    for (;;)
    {
        uint8_t torque;
        xTaskNotifyAndQueryIndexed(handles->send_torque_th, 2, 0, eNoAction, &calculated_torque_long);
        if (!xTaskNotifyWaitIndexed(1, 0, 0, &direct_torque_long, 0))
        {
            // we didn't recieve a torque update, despite that updating twice as often. Abort immediately
        }
        if (direct_torque_long == 0)
        {
            // The driver has lifted their foot off the pedal. No matter what, we need to stop
            torques = {0, 0, 0, 0};
        }
        else
        {
            torques[0] = calculated_torque_long;       // fl
            torques[1] = calculated_torque_long >> 8;  // fr
            torques[2] = calculated_torque_long >> 16; // bl
            torques[3] = calculated_torque_long >> 24; // br
        }

        for (int i = 0; i < 4; ++i)
        {
            can_message_t message;
            message.identifier = 0x600 + id_array[i];
            message.data_length_code = 8;
            message.data = {
                0x2B, // magic numbers for CANopen
                0x71,
                0x60, // 0x6071 is the specifier
                0x00,
                torques[i], // 0-255
                0x0,
                0x0, // CANopen padding
                0x0,
            };

            // Queue message for transmission
            if (can_transmit(&message, pdMS_TO_TICKS(PREVCU_CAN_TIMEOUT)) != ESP_OK)
            {
                // TODO we weren't able to send our message in a reasonable amount of time. We should probably tell someone and reboot
            }
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PREVCU_SEND_TORQUE_PERIOD));
    }
}
typedef struct speed_update_t
{
    TickType_t time;
    int32_t[4] speeds;
} speed_update_t;

typedef struct mc_temp_update_t
{
    TickType_t time;
    uint8_t[4] temperatures;
} mc_temp_update_t;

typedef struct mc_torque_update_t
{
    TickType_t time;
    uint16_t[4] torques;
} mc_torque_update_t;

// takes care of all of the receiving
void can_receive_task(void *handle_void)
{
    wait_to_start();
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
        current_time = xTaskGetTickCount();
        // TODO: Replace subtraction with something overflow aware
        TickType_t max_time_diff = max(
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
        uint16_t command = message.identifier & 0x780;
        uint16_t node_id = message.identifier - 0x7F;

        uint8_t index;
        switch (node_id)
        {
        case PREVCU_FL_ADDR:
            index = 0;
            break;
        case PREVCU_FR_ADDR:
            index = 1;
            break;
        case PREVCU_BL_ADDR:
            index = 2;
            break;
        case PREVCU_BR_ADDR:
            index = 3;
            break;
        default:
            // we got a message we didn't expect.
        }
        if (command == 0x700)
        {
            last_heartbeat_messsage[index] = xTaskGetTickCount();
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
                latest_speeds[index] <<= 8;
                latest_speeds[index] |= message.data[7];
                waiting_speeds |= 1 << index;
                if (waiting_speeds == 0xF)
                {
                    waiting_speeds = 0;
                    queue_element_t queue_element;
                    queue_element.header = SPEED_UPDATE_HEADER;
                    speed_update_t update;
                    update.time = xTaskGetTickCount();
                    update.speeds = latest_speeds;
                    memcpy(&queue_element.buffer, &update, sizeof(update));
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
                    update.temperatures = latest_mc_temps;
                    memcpy(&queue_element.buffer, &update, sizeof(update));
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
                    update.torques = latest_torques;
                    memcpy(&queue_element.buffer, &update, sizeof(update));
                    xQueueSendToBack(handles->queue_handle, &queue_element, portMAX_DELAY);
                }
                break;
            default:
                // we got a message we didnt expect
            }
        }
    }
}

void app_main(void)
{
    setup();
}
