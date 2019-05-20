/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "esp_intr_alloc.h"



// #include <stdio.h>
// #include "esp_types.h"
// #include "esp_attr.h"
// #include "esp_intr.h"
// #include "esp_log.h"
// #include "malloc.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/semphr.h"
// #include "freertos/xtensa_api.h"
// #include "freertos/task.h"
// #include "freertos/ringbuf.h"
#include "soc/dport_reg.h"
#include "soc/i2c_struct.h"
#include "soc/i2c_reg.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/rtc_io.h"
#include "i2c_example_main.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define BH1750_SENSOR_ADDR CONFIG_BH1750_ADDR   /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START CONFIG_BH1750_OPMODE   /*!< Operation mode */
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

static const char *TAG = "i2c-example";

static DRAM_ATTR i2c_dev_t* const I2C[I2C_NUM_MAX] = { &I2C0, &I2C1 };  // 具体参考i2c_dev_t，位于esp-idf/components/soc/esp32/include/soc/i2c_struct.h

static intr_handle_t my_i2c_slave_irq_handle;

SemaphoreHandle_t print_mux = NULL;

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief test code to operate on BH1750 sensor
 *
 * 1. set operation mode(e.g One time L-resolution mode)
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, BH1750_CMD_START, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_l, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief i2c slave initialization
 */
static esp_err_t i2c_slave_init()
{
    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave;
    conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
    i2c_param_config(i2c_slave_port, &conf_slave);
    return i2c_driver_install(i2c_slave_port, conf_slave.mode,
                              I2C_SLAVE_RX_BUF_LEN,
                              I2C_SLAVE_TX_BUF_LEN, 0);
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}

static void i2c_test_task(void *arg)
{
    int i = 0;
    int ret;
    uint32_t task_idx = (uint32_t)arg;
    uint8_t *data = (uint8_t *)malloc(DATA_LENGTH);
    uint8_t *data_wr = (uint8_t *)malloc(DATA_LENGTH);
    uint8_t *data_rd = (uint8_t *)malloc(DATA_LENGTH);
    uint8_t sensor_data_h, sensor_data_l;
    int cnt = 0;

    uint8_t rcv_buffer[16];
    size_t rcv_num;
    char *p;

    while (1) {
        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        // if(1 == I2C[I2C_SLAVE_NUM]->int_raw.tx_fifo_empty)  // 如果I2C发送缓冲区空
        // {
        //     vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
        // }
//tx_send_empty
        // This register stores the value of state machine for i2c module.  3'h0: SCL_MAIN_IDLE  3'h1: SCL_ADDRESS_SHIFT 3'h2: SCL_ACK_ADDRESS  3'h3: SCL_RX_DATA
        // if(3 == I2C[I2C_SLAVE_NUM]->status_reg.scl_main_state_last)
        // {
            rcv_num = i2c_slave_read_buffer(I2C_SLAVE_NUM, rcv_buffer, 8, 1000 / portTICK_RATE_MS);
            if(rcv_num > 0)
            {
                printf("receive data:\n");
                disp_buf(rcv_buffer, 8);
                if (strstr((char*)rcv_buffer, IIC_COMMAND_WIFI_SCAN_BEGIN) != NULL)	// 设置i2c测试_write command
                {
                    printf("start wifi scan!\n");
                    for (i = 0; i < DATA_LENGTH; i++) {
                        data[i] = i;
                    }

                    size_t d_size = i2c_slave_write_buffer(I2C_SLAVE_NUM, data, RW_TEST_LENGTH, 1000 / portTICK_RATE_MS);
                    if (d_size == 0) {
                        ESP_LOGW(TAG, "i2c slave tx buffer full");
                    } else {
                        printf("====TASK[%d] Slave buffer data ====\n", task_idx);
                        disp_buf(data, d_size);
                    }
                }
                else if (strstr((char*)rcv_buffer, IIC_COMMAND_WIFI_SCAN_STOP) != NULL)	// i2c wifi scan stop
                {
                    printf("IIC_COMMAND_WIFI_SCAN_STOP!\n");
                    ESP_ERROR_CHECK(i2c_reset_tx_fifo(I2C_SLAVE_NUM));
                    ESP_ERROR_CHECK(i2c_reset_rx_fifo(I2C_SLAVE_NUM));
                }
                else if (strstr((char*)rcv_buffer, IIC_COMMAND_OFF) != NULL)	// i2c off
                {
                    printf("IIC_COMMAND_OFF!\n");
                    ESP_ERROR_CHECK(i2c_driver_delete(I2C_SLAVE_NUM));
                }
                else
                {
                }
                
                ESP_ERROR_CHECK(i2c_reset_rx_fifo(I2C_SLAVE_NUM));
            }

        printf("> i2c_slave_write_buffer(),int_raw(rx_fifo_full/tx_fifo_empty):%x-%x,int_status(rx_fifo_full/tx_fifo_empty):%x-%x\n",
                I2C[I2C_SLAVE_NUM]->int_raw.rx_fifo_full,
                I2C[I2C_SLAVE_NUM]->int_raw.tx_fifo_empty,
                I2C[I2C_SLAVE_NUM]->int_status.rx_fifo_full,
                I2C[I2C_SLAVE_NUM]->int_status.tx_fifo_empty);

        printf("count of fifo data(4 bytes) rx/tx: %d/%d\n", 
                I2C[I2C_SLAVE_NUM]->status_reg.rx_fifo_cnt,
                I2C[I2C_SLAVE_NUM]->status_reg.tx_fifo_cnt);
        printf("int_status:%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x\n",
                I2C[I2C_SLAVE_NUM]->int_status.rx_fifo_full,
                I2C[I2C_SLAVE_NUM]->int_status.tx_fifo_empty,
                I2C[I2C_SLAVE_NUM]->int_status.rx_fifo_ovf,
                I2C[I2C_SLAVE_NUM]->int_status.end_detect,
                I2C[I2C_SLAVE_NUM]->int_status.slave_tran_comp,
                I2C[I2C_SLAVE_NUM]->int_status.arbitration_lost,
                I2C[I2C_SLAVE_NUM]->int_status.master_tran_comp,
                I2C[I2C_SLAVE_NUM]->int_status.trans_complete,
                I2C[I2C_SLAVE_NUM]->int_status.time_out,
                I2C[I2C_SLAVE_NUM]->int_status.trans_start,
                I2C[I2C_SLAVE_NUM]->int_status.ack_err,
                I2C[I2C_SLAVE_NUM]->int_status.rx_rec_full,
                I2C[I2C_SLAVE_NUM]->int_status.tx_send_empty);
        printf("int_ena:%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x\n",
                I2C[I2C_SLAVE_NUM]->int_ena.rx_fifo_full,
                I2C[I2C_SLAVE_NUM]->int_ena.tx_fifo_empty,
                I2C[I2C_SLAVE_NUM]->int_ena.rx_fifo_ovf,
                I2C[I2C_SLAVE_NUM]->int_ena.end_detect,
                I2C[I2C_SLAVE_NUM]->int_ena.slave_tran_comp,
                I2C[I2C_SLAVE_NUM]->int_ena.arbitration_lost,
                I2C[I2C_SLAVE_NUM]->int_ena.master_tran_comp,
                I2C[I2C_SLAVE_NUM]->int_ena.trans_complete,
                I2C[I2C_SLAVE_NUM]->int_ena.time_out,
                I2C[I2C_SLAVE_NUM]->int_ena.trans_start,
                I2C[I2C_SLAVE_NUM]->int_ena.ack_err,
                I2C[I2C_SLAVE_NUM]->int_ena.rx_rec_full,
                I2C[I2C_SLAVE_NUM]->int_ena.tx_send_empty);

        printf("int_raw:%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x\n",
                I2C[I2C_SLAVE_NUM]->int_raw.rx_fifo_full,
                I2C[I2C_SLAVE_NUM]->int_raw.tx_fifo_empty,
                I2C[I2C_SLAVE_NUM]->int_raw.rx_fifo_ovf,
                I2C[I2C_SLAVE_NUM]->int_raw.end_detect,
                I2C[I2C_SLAVE_NUM]->int_raw.slave_tran_comp,
                I2C[I2C_SLAVE_NUM]->int_raw.arbitration_lost,
                I2C[I2C_SLAVE_NUM]->int_raw.master_tran_comp,
                I2C[I2C_SLAVE_NUM]->int_raw.trans_complete,
                I2C[I2C_SLAVE_NUM]->int_raw.time_out,
                I2C[I2C_SLAVE_NUM]->int_raw.trans_start,
                I2C[I2C_SLAVE_NUM]->int_raw.ack_err,
                I2C[I2C_SLAVE_NUM]->int_raw.rx_rec_full,
                I2C[I2C_SLAVE_NUM]->int_raw.tx_send_empty);

        printf("int_clr:%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x\n",
                I2C[I2C_SLAVE_NUM]->int_clr.rx_fifo_full,
                I2C[I2C_SLAVE_NUM]->int_clr.tx_fifo_empty,
                I2C[I2C_SLAVE_NUM]->int_clr.rx_fifo_ovf,
                I2C[I2C_SLAVE_NUM]->int_clr.end_detect,
                I2C[I2C_SLAVE_NUM]->int_clr.slave_tran_comp,
                I2C[I2C_SLAVE_NUM]->int_clr.arbitration_lost,
                I2C[I2C_SLAVE_NUM]->int_clr.master_tran_comp,
                I2C[I2C_SLAVE_NUM]->int_clr.trans_complete,
                I2C[I2C_SLAVE_NUM]->int_clr.time_out,
                I2C[I2C_SLAVE_NUM]->int_clr.trans_start,
                I2C[I2C_SLAVE_NUM]->int_clr.ack_err,
                I2C[I2C_SLAVE_NUM]->int_clr.rx_rec_full,
                I2C[I2C_SLAVE_NUM]->int_clr.tx_send_empty);

        printf("status_reg:%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x\n",
                I2C[I2C_SLAVE_NUM]->status_reg.ack_rec,
                I2C[I2C_SLAVE_NUM]->status_reg.slave_rw,
                I2C[I2C_SLAVE_NUM]->status_reg.time_out,
                I2C[I2C_SLAVE_NUM]->status_reg.arb_lost,
                I2C[I2C_SLAVE_NUM]->status_reg.bus_busy,
                I2C[I2C_SLAVE_NUM]->status_reg.slave_addressed,
                I2C[I2C_SLAVE_NUM]->status_reg.byte_trans,
                I2C[I2C_SLAVE_NUM]->status_reg.rx_fifo_cnt,
                I2C[I2C_SLAVE_NUM]->status_reg.tx_fifo_cnt,
                I2C[I2C_SLAVE_NUM]->status_reg.scl_main_state_last,
                I2C[I2C_SLAVE_NUM]->status_reg.scl_state_last);
        xSemaphoreGive(print_mux);
        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);

// if(1 == I2C[I2C_SLAVE_NUM]->int_raw.tx_fifo_empty)
// {
// // 芯片 boot 后读取传感器数据，发出警报或者上传数据；
// // 调用 rtc_gpio_pulldown_en(MY_RTC_WAKEUP_IO) 函数或 rtc_gpio_pullup_en(MY_RTC_WAKEUP_IO) 函数，设置内部下拉或上拉类型；
// // 调用 esp_deep_sleep_enable_ext0_wakeup(MY_RTC_WAKEUP_IO, WAKEUP_IO_LEVEL) 函数或 esp_deep_sleep_enable_ext1_wakeup(WAKEUP_PIN_MASK, WAKEUP_TYPE) 函数，设置从 Deep-sleep 模式下唤醒的 RTC GPIO 电压条件；1
// // 调用 esp_deep_sleep_start() 函数进入 Deep-sleep 模式。
//     // 以下使esp32进入深度睡眠模式
//     // printf("esp_deep_sleep_start()\n");
//     // esp_deep_sleep_start();
//     // printf("esp_deep_sleep_start(),ok!\n");

//     //use RTC_IO_1 as wakeup pin, low level will wakeup system.
//     const uint32_t pin_num = 37;

//     esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
//     rtc_gpio_pullup_en(pin_num);
//     esp_sleep_enable_ext0_wakeup(pin_num, 0);
//     printf("ready to deep sleep,z^z^z^z^z^z^\n");
//     esp_deep_sleep_start();
//     //rtc_gpio_deinit(pin_num);
//     printf("wake up!!!!!!!!\n");
// }

    }
    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}

static void i2c_irq_function_handler(void *arg)
{
    printf("i2c slave zhongduan!!!!!!!!!!!!!!!!!\n");
}

void qinbao_i2c()
{
    print_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_slave_init());  // i2c之slave模式初始化
// i2c中断相关
//i2c_isr_register(I2C_SLAVE_NUM, i2c_irq_function_handler, NULL, ESP_INTR_FLAG_LOWMED, &my_i2c_slave_irq_handle); //重新注册中断服务函数


//i2c_isr_free(my_i2c_slave_irq_handle);
//    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
//    xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void *)1, 10, NULL);

    // uint32_t status = I2C[I2C_SLAVE_NUM]->int_status.val;
}



// I2C[I2C_SLAVE_NUM]->int_ena.rx_fifo_full = 1;
// I2C[I2C_SLAVE_NUM]->int_ena.tx_fifo_empty = 1;
// I2C[I2C_SLAVE_NUM]->int_ena.rx_fifo_ovf = 1;
// I2C[I2C_SLAVE_NUM]->int_ena.end_detect = 1;
// I2C[I2C_SLAVE_NUM]->int_ena.slave_tran_comp = 1;
// I2C[I2C_SLAVE_NUM]->int_ena.arbitration_lost = 1;
// I2C[I2C_SLAVE_NUM]->int_ena.master_tran_comp = 1;
// I2C[I2C_SLAVE_NUM]->int_ena.trans_complete = 1;
// I2C[I2C_SLAVE_NUM]->int_ena.time_out = 1;
// I2C[I2C_SLAVE_NUM]->int_ena.trans_start = 1;
// I2C[I2C_SLAVE_NUM]->int_ena.ack_err = 1;
// I2C[I2C_SLAVE_NUM]->int_ena.rx_rec_full = 1;
// I2C[I2C_SLAVE_NUM]->int_ena.tx_send_empty = 1;
// I2C[I2C_SLAVE_NUM]->int_ena.reserved13 = 1;
