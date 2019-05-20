#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "sdkconfig.h"

#include "my_gpio.h"
#include "qinbao_wifi.h"
//#include "wifi_scan.h"
//#include "mqtt_tcp.h"

#include "smartconfig_main.h" //
#include "i2c_example_main.h" // i2c相关的文件
#include "ali_linkkit_sdk.h"
#include "dht_espidf.h"
#include "nmea_app.h"


/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_MY_BLINK_GPIO

static const char *TAG = "QB_MAIN";

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void blink_task(void *pvParameter)
{
    int res;

    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while (1)
    {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("led off\n");
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("led on\n");
    }
}


void app_main()
{
    // 以下使esp32进入深度睡眠模式
    // printf("esp_deep_sleep_start()\n");
    // esp_deep_sleep_start();
    // printf("esp_deep_sleep_start(),ok!\n");

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
qb_dht_init();
get_position_from_nmea_parser();
    xTaskCreate(&blink_task, "blink_task", /*configMINIMAL_STACK_SIZE*/2048, NULL, 5, NULL);
    // wifi_smartconfig_app_main();
    //qinbao_main();
    //my_mqtt_over_tcp_app_main();    // mqtt测试程序
    qinbao_wifi_init();
    linkkit_solo_main();
    //wifi_scan();    //qinbao_wifi_init();

    // my_gpio_test();
    //    qinbao_i2c();	// i2c相关的程序

    //   ali_iot_mqtt_example_main();
}