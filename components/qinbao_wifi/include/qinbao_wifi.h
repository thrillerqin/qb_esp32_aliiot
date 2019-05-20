#ifndef _QINBAO_WIFI_H_
#define _QINBAO_WIFI_H_

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_MODE_AP   CONFIG_ESP_WIFI_MODE_AP //TRUE:AP FALSE:STA
#define EXAMPLE_MAX_STA_CONN       CONFIG_MAX_STA_CONN

void wifi_init_softap();
void wifi_init_sta();
void qinbao_wifi_init();

#endif