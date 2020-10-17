#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "app_config.h"
#include "app_config_wifi.h"
#include "app_config_http.h"

#define TAG "MAIN"

void app_main(void){
    ESP_ERROR_CHECK(app_config_init());		    // Initializing and loading configuration
    ESP_ERROR_CHECK(app_config_wifi_init());	// Initializing wifi according saved configuration
    ESP_ERROR_CHECK(app_config_http_init());	// Initializing HTTP
}