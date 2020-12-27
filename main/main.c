#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "app_config.h"
#include "app_config_wifi.h"
#include "app_config_http.h"
#include "app_config_mqtt.h"
#include "app_config_ble_mesh.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_sensor_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "app_ds.h"
#include "sdkconfig.h"

#define TAG "MAIN"
#define TASK_STACK_SIZE         4096
#define TEMP_POLLING_PERIOD_MS  5000
#define TEMP_QUEUE_LENGTH       5
#define ALARM_PERIOD_MS         30000
#define WORKER_PERIOD_MS        1000
#define CHANNEL_NUMBER          3
#define ON_LEVEL                1
#define OFF_LEVEL               !ON_LEVEL
#define BUTTON_PIN              0
#define RESET_TIME_MS           3000
#define CUR_TEMP_MS             2000
#define MAX_TEMP                40
#define MIN_TEMP                20

#define MODE_STATE_TOPIC            "/mode/state"
#define MODE_COMMAND_TOPIC          "/mode/set"
#define TEMPERATURE_STATE_TOPIC     "/temperature/state"
#define TEMPERATURE_COMMAND_TOPIC   "/temperature/set"
#define TEMPERATURE_CURRENT_TOPIC   "/temperature/current"
#define ALLOC_ERR_STR               "Error allocating buffer!"

ds_bus_t    ds_buses[DS_BUS_NUMBER] = {0};
app_config_cbs_t app_config_cbs;
TimerHandle_t alarm_timers[CHANNEL_NUMBER];
TimerHandle_t reset_timer;
TimerHandle_t temperature_timer;
bool alarms[CHANNEL_NUMBER] = {true};
bool enabled[CHANNEL_NUMBER] = {true};
float measurements[CHANNEL_NUMBER] = {0xFF};

gpio_num_t outputs[CHANNEL_NUMBER] = {
    GPIO_NUM_13,
    GPIO_NUM_15,
    GPIO_NUM_16
};

typedef enum {
    ds18b20 = 0,
    ble_mesh,
    mqtt,
    none
} sensor_type_t;

typedef struct {
    sensor_type_t   sensor_type;
    uint8_t         sensor_num;
    uint8_t         sensor_addr;
    float           value;
} temp_queue_t;

xQueueHandle temp_queue;

static void app_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event, esp_ble_mesh_cfg_server_cb_param_t *param){
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT){
        switch (param->ctx.recv_op) {
            case ESP_BLE_MESH_MODEL_OP_NODE_RESET:
                ESP_LOGW(TAG, "Resetting Ble mesh node!");
                esp_ble_mesh_node_local_reset();
                app_config_restart();
                break;
            default:
                break;
        }
        
    }
}

static void app_ble_mesh_sensor_client_cb(esp_ble_mesh_sensor_client_cb_event_t event, esp_ble_mesh_sensor_client_cb_param_t *param){
    ESP_LOGD(TAG, "Sensor client, event %u, addr 0x%04x", event, param->params->ctx.addr);

    switch (event) {
    case ESP_BLE_MESH_SENSOR_CLIENT_PUBLISH_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_SENSOR_CLIENT_PUBLISH_EVT, dst %d, len %d, op %d", param->params->ctx.recv_dst, param->status_cb.sensor_status.marshalled_sensor_data->len, param->params->opcode);
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS){
            uint8_t *data = param->status_cb.sensor_status.marshalled_sensor_data->data;
            uint8_t len = 0;
            uint16_t prop_id = 0;
            uint8_t mpid_len = 0;
            uint8_t *data_ptr = data;
            uint16_t primary_addr = esp_ble_mesh_get_primary_element_address();
            uint8_t channel = param->params->model->element->element_addr - primary_addr;
            ESP_LOGI(TAG, "Channel %d", channel);
            for (; len < param->status_cb.sensor_status.marshalled_sensor_data->len;) {
                uint8_t fmt = ESP_BLE_MESH_GET_SENSOR_DATA_FORMAT(data);
                uint8_t data_len = ESP_BLE_MESH_GET_SENSOR_DATA_LENGTH(data, fmt);
                prop_id = ESP_BLE_MESH_GET_SENSOR_DATA_PROPERTY_ID(data, fmt);
                mpid_len = (fmt == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A ? ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN : ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN);
                ESP_LOGD(TAG, "Fmt %d, len %d, pid %d", fmt, data_len, prop_id);
                if (data_len != ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN) {
                    ESP_LOG_BUFFER_HEX("Sensor data", data + mpid_len, data_len + 1);
                    data_ptr = data + mpid_len;
                    len += mpid_len + data_len + 1;
                    if(prop_id == 0x54){
                        float value = (data_ptr[0] + (data_ptr[1] << 8)) / 100;
                        temp_queue_t item;
                        item.sensor_type = ble_mesh;
                        item.sensor_num = channel;            // One sensor per bus
                        item.value = value;
                        portBASE_TYPE status = xQueueSend(temp_queue, &item, 1000 / portTICK_RATE_MS);
                        if(status != pdPASS){
                            ESP_LOGE(TAG, "Error queuing temprature!");
                        }
                        ESP_LOGD(TAG, "Queueing %f", value);
                    } else {
                        ESP_LOGW(TAG, "Not allowed propery ID 0x%04x", prop_id);
                    }
                } else {
                    len += mpid_len;
                }
                data += len;
            }            
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event");
        break;
    }
}

static void ds_task(void *pvParameters){
    while(1){
        for(uint8_t i=0; i<DS_BUS_NUMBER; i++){
            if(ds_buses[i].device_count > 0){
                float readings[DS_MAX_DEVICES];
                esp_err_t err = app_ds_read_temperature_all(&ds_buses[i], readings);
                if(err != ESP_OK) {
                    ESP_LOGE(TAG, "Error reading temperature");
                    continue;
                }
                for (uint8_t j=0; j < ds_buses[i].device_count; j++){
                    printf("Bus %d, device %d, value %2.1f\n", i, j, readings[j]);
                    temp_queue_t item;
                    item.sensor_type = ds18b20;
                    item.sensor_num = i;            // One sensor per bus
                    item.value = readings[j];
                    portBASE_TYPE status = xQueueSend(temp_queue, &item, 1000 / portTICK_RATE_MS);
                    if(status != pdPASS){
                        ESP_LOGE(TAG, "Error queuing temprature!");
                    }
                }
            }
        }
        vTaskDelay(TEMP_POLLING_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

sensor_type_t get_zone_type(uint8_t zone){
    char element[CONFIG_APP_CONFIG_SHORT_NAME_LEN] = {0};
    sprintf(element, "zone%d_sensor", zone + 1);
    uint8_t zone_type;
    esp_err_t err = app_config_getValue(element, int8, &zone_type);
    if (err != ESP_OK){
        ESP_LOGW(TAG, "Zone %d type not found", zone + 1);
        return none;
    }
    switch (zone_type) {
        case 0:
            return ds18b20;
            break;
        case 1:
            return ble_mesh;
            break;
        case 2:
            return mqtt;
            break;
        default:
            ESP_LOGW(TAG, "Zone type %d unrecognized", zone_type);
            return none;
    }
}

static void sensor_task(void *pvParameters){
    for (;;){
        temp_queue_t item;
        portBASE_TYPE status = xQueueReceive(temp_queue, &item, portMAX_DELAY);
        if (status != pdPASS){
            ESP_LOGE(TAG, "Error receiving from queue");
        } else {
            ESP_LOGI(TAG, "Received type %d, num %d, value %2.1f", item.sensor_type, item.sensor_num, item.value);
            for (uint8_t i=0; i<CHANNEL_NUMBER; i++){
                if (get_zone_type(i) == item.sensor_type){
                    if (item.sensor_type == ds18b20){
                        if (item.sensor_num == i){
                            measurements[i] = item.value;
                            alarms[i] = 0;
                            xTimerReset(alarm_timers[i], 0);
                        }
                    } else if (item.sensor_type == ble_mesh){
                        if (item.sensor_num == i){
                            measurements[i] = item.value;
                            alarms[i] = 0;
                            xTimerReset(alarm_timers[i], 0);
                        }
                    } else if (item.sensor_type == mqtt){
                    }
                }
            }
        }
    }
}

static void worker_task(void *pvParameters){
    for (;;){
        for(uint8_t i=0; i<CHANNEL_NUMBER; i++){
            char zone_temp[11];
            char zone_hyst[11];
            uint8_t target;
            uint8_t hyst;
            sprintf(zone_temp, "zone%1d_temp", i+1);
            sprintf(zone_hyst, "zone%1d_hyst", i+1);
            esp_err_t err = app_config_getValue(zone_temp, int8, &target);
            if(err != ESP_OK){
                ESP_LOGE(TAG, "Error retreiving target temperature for zone %d", i);
                alarms[i] = true;
            }
            err = app_config_getValue(zone_hyst, int8, &hyst);
            if(err != ESP_OK){
                ESP_LOGE(TAG, "Error retreiving hysteresis for zone %d", i);
                alarms[i] = true;
            }
            if((!alarms[i])&&(enabled[i])){
                if(measurements[i] < (target - hyst)) {
                    //printf("Turning on\n");
                    gpio_set_level(outputs[i], ON_LEVEL);
                } else if (measurements[i] > target) {
                    //printf("Turning off\n");
                    gpio_set_level(outputs[i], OFF_LEVEL);
                }
            }
        }
        vTaskDelay(WORKER_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

void app_mqtt_notify_mode(uint8_t channel){
    char element[13] = {0};
    sprintf(element, "base_path%d", channel + 1);
    char *base_path;
    app_config_getValue(element, string, &base_path);
    if(strlen(base_path)){
        char *mode_state_topic = (char *)calloc(strlen(base_path) + sizeof(MODE_STATE_TOPIC) + 1, sizeof(char));
        if (!mode_state_topic){
            ESP_LOGE(TAG, ALLOC_ERR_STR);
            free(base_path);
            return;
        }
        strncat(mode_state_topic, base_path, strlen(base_path) + 1);
        strncat(mode_state_topic, MODE_STATE_TOPIC, sizeof(MODE_STATE_TOPIC) + 1);
        ESP_LOGI(TAG, "Publishing mode state");
        if(enabled[channel] && !alarms[channel]){
            ESP_LOGI(TAG, "Topic %s, heat", mode_state_topic);
            app_config_mqtt_publish(mode_state_topic, "heat");
        } else {
            ESP_LOGI(TAG, "Topic %s, off", mode_state_topic);
            app_config_mqtt_publish(mode_state_topic, "off");
        }
        free(mode_state_topic);
    }
}

void app_mqtt_notify_temp(uint8_t channel){
    char element[13] = {0};
    sprintf(element, "base_path%d", channel + 1);
    char *base_path;
    app_config_getValue(element, string, &base_path);
    if(strlen(base_path)){
        char *temperature_state_topic = calloc(strlen(base_path) + sizeof(TEMPERATURE_STATE_TOPIC) + 1, sizeof(char));
        if (temperature_state_topic) {
            strncat(temperature_state_topic, base_path, strlen(base_path) + 1);
            strncat(temperature_state_topic, TEMPERATURE_STATE_TOPIC, sizeof(TEMPERATURE_STATE_TOPIC) + 1);
            char zone_temp_str[13];
            sprintf(zone_temp_str, "zone%d_temp", channel + 1);
            uint8_t value;
            char value_str[4];
            esp_err_t err = app_config_getValue(zone_temp_str, int8, &value);
            if(err == ESP_OK){
                sprintf(value_str, "%d", value);
                ESP_LOGI(TAG, "Publishing temperature state, topic %s, value %s", temperature_state_topic, value_str);
                app_config_mqtt_publish(temperature_state_topic, value_str);
            } else {
                ESP_LOGW(TAG, "Error retreiving temperature for zone %d", channel + 1);
            }
        } else {
            ESP_LOGE(TAG, ALLOC_ERR_STR);
            free(base_path);
        }
        free(temperature_state_topic);
    }
}

void app_mqtt_handler_cb(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_client_handle_t client = event->client;
    switch (event->event_id) {
            case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            for(uint8_t i=0; i<CHANNEL_NUMBER; i++){
                char element[11] = {0};
                sprintf(element, "base_path%d", i + 1);
                char *base_path;
                esp_err_t err = app_config_getValue(element, string, &base_path);
                ESP_LOGI(TAG,"%s", base_path);
                if (strlen(base_path) > 0){
                    char *mode_command_topic = (char *)calloc(strlen(base_path) + sizeof(MODE_COMMAND_TOPIC) + 1, sizeof(char));
                    if(!mode_command_topic){
                        ESP_LOGE(TAG, ALLOC_ERR_STR);
                        free(base_path);
                        return;
                    }
                    char *temperature_command_topic= (char *)calloc(strlen(base_path) + sizeof(TEMPERATURE_COMMAND_TOPIC) + 1, sizeof(char));
                    if(!mode_command_topic){
                        ESP_LOGE(TAG, ALLOC_ERR_STR);
                        free(mode_command_topic);
                        free(base_path);
                        return;
                    }
                    strncat(mode_command_topic, base_path, strlen(base_path) + 1);
                    strncat(temperature_command_topic, base_path, strlen(base_path) + 1);
                    strncat(mode_command_topic, MODE_COMMAND_TOPIC, sizeof(MODE_COMMAND_TOPIC) + 1);
                    strncat(temperature_command_topic, TEMPERATURE_COMMAND_TOPIC, sizeof(TEMPERATURE_COMMAND_TOPIC) + 1);
                    ESP_LOGI(TAG, "Subscribing %s", mode_command_topic);
                    esp_mqtt_client_subscribe(client, mode_command_topic, 0);
                    ESP_LOGI(TAG, "Subscribing %s", temperature_command_topic);
                    esp_mqtt_client_subscribe(client, temperature_command_topic, 0);
                    free(mode_command_topic);
                    free(temperature_command_topic);

                    app_mqtt_notify_mode(i);
                    app_mqtt_notify_temp(i);
                    //xTimerStart(temperature_timer, 0);
                }
            }
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            for(uint8_t i=0; i < CHANNEL_NUMBER; i++){
                char element[13] = {0};
                sprintf(element, "base_path%d", i + 1);
                char *base_path;
                app_config_getValue(element, string, &base_path);
                if (strlen(base_path) > 0){
                    char mode_command_topic[58] = {0};
                    char temperature_command_topic[58] = {0};
                    strncat(mode_command_topic, base_path, strlen(base_path) + 1);
                    strncat(temperature_command_topic, base_path, strlen(base_path) + 1);
                    strncat(mode_command_topic, MODE_COMMAND_TOPIC, sizeof(MODE_COMMAND_TOPIC) + 1);
                    strncat(temperature_command_topic, TEMPERATURE_COMMAND_TOPIC, sizeof(TEMPERATURE_COMMAND_TOPIC) + 1);
                    if(strncmp(event->topic, mode_command_topic, event->topic_len) == 0){
                        if(strncmp(event->data, "off", event->data_len) == 0){
                            ESP_LOGI(TAG, "Disabling channel %d", i);
                            enabled[i] = false;
                            app_mqtt_notify_mode(i);
                            gpio_set_level(outputs[i], OFF_LEVEL);
                        } else if(strncmp(event->data, "heat", event->data_len) == 0){    
                            ESP_LOGI(TAG, "Enabling channel %d", i);
                            enabled[i] = true;
                            app_mqtt_notify_mode(i);
                        } else {
                            ESP_LOGW(TAG, "Unrecognized mode command");
                        }
                        break;
                    } else if(strncmp(event->topic, temperature_command_topic, event->topic_len) == 0){
                        char val_str[4] = {0};
                        strncpy(val_str, event->data, (event->data_len > 3)?(3):(event->data_len));
                        long val = strtol(val_str, NULL, 10);
                        if((val > MAX_TEMP) || (val < MIN_TEMP)){
                            ESP_LOGW(TAG, "Exceeds temperature range %d", (int)val);
                        } else {
                            char zone_temp_str[11];
                            sprintf(zone_temp_str, "zone%d_temp", i + 1);
                            ESP_LOGI(TAG, "Setting zone %d temp to %d", i, (uint8_t)val);
                            esp_err_t err = app_config_setValue(zone_temp_str, &val);
                            if(err != ESP_OK){
                                ESP_LOGW(TAG, "Error setting temperature!");
                            } else {
                                app_mqtt_notify_temp(i);
                            }
                        }
                        break;
                    } else {
                        ESP_LOGW(TAG, "Unknown MQTT topic");
                    }
                }
            }
            break;
            case MQTT_EVENT_DISCONNECTED:
                //xTimerStop(temperature_timer, 0);
            break;
        default:
            break;
    }
}

void alarm_timer_cb(TimerHandle_t xTimer){
    uint8_t channel;
    gpio_num_t pin = *(gpio_num_t *)pvTimerGetTimerID(xTimer);
    for (uint8_t i=0; i<CHANNEL_NUMBER; i++) if(outputs[i] == pin) channel = i;
    
    ESP_LOGE(TAG, "ALARM! No data from temperature sensor for pin %d!", pin);
    alarms[channel] = true;
    gpio_set_level(outputs[channel], OFF_LEVEL);
}

void reset_timer_cb(TimerHandle_t xTimer){
    app_config_erase();
}

void temperature_timer_cb(TimerHandle_t xTimer)
{
    bool config_mqtt_enable;
    app_config_getBool("ble_mesh_enable", &config_mqtt_enable);
    if (config_mqtt_enable)
    {
        for (uint8_t i = 0; i < CHANNEL_NUMBER; i++)
        {
            if (!alarms[i])
            {
                char element[13];
                sprintf(element, "base_path%d", i + 1);
                char *base_path;
                app_config_getValue(element, string, &base_path);
                char *cur_temp_topic = calloc(strlen(base_path) + sizeof(TEMPERATURE_CURRENT_TOPIC) + 1, sizeof(char));
                if (cur_temp_topic)
                {
                    strncat(cur_temp_topic, base_path, strlen(base_path) + 1);
                    strncat(cur_temp_topic, TEMPERATURE_CURRENT_TOPIC, sizeof(TEMPERATURE_CURRENT_TOPIC) + 1);
                    char value_str[4];
                    sprintf(value_str, "%2.1f", measurements[i]);
                    ESP_LOGI(TAG, "Publishing current temperature %s", value_str);
                    app_config_mqtt_publish(cur_temp_topic, value_str);
                    free(cur_temp_topic);
                }
                else
                {
                    ESP_LOGE(TAG, ALLOC_ERR_STR);
                    continue;
                }
            }
        }
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg){
    if (gpio_get_level(BUTTON_PIN == 0)){
        xTimerStartFromISR(reset_timer, 0);
    } else {
        xTimerStopFromISR(reset_timer, 0);
    }
}

void app_main(void){
    reset_timer = xTimerCreate("reset_timer", RESET_TIME_MS / portTICK_PERIOD_MS, pdFALSE, NULL, reset_timer_cb);

    for (uint8_t i=0; i<CHANNEL_NUMBER; i++){
        gpio_reset_pin(outputs[i]);
        gpio_intr_disable(outputs[i]);
        gpio_set_direction(outputs[i], GPIO_MODE_OUTPUT);
        gpio_pullup_dis(outputs[i]);
        gpio_pulldown_en(outputs[i]);
        gpio_set_level(outputs[i], OFF_LEVEL);
    }
    gpio_reset_pin(BUTTON_PIN);
    gpio_pullup_dis(BUTTON_PIN);
    gpio_pulldown_dis(BUTTON_PIN);
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, (void*) NULL);

    app_config_cbs.config_srv = app_ble_mesh_config_server_cb;
    app_config_cbs.sensor_client = app_ble_mesh_sensor_client_cb;
    app_config_cbs.mqtt = app_mqtt_handler_cb;
    ESP_ERROR_CHECK(app_config_init(&app_config_cbs));		    // Initializing and loading configuration

    ds_buses[0].pin = GPIO_NUM_32;
    ds_buses[0].resolution = DS18B20_RESOLUTION_12_BIT;
    ds_buses[1].pin = GPIO_NUM_33;
    ds_buses[1].resolution = DS18B20_RESOLUTION_12_BIT;
    app_ds_init(ds_buses);

    ESP_LOGI(TAG, "Initialize temp queue");
    temp_queue = xQueueCreate(TEMP_QUEUE_LENGTH, sizeof(temp_queue_t));

    ESP_LOGI(TAG, "Starting DS18b20 thread");
    if (xTaskCreate(ds_task, "DS18b20", TASK_STACK_SIZE, NULL, 1, NULL) != pdPASS){
        ESP_LOGE(TAG, "Error creating task!");
        abort();
    }

    ESP_LOGI(TAG, "Creating alarm timers");
    for( uint8_t x = 0; x < CHANNEL_NUMBER; x++ ){
         alarm_timers[x] = xTimerCreate("Alarm", ALARM_PERIOD_MS / portTICK_PERIOD_MS, pdTRUE, &outputs[x], alarm_timer_cb);
         if( alarm_timers[x] == NULL ){
             ESP_LOGE(TAG, "Alarm timer was not created.");
             abort();
         } else {
             if( xTimerStart( alarm_timers[x], 0 ) != pdPASS ){
                 ESP_LOGE(TAG, "Alarm timer was not started.");
                 abort();
             }
         }
    }

    ESP_LOGI(TAG, "Starting worker thread");
    if (xTaskCreate(worker_task, "worker", TASK_STACK_SIZE, NULL, 1, NULL) != pdPASS){
        ESP_LOGE(TAG, "Error creating task!");
        abort();
    }

    ESP_LOGI(TAG, "Starting sensor thread");
    if (xTaskCreate(sensor_task, "Sensor", TASK_STACK_SIZE, NULL, 1, NULL) != pdPASS){
        ESP_LOGE(TAG, "Error creating task!");
        abort();
    }
    bool config_mqtt_enable;
    app_config_getBool("ble_mesh_enable", &config_mqtt_enable);
    if (config_mqtt_enable){
        ESP_LOGI(TAG, "Starting temperature notification timer");
        temperature_timer = xTimerCreate("temperature_timer", CUR_TEMP_MS / portTICK_PERIOD_MS, pdTRUE, NULL, temperature_timer_cb);
        xTimerStart(temperature_timer, 0);
    }
}

