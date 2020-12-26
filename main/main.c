#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "app_config.h"
#include "app_config_wifi.h"
#include "app_config_http.h"
#include "app_config_ble_mesh.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_sensor_model_api.h"
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

ds_bus_t    ds_buses[DS_BUS_NUMBER] = {0};
app_config_cbs_t app_config_cbs;
TimerHandle_t alarm_timers[CHANNEL_NUMBER];
TimerHandle_t reset_timer;
bool alarms[CHANNEL_NUMBER] = {true};
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

static uint16_t sensor_prop_id;

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

void example_ble_mesh_send_sensor_message(uint32_t opcode)
{
    esp_ble_mesh_sensor_client_get_state_t get = {0};
    esp_ble_mesh_client_common_param_t common = {0};
    esp_err_t err = ESP_OK;

    switch (opcode) {
    case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET:
        get.cadence_get.property_id = sensor_prop_id;
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
        get.settings_get.sensor_property_id = sensor_prop_id;
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET:
        get.series_get.property_id = sensor_prop_id;
        break;
    default:
        break;
    }

    err = esp_ble_mesh_sensor_client_get_state(&common, &get);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send sensor message 0x%04x", opcode);
    }
}

static void example_ble_mesh_sensor_timeout(uint32_t opcode)
{
    switch (opcode) {
    case ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET:
        ESP_LOGW(TAG, "Sensor Descriptor Get timeout, opcode 0x%04x", opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET:
        ESP_LOGW(TAG, "Sensor Cadence Get timeout, opcode 0x%04x", opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET:
        ESP_LOGW(TAG, "Sensor Cadence Set timeout, opcode 0x%04x", opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
        ESP_LOGW(TAG, "Sensor Settings Get timeout, opcode 0x%04x", opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_GET:
        ESP_LOGW(TAG, "Sensor Setting Get timeout, opcode 0x%04x", opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET:
        ESP_LOGW(TAG, "Sensor Setting Set timeout, opcode 0x%04x", opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_GET:
        ESP_LOGW(TAG, "Sensor Get timeout 0x%04x", opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET:
        ESP_LOGW(TAG, "Sensor Column Get timeout, opcode 0x%04x", opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET:
        ESP_LOGW(TAG, "Sensor Series Get timeout, opcode 0x%04x", opcode);
        break;
    default:
        ESP_LOGE(TAG, "Unknown Sensor Get/Set opcode 0x%04x", opcode);
        return;
    }

    example_ble_mesh_send_sensor_message(opcode);
}

static void app_ble_mesh_sensor_client_cb(esp_ble_mesh_sensor_client_cb_event_t event, esp_ble_mesh_sensor_client_cb_param_t *param){
    ESP_LOGI(TAG, "Sensor client, event %u, addr 0x%04x", event, param->params->ctx.addr);

    if (param->error_code) {
        ESP_LOGE(TAG, "Send sensor client message failed (err %d)", param->error_code);
        return;
    }

    switch (event) {
    case ESP_BLE_MESH_SENSOR_CLIENT_GET_STATE_EVT:
        switch (param->params->opcode) {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET:
            ESP_LOGI(TAG, "Sensor Descriptor Status, opcode 0x%04x", param->params->ctx.recv_op);
            if (param->status_cb.descriptor_status.descriptor->len != ESP_BLE_MESH_SENSOR_SETTING_PROPERTY_ID_LEN &&
                param->status_cb.descriptor_status.descriptor->len % ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN) {
                ESP_LOGE(TAG, "Invalid Sensor Descriptor Status length %d", param->status_cb.descriptor_status.descriptor->len);
                return;
            }
            if (param->status_cb.descriptor_status.descriptor->len) {
                ESP_LOG_BUFFER_HEX("Sensor Descriptor", param->status_cb.descriptor_status.descriptor->data,
                    param->status_cb.descriptor_status.descriptor->len);
                /* If running with sensor server example, sensor client can get two Sensor Property IDs.
                 * Currently we use the first Sensor Property ID for the following demonstration.
                 */
                sensor_prop_id = param->status_cb.descriptor_status.descriptor->data[1] << 8 |
                                 param->status_cb.descriptor_status.descriptor->data[0];
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET:
            ESP_LOGI(TAG, "Sensor Cadence Status, opcode 0x%04x, Sensor Property ID 0x%04x",
                param->params->ctx.recv_op, param->status_cb.cadence_status.property_id);
            ESP_LOG_BUFFER_HEX("Sensor Cadence", param->status_cb.cadence_status.sensor_cadence_value->data,
                param->status_cb.cadence_status.sensor_cadence_value->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
            ESP_LOGI(TAG, "Sensor Settings Status, opcode 0x%04x, Sensor Property ID 0x%04x",
                param->params->ctx.recv_op, param->status_cb.settings_status.sensor_property_id);
            ESP_LOG_BUFFER_HEX("Sensor Settings", param->status_cb.settings_status.sensor_setting_property_ids->data,
                param->status_cb.settings_status.sensor_setting_property_ids->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_GET:
            ESP_LOGI(TAG, "Sensor Setting Status, opcode 0x%04x, Sensor Property ID 0x%04x, Sensor Setting Property ID 0x%04x",
                param->params->ctx.recv_op, param->status_cb.setting_status.sensor_property_id,
                param->status_cb.setting_status.sensor_setting_property_id);
            if (param->status_cb.setting_status.op_en) {
                ESP_LOGI(TAG, "Sensor Setting Access 0x%02x", param->status_cb.setting_status.sensor_setting_access);
                ESP_LOG_BUFFER_HEX("Sensor Setting Raw", param->status_cb.setting_status.sensor_setting_raw->data,
                    param->status_cb.setting_status.sensor_setting_raw->len);
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_GET:
            ESP_LOGI(TAG, "Sensor Status, opcode 0x%04x", param->params->ctx.recv_op);
            if (param->status_cb.sensor_status.marshalled_sensor_data->len) {
                ESP_LOG_BUFFER_HEX("Sensor Data", param->status_cb.sensor_status.marshalled_sensor_data->data,
                    param->status_cb.sensor_status.marshalled_sensor_data->len);
                uint8_t *data = param->status_cb.sensor_status.marshalled_sensor_data->data;
                uint16_t length = 0;
                for (; length < param->status_cb.sensor_status.marshalled_sensor_data->len; ) {
                    uint8_t fmt = ESP_BLE_MESH_GET_SENSOR_DATA_FORMAT(data);
                    uint8_t data_len = ESP_BLE_MESH_GET_SENSOR_DATA_LENGTH(data, fmt);
                    uint16_t prop_id = ESP_BLE_MESH_GET_SENSOR_DATA_PROPERTY_ID(data, fmt);
                    uint8_t mpid_len = (fmt == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A ?
                                        ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN : ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN);
                    ESP_LOGI(TAG, "Format %s, length 0x%02x, Sensor Property ID 0x%04x",
                        fmt == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A ? "A" : "B", data_len, prop_id);
                    if (data_len != ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN) {
                        ESP_LOG_BUFFER_HEX("Sensor Data", data + mpid_len, data_len + 1);
                        length += mpid_len + data_len + 1;
                        data += mpid_len + data_len + 1;
                    } else {
                        length += mpid_len;
                        data += mpid_len;
                    }
                }
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET:
            ESP_LOGI(TAG, "Sensor Column Status, opcode 0x%04x, Sensor Property ID 0x%04x",
                param->params->ctx.recv_op, param->status_cb.column_status.property_id);
            ESP_LOG_BUFFER_HEX("Sensor Column", param->status_cb.column_status.sensor_column_value->data,
                param->status_cb.column_status.sensor_column_value->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET:
            ESP_LOGI(TAG, "Sensor Series Status, opcode 0x%04x, Sensor Property ID 0x%04x",
                param->params->ctx.recv_op, param->status_cb.series_status.property_id);
            ESP_LOG_BUFFER_HEX("Sensor Series", param->status_cb.series_status.sensor_series_value->data,
                param->status_cb.series_status.sensor_series_value->len);
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Get opcode 0x%04x", param->params->ctx.recv_op);
            break;
        }
        break;
    case ESP_BLE_MESH_SENSOR_CLIENT_SET_STATE_EVT:
        switch (param->params->opcode) {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET:
            ESP_LOGI(TAG, "Sensor Cadence Status, opcode 0x%04x, Sensor Property ID 0x%04x",
                param->params->ctx.recv_op, param->status_cb.cadence_status.property_id);
            ESP_LOG_BUFFER_HEX("Sensor Cadence", param->status_cb.cadence_status.sensor_cadence_value->data,
                param->status_cb.cadence_status.sensor_cadence_value->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET:
            ESP_LOGI(TAG, "Sensor Setting Status, opcode 0x%04x, Sensor Property ID 0x%04x, Sensor Setting Property ID 0x%04x",
                param->params->ctx.recv_op, param->status_cb.setting_status.sensor_property_id,
                param->status_cb.setting_status.sensor_setting_property_id);
            if (param->status_cb.setting_status.op_en) {
                ESP_LOGI(TAG, "Sensor Setting Access 0x%02x", param->status_cb.setting_status.sensor_setting_access);
                ESP_LOG_BUFFER_HEX("Sensor Setting Raw", param->status_cb.setting_status.sensor_setting_raw->data,
                    param->status_cb.setting_status.sensor_setting_raw->len);
            }
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Set opcode 0x%04x", param->params->ctx.recv_op);
            break;
        }
        break;
    case ESP_BLE_MESH_SENSOR_CLIENT_PUBLISH_EVT:
        break;
    case ESP_BLE_MESH_SENSOR_CLIENT_TIMEOUT_EVT:
        example_ble_mesh_sensor_timeout(param->params->opcode);
    default:
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
            if(!alarms[i]){
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

}

