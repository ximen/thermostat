/**
 * @file
 * @brief Interface definitions for the ESP32 GPIO driver used to communicate with devices
 *        on the One Wire Bus.
 *
 * This library provides structures and functions that are useful for communicating
 * with DS18B20 devices connected via a Maxim Integrated 1-Wire® bus.
  *
 */
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#define DS_BUS_NUMBER   2   // Number of 1-Wire buses
#define DS_MAX_DEVICES  1   // Max device number on single bus

typedef struct {
    OneWireBus          *owb;
    owb_rmt_driver_info rmt_driver;
    uint8_t             device_count;
    gpio_num_t          pin;    
    DS18B20_RESOLUTION  resolution;    
    DS18B20_Info        *devices[DS_MAX_DEVICES];
    OneWireBus_ROMCode  device_rom_codes[DS_MAX_DEVICES];
} ds_bus_t;

/**
 * @brief      Initialize 1-Wire subsystem
 *
 * Initialization RMT drivers and 1-Wire buses. After bus initialization initializes devices on each bus
 * .pin parameter in ds_bus_t structure must be filled prior call.
 * 
 * @param[in]   buses  pointer to array of structures containing information on 1-Wire buses
 * 
 * @return
 *             - ESP_OK if configuration was initialized successfully
 *             - one of the error codes from the underlying flash storage driver
 */
void app_ds_init(ds_bus_t *buses);

/**
 * @brief      DeInitialize 1-Wire subsystem resources
 *
 * Free dynamically allocated resources
  * 
 * @param[in]   buses  pointer to array of structures containing information on 1-Wire buses
 * 
 * @return
 *             - ESP_OK if configuration was initialized successfully
 *             - one of the error codes from the underlying flash storage driver
 */void app_ds_deinit(ds_bus_t *buses);

 esp_err_t app_ds_read_temperature_all(ds_bus_t *bus, float *readings);