#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
#include "app_ds.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "DS18B20"

void app_ds_find_devices(ds_bus_t *bus){
    ESP_LOGI(TAG, "Find devices on bus:");
    int num_devices = 0;
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(bus->owb, &search_state, &found);
    while (found){
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        printf("  %d : %s\n", num_devices, rom_code_s);
        bus->device_rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        owb_search_next(bus->owb, &search_state, &found);
    }
    printf("Found %d device%s\n", num_devices, num_devices == 1 ? "" : "s");
    if (num_devices == 1){
        // For a single device only:
        OneWireBus_ROMCode rom_code;
        owb_status status = owb_read_rom(bus->owb, &rom_code);
        if (status == OWB_STATUS_OK){
            char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
            owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
            printf("Single device %s present\n", rom_code_s);
        } else {
            printf("An error occurred reading ROM code: %d", status);
        }
    } else {
        // Search for a known ROM code (LSB first):
        // For example: 0x1502162ca5b2ee28
        OneWireBus_ROMCode known_device = {
            .fields.family = { 0x28 },
            .fields.serial_number = { 0xee, 0xb2, 0xa5, 0x2c, 0x16, 0x02 },
            .fields.crc = { 0x15 },
        };
        char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
        owb_string_from_rom_code(known_device, rom_code_s, sizeof(rom_code_s));
        bool is_present = false;

        owb_status search_status = owb_verify_rom(bus->owb, known_device, &is_present);
        if (search_status == OWB_STATUS_OK){
            printf("Device %s is %s\n", rom_code_s, is_present ? "present" : "not present");
        } else {
            printf("An error occurred searching for known device: %d", search_status);
        }
    }
    bus->device_count = num_devices;
}

void app_ds_init_devices(ds_bus_t *bus){
    // Create DS18B20 devices on the 1-Wire bus
    for (int i = 0; i < bus->device_count; ++i)
    {
        bus->devices[i] = ds18b20_malloc();  // heap allocation

        if (bus->device_count == 1){
            printf("Single device optimisations enabled\n");
            ds18b20_init_solo(bus->devices[i], bus->owb);          // only one device on bus
        } else {
            ds18b20_init(bus->devices[i], bus->owb, bus->device_rom_codes[i]); // associate with bus and device
        }
        ds18b20_use_crc(bus->devices[i], true);           // enable CRC check on all reads
        ds18b20_set_resolution(bus->devices[i], bus->resolution);
    }

    // Check for parasitic-powered devices
    bool parasitic_power = false;
    ds18b20_check_for_parasite_power(bus->owb, &parasitic_power);
    if (parasitic_power) {
        printf("Parasitic-powered devices detected");
    }

    // In parasitic-power mode, devices cannot indicate when conversions are complete,
    // so waiting for a temperature conversion must be done by waiting a prescribed duration
    owb_use_parasitic_power(bus->owb, parasitic_power);
}

void app_ds_read_temperature_all(ds_bus_t *bus, float *readings){
    // Read temperatures more efficiently by starting conversions on all devices at the same time
    int errors_count[DS_MAX_DEVICES] = {0};
    int sample_count = 0;
    if (bus->device_count > 0){
        ds18b20_convert_all(bus->owb);

        // In this application all devices use the same resolution,
        // so use the first device to determine the delay
        ds18b20_wait_for_conversion(bus->devices[0]);

        // Read the results immediately after conversion otherwise it may fail
        // (using printf before reading may take too long)
        DS18B20_ERROR errors[DS_MAX_DEVICES] = { 0 };

        for (int i = 0; i < bus->device_count; ++i){
            errors[i] = ds18b20_read_temp(bus->devices[i], &readings[i]);
        }
    } else {
        printf("\nNo DS18B20 devices detected!\n");
    }
}

void app_ds_init(ds_bus_t *buses){
    assert(DS_BUS_NUMBER < 5);
    ESP_LOGI(TAG, "OWB Init %d buses", DS_BUS_NUMBER);
    for (uint8_t i=0; i<DS_BUS_NUMBER; i++){
        buses[i].owb= owb_rmt_initialize(&buses[i].rmt_driver, buses[i].pin, i*2+1, i*2);
        owb_use_crc(buses[i].owb, true);  // enable CRC check for ROM code
        app_ds_find_devices(&buses[i]);
        ESP_LOGI(TAG, "Devices init on bus %d", i);
        for (uint8_t j=0; j<buses[i].device_count; j++){
            ESP_LOGI(TAG, "Device %d", j);
            app_ds_init_devices(&buses[i]);
        }
    }
}
    
void app_ds_deinit(ds_bus_t *buses){
    // clean up dynamically allocated data
    for(uint8_t i=0; i<DS_BUS_NUMBER; i++){
        ESP_LOGI(TAG, "Cleaning up DS18b20 devices");
        for (int j = 0; j < buses[i].device_count; ++j){
            ds18b20_free(&buses[i].devices[j]);
        }
        ESP_LOGI(TAG, "Cleaning up DS18b20 buses");
        for (uint8_t j=0; j<DS_BUS_NUMBER; j++){
            owb_uninitialize(buses[j].owb);
        }
    }
}
