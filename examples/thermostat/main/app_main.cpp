/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/* HomeKit Thermostat
*/

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_event.h>
#include <esp_log.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>

#include <hap_fw_upgrade.h>
#include <iot_button.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>

#include <HeatPump.h>

/*  Required for server verification during OTA, PEM format as string  */
char server_cert[] = {};

static const char *TAG = "HAP Thermostat";

#define THERMOSTAT_TASK_PRIORITY  1
#define THERMOSTAT_TASK_STACKSIZE 4 * 1024
#define THERMOSTAT_TASK_NAME      "hap_thermostat"

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
#define RESET_NETWORK_BUTTON_TIMEOUT        3

/* Reset to factory if button is pressed and held for more than 10 seconds */
#define RESET_TO_FACTORY_BUTTON_TIMEOUT     10

/* The button "Boot" will be used as the Reset button for the example */
#define RESET_GPIO  GPIO_NUM_0

HeatPump heatpump;

static void set_char_value(const char* char_uuid, hap_val_t *val) 
{
    hap_serv_t *hs = hap_acc_get_serv_by_uuid(hap_get_first_acc(), HAP_SERV_UUID_THERMOSTAT);
    if (hs) {
        hap_char_t *hc =  hap_serv_get_char_by_uuid(hs, char_uuid);
        if (hc) {
            hap_char_update_val(hc, val);
        }
    }
}

static char* get_heatpump_mode(int value) {
  switch (value) {
    case 0:
      return "OFF";
    case 1:
      return "HEAT";
    case 2:
      return "COOL";
    case 3:
      return "AUTO";
    default:
      return "OFF";
  }
}

static int get_homekit_target_heating_cooling_state(bool power, const char* mode) {
  if (!power) {
    return 0; // Off
  }

  if (strcmp(mode, "HEAT") == 0) {
    return 1; // Heat
  } else if (strcmp(mode, "COOL") == 0 || strcmp(mode, "DRY") == 0 || strcmp(mode, "FAN") == 0) {
    return 2; // Cool
  } else if (strcmp(mode, "AUTO") == 0) {
    return 3; // Auto
  } else {
    return 0; // Unknown mode, Off
  }
}

static int get_homekit_current_heating_cooling_state(bool operating, const char* mode) {
  if (!operating) {
    return 0; // Off
  }

  if (strcmp(mode, "HEAT") == 0) {
    return 1; // Heat
  } else if (strcmp(mode, "COOL") == 0) {
    return 2; // Cool
  } else {
    return 0; // Unknown - Off
  }
}

/**
 * @brief The network reset button callback handler.
 * Useful for testing the Wi-Fi re-configuration feature of WAC2
 */
static void reset_network_handler(void* arg)
{
    hap_reset_network();
}
/**
 * @brief The factory reset button callback handler.
 */
static void reset_to_factory_handler(void* arg)
{
    hap_reset_to_factory();
}

/**
 * The Reset button  GPIO initialisation function.
 * Same button will be used for resetting Wi-Fi network as well as for reset to factory based on
 * the time for which the button is pressed.
 */
static void reset_key_init(uint32_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);
    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
}

/* Mandatory identify routine for the accessory.
 * In a real accessory, something like LED blink should be implemented
 * got visual identification
 */
static int thermostat_identify(hap_acc_t *ha)
{
    ESP_LOGI(TAG, "Accessory identified");
    return HAP_SUCCESS;
}

/*
 * An optional HomeKit Event handler which can be used to track HomeKit
 * specific events.
 */
static void thermostat_hap_event_handler(void* arg, esp_event_base_t event_base, int event, void *data)
{
    switch(event) {
        case HAP_EVENT_PAIRING_STARTED :
            ESP_LOGI(TAG, "Pairing Started");
            break;
        case HAP_EVENT_PAIRING_ABORTED :
            ESP_LOGI(TAG, "Pairing Aborted");
            break;
        case HAP_EVENT_CTRL_PAIRED :
            ESP_LOGI(TAG, "Controller %s Paired. Controller count: %d",
                        (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_UNPAIRED :
            ESP_LOGI(TAG, "Controller %s Removed. Controller count: %d",
                        (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_CONNECTED :
            ESP_LOGI(TAG, "Controller %s Connected", (char *)data);
            break;
        case HAP_EVENT_CTRL_DISCONNECTED :
            ESP_LOGI(TAG, "Controller %s Disconnected", (char *)data);
            break;
        case HAP_EVENT_ACC_REBOOTING : {
            char *reason = (char *)data;
            ESP_LOGI(TAG, "Accessory Rebooting (Reason: %s)",  reason ? reason : "null");
            break;
        }
        default:
            /* Silently ignore unknown events */
            break;
    }
}

/* A callback for handling a read on the characteristics of Thermostat.
 * Read routines are generally not required as the value is available with th HAP core
 * when it is updated from write routines. For external triggers (like theremostat switched on/off
 * using physical button), accessories should explicitly call hap_char_update_val()
 * instead of waiting for a read request.
 */
static int thermostat_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
    if (hap_req_get_ctrl_id(read_priv)) {
        ESP_LOGI(TAG, "Received read from %s", hap_req_get_ctrl_id(read_priv));
    }
    
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_HEATING_COOLING_STATE)) {
        // 0 "Off", 1 "Heat", 2 "Cool", 3 "Auto"
        char* mode = heatpump.getModeSetting();
        bool operating = heatpump.getOperating();
        if (!mode) {
            *status_code = HAP_STATUS_RES_BUSY;
            return;
        }
        hap_val_t current_state_val;
        current_state_val.i = get_homekit_current_heating_cooling_state(operating, mode);
        hap_char_update_val(hc, &current_state_val);
        *status_code = HAP_STATUS_SUCCESS;

        ESP_LOGI(TAG, "Received Read. Thermostat Current State: %i",  current_state_val->i);
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_TEMPERATURE)) {
        // min 0, max 100, step 0.1, unit celsius
        hap_val_t current_temp_val;
        current_temp_val.f = heatpump.getRoomTemperature();
        ESP_LOGI(TAG, "Received Read. Thermostat Current Temp: %f", current_temp_val->f);

        hap_char_update_val(hc, &current_temp_val);
        *status_code = HAP_STATUS_SUCCESS;
    }
    // else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_ROTATION_DIRECTION)) {
    //    /* Read the current value, toggle it and set the new value.
    //     * A separate variable should be used for the new value, as the hap_char_get_val()
    //     * API returns a const pointer
    //     */
    //     const hap_val_t *cur_val = hap_char_get_val(hc);

    //     hap_val_t new_val;
    //     if (cur_val->i == 1) {
    //         new_val.i = 0;
    //     } else {
    //         new_val.i = 1;
    //     }
    //     hap_char_update_val(hc, &new_val);
    //     *status_code = HAP_STATUS_SUCCESS;
    // }

    return HAP_SUCCESS;
}

/* A callback for handling a write on the characteristics of Thermostat. */
static int thermostat_write(hap_write_data_t write_data[], int count,
        void *serv_priv, void *write_priv)
{
    if (hap_req_get_ctrl_id(write_priv)) {
        ESP_LOGI(TAG, "Received write from %s", hap_req_get_ctrl_id(write_priv));
    }
    ESP_LOGI(TAG, "Thermostat Write called with %d chars", count);
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_TARGET_HEATING_COOLING_STATE)) {
            //  0 "Off", 1 "Heat", 2 "Cool", 3 "Auto"
            bool power = (write->val.i == 1);
            char* mode = get_heatpump_mode(write->val.i);
            ESP_LOGI(TAG, "Received Write. Thermostat Target State: %s", mode);

            heatpump.setPowerSetting(power);
            heatpump.setModeSetting(mode);

            hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
        } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_TARGET_TEMPERATURE)) {
            // min 10, max 38, step 0.1, unit celsius
            ESP_LOGI(TAG, "Received Write. Thermostat Target Temp: %s", write->val.f);

            heatpump.setTemperature(write->val.f);

            hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
        } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_TEMPERATURE_DISPLAY_UNITS)) {
            // 0 "Celsius", 1 "Fahrenheit"
            // no-op there is no display on the heatpump.
        }
        // else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_ROTATION_DIRECTION)) {
        //     if (write->val.i > 1) {
        //         *(write->status) = HAP_STATUS_VAL_INVALID;
        //         ret = HAP_FAIL;
        //     } else {
        //         ESP_LOGI(TAG, "Received Write. Thermostat %s", write->val.i ? "AntiClockwise" : "Clockwise");
        //         hap_char_update_val(write->hc, &(write->val));
        //         *(write->status) = HAP_STATUS_SUCCESS;
        //     }
        // } 
        else {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
    }
    return ret;
}

static void heatpumpSettingsChanged() {
  const char* mode = heatpump.getModeSetting();
  bool power = heatpump.getPowerSettingBool();
  if (mode) {
    hap_val_t target_state_val;
    current_state_val.i = get_homekit_target_heating_cooling_state(power, mode);;
    set_char_value(HAP_CHAR_UUID_TARGET_HEATING_COOLING_STATE, target_state_val);
  }

  float temperature = heatpump.getTemperature();
  if (temperature) {
        hap_val_t temperature_val;
        temperature_val.f = temperature;
        set_char_value(HAP_CHAR_UUID_TARGET_TEMPERATURE, temperature_val);
    }
}

static void heatpumpStatusChanged(heatpumpStatus status) {
    float temperature = heatpump.getRoomTemperature();
    if (temperature) {
        hap_val_t temperature_val;
        temperature_val.f = temperature;
        set_char_value(HAP_CHAR_UUID_CURRENT_TEMPERATURE, temperature_val);
    }

    char* mode = heatpump.getModeSetting();
    bool operating = heatpump.getOperating();
    if (mode) {
        hap_val_t current_state_val;
        current_state_val.i = get_homekit_current_heating_cooling_state(operating, mode);
        set_char_value(HAP_CHAR_UUID_CURRENT_HEATING_COOLING_STATE, current_state_val);
    }
}

/* The main thread for handling the Thermostat Accessory */
static void thermostat_thread_entry(void *p)
{
    hap_acc_t *accessory;
    hap_serv_t *service;

    /* Configure HomeKit core to make the Accessory name (and thus the WAC SSID) unique,
     * instead of the default configuration wherein only the WAC SSID is made unique.
     */
    hap_cfg_t hap_cfg;
    hap_get_config(&hap_cfg);
    hap_cfg.unique_param = UNIQUE_NAME;
    hap_set_config(&hap_cfg);

    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally
     */
    hap_acc_cfg_t cfg = {
        .name = "Heat Pump",
        .manufacturer = "Mitsubishi",
        .model = "MSZ-GL06NA",
        .serial_num = "001122334455",
        .fw_rev = "0.9.0",
        .hw_rev = NULL,
        .pv = "1.1.0",
        .identify_routine = thermostat_identify,
        .cid = HAP_CID_THERMOSTAT,
    };
    /* Create accessory object */
    accessory = hap_acc_create(&cfg);

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E','S','P','3','2','H','A','P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    /* Create the Thermostat Service. Include the "name" since this is a user visible service  */
    
    service = hap_serv_thermostat_create(
        0 /* curr_heating_cooling_state: 0 "Off", 1 "Heat", 2 "Cool" */, 
        0 /* targ_heating_cooling_state: 0 "Off", 1 "Heat", 2 "Cool", 3 "Auto" */, 
        20.5556 /* curr_temp: min 0, max 100, step 0.1, unit celsius */, 
        20.5556 /* targ_temp: min 10, max 38, step 0.1, unit celsius */, 
        1 /* temp_disp_units: 0 "Celsius", 1 "Fahrenheit" */);

    hap_serv_add_char(service, hap_char_name_create("Mitsubishi Heat Pump"));

    /* Set the write callback for the service */
    hap_serv_set_write_cb(service, thermostat_write);

    /* Set the read callback for the service (optional) */
    hap_serv_set_read_cb(service, thermostat_read);

    /* Add the Thermostat Service to the Accessory Object */
    hap_acc_add_serv(accessory, service);

    /* Create the Firmware Upgrade HomeKit Custom Service.
     * Please refer the FW Upgrade documentation under components/homekit/extras/include/hap_fw_upgrade.h
     * and the top level README for more information.
     */
    hap_fw_upgrade_config_t ota_config = {
        .server_cert_pem = server_cert,
    };
    service = hap_serv_fw_upgrade_create(&ota_config);
    /* Add the service to the Accessory Object */
    hap_acc_add_serv(accessory, service);

    /* Add the Accessory to the HomeKit Database */
    hap_add_accessory(accessory);

    /* Register a common button for reset Wi-Fi network and reset to factory.
     */
    reset_key_init(RESET_GPIO);

    /* Query the controller count (just for information) */
    ESP_LOGI(TAG, "Accessory is paired with %d controllers",
                hap_get_paired_controller_count());

    /* Setup the heatpump connection */
    heatpump.setSettingsChangedCallback(heatpumpSettingsChanged);
    heatpump.setStatusChangedCallback(heatpumpStatusChanged);
    heatpump.connect(&Serial);

    /* For production accessories, the setup code shouldn't be programmed on to
     * the device. Instead, the setup info, derived from the setup code must
     * be used. Use the factory_nvs_gen utility to generate this data and then
     * flash it into the factory NVS partition.
     *
     * By default, the setup ID and setup info will be read from the factory_nvs
     * Flash partition and so, is not required to set here explicitly.
     *
     * However, for testing purpose, this can be overridden by using hap_set_setup_code()
     * and hap_set_setup_id() APIs, as has been done here.
     */
#ifdef CONFIG_EXAMPLE_USE_HARDCODED_SETUP_CODE
    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code(CONFIG_EXAMPLE_SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_EXAMPLE_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* Enable Hardware MFi authentication (applicable only for MFi variant of SDK) */
    hap_enable_mfi_auth(HAP_MFI_AUTH_HW);

    /* Initialize Wi-Fi */
    app_wifi_init();

    /* Register an event handler for HomeKit specific events.
     * All event handlers should be registered only after app_wifi_init()
     */
    esp_event_handler_register(HAP_EVENT, ESP_EVENT_ANY_ID, &thermostat_hap_event_handler, NULL);

    /* After all the initializations are done, start the HAP core */
    hap_start();
    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);
    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);
}

extern "C" void app_main()
{
    xTaskCreate(thermostat_thread_entry, THERMOSTAT_TASK_NAME, THERMOSTAT_TASK_STACKSIZE, NULL, THERMOSTAT_TASK_PRIORITY, NULL);
}

