/*
 * SPDX-FileCopyrightText: 2016-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "string.h"
#include "esp_log.h"
#include "modbus_params.h" // for modbus parameters structures
#include "mbcontroller.h"
#include "sdkconfig.h"

#define MB_PORT_NUM (CONFIG_MB_UART_PORT_NUM)   // Number of UART port used for Modbus connection
#define MB_DEV_SPEED (CONFIG_MB_UART_BAUD_RATE) // The communication speed of the UART

// Note: Some pins on target chip cannot be assigned for UART communication.
// See UART documentation for selected board and target to configure pins using Kconfig.

// The number of parameters that intended to be used in the particular control process
#define MASTER_MAX_CIDS num_device_parameters

// Number of reading of parameters from slave
#define MASTER_MAX_RETRY 3000

// Timeout to update cid over Modbus
#define UPDATE_CIDS_TIMEOUT_MS (5000)
#define UPDATE_CIDS_TIMEOUT_TICS (UPDATE_CIDS_TIMEOUT_MS / portTICK_PERIOD_MS)

// Timeout between polls
#define POLL_TIMEOUT_MS (500)
#define POLL_TIMEOUT_TICS (POLL_TIMEOUT_MS / portTICK_PERIOD_MS)

// The macro to get offset for parameter in the appropriate structure
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) + 1))
#define COIL_OFFSET(field) ((uint16_t)(offsetof(coil_reg_params_t, field) + 1))
// Discrete offset macro
#define DISCR_OFFSET(field) ((uint16_t)(offsetof(discrete_reg_params_t, field) + 1))

#define STR(fieldname) ((const char *)(fieldname))

#define OPTS(min_val, max_val, step_val)                   \
    {                                                      \
        .opt1 = min_val, .opt2 = max_val, .opt3 = step_val \
    }

static const char *TAG = "app_modbus";

// Enumeration of modbus device addresses accessed by master device
enum
{
    MB_DEVICE_ADDR1 = 1 // Only one slave device used for the test (add other slave addresses here)
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum
{
    CIS_HUM = 0,
    CID_TEMP,
    CID_CAL_HUM,
    CID_CAL_TEMP,
    CID_ADDR,
    CID_BAUD,
};

esp_err_t modbus_read_characteristic(const mb_parameter_descriptor_t *param_descriptor, uint16_t *value)
{
    esp_err_t err = ESP_OK;

    // Get data from parameters description table
    // and use this information to fill the characteristics description table
    // and having all required fields in just one table
    err = mbc_master_get_cid_info(param_descriptor->cid, &param_descriptor);
    if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL))
    {
        uint8_t type = 0;

        err = mbc_master_get_parameter(param_descriptor->cid, (char *)param_descriptor->param_key,
                                       (uint8_t *)value, &type);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                     param_descriptor->cid,
                     (char *)param_descriptor->param_key,
                     (int)err,
                     (char *)esp_err_to_name(err));
        }

        ESP_LOGI(TAG, "Characteristic #%d - %s = %0.1f %s read successful.",
                 param_descriptor->cid,
                 (char *)param_descriptor->param_key,
                 (float)*value / 10,
                 (char *)param_descriptor->param_units);

        vTaskDelay(POLL_TIMEOUT_TICS); // timeout between polls
    }
    return err;
}

// User operation function to read slave values and check alarm
static void modbus_read_sensor(void)
{

    uint16_t hum = 0;
    uint16_t temp = 0;
    mb_parameter_descriptor_t hum_descriptor = {CIS_HUM, STR("Humidity"), STR("%"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, {}, PAR_PERMS_READ};
    mb_parameter_descriptor_t temp_descriptor = {CID_TEMP, STR("Temperature"), STR("C"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 1, 1, HOLD_OFFSET(holding_data1), PARAM_TYPE_U16, 2, {}, PAR_PERMS_READ};

    // {CID_CAL_HUM, STR("Humidity calibration"), STR("%"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 80, 1, HOLD_OFFSET(holding_data2), PARAM_TYPE_U16, 2, OPTS(-100, 100, 1), PAR_PERMS_READ},
    // {CID_CAL_TEMP, STR("Temperature Calibration"), STR("C"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 81, 1, HOLD_OFFSET(holding_data3), PARAM_TYPE_U16, 2, OPTS(-100, 100, 1), PAR_PERMS_READ},
    // {CID_ADDR, STR("ModBus address"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 2000, 1, HOLD_OFFSET(holding_data4), PARAM_TYPE_U8, 1, OPTS(0, 100, 1), PAR_PERMS_READ},
    // {CID_BAUD, STR("ModBus baud rate"), STR("Bauds"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 2001, 1, HOLD_OFFSET(holding_data5), PARAM_TYPE_U8, 1, OPTS(0, 10000, 1), PAR_PERMS_READ},

    ESP_LOGI(TAG, "Reading Modbus sensor.");

    ESP_ERROR_CHECK(modbus_read_characteristic(&hum_descriptor, &hum));
    ESP_ERROR_CHECK(modbus_read_characteristic(&temp_descriptor, &temp));
}

static esp_err_t modbus_master_init(void)
{
    mb_communication_info_t comm = {
        .port = MB_PORT_NUM,
        .mode = MB_MODE_RTU,
        .baudrate = MB_DEV_SPEED,
        .parity = MB_PARITY_NONE};
    void *master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MB_RETURN_ON_FALSE((master_handler != NULL), ESP_ERR_INVALID_STATE, TAG,
                       "ModBus controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "ModBus controller initialization fail, returns(0x%x).",
                       (uint32_t)err);
    err = mbc_master_setup((void *)&comm);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "ModBus controller setup fail, returns(0x%x).",
                       (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD,
                       CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "ModBus serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);

    err = mbc_master_start();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "ModBus controller start fail, returns(0x%x).",
                       (uint32_t)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "ModBus serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "ModBus controller set descriptor fail, returns(0x%x).",
                       (uint32_t)err);

    ESP_LOGI(TAG, "Modbus master stack initialized.");
    return err;
}

void app_main(void)
{
    ESP_ERROR_CHECK(modbus_master_init());
    vTaskDelay(10);

    while (true)
    {
        modbus_read_sensor();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Destroying master modbus handle.");
    ESP_ERROR_CHECK(mbc_master_destroy());
}
