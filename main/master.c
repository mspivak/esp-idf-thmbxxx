/*
 * SPDX-FileCopyrightText: 2016-2023 Espressif Systems (Shanghai) CO LTD
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

// Timeout to update cid over Modbus

#define SECOND (1000 / portTICK_PERIOD_MS)
#define MINUTE (60 * SECOND)

#define TIME_BETWEEN_CHARS (1 * SECOND)
#define TIME_BETWEEN_READINGS (1 * SECOND)

// The macro to get offset for parameter in the appropriate structure
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) + 1))
#define COIL_OFFSET(field) ((uint16_t)(offsetof(coil_reg_params_t, field) + 1))
// Discrete offset macro
#define DISCR_OFFSET(field) ((uint16_t)(offsetof(discrete_reg_params_t, field) + 1))

#define STR(fieldname) ((const char *)(fieldname))
// Options can be used as bit masks or parameter limits
#define OPTS(min_val, max_val, step_val)                   \
    {                                                      \
        .opt1 = min_val, .opt2 = max_val, .opt3 = step_val \
    }

static const char *TAG = "modbus_example";

const int BAUDRATE_MAP[] = {2400, 4800, 9600};

// Enumeration of modbus device addresses accessed by master device
enum
{
    MB_DEVICE_ADDR1 = 1 // Only one slave device used for the test (add other slave addresses here)
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum
{
    CID_HUM = 0,
    CID_TEMP,
    CID_CAL_HUM,
    CID_CAL_TEMP,
    CID_ADDR,
    CID_BAUD,
};

// Example Data (Object) Dictionary for Modbus parameters:
// The CID field in the table must be unique.
// Modbus Slave Addr field defines slave address of the device with correspond parameter.
// Modbus Reg Type - Type of Modbus register area (Holding register, Input Register and such).
// Reg Start field defines the start Modbus register number and Reg Size defines the number of registers for the characteristic accordingly.
// The Instance Offset defines offset in the appropriate parameter structure that will be used as instance to save parameter value.
// Data Type, Data Size specify type of the characteristic and its data size.
// Parameter Options field specifies the options that can be used to process parameter value (limits or masks).
// Access Mode - can be used to implement custom options for processing of characteristic (Read/Write restrictions, factory mode values and etc).
const mb_parameter_descriptor_t device_parameters[] = {
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode }
    {CID_HUM, STR("Humidity"), STR("%"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0, 1, HOLD_OFFSET(holding_data0), PARAM_TYPE_U16, 2, {}, PAR_PERMS_READ},
    {CID_TEMP, STR("Temperature"), STR("C"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 1, 1, HOLD_OFFSET(holding_data1), PARAM_TYPE_U16, 2, {}, PAR_PERMS_READ},
    {CID_CAL_HUM, STR("Humidity calibration"), STR("%"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 80, 1, HOLD_OFFSET(holding_data2), PARAM_TYPE_U16, 2, OPTS(-100, 100, 1), PAR_PERMS_READ},
    {CID_CAL_TEMP, STR("Temperature Calibration"), STR("C"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 81, 1, HOLD_OFFSET(holding_data3), PARAM_TYPE_U16, 2, OPTS(-100, 100, 1), PAR_PERMS_READ},
    {CID_ADDR, STR("ModBus address"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 2000, 1, HOLD_OFFSET(holding_data4), PARAM_TYPE_U8, 1, OPTS(0, 100, 1), PAR_PERMS_READ},
    {CID_BAUD, STR("ModBus baud rate"), STR("bauds"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 2001, 1, HOLD_OFFSET(holding_data5), PARAM_TYPE_U8, 1, OPTS(0, 10000, 1), PAR_PERMS_READ},
};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters) / sizeof(device_parameters[0]));

// The function to get pointer to parameter storage (instance) according to parameter description table
static void *master_get_param_data(const mb_parameter_descriptor_t *param_descriptor)
{
    assert(param_descriptor != NULL);
    void *instance_ptr = NULL;
    if (param_descriptor->param_offset != 0)
    {
        switch (param_descriptor->mb_param_type)
        {
        case MB_PARAM_HOLDING:
            instance_ptr = ((void *)&holding_reg_params + param_descriptor->param_offset - 1);
            break;
        case MB_PARAM_INPUT:
            instance_ptr = ((void *)&input_reg_params + param_descriptor->param_offset - 1);
            break;
        case MB_PARAM_COIL:
            instance_ptr = ((void *)&coil_reg_params + param_descriptor->param_offset - 1);
            break;
        case MB_PARAM_DISCRETE:
            instance_ptr = ((void *)&discrete_reg_params + param_descriptor->param_offset - 1);
            break;
        default:
            instance_ptr = NULL;
            break;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Wrong parameter offset for CID #%u", (unsigned)param_descriptor->cid);
        assert(instance_ptr != NULL);
    }
    return instance_ptr;
}

// User operation function to read slave values and check alarm
static void master_operation_func(void *arg)
{
    esp_err_t err = ESP_OK;
    u_int16_t value = 0;
    const mb_parameter_descriptor_t *param_descriptor = NULL;

    ESP_LOGI(TAG, "Start modbus test...");

    while (true)
    {
        // Read all found characteristics from slave(s)
        for (uint16_t cid = 0; (err != ESP_ERR_NOT_FOUND) && cid < MASTER_MAX_CIDS; cid++)
        {
            // Get data from parameters description table
            // and use this information to fill the characteristics description table
            // and having all required fields in just one table
            err = mbc_master_get_cid_info(cid, &param_descriptor);
            if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL))
            {
                void *temp_data_ptr = master_get_param_data(param_descriptor);
                assert(temp_data_ptr);
                uint8_t type = 0;
                err = mbc_master_get_parameter(cid,
                                               (char *)param_descriptor->param_key,
                                               (uint8_t *)temp_data_ptr, &type);
                if (err != ESP_OK)
                {

                    ESP_LOGE(TAG, "Characteristic #%u %s read fail, err 0x%x: %s.",
                             param_descriptor->cid,
                             param_descriptor->param_key,
                             (int)err,
                             (char *)esp_err_to_name(err));

                    continue;
                }

                value = *(u_int16_t *)temp_data_ptr;

                if (param_descriptor->cid == CID_ADDR || param_descriptor->cid == CID_BAUD)
                {

                    if (param_descriptor->cid == CID_BAUD)
                    {
                        value = BAUDRATE_MAP[value];
                    }

                    ESP_LOGI(TAG, "Characteristic #%u %s: %d %s",
                             param_descriptor->cid,
                             param_descriptor->param_key,
                             value,
                             param_descriptor->param_units);
                }
                else
                {

                    ESP_LOGI(TAG, "Characteristic #%u %s: %.1f %s",
                             param_descriptor->cid,
                             param_descriptor->param_key,
                             (float)value / 10,
                             param_descriptor->param_units);
                }
            }
            vTaskDelay(TIME_BETWEEN_CHARS);
        }
        vTaskDelay(TIME_BETWEEN_READINGS);
    }

    ESP_LOGI(TAG, "Destroy master...");
    ESP_ERROR_CHECK(mbc_master_destroy());
}

// Modbus master initialization
static esp_err_t master_init(void)
{
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
        .port = MB_PORT_NUM,
#if CONFIG_MB_COMM_MODE_ASCII
        .mode = MB_MODE_ASCII,
#elif CONFIG_MB_COMM_MODE_RTU
        .mode = MB_MODE_RTU,
#endif
        .baudrate = MB_DEV_SPEED,
        .parity = MB_PARITY_NONE
    };
    void *master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MB_RETURN_ON_FALSE((master_handler != NULL), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller initialization fail, returns(0x%x).", (int)err);
    err = mbc_master_setup((void *)&comm);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller setup fail, returns(0x%x).", (int)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD,
                       CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb serial set pin failure, uart_set_pin() returned (0x%x).", (int)err);

    err = mbc_master_start();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller start fail, returned (0x%x).", (int)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb serial set mode failure, uart_set_mode() returned (0x%x).", (int)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller set descriptor fail, returns(0x%x).", (int)err);
    ESP_LOGI(TAG, "Modbus master stack initialized...");
    return err;
}

void app_main(void)
{
    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(master_init());
    vTaskDelay(10);

    master_operation_func(NULL);
}