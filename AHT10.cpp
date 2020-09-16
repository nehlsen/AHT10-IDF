#include "AHT10.h"
#include <driver/i2c.h>
#include <esp_log.h>
#include <cstring>

static const char *LOG_TAG = "AHT10";

#define AHT10_INIT_CMD             0xE1  //initialization command for AHT10
#define AHT10_START_MEASURMENT_CMD 0xAC  //start measurment command
#define AHT10_NORMAL_CMD           0xA8  //normal cycle mode command, no info in datasheet!!!
#define AHT10_SOFT_RESET_CMD       0xBA  //soft reset command

#define AHT10_INIT_NORMAL_MODE     0x00  //enable normal mode
#define AHT10_INIT_CYCLE_MODE      0x20  //enable cycle mode
#define AHT10_INIT_CMD_MODE        0x40  //enable command mode
#define AHT10_INIT_CAL_ENABLE      0x08  //load factory calibration coeff

#define AHT10_DATA_MEASURMENT_CMD  0x33  //no info in datasheet!!! my guess it is DAC resolution, saw someone send 0x00 instead
#define AHT10_DATA_NOP             0x00  //no info in datasheet!!!

#define AHT10_MEASURMENT_DELAY     80    //at least 75 milliseconds
#define AHT10_POWER_ON_DELAY       40    //at least 20..40 milliseconds
#define AHT10_CMD_DELAY            350   //at least 300 milliseconds, no info in datasheet!!!
#define AHT10_SOFT_RESET_DELAY     20    //less than 20 milliseconds

#if defined(CONFIG_AHT10_I2C_PORT_NUMBER_1)
static const i2c_port_t I2CPortNumber = I2C_NUM_1;
#else
static const i2c_port_t I2CPortNumber = I2C_NUM_0;
#endif

#if defined(CONFIG_AHT10_I2C_ADDRESS_0X39)
static const uint8_t I2CAddress = 0x39;
#else
static const uint8_t I2CAddress = 0x38;
#endif

AHT10::AHT10()
{
    memset(&m_rawData, 0, 6 * sizeof(uint8_t));

    initBus();
    activateOneTimeMode();
    readStatusByte();
    loadFactoryCalibration();
}

void AHT10::initBus()
{
    ESP_LOGV(LOG_TAG, "initBus()");

    i2c_config_t config;
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = static_cast<gpio_num_t>(CONFIG_AHT10_PIN_SDA);
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_io_num = static_cast<gpio_num_t>(CONFIG_AHT10_PIN_SCL);
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = CONFIG_AHT10_I2C_CLOCK_SPEED;

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_param_config(I2CPortNumber, &config));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_driver_install(I2CPortNumber, config.mode, 0, 0, 0));
    vTaskDelay(pdMS_TO_TICKS(AHT10_POWER_ON_DELAY));
}

bool AHT10::activateOneTimeMode()
{
    ESP_LOGV(LOG_TAG, "activateOneTimeMode()");
    return write3byte(
            AHT10_NORMAL_CMD,
            AHT10_DATA_NOP,
            AHT10_DATA_NOP
    );
}

bool AHT10::activateContinuousMode()
{
    ESP_LOGV(LOG_TAG, "activateContinuousMode()");
    return write3byte(
            AHT10_INIT_CMD,
            AHT10_INIT_CYCLE_MODE | AHT10_INIT_CAL_ENABLE,
            AHT10_DATA_NOP
    );
}

bool AHT10::loadFactoryCalibration()
{
    ESP_LOGV(LOG_TAG, "loadFactoryCalibration()");
    return write3byte(
            AHT10_INIT_CMD, // FIXME this seems to be the "real" init
            AHT10_INIT_CAL_ENABLE,
            AHT10_DATA_NOP
            ) && isCalibrated();
}

bool AHT10::write3byte(uint8_t byte0, uint8_t byte1, uint8_t byte2)
{
    ESP_LOGV(LOG_TAG, "write3byte()");

    i2c_cmd_handle_t commandHandle = i2c_cmd_link_create();
    if (commandHandle == nullptr) {
        ESP_LOGE(LOG_TAG, "write3byte(), failed to acquire i2c command handle");
        return false;
    }

    i2c_master_start(commandHandle);
    i2c_master_write_byte(commandHandle, (I2CAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(commandHandle, byte0, true);
    i2c_master_write_byte(commandHandle, byte1, true);
    i2c_master_write_byte(commandHandle, byte2, true);
    i2c_master_stop(commandHandle);
    bool i2cCommandResult = i2c_master_cmd_begin(I2CPortNumber, commandHandle, pdMS_TO_TICKS(1000)) == ESP_OK;
    i2c_cmd_link_delete(commandHandle);
    vTaskDelay(pdMS_TO_TICKS(AHT10_CMD_DELAY));

    ESP_LOGD(LOG_TAG, "write3byte(), Result: %s", (i2cCommandResult ? "OK" : "FAIL"));
    return i2cCommandResult;
}

bool AHT10::readRawData()
{
    ESP_LOGV(LOG_TAG, "readRawData()");

    if (!write3byte(
            AHT10_START_MEASURMENT_CMD,
            AHT10_DATA_MEASURMENT_CMD,
            AHT10_DATA_NOP
        )) {
        return false;
    }

    if (!isCalibrated()) {
        ESP_LOGW(LOG_TAG, "readRawData(), NOT calibrated");
//        return false;
    }
    if (isBusy()) {
        vTaskDelay(pdMS_TO_TICKS(AHT10_MEASURMENT_DELAY));
        ESP_LOGD(LOG_TAG, "readRawData(), isBusy!, now? %s / status:%d", (isBusy() ? "Y" : "N"), m_statusByte);
    }

    i2c_cmd_handle_t CommandHandle = i2c_cmd_link_create();
    if (CommandHandle == nullptr) {
        ESP_LOGE(LOG_TAG, "readRawData(), failed to acquire i2c command handle");
        return false;
    }

    uint8_t data[6];
    i2c_master_start(CommandHandle);
    i2c_master_write_byte(CommandHandle, (I2CAddress << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(CommandHandle, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(CommandHandle, &data[1], I2C_MASTER_ACK);
    i2c_master_read_byte(CommandHandle, &data[2], I2C_MASTER_ACK);
    i2c_master_read_byte(CommandHandle, &data[3], I2C_MASTER_ACK);
    i2c_master_read_byte(CommandHandle, &data[4], I2C_MASTER_ACK);
    i2c_master_read_byte(CommandHandle, &data[5], I2C_MASTER_NACK);
    i2c_master_stop(CommandHandle);

    bool Result = i2c_master_cmd_begin(I2CPortNumber, CommandHandle, pdMS_TO_TICKS(1000)) == ESP_OK;
    i2c_cmd_link_delete(CommandHandle);

    ESP_LOGD(LOG_TAG, "readRawData(), READ Result: %s", (Result ? "OK" : "FAIL"));

    if (Result) {
        memcpy(m_rawData, data, 6);
    }

    return Result;
}

bool AHT10::readStatusByte()
{
    ESP_LOGV(LOG_TAG, "readStatusByte()");

    i2c_cmd_handle_t commandHandle = i2c_cmd_link_create();
    if (commandHandle == nullptr) {
        ESP_LOGE(LOG_TAG, "readStatusByte(), failed to acquire i2c command handle");
        return false;
    }

    i2c_master_start(commandHandle);
    i2c_master_write_byte(commandHandle, (I2CAddress << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(commandHandle, &m_statusByte, I2C_MASTER_ACK);
    i2c_master_stop(commandHandle);
    bool i2cCommandResult = i2c_master_cmd_begin(I2CPortNumber, commandHandle, pdMS_TO_TICKS(1000)) == ESP_OK;
    i2c_cmd_link_delete(commandHandle);

    ESP_LOGD(LOG_TAG, "readStatusByte(), READ Result: %s, statusByte: %d", (i2cCommandResult ? "OK" : "FAIL"), m_statusByte);
    return i2cCommandResult;
}

bool AHT10::hasStatus() const
{
    return m_statusByte != 0x0;
}

bool AHT10::isCalibrated(bool forceFresh)
{
    if (forceFresh || !hasStatus()) readStatusByte();

    uint8_t mask = 0b00001000;
    return m_statusByte & mask;
}

bool AHT10::isBusy(bool forceFresh)
{
    if (forceFresh || !hasStatus()) readStatusByte();

    uint8_t mask = 0b10000000;
    return m_statusByte & mask;
}

float AHT10::getTemperature()
{
    uint32_t temperature = ((uint32_t)(m_rawData[3] & 0x0F) << 16) | ((uint16_t)m_rawData[4] << 8) | m_rawData[5]; //20-bit raw temperature data

    return (float)temperature * 0.000191 - 50;
}

float AHT10::getHumidity()
{
    uint32_t rawData = (((uint32_t)m_rawData[1] << 16) | ((uint16_t)m_rawData[2] << 8) | (m_rawData[3])) >> 4; //20-bit raw humidity data

    float humidity = (float)rawData * 0.000095;

    if (humidity < 0)   return 0;
    if (humidity > 100) return 100;
    return humidity;
}

//bool AHT10::softReset(void)
//{
//    Wire.beginTransmission(_address);
//
//#if (ARDUINO) >= 100
//    Wire.write(AHT10_SOFT_RESET_CMD);
//#else
//    Wire.send(AHT10_SOFT_RESET_CMD);
//#endif
//
//    if (Wire.endTransmission(true) != 0) return false; //safety check, make sure sensor reset
//
//    delay(AHT10_SOFT_RESET_DELAY);
//
//    setNormalMode();                                   //reinitialize sensor registers after reset
//
//    return enableFactoryCalCoeff();                    //reinitialize sensor registers after reset
//}
