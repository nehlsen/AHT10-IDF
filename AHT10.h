#ifndef AHT10_H
#define AHT10_H

#include <cstdint>
#include <sdkconfig.h>

class AHT10
{
public:
    explicit AHT10(int i2cPinSda = CONFIG_AHT10_PIN_SDA, int i2cPinScl = CONFIG_AHT10_PIN_SCL);

    bool readRawData();
    float getTemperature();
    float getHumidity();

private:
    void initBus(int i2cPinSda, int i2cPinScl);
    bool activateOneTimeMode();
    bool activateContinuousMode();
    bool loadFactoryCalibration();
    void softReset();

    bool write3byte(uint8_t byte0, uint8_t byte1, uint8_t byte2);
    bool writeBytes(uint8_t *bytes, uint8_t numberOfBytes);

    uint8_t m_rawData[6];

    bool readStatusByte();
    // 7:busy, 6: cmd mode, 5: cyclic mode, 4: reserved, 3: calibrated, 2-0: reserved
    uint8_t m_statusByte = 0x0;
    bool hasStatus() const;
    bool isCalibrated(bool forceFresh = false);
    bool isBusy(bool forceFresh = true);
};

#endif //AHT10_H
