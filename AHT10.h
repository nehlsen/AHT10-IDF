#ifndef AHT10_H
#define AHT10_H

#include <stdint.h>

class AHT10
{
public:
    AHT10();

    float getTemperature();
    float getHumidity();

    //private:
    void initBus();
    bool initSensor();
    bool loadFactoryCalibration();

    bool readRawData();
    uint8_t m_rawData[6];

    uint8_t readStatusByte();
    uint8_t m_statusByte = 0x0;
    bool hasStatus();
    bool isCalibrated(bool forceFresh = false);
    bool isBusy(bool forceFresh = true);
};

#endif //AHT10_H
