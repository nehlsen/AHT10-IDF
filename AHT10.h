#ifndef AHT10_H
#define AHT10_H

#include <cstdint>

class AHT10
{
public:
    AHT10();

    float getTemperature();
    float getHumidity();

    //private:
    void initBus();
    bool activateOneTimeMode();
    bool activateContinuousMode();
    bool loadFactoryCalibration();

    bool write3byte(uint8_t byte0, uint8_t byte1, uint8_t byte2);

    bool readRawData();
    uint8_t m_rawData[6];

    bool readStatusByte();
    // 7:busy, 6: cmd mode, 5: cyclic mode, 4: reserved, 3: calibrated, 2-0: reserved
    uint8_t m_statusByte = 0x0;
    bool hasStatus() const;
    bool isCalibrated(bool forceFresh = false);
    bool isBusy(bool forceFresh = true);
};

#endif //AHT10_H
