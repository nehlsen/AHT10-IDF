# AHT10 component for the EPS32 IDF

This is mostly a copy/fork of https://github.com/enjoyneering/AHT10/

## Sample Code

```c++
#include <esp_log.h>
#include <AHT10.h>
#include <esp_event.h>

#ifdef __cplusplus
extern "C" {
#endif

void app_main()
{
    ESP_LOGI("AHT10", "Test...");

    AHT10 sensor;

    do {
        sensor.readRawData();
        ESP_LOGI("AHT10", "Temp: %.2f, Hum: %.2f", sensor.getTemperature(), sensor.getHumidity());
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    } while (true);
}

#ifdef __cplusplus
} // extern "C"
#endif
```