#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"

#include "MS5611_Baro.h"

#include "stdint.h"

MS5611 g_Barometer;

void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.println("MS5611 example program. Queries sensor every second and prints pressure (mBar) and temperature (C)");

    Wire.begin();
    SPI.begin();

    MS5611::Configuration baroConfig;
    baroConfig.m_ChipSelectPin = 5; // D10 Arduino | D5 ESP32
    baroConfig.m_Mode = MS5611::Mode::M_SPI;
    baroConfig.m_CSBStatus = LOW;
    g_Barometer.Initialize(baroConfig);
}

void loop()
{
    float temperature = 0.0f;
    float pressure = 0.0f;
    g_Barometer.ReadSensor(temperature, pressure, MS5611::Sampling::R_512, MS5611::Sampling::R_512);

    Serial.print(pressure); Serial.print(","); Serial.println(temperature);

    delay(1000);
}