#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"

#include "MS5611_Baro.h"

#include "stdint.h"

MS5611 g_Barometer;

void setup()
{
    Serial.begin(9600);
    Serial.println("");

    Wire.begin();
    SPI.begin();

    MS5611::Configuration baroConfig;
    baroConfig.m_ChipSelectPin = 0xFF;
    baroConfig.m_Mode = MS5611::Mode::I2C;
    baroConfig.m_PSStatus = 1;
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