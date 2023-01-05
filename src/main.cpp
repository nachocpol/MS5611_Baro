#include "Arduino.h"
#include "Wire.h"

#include "MS5611_Baro.h"

#include "stdint.h"

MS5611 g_Barometer;

void setup()
{
    Serial.begin(9600);
    Serial.println("");

    Wire.begin();

    g_Barometer.Initialize(MS5611::Mode::I2C);   
}

void loop()
{
    float temperature = 0.0f;
    float pressure = 0.0f;
    g_Barometer.ReadSensor(temperature, pressure, MS5611::Sampling::R_512, MS5611::Sampling::R_512);

    Serial.print(pressure); Serial.print(","); Serial.println(temperature);

    delay(1000);
}