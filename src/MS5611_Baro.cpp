#include "MS5611_Baro.h"

#include "Arduino.h"
#include "Wire.h"

static int32_t ClampInt32(int32_t value, int32_t min, int32_t max)
{
    return  (value > max) ? max :
            (value < min) ? min : value;
}

static int64_t ClampInt64(int64_t value, int64_t min, int64_t max)
{
    return  (value > max) ? max :
            (value < min) ? min : value;
}

MS5611::MS5611()
{

}

MS5611::~MS5611()
{

}

void MS5611::Initialize(Mode mode)
{
    m_I2CAddress = 0x76; // TODO: Can also be 0x77

    Reset();

    // Read PROM calibration data
    for(uint8_t i = 0; i < 8; ++i)
    {
        const uint8_t promAddr = 0xA0 | (uint8_t)(i << 1);
        
        Wire.beginTransmission(m_I2CAddress);
        Wire.write(promAddr);
        Wire.endTransmission(1);

        Wire.requestFrom(m_I2CAddress, (uint8_t)2);
        uint8_t data0 = (uint8_t)Wire.read(); 
        uint8_t data1 = (uint8_t)Wire.read();
        m_PROMData.m_RawData[i] = (uint16_t)(data0 << 8) | (uint16_t)data1;
    }

#if MS5611_DEBUG
    Serial.println(m_PROMData.m_Data.m_FactoryData);
    Serial.println(m_PROMData.m_Data.m_C1);
    Serial.println(m_PROMData.m_Data.m_C2);
    Serial.println(m_PROMData.m_Data.m_C3);
    Serial.println(m_PROMData.m_Data.m_C4);
    Serial.println(m_PROMData.m_Data.m_C5);
    Serial.println(m_PROMData.m_Data.m_C6);
    Serial.println(m_PROMData.m_Data.m_CRC);
#endif

    //Test();
}

void MS5611::ReadSensor(float& temperature, float& pressure, Sampling temperatureSampling, Sampling pressureSampling)
{
#if 1
    const uint16_t C1 = m_PROMData.m_Data.m_C1;
    const uint16_t C2 = m_PROMData.m_Data.m_C2;
    const uint16_t C3 = m_PROMData.m_Data.m_C3;
    const uint16_t C4 = m_PROMData.m_Data.m_C4;
    const uint16_t C5 = m_PROMData.m_Data.m_C5;
    const uint16_t C6 = m_PROMData.m_Data.m_C6;
#else
    const uint16_t C1 = 40127;
    const uint16_t C2 = 36924;
    const uint16_t C3 = 23317;
    const uint16_t C4 = 23282;
    const uint16_t C5 = 33464;
    const uint16_t C6 = 28312;
#endif

    // Retrieve raw sensor values
    const uint32_t D1 = GetRawValue(false, pressureSampling);     // Pressure
    const uint32_t D2 = GetRawValue(true, temperatureSampling);   // Temperature

    // Calculate correction terms
    const int32_t dT = ClampInt32(ComputeDT(D2, C5), -16776960, 16777216);
    int32_t TEMP = ClampInt32(ComputeTEMP(dT, C6), -4000, 8500);
    int64_t OFF = ClampInt64(ComputeOFF(C2, C4, dT), -8589672450, 12884705280);
    int64_t SENS = ClampInt64(ComputeSENS(C1, C3, dT), -4294836225, 6442352640);

    int64_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;
    
    // Adjust based on current temperature reading
    if(TEMP < 2000)
    {
        const int64_t TEMP_SQ = ((int64_t)TEMP - (int64_t)2000) * ((int64_t)TEMP - (int64_t)2000);
        T2 = ((int64_t)dT * (int64_t)dT) / (int64_t)2147483648;
        OFF2 = (int64_t)5 * TEMP_SQ / (int64_t)2;
        SENS2 = (int64_t)5 * TEMP_SQ / (int64_t)4;
        if(TEMP < -1500)
        {
            const int64_t TEMP_SQ_2 = ((int64_t)TEMP + (int64_t)1500) * ((int64_t)TEMP + (int64_t)1500);
            OFF2 = (int64_t)OFF2 + (int64_t)7 * TEMP_SQ_2;
            SENS2 = SENS2 + (int64_t)11 * TEMP_SQ_2 / (int64_t)2;
        }
    }

    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;

    int32_t P = ClampInt64(ComputeP(D1, SENS, OFF), 1000, 120000);

    temperature = (float)TEMP / 100.0f;
    pressure = (float)P / 100.0f;

    //Serial.print(TEMP); Serial.print(","); Serial.print(D2); Serial.print(","); Serial.print(m_PROMData.m_Data.m_C5); Serial.print(","); Serial.println(dT);
    //Serial.print("PData,"); Serial.print(D1); Serial.print(","); Serial.print((double)SENS); Serial.print(","); Serial.println((double)OFF);
}

void MS5611::Reset()
{
    if(m_Mode == I2C)
    {
        Wire.beginTransmission(m_I2CAddress);
        Wire.write(RESET);
        Wire.endTransmission(1);
        delay(10);
    }
    else
    {

    }
}

uint8_t MS5611::EncodeConvertCommand(bool isTemperature, Sampling rate)
{
    uint8_t command = 0x40;
    command |= isTemperature ? 0x10 : 0;
    command |= (uint8_t)(rate << 1);
    return command;
}

uint32_t MS5611::GetRawValue(bool isTemperature, Sampling rate)
{
    uint32_t rawValue = 0;
    uint8_t command = 0xFF;
    
    // Send conversion command
    command = EncodeConvertCommand(isTemperature, rate);
    Wire.beginTransmission(m_I2CAddress);
    Wire.write(command);
    Wire.endTransmission();

    delay(10); // TO-DO: Is this the best we can do? Could we have an active check instead?

    // Prepare data to be read
    Wire.beginTransmission(m_I2CAddress);
    Wire.write(ADC_READ);
    Wire.endTransmission();

    // Read it
    Wire.requestFrom(m_I2CAddress, (uint8_t)3);
    rawValue |= (uint32_t)Wire.read() << 16;
    rawValue |= (uint32_t)Wire.read() << 8;
    rawValue |= (uint32_t)Wire.read() << 0;

    return rawValue;
}

int32_t MS5611::ComputeDT(uint32_t D2, uint16_t C5)
{
    return (int32_t)D2 - (int32_t)C5 * (int32_t)256;
}

int32_t MS5611::ComputeTEMP(int32_t dT, uint16_t C6)
{
    return (int32_t)((int64_t)2000 + (int64_t)dT * (int64_t)C6 / (int64_t)8388608);
}

int64_t MS5611::ComputeOFF(uint16_t C2, uint16_t C4, int32_t dT)
{
    return (int64_t)C2 * (int64_t)65536 + (int64_t)C4 * (int64_t)dT / (int64_t)128;
}

int64_t MS5611::ComputeSENS(uint16_t C1, uint16_t C3, int32_t dT)
{
    return (int64_t)C1 * (int64_t)32768 + (int64_t)C3 * (int64_t)dT / (int64_t)265;
}

int32_t MS5611::ComputeP(uint32_t D1, int64_t SENS, int64_t OFF)
{
    return (int32_t)(((int64_t)D1 * (int64_t)SENS / (int64_t)2097152 - (int64_t)OFF) / (int64_t)32768);
}

void MS5611::Test()
{
    const uint16_t C1 = 40127;
    const uint16_t C2 = 36924;
    const uint16_t C3 = 23317;
    const uint16_t C4 = 23282;
    const uint16_t C5 = 33464;
    const uint16_t C6 = 28312;

    const uint32_t D1 = 9085466;
    const uint32_t D2 = 8569150;

    int32_t dT = ComputeDT(D2, C5);
    int32_t TEMP = ComputeTEMP(2366, C6);

    Serial.println(dT);
    Serial.println(TEMP);

    // Looks to be correct, however, not quite sure why
    // https://github.com/Troynica/MS5611/blob/master/documentation/APPNote_520_C_code.pdf
    uint16_t nprom[] = {0x3132,0x3334,0x3536,0x3738,0x3940,0x4142,0x4344,0x4500};
    uint16_t result = TestCRC(nprom); // m_PROMData.m_RawData
    Serial.print("CRC original = "); Serial.print(nprom[7]); Serial.print(" , Calculated = "); Serial.println(result);

    while(true) {}
}

uint16_t MS5611::TestCRC(uint16_t* data)
{
    int cnt;            // simple counter
    uint16_t n_rem;     // crc reminder
    uint16_t crc_read;  // original value of the crc
    unsigned char n_bit;
    n_rem = 0x00;
    crc_read = data[7];             //save read CRC
    data[7] = (0xFF00 & (data[7])); //CRC byte is replaced by 0
    for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
    {// choose LSB or MSB
        if (cnt%2==1) n_rem ^= (unsigned short) ((data[cnt>>1]) & 0x00FF);
        else n_rem ^= (unsigned short) (data[cnt>>1]>>8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
            {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else
            {
                n_rem = (n_rem << 1);
            }
        }
    }
    n_rem= (0x000F & (n_rem >> 12));    // final 4-bit reminder is CRC code
    data[7] = crc_read;                 // restore the crc_read to its original place

    return (n_rem ^ 0x0); 
}