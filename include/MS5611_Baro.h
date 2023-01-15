#ifndef MS5611_BARO
#define MS5611_BARO

#include "stdint.h"

#define MS5611_DEBUG 1

class MS5611
{
public:
    enum Mode : uint8_t
    {
        M_I2C,
        M_SPI
    };

    struct Configuration
    {
        Mode m_Mode;
        uint8_t m_CSBStatus : 1;    // 1 = HIGH  0 = LOW (Only for I2C, defines the address to use)
        uint8_t m_ChipSelectPin;    // Pin used for CS (Only for SPI)
    };

    enum Sampling : uint8_t
    {
        R_256,
        R_512,
        R_1024,
        R_2048,
    };

    MS5611();
    MS5611(const MS5611& other) = delete;
    ~MS5611();

    // Must be called before reading sensor data.
    bool Initialize(const Configuration& config);

    // Reads the raw sensor data and returns the temperature (ºC) and corrected pressure (mBar)
    void ReadSensor(float& temperature, float& pressure, Sampling temperatureSampling = R_1024, Sampling pressureSampling = R_1024);

private:
    enum Commands : uint8_t
    {
        RESET       = 0x1E,
        ADC_READ    = 0,
    };

    void Reset();

    uint16_t ReadPROM(uint8_t address);

    void SendCommand(uint8_t command);

    uint32_t ReadADC();

    uint8_t EncodeConvertCommand(bool isTemperature, Sampling rate);

    uint32_t GetRawValue(bool isTemperature, Sampling rate);

    int32_t ComputeDT(uint32_t D2, uint16_t C5);

    int32_t ComputeTEMP(int32_t dT, uint16_t C6);

    int64_t ComputeOFF(uint16_t C2, uint16_t C4, int32_t dT);

    int64_t ComputeSENS(uint16_t C1, uint16_t C3, int32_t dT);

    int32_t ComputeP(uint32_t D1, int64_t SENS, int64_t OFF);

    void Test();

    uint16_t TestCRC(uint16_t* data);

    union PROMData
    {
        struct 
        {
            uint16_t m_FactoryData;
            uint16_t m_C1;
            uint16_t m_C2;
            uint16_t m_C3;
            uint16_t m_C4;
            uint16_t m_C5;
            uint16_t m_C6;
            uint16_t m_CRC;
        } m_Data;
        uint16_t m_RawData[8];
    }m_PROMData;
    uint8_t m_I2CAddress;
    Configuration m_Config;
    bool m_Initialized;
};

#endif