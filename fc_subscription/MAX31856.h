/// MAX31856.h Thermocouple Breakout Board Code (missing stuff for fault detection) ///

#pragma once
#include <stdio.h>          /// IO functions

#pragma once
#include <unistd.h>         /// Used to fork and pipe

#pragma once
#include <pigpio.h>         /// Used to control PI GPIO Pins

#define spiChan 0  // This is the normal SPI port on the pi not tested on 1

/// Some of the enumeration for the various MAX31856 settings (missing stuff for fault detection) ///
enum MAX31856_Unit_E
{
    MAX31856_Unit_Fahrenheit,
    MAX31856_Unit_Celcius,
};

enum MAX31856_ConversionMode_E
{
    MAX31856_ConversionMode_NormallyOff = 0b00000000,
    MAX31856_ConversionMode_Automatic =   0b10000000,
};

// WIP FAULTS and Cold-Junction stuff I don't need right now //

enum MAX31856_NoiseRejectionFilter_E
{
    MAX31856_NoiseRejectionFilter_60 = 0b00000000,
    MAX31856_NoiseRejectionFilter_50 = 0b00000001,
};

enum MAX31856_AveragingMode_E
{
    MAX31856_AveragingMode_1 =  0b00000000,
    MAX31856_AveragingMode_2 =  0b00010000,
    MAX31856_AveragingMode_4 =  0b00100000,
    MAX31856_AveragingMode_8 =  0b00110000,
    MAX31856_AveragingMode_16 = 0b01000000,
};

enum MAX31856_ThermocoupleType_En
{
    MAX31856_ThermocoupleType_B = 0b00000000,
    MAX31856_ThermocoupleType_E = 0b00000001,
    MAX31856_ThermocoupleType_J = 0b00000010,
    MAX31856_ThermocoupleType_K = 0b00000011,
    MAX31856_ThermocoupleType_N = 0b00000100,
    MAX31856_ThermocoupleType_R = 0b00000101,
    MAX31856_ThermocoupleType_S = 0b00000110,
    MAX31856_ThermocoupleType_T = 0b00000111,
    // WIP OTHER CUSTOM OPTIONS SEE DATA SHEET //
};


/// Structure to hold all info for one board ///
struct MAX31856_S
{
    int Hadle;              // SPI Handle
    int ChipSelectPin;      // Pin used to address the chip

    enum MAX31856_Unit_E Unit;
    enum MAX31856_ConversionMode_E ConversionMode;
    enum MAX31856_NoiseRejectionFilter_E NoiseRejectionFilter;
    enum MAX31856_AveragingMode_E AveragingMode;
    enum MAX31856_ThermocoupleType_En ThermocoupleType;

    char Configuration0Register;
    char Configuration1Register;
};

// Combines the settings into the Configuration Registers and sends to MAX31856 //
void MAX31856_CombineSettingsAndSend(struct MAX31856_S *MAX31856)
{
    char Buf[4];

    MAX31856->Configuration0Register = MAX31856->NoiseRejectionFilter | MAX31856->ConversionMode;
    MAX31856->Configuration1Register = MAX31856->AveragingMode | MAX31856->ThermocoupleType;

    Buf[0] = 0x80;
    Buf[1] = MAX31856->Configuration0Register;
    Buf[2] = 0x81;
    Buf[3] = MAX31856->Configuration1Register;

    gpioWrite(MAX31856->ChipSelectPin, 0);
    spiWrite(MAX31856->Hadle, Buf, 4);
    gpioWrite(MAX31856->ChipSelectPin, 1);

    return;
}

// Sets up MAX31856 com and applies settings //
struct MAX31856_S SetupMAX31856(int ChipSelectPin, enum MAX31856_ThermocoupleType_En Type, enum MAX31856_Unit_E Unit)
{
    struct MAX31856_S MAX31856;

    // Set up input settings and default settings //
    MAX31856.ChipSelectPin = ChipSelectPin;
    MAX31856.ThermocoupleType = Type;
    MAX31856.Unit = Unit;

    MAX31856.ConversionMode = MAX31856_ConversionMode_Automatic;
    MAX31856.NoiseRejectionFilter = MAX31856_ConversionMode_Automatic;

    // Setup the communication //
    MAX31856.Hadle = spiOpen(spiChan, 500000, 0b0010000000000000000011);
    gpioSetMode(MAX31856.ChipSelectPin, PI_OUTPUT);
    gpioWrite(MAX31856.ChipSelectPin, 1);

    return MAX31856;
};

// Returns the temperature with units based on MAX31856.Units //
double MAX31856GetTemperature(struct MAX31856_S *MAX31856)
{
    char TEMPBUF[4], SENDBUFF[4];
    int AssembledValue;

    SENDBUFF[0] = 0x0C;
    SENDBUFF[1] = 0x00;
    SENDBUFF[2] = 0x00;
    SENDBUFF[3] = 0x00;

    gpioWrite(MAX31856->ChipSelectPin, 0);
    spiXfer(MAX31856->Hadle, SENDBUFF, TEMPBUF, 4);
    gpioWrite(MAX31856->ChipSelectPin, 1);

    AssembledValue = ((int)TEMPBUF[3])<<8;
    AssembledValue += ((int)TEMPBUF[2])<<16;
    AssembledValue += ((int)TEMPBUF[1])<<24;

    if(MAX31856->Unit == MAX31856_Unit_Fahrenheit)
        return (AssembledValue / 1048576.0) * 9.0/5.0 + 32.0;
    else
        return (AssembledValue / 1048576.0);
}

// Ends communication with MAX31856 board //
#define MAX31856_Stop(A) spiClose(A.Hadle)