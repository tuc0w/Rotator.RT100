#include "CustomEEPROM.h"
#include "Motor.h"

#pragma once

#define RESPONSE_OK "(OK)"
#define RESPONSE_KO "(KO)"

class StringProxy
{
private:
    CustomEEPROM *_eeprom;
    Motor *_motor;
    char _resultBuffer1[50];
    char _resultBuffer2[50];
    char *_uintToChar(unsigned int value);
    bool _commandEndsWith(char c, char commandParam[], int commandParamLength);

public:
    void init(CustomEEPROM &eeprom, Motor &motor);
    float getStepsPerDeg();
    float stepsToDeg(unsigned long steps);
    unsigned long degToSteps(float deg);
    char const *processFalconCommand(char *command, char *commandParam, int commandParamLength);
};
