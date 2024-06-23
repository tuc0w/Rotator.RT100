#include "CustomEEPROM.h"
#include "Motor.h"
#include "StringProxy.h"

#pragma once
#define VOLTAGE_RES 5
#define ADC_MAX 1024
#define NUM_BITS 10
#define HOME_SENSOR_THRESHOLD 500
#define HOME_SENSOR_PIN A0
#define HOME_SENSOR_READINGS 12
#define HOME_SENSOR_READING_DELAY 50

class Homing
{
private:
    CustomEEPROM *_eeprom;
    unsigned long _currentPosition = 0L;
    unsigned long _homePosition = 0L;
    unsigned long _previousPosition = 0L;
    int _currentSensorValue = 9999;
    int _previousSensorValue = 9999;
    bool _isHomed = false;
    bool _isHoming = false;
    Motor *_motor;
    bool _pinsInitialized = false;
    bool _readingHomeValuesFinished = false;
    StringProxy *_stringProxy;
    uint16_t _getSensorReading();
    void _handleMovement();
    void _moveOneDegree();
    void _moveToHome();

public:
    void findHomePosition();
    bool init(CustomEEPROM &eeprom, Motor &motor, StringProxy &stringProxy);
    bool isHomed();
    bool isHoming();
};
