#include <Arduino.h>
#include "Homing.h"

uint16_t Homing::_getSensorReading()
{
    int hallReadings = 0;
    for (int i = 0; i < HOME_SENSOR_READINGS; i++)
    {
        hallReadings += analogRead(HOME_SENSOR_PIN);
        delayMicroseconds(HOME_SENSOR_READING_DELAY);
    }

    return hallReadings / HOME_SENSOR_READINGS;
}

void Homing::_handleMovement()
{
    do
    {
        _motor->handleMotor();
    } while (_motor->isMoving());
}

void Homing::_moveOneDegree()
{
    unsigned long steps = _stringProxy->getStepsPerDeg();
    _eeprom->setTargetPosition(_eeprom->getPosition() + steps);
    _motor->applyStepMode();
    _motor->startMotor();
}

void Homing::_moveToHome()
{
    _eeprom->setTargetPosition(_homePosition);
    _motor->applyStepMode();
    _motor->startMotor();

    this->_handleMovement();

    _homePosition = 0;
    _isHomed = true;
    _isHoming = false;

    _eeprom->setHoming(false);
    _eeprom->setPosition(0);
    _eeprom->setTargetPosition(0);
    _eeprom->handleEeprom();
}

bool Homing::init(CustomEEPROM &eeprom, Motor &motor, StringProxy &stringProxy)
{
    _eeprom = &eeprom;
    _motor = &motor;
    _stringProxy = &stringProxy;

    if (!_pinsInitialized)
    {
        pinMode(HOME_SENSOR_PIN, INPUT);
        _pinsInitialized = true;
    }

    return true;
}

bool Homing::isHomed()
{
    return _isHomed;
}

bool Homing::isHoming()
{
    return _isHoming;
}

void Homing::findHomePosition()
{
    _isHoming = true;
    _eeprom->setHoming(true);
    _eeprom->setPosition(0);
    _eeprom->handleEeprom();

    for (int i = 0; i <= 360; i++)
    {
        if (i > 0)
        {
            this->_moveOneDegree();
            this->_handleMovement();
        }

        _currentSensorValue = this->_getSensorReading();
        _currentPosition = _eeprom->getPosition();
        _previousPosition = _currentPosition - _stringProxy->getStepsPerDeg();

        if (_currentSensorValue <= _previousSensorValue)
        {
            _previousSensorValue = _currentSensorValue;
        }
        else if (
            _currentSensorValue < HOME_SENSOR_THRESHOLD &&
            _currentSensorValue > _previousSensorValue)
        {
            break;
        }
    }

    _homePosition = _previousPosition;
    this->_moveToHome();

    _readingHomeValuesFinished = true;
}
