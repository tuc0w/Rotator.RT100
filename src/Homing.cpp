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

    _eeprom->handleEeprom();
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
    _motor->startMotor();

    do
    {
        _motor->handleMotor();
    } while (_motor->isMoving());

    _eeprom->setPosition(0);
    _eeprom->setTargetPosition(0);
    _eeprom->handleEeprom();

    _homePosition = 0;
    _isHomed = true;
    _isHoming = false;
}

bool Homing::init(CustomEEPROM &eeprom, Motor &motor, StringProxy &stringProxy)
{
    _eeprom = &eeprom;
    _motor = &motor;
    _stringProxy = &stringProxy;

    ArrayList<int> _homeSensorList(ArrayList<int>::FIXED, 360);

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
    _eeprom->setPosition(0);
    _eeprom->handleEeprom();

    for (int i = 0; i <= 360; i++)
    {
        if (i > 0)
        {
            this->_moveOneDegree();
            do
            {
                _motor->handleMotor();
            } while (_motor->isMoving());

            _eeprom->handleEeprom();
        }

        int hallReading = this->_getSensorReading();
        this->_homeSensorList.insert(i, hallReading);
    }

    int angleOfLowestValue = 1;
    int lowestValue = 9999;
    for (int i = 0; i <= 360; i++)
    {
        if (this->_homeSensorList.get(i) <= lowestValue)
        {
            lowestValue = this->_homeSensorList.get(i);
            angleOfLowestValue = i;
        }
    }

    _homePosition = _stringProxy->degToSteps(angleOfLowestValue);
    this->_moveToHome();

    _readingHomeValuesFinished = true;
}
