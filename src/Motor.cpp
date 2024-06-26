#include <Arduino.h>
#include "Motor.h"

void Motor::_startMotor()
{
    _motorIsMoving = true;
    _debouncingLastRunMs = millis();

    switch (_eeprom->getSpeedMode())
    {
    case 1:
        _motorMoveDelay = 96000;
        break;
    case 2:
        _motorMoveDelay = 48000;
        break;
    case 3:
        _motorMoveDelay = 24000;
        break;
    case 4:
        _motorMoveDelay = 8000;
        break;
    case 5:
        _motorMoveDelay = 4000;
        break;
    }

    _motorMoveDelay /= _eeprom->getStepMode();
}

void Motor::_stopMotor()
{
    _lastMoveFinishedMs = millis();
    _motorIsMoving = false;
    _eeprom->setTargetPosition(_eeprom->getPosition());
}

void Motor::_applyStepMode()
{
    unsigned short sm = _eeprom->getStepMode();
    // select microsteps (0,2,4,8,16,32,64,128,256)
    _tmcDriver.microsteps(sm == 1 ? 0 : sm);
}

void Motor::_applyStepModeManual()
{
    unsigned short sm = _eeprom->getStepModeManual();
    // select microsteps (0,2,4,8,16,32,64,128,256)
    _tmcDriver.microsteps(sm == 1 ? 0 : sm);
}

void Motor::_applyMotorCurrent()
{
    _motorI = MOTOR_I;

    int mA = _motorI * (((float)_eeprom->getMotorIMoveMultiplier()) / 100.0);
    float multiplier = ((float)_eeprom->getMotorIHoldMultiplier()) / 100.0;

    _tmcDriver.rms_current(mA, multiplier * 0.8);
}

bool Motor::init(CustomEEPROM &eeprom)
{
    _eeprom = &eeprom;

    if (!_pinsInitialized)
    {
        pinMode(TMC220X_PIN_DIR, OUTPUT);
        pinMode(TMC220X_PIN_STEP, OUTPUT);
        pinMode(TMC220X_PIN_MS1, OUTPUT);
        pinMode(TMC220X_PIN_MS2, OUTPUT);
        pinMode(TMC220X_PIN_ENABLE, OUTPUT);

        digitalWrite(TMC220X_PIN_DIR, LOW);
        digitalWrite(TMC220X_PIN_STEP, LOW);
        digitalWrite(TMC220X_PIN_ENABLE, HIGH);
        _pinsInitialized = true;
    }

    if (MOTOR_DRIVER == "TMC220X")
    {
        _tmcDriver.begin();

        int testConnection;
        for (int i = 0; i < 5; i++)
        {
            testConnection = _tmcDriver.test_connection();
            if (testConnection == 0)
                break;
        }

        if (testConnection != 0)
            return false;

        _tmcDriver.pdn_disable(true); // enable UART
        _applyMotorCurrent();
        _tmcDriver.mstep_reg_select(true); // enable microstep selection over UART
        _tmcDriver.I_scale_analog(false);  // disable Vref scaling
        _applyStepMode();
        _tmcDriver.blank_time(24);             // Comparator blank time. This time needs to safely cover the switching event and the duration of the ringing on the sense resistor. Choose a setting of 24 or 32 for typical applications. For higher capacitive loads, 3 may be required. Lower settings allow stealthChop to regulate down to lower coil current values.
        _tmcDriver.toff(5);                    // enable stepper driver (For operation with stealthChop, this parameter is not used, but >0 is required to enable the motor)
        _tmcDriver.intpol(true);               // use interpolation
        _tmcDriver.TPOWERDOWN(255);            // time until current reduction after the motor stops. Use maximum (5.6s)
        digitalWrite(TMC220X_PIN_ENABLE, LOW); // enable coils

        _uartInitialized = true;
    }
    else if (MOTOR_DRIVER == "ULN2003")
    {
        /**
         * accepted RPM range: 6RPM (may overheat) - 24RPM (may skip)
         * ideal range: 10RPM (safe, high torque) - 22RPM (fast, low torque)
         */
        _ulnDriver.setRpm(10);
        _uartInitialized = true;
    }

    return true;
}

bool Motor::isUartInitialized()
{
    return _uartInitialized;
}

bool Motor::handleMotor()
{
    if (_motorIsMoving)
    {
        // give priority to motor with dedicated 50ms loops (effectivly pausing main loop, including serial event processing)
        while (millis() - _debouncingLastRunMs < 50)
        {
            if (_motorManualIsMovingContinuous)
            {
                if (_motorManualIsMovingContinuousDir)
                {
                    _eeprom->setTargetPosition(_eeprom->getPosition() + 1);
                }
                else
                {
                    _eeprom->setTargetPosition(_eeprom->getPosition() - 1);
                }
            }

            if (_eeprom->getTargetPosition() < _eeprom->getPosition())
            {
                if (MOTOR_DRIVER == "TMC220X")
                {
                    digitalWrite(TMC220X_PIN_DIR, _eeprom->getReverseDirection() ? HIGH : LOW);
                    digitalWrite(TMC220X_PIN_STEP, HIGH);
                    delayMicroseconds(1);
                    digitalWrite(TMC220X_PIN_STEP, LOW);
                }
                else if (MOTOR_DRIVER == "ULN2003")
                {
                    _ulnDriver.stepCCW();
                }

                _eeprom->setPosition(_eeprom->getPosition() - 1);
                delayMicroseconds(_motorMoveDelay);
            }
            else if (_eeprom->getTargetPosition() > _eeprom->getPosition())
            {
                if (MOTOR_DRIVER == "TMC220X")
                {
                    digitalWrite(TMC220X_PIN_DIR, _eeprom->getReverseDirection() ? LOW : HIGH);
                    digitalWrite(TMC220X_PIN_STEP, HIGH);
                    delayMicroseconds(1);
                    digitalWrite(TMC220X_PIN_STEP, LOW);
                }
                else if (MOTOR_DRIVER == "ULN2003")
                {
                    _ulnDriver.stepCW();
                }

                _eeprom->setPosition(_eeprom->getPosition() + 1);
                delayMicroseconds(_motorMoveDelay);
            }
            else
            {
                _stopMotor();
            }
        }

        _debouncingLastRunMs = millis();
    }

    return _motorIsMoving;
}

void Motor::startMotor()
{
    _startMotor();
}

void Motor::stopMotor()
{
    _stopMotor();
}

void Motor::applyStepMode()
{
    _applyStepMode();
}

void Motor::applyStepModeManual()
{
    _applyStepModeManual();
}

void Motor::applyMotorCurrent()
{
    _applyMotorCurrent();
}

long Motor::getLastMoveFinishedMs()
{
    long ms = _lastMoveFinishedMs;
    _lastMoveFinishedMs = 0L;

    return ms;
}

bool Motor::isMoving()
{
    return _motorIsMoving;
}
