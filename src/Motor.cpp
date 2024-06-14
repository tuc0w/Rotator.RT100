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
    if (_motorIsMoving && _eeprom->getSettleBufferMs() > 0)
        _settleBufferPrevMs = millis();
    _lastMoveFinishedMs = millis();

    _motorManualIsMoving = false;
    _motorManualIsMovingContinuous = false;
    _motorIsMoving = false;
    _eeprom->setTargetPosition(_eeprom->getPosition());
}

void Motor::_applyStepMode()
{
    unsigned short sm = _eeprom->getStepMode();
    // select microsteps (0,2,4,8,16,32,64,128,256)
    _driver.microsteps(sm == 1 ? 0 : sm);
}

void Motor::_applyStepModeManual()
{
    unsigned short sm = _eeprom->getStepModeManual();
    // select microsteps (0,2,4,8,16,32,64,128,256)
    _driver.microsteps(sm == 1 ? 0 : sm);
}

void Motor::_applyMotorCurrent()
{
    _motorI = MOTOR_I;

    int mA = _motorI * (((float)_eeprom->getMotorIMoveMultiplier()) / 100.0);
    float multiplier = ((float)_eeprom->getMotorIHoldMultiplier()) / 100.0;

    _driver.rms_current(mA, multiplier * 0.8);
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

    _driver.begin();

    int testConnection;
    for (int i = 0; i < 5; i++)
    {
        testConnection = _driver.test_connection();
        if (testConnection == 0)
            break;
    }

    if (testConnection != 0)
        return false;

    _driver.pdn_disable(true); // enable UART
    _applyMotorCurrent();
    _driver.mstep_reg_select(true); // enable microstep selection over UART
    _driver.I_scale_analog(false);  // disable Vref scaling
    _applyStepMode();
    _driver.blank_time(24);                // Comparator blank time. This time needs to safely cover the switching event and the duration of the ringing on the sense resistor. Choose a setting of 24 or 32 for typical applications. For higher capacitive loads, 3 may be required. Lower settings allow stealthChop to regulate down to lower coil current values.
    _driver.toff(5);                       // enable stepper driver (For operation with stealthChop, this parameter is not used, but >0 is required to enable the motor)
    _driver.intpol(true);                  // use interpolation
    _driver.TPOWERDOWN(255);               // time until current reduction after the motor stops. Use maximum (5.6s)
    digitalWrite(TMC220X_PIN_ENABLE, LOW); // enable coils

    _uartInitialized = true;

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
                digitalWrite(TMC220X_PIN_DIR, _eeprom->getReverseDirection() ? HIGH : LOW);
                digitalWrite(TMC220X_PIN_STEP, HIGH);
                delayMicroseconds(1);
                digitalWrite(TMC220X_PIN_STEP, LOW);
                _eeprom->setPosition(_eeprom->getPosition() - 1);
                delayMicroseconds(_motorMoveDelay);
            }
            else if (_eeprom->getTargetPosition() > _eeprom->getPosition())
            {
                digitalWrite(TMC220X_PIN_DIR, _eeprom->getReverseDirection() ? LOW : HIGH);
                digitalWrite(TMC220X_PIN_STEP, HIGH);
                delayMicroseconds(1);
                digitalWrite(TMC220X_PIN_STEP, LOW);
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

void Motor::setMoveManual(bool motorManualIsMoving, bool motorManualIsMovingContinuous, bool motorManualIsMovingContinuousDir)
{
    _motorManualIsMoving = motorManualIsMoving;
    _motorManualIsMovingContinuous = motorManualIsMovingContinuous;
    _motorManualIsMovingContinuousDir = motorManualIsMovingContinuousDir;
}

bool Motor::getMotorManualIsMoving()
{
    return _motorManualIsMoving;
}

bool Motor::getMotorManualIsMovingContinuous()
{
    return _motorManualIsMovingContinuous;
}

bool Motor::getMotorManualIsMovingContinuousDir()
{
    return _motorManualIsMovingContinuousDir;
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

bool Motor::isMovingWithSettle()
{
    if (_motorIsMoving)
    {
        return true;
    }
    else
    {
        /*
            if your focuser has any play, this can affect the autofocuser performance. SGP for example goes aways from motorDir position and
            than starts traversing back. When it changes focus direction (traverse back), focuser play can cause FOV to wiggle just a bit,
            which causes enlongated stars on the next exposure. Settle buffer option returns IsMoving as TRUE after focuser reaches target position,
            letting it to settle a bit. Advices by Jared Wellman of SGP.
        */
        if (_eeprom->getSettleBufferMs() > 0L && _settleBufferPrevMs != 0L)
        {
            long settleBufferCurrenMs = millis();
            if ((settleBufferCurrenMs - _settleBufferPrevMs) < (unsigned long)_eeprom->getSettleBufferMs())
            {
                return true;
            }
            else
            {
                _settleBufferPrevMs = 0L;

                return true;
            }
        }
        else
        {
            return false;
        }
    }
}

void Motor::debug()
{
    Serial.print("Motor full current (mA): ");
    Serial.println(_motorI);
    Serial.print("TMC DRV_STATUS: ");
    Serial.println(_driver.DRV_STATUS());
    Serial.print("TMC CRC error: ");
    Serial.println(_driver.CRCerror);
}

void Motor::legacyTest()
{
    Serial.println("LEGACY MODE - START");
    digitalWrite(TMC220X_PIN_ENABLE, LOW);
    Serial.println("ENABLE LOW");

    Serial.println("MOVING STARTED");
    for (int i = 0; i < 500; i++)
    {
        digitalWrite(TMC220X_PIN_DIR, LOW);
        digitalWrite(TMC220X_PIN_STEP, HIGH);
        delayMicroseconds(1);
        digitalWrite(TMC220X_PIN_STEP, LOW);
        delayMicroseconds(8000);
    }
    Serial.println("MOVING FINISHED");

    digitalWrite(TMC220X_PIN_ENABLE, HIGH);
    Serial.println("ENABLE HIGH");

    Serial.println("LEGACY MODE - END");
}
