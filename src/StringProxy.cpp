#include "StringProxy.h"

char const *StringProxy::_formatResponse(float value)
{
    dtostrf(value, 1, 2, _resultBuffer1);
    sprintf(_resultBuffer2, "(%s)", _resultBuffer1);

    return _resultBuffer2;
}

char const *StringProxy::_formatResponse(unsigned long value)
{
    sprintf(_resultBuffer1, "(%lu)", value);

    return _resultBuffer1;
}

char const *StringProxy::_formatResponse(unsigned short value)
{
    sprintf(_resultBuffer1, "(%hu)", value);

    return _resultBuffer1;
}

void StringProxy::init(CustomEEPROM &eeprom, Motor &motor)
{
    _eeprom = &eeprom;
    _motor = &motor;
}

float StringProxy::getStepsPerDeg()
{
    double stepsPerDeg = 400.0f * _eeprom->getStepMode(); // steps per 360 of motor shaft
    stepsPerDeg *= (100.0f / 20.0f);                      // steps per 360 deg of rotator
    stepsPerDeg /= 360.0f;                                // steps per deg

    return stepsPerDeg;
}

float StringProxy::stepsToDeg(unsigned long steps)
{
    float stepsPerDeg = this->getStepsPerDeg();

    float deg = steps / stepsPerDeg;

    deg -= 90.0f; // shift by 90deg

    while (deg < 0.0f)
    {
        deg += 360.0f;
    }
    while (deg >= 360.0f)
    {
        deg -= 360.0f;
    }

    return deg;
}

unsigned long StringProxy::degToSteps(float deg)
{
    float stepsPerDeg = this->getStepsPerDeg();

    deg += 90.0f; // shift by 90deg

    while (deg < 0.0f)
    {
        deg += 360.0f;
    }
    while (deg >= 360.0f)
    {
        deg -= 360.0f;
    }

    unsigned long steps = deg * stepsPerDeg;

    return steps;
}

char const *StringProxy::processFalconCommand(char *command, char *commandParam, int commandParamLength)
{
    float deg;
    unsigned long maxSteps;
    unsigned long steps;

    if (command[0] == 'F')
    {
        switch (command[1])
        {
        case '#': // status
            return "FR_OK";

        case 'S':
            dtostrf(this->getStepsPerDeg(), 1, 2, _resultBuffer2);
            sprintf(_resultBuffer1, "FS:%s", _resultBuffer2);

            return _resultBuffer1;

        case 'A': // full status
            /*
            Receive: FR_OK:43:32:50.00:0:0:0:0
            status FR_OK means that focuser is up and running
            position_in_deg Position in degrees (double number, 2 decimals)
            is_running Boolean value: Prints 1 if falcon motor is running, 0 if not
            limit_detect Boolean value: Prints 1 if limit is detected, Print 0 if limit is not detected
            do_derotation Boolean value: Print 1 if derotation is active, Print 0 if is deactivated
            motor_reverse Boolean value: Print 1 if reverse is enabled, 0 if is disabled
            */

            dtostrf(this->stepsToDeg(_eeprom->getPosition()), 1, 2, _resultBuffer2);
            sprintf(
                _resultBuffer1,
                "FR_OK:%lu:%s:%c:0:0:%c",
                _eeprom->getPosition(),
                _resultBuffer2,
                _motor->isMoving() ? '1' : '0',
                _eeprom->getReverseDirection() ? '1' : '0');

            return _resultBuffer1;

        case 'V': // Report firmware version - FV:n.n
            return "FV:1.3";

        case 'D': // Report position in degrees - FD:nn.nn
            dtostrf(this->stepsToDeg(_eeprom->getPosition()), 1, 2, _resultBuffer2);
            sprintf(_resultBuffer1, "FD:%s", _resultBuffer2);

            return _resultBuffer1;

        case 'P': // Report position in steps - FP:n..
            sprintf(_resultBuffer1, "FP:%lu", _eeprom->getPosition());

            return _resultBuffer1;

        case 'H': // Halt Falcon Rotator FH:1
            _motor->stopMotor();
            return "FH:1";

        case 'R': // Print 1 if rotator is running, Print 0 if rotator is idle - FR:1 or FR:0
            sprintf(_resultBuffer1, "FR:%c", _motor->isMoving() ? '1' : '0');

            return _resultBuffer1;

        case 'N': // Reverse Motor (1 = reverse, 0 = normal), One off setting – stored in EEPROM - FN:1 or FN:0
            _eeprom->setReverseDirection(atoi(commandParam));
            sprintf(_resultBuffer1, "FN:%c", _eeprom->getReverseDirection() ? '1' : '0');

            return _resultBuffer1;

        case 'F': // Reload Rotator Firmware
            return "FR_OK";
        }
    }
    else if (command[0] == 'V' && command[1] == 'S')
    { // Report input voltage in raw format - VS:n..
        return "VS:12.00";
    }
    else if (command[0] == 'D' && command[1] == 'R')
    { // Enable Derotation. Provided number is the derotation time (in millisec) interval per step e.g (1 step per 1000 millisec) (DR:0 disables derotation) - DR:nn..
        return "DR:0";
    }
    else if (command[0] == 'S' && command[1] == 'D')
    { // Set Degrees: Set New position in degrees as the actual rotator position (without turning rotator) - SD:nn.nn
        deg = atof(commandParam);

        steps = this->degToSteps(deg);
        maxSteps = (this->getStepsPerDeg() * 360.0f);

        _eeprom->syncPosition(steps);
        _eeprom->setMaxPosition(maxSteps);
        _eeprom->setMaxMovement(maxSteps);

        dtostrf(this->stepsToDeg(steps), 1, 2, _resultBuffer2);
        sprintf(_resultBuffer1, "SD:%s", _resultBuffer2);

        return _resultBuffer1;
    }
    else if (command[0] == 'M' && command[1] == 'D')
    { // Move to Degrees: Move motor to new degrees. (accepts a decimal number e.g 33.55) - MD:nn.nn
        deg = atof(commandParam);

        steps = this->degToSteps(deg);

        _eeprom->setTargetPosition(steps);
        _motor->applyStepMode();
        _motor->startMotor();

        dtostrf(this->stepsToDeg(steps), 1, 2, _resultBuffer2);
        sprintf(_resultBuffer1, "MD:%s", _resultBuffer2);

        return _resultBuffer1;
    }
    else if (command[0] == 'M' && command[1] == 'S')
    { // Move to Position: Move motor to new position MS:nn..  - MS:nn..
        steps = strtoul(commandParam, NULL, 10);
        _eeprom->setTargetPosition(steps);
        _motor->applyStepMode();
        _motor->startMotor();

        sprintf(_resultBuffer1, "MS:%lu", _eeprom->getTargetPosition());

        return _resultBuffer1;
    }
    else if (command[0] == 'G' && command[1] == 'S')
    {
        sprintf(_resultBuffer1, "GS:%u", _eeprom->getStepMode());

        return _resultBuffer1;
    }
    else if (command[0] == 'S' && command[1] == 'S')
    {
        long sm = (unsigned short)strtoul(commandParam, NULL, 10);
        if (_eeprom->setStepMode(sm))
        {
            return RESPONSE_OK;
        }
        else
        {
            return RESPONSE_KO;
        }
    }
    else if (command[0] == 'G' && command[1] == 'G')
    {
        sprintf(_resultBuffer1, "SG:%u", _eeprom->getSpeedMode());

        return _resultBuffer1;
    }
    else if (command[0] == 'S' && command[1] == 'G')
    {
        long sm = (unsigned short)commandParam;
        if (_eeprom->setSpeedMode(sm))
        {
            return RESPONSE_OK;
        }
        else
        {
            return RESPONSE_KO;
        }
    }
    else if (command[0] == 'S' && command[1] == 'D')
    {
        dtostrf(this->getStepsPerDeg(), 1, 2, _resultBuffer2);
        sprintf(_resultBuffer1, "SD:%s", _resultBuffer2);

        return _resultBuffer1;
    }
    else if (command[0] == 'R' && command[1] == 'S')
    {
        _eeprom->resetToDefaults();

        return RESPONSE_OK;
    }

    return "";
}
