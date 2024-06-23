#include <Arduino.h>
#include "CustomEEPROM.h"
#include "Homing.h"
#include "Motor.h"
#include "StringProxy.h"
#include "CustomSerial.h"
#include <SoftwareSerial.h>
#include "EEPROM.h"

CustomEEPROM _eeprom;
Homing _homing;
Motor _motor;
StringProxy _stringProxy;
CustomSerial _serial;

SoftwareSerial loopbackSerial(4, 6); // RX, TX

int pm = 0;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    while (!Serial)
    {
        digitalWrite(LED_BUILTIN, (pm++ % 2) == 0 ? HIGH : LOW);
        delay(10);
    }
    Serial.begin(9600, SERIAL_8N1);
    loopbackSerial.begin(9600);
    loopbackSerial.println("Hello, world?");
    _eeprom.init();
    _motor.init(_eeprom);
    _stringProxy.init(_eeprom, _motor);
    _serial.init(_stringProxy);
    _homing.init(_eeprom, _motor, _stringProxy);
}

void loop()
{
    if (!_motor.isUartInitialized())
    {
        if (!_motor.init(_eeprom))
        {
            digitalWrite(LED_BUILTIN, (pm++ % 2) == 0 ? HIGH : LOW);
            _serial.serialEvent(loopbackSerial);
            delay(500);
            return;
        }
        else
        {
            digitalWrite(LED_BUILTIN, LOW);
        }
    }

    if (!_homing.isHomed())
    {
        if (_homing.isHoming())
        {
            return;
        }

        _homing.findHomePosition();
    }

    _motor.handleMotor();
    _serial.serialEvent(loopbackSerial);

    if (_motor.isMoving())
        return;

    _eeprom.handleEeprom();
}
