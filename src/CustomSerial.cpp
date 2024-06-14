#include "CustomSerial.h"

void CustomSerial::init(StringProxy &stringProxy)
{
    _stringProxy = &stringProxy;
}

void CustomSerial::serialEvent(SoftwareSerial &loopbackSerial)
{
    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '\n')
        {
            _serialCommandRawLength = _serialCommandRawIdx;
            _serialCommandRawIdx = 0;
            _commandParamLength = 0;

            memset(_command, 0, 5);
            memset(_commandParam, 0, 65);

            if (_serialCommandRawLength >= 2)
            {
                strncpy(_command, _serialCommandRaw, 2);
                _command[3] = 0;
            }

            if (_serialCommandRawLength > 3)
            {
                _commandParamLength = _serialCommandRawLength - 3;
                strncpy(_commandParam, _serialCommandRaw + 3, _commandParamLength);
                _commandParam[_commandParamLength] = 0;
            }

            String output = _stringProxy->processFalconCommand(_command, _commandParam, _commandParamLength);
            if (output.length() > 0)
            {
                Serial.println(output + TERMINATION_CHAR);
            }
        }
        else if (_serialCommandRawIdx < 70)
        {
            _serialCommandRaw[_serialCommandRawIdx] = c;
            _serialCommandRawIdx++;
        }
    }
}
