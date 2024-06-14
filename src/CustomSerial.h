#include "StringProxy.h"

#pragma once

#define TERMINATION_CHAR ';'

enum CmdType
{
    INVALID,
    AF3,
    FALCON
};

class CustomSerial
{
private:
    StringProxy *_stringProxy;
    char _serialCommandRaw[70];
    CmdType _cmdType;
    int _serialCommandRawIdx;
    int _serialCommandRawLength;
    char _command[5];
    char _commandParam[65];
    int _commandParamLength;

public:
    void init(StringProxy &stringProxy);
    void serialEvent(SoftwareSerial &loopbackSerial);
};