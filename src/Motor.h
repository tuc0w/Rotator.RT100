#include <CheapStepper.h>
#include <TMCStepper.h>
#include "CustomEEPROM.h"

#pragma once
#define TMC220X_PIN_ENABLE 9
#define TMC220X_PIN_DIR 2
#define TMC220X_PIN_STEP 3
#define TMC220X_PIN_MS2 7
#define TMC220X_PIN_MS1 8
#define TMC220X_PIN_UART_RX 11
#define TMC220X_PIN_UART_TX 12

#define ULN2003_PIN_IN1 8
#define ULN2003_PIN_IN2 9
#define ULN2003_PIN_IN3 10
#define ULN2003_PIN_IN4 11
#define ULN2003_STEPS_PER_REVOLUTION 4096

#define MOTOR_DRIVER "ULN2003"

#define MOTOR_I 500

class Motor
{
private:
    CustomEEPROM *_eeprom;
    bool _pinsInitialized = false;
    bool _uartInitialized = false;
    long _motorI;
    bool _motorIsMoving;
    unsigned long _debouncingLastRunMs = 0L;
    unsigned long _lastMoveFinishedMs = 0L;
    long _motorMoveDelay;
    TMC2208Stepper _tmcDriver = TMC2208Stepper(TMC220X_PIN_UART_RX, TMC220X_PIN_UART_TX, 0.11);
    CheapStepper _ulnDriver = CheapStepper(ULN2003_PIN_IN1, ULN2003_PIN_IN2, ULN2003_PIN_IN3, ULN2003_PIN_IN4);
    void _startMotor();
    void _stopMotor();
    void _applyStepMode();
    void _applyStepModeManual();
    void _applyMotorCurrent();

public:
    bool init(CustomEEPROM &eeprom);
    bool isUartInitialized();
    bool handleMotor();
    void startMotor();
    void stopMotor();
    void applyStepMode();
    void applyStepModeManual();
    void applyMotorCurrent();
    long getLastMoveFinishedMs();
    bool isMoving();
};
