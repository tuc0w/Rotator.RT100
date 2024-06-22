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

#define MOTOR_I 500

class Motor
{
private:
    CustomEEPROM *_eeprom;
    bool _pinsInitialized = false;
    bool _uartInitialized = false;
    long _motorI;
    bool _motorIsMoving;
    bool _motorManualIsMoving;
    bool _motorManualIsMovingContinuous;
    bool _motorManualIsMovingContinuousDir;
    unsigned long _settleBufferPrevMs;
    unsigned long _debouncingLastRunMs = 0L;
    unsigned long _lastMoveFinishedMs = 0L;
    long _motorMoveDelay;
    TMC2208Stepper _driver = TMC2208Stepper(TMC220X_PIN_UART_RX, TMC220X_PIN_UART_TX, 0.11);
    void _startMotor();
    void _stopMotor();
    void _applyStepMode();
    void _applyStepModeManual();
    void _applyMotorCurrent();

public:
    bool init(CustomEEPROM &eeprom);
    bool isUartInitialized();
    bool handleMotor();
    void setMoveManual(bool motorManualIsMoving, bool motorManualIsMovingContinuous, bool motorManualIsMovingContinuousDir);
    bool getMotorManualIsMoving();
    bool getMotorManualIsMovingContinuous();
    bool getMotorManualIsMovingContinuousDir();
    void startMotor();
    void stopMotor();
    void applyStepMode();
    void applyStepModeManual();
    void applyMotorCurrent();
    long getLastMoveFinishedMs();
    bool isMoving();
    bool isMovingWithSettle();
    void debug();
    void legacyTest();
};
