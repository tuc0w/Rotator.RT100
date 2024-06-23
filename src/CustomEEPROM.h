#if defined(ARDUINO_AVR_NANO_EVERY)
#define EEPROM_SIZE 256
#else
#define EEPROM_SIZE 1024
#endif
#define EEPROM_CHECK_PERIOD_MS 5000

#pragma once

class EEPROMState
{
public:
  unsigned long maxPosition;
  unsigned long maxMovement;
  unsigned short stepMode;
  unsigned short stepModeManual;
  unsigned char speedMode;
  unsigned long settleBufferMs;
  unsigned long idleEepromWriteMs;
  bool reverseDirection;
  unsigned char motorIMoveMultiplier;
  unsigned char motorIHoldMultiplier;
  unsigned long position;
  unsigned long targetPosition;
  unsigned long checksum;
};

class CustomEEPROM
{
private:
  EEPROMState _state = {0, 0, 16, 1, 4, 0, 0, 0, 0, 0, 0, 0, 9999};
  EEPROMState _stateDefaults = {1000000, 5000000, 16, 2, 4, 0, 180000, 0, 90, 40, 0, 0, 0};
  int _slidingSize = sizeof(_state.position) + sizeof(_state.targetPosition) + sizeof(_state.checksum);
  int _configurationSize = sizeof(EEPROMState) - _slidingSize;
  int _slidingAddressCount = EEPROM_SIZE / _slidingSize;
  int _slidingCurrentAddress = _configurationSize;
  bool _isConfigDirty;
  bool _isHoming;
  unsigned long _lastEepromCheckMs;
  unsigned long _lastPositionChangeMs = 0L;

  unsigned long _calculateChecksum(EEPROMState state);
  void _readEeprom();
  void _writeEeprom(bool isReset);
  void _resetEeprom();

public:
  void init();
  void handleEeprom();
  void resetToDefaults();
  void debug();

  bool isHoming();
  void setHoming(bool value);
  unsigned long getPosition();
  void setPosition(unsigned long value);
  void syncPosition(unsigned long value);
  unsigned long getTargetPosition();
  bool setTargetPosition(unsigned long value);
  unsigned long getMaxPosition();
  void setMaxPosition(unsigned long value);
  unsigned long getMaxMovement();
  void setMaxMovement(unsigned long value);
  unsigned short getStepMode();
  bool setStepMode(unsigned short value);
  unsigned short getStepModeManual();
  bool setStepModeManual(unsigned short value);
  unsigned char getSpeedMode();
  bool setSpeedMode(unsigned char value);
  unsigned long getSettleBufferMs();
  void setSettleBufferMs(unsigned long value);
  bool getReverseDirection();
  void setReverseDirection(bool value);
  unsigned long getIdleEepromWriteMs();
  void setIdleEepromWriteMs(unsigned long value);
  unsigned char getMotorIMoveMultiplier();
  void setMotorIMoveMultiplier(unsigned char value);
  unsigned char getMotorIHoldMultiplier();
  void setMotorIHoldMultiplier(unsigned char value);
  unsigned long getChecksum();
  void setChecksum(unsigned long value);
};