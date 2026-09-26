/*
  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2010-2011 Paul Stoffregen.  All rights reserved.
  Copyright (C) 2009 Shigeru Kobayashi.  All rights reserved.
  Copyright (C) 2013 Norbert Truchsess. All rights reserved.
  Copyright (C) 2009-2017 Jeff Hoefs.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.

  Last updated: September 16th, 2017
*/

#include <ConfigurableFirmata.h>
#include "AccelStepperFirmata.h"
#include "utility/AccelStepper.h"
#include "utility/MultiStepper.h"

boolean AccelStepperFirmata::handlePinMode(byte pin, int mode)
{
  if (mode == PIN_MODE_STEPPER) {
    if (IS_PIN_DIGITAL(pin)) {
      pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
      return true;
    }
  }
  return false;
}

void AccelStepperFirmata::handleCapability(byte pin)
{
  if (IS_PIN_DIGITAL(pin)) {
    Firmata.write(PIN_MODE_STEPPER);
    Firmata.write(21); //21 bits used for number of steps
  }
}

// Send position data when it's requested or a move completes
void AccelStepperFirmata::reportPosition(byte deviceNum, bool complete)
{
  if (deviceNum < MAX_ACCELSTEPPERS && stepper[deviceNum]) {
    byte data[5];
    long position = stepper[deviceNum]->currentPosition();
    encode32BitSignedInteger(position, data);

    Firmata.write(START_SYSEX);
    Firmata.write(ACCELSTEPPER_DATA);
    if (complete) {
      Firmata.write(ACCELSTEPPER_MOVE_COMPLETE);
    } else {
      Firmata.write(ACCELSTEPPER_REPORT_POSITION);
    }
    Firmata.write(deviceNum);
    Firmata.write(data[0]);
    Firmata.write(data[1]);
    Firmata.write(data[2]);
    Firmata.write(data[3]);
    Firmata.write(data[4]);
    Firmata.write(END_SYSEX);
  }
}

void AccelStepperFirmata::reportGroupComplete(byte deviceNum)
{
  if (deviceNum < MAX_GROUPS && group[deviceNum]) {
    Firmata.write(START_SYSEX);
    Firmata.write(ACCELSTEPPER_DATA);
    Firmata.write(MULTISTEPPER_MOVE_COMPLETE);
    Firmata.write(deviceNum);
    Firmata.write(END_SYSEX);
  }
}

/*==============================================================================
 * SYSEX-BASED commands
 *============================================================================*/

boolean AccelStepperFirmata::handleSysex(byte command, byte argc, byte *argv)
{
  if (command != ACCELSTEPPER_DATA) {
    return false;
  }

  if (argc < 2) {
    Firmata.sendString(F("ACCELSTEPPER_DATA: message too short"));
    return false;
  }

  const byte stepCommand = argv[0];
  const byte id = argv[1];

  switch (stepCommand) {
    case ACCELSTEPPER_CONFIG: {
      if (id >= MAX_ACCELSTEPPERS || argc < 5) {
        Firmata.sendString(F("ACCELSTEPPER_CONFIG: invalid message"));
        return false;
      }

      const byte interface = argv[2];
      const byte wireCount = (interface & 0x70) >> 4;
      const byte stepType = (interface & 0x0e) >> 1;
      const bool supportedInterface = wireCount == 1 || wireCount == 2 ||
          ((wireCount == 3 || wireCount == 4) &&
           (stepType == STEP_TYPE_WHOLE || stepType == STEP_TYPE_HALF));

      if (!supportedInterface) {
        Firmata.sendString(F("ACCELSTEPPER_CONFIG: unsupported interface"));
        return false;
      }

      byte requiredArgc = 5;
      if (wireCount >= 3) requiredArgc++;
      if (wireCount >= 4) requiredArgc++;
      if (interface & 0x01) requiredArgc++;

      if (argc != requiredArgc && argc != requiredArgc + 1) {
        Firmata.sendString(F("ACCELSTEPPER_CONFIG: invalid length"));
        return false;
      }

      if (stepper[id]) {
        Firmata.sendString(F("ACCELSTEPPER_CONFIG: device already configured"));
        return false;
      }

      const byte stepOrMotorPin1 = argv[3];
      const byte directionOrMotorPin2 = argv[4];
      const byte motorPin3 = wireCount >= 3 ? argv[5] : 0;
      const byte motorPin4 = wireCount >= 4 ? argv[6] : 0;
      const byte enablePin = (interface & 0x01) ? argv[requiredArgc - 1] : 0;
      const byte motorPins[] = {stepOrMotorPin1, directionOrMotorPin2, motorPin3, motorPin4};
      const byte motorPinCount = wireCount == 1 ? 2 : wireCount;

      // All pin modes must be valid before any pin state changes.
      for (byte i = 0; i < motorPinCount; i++) {
        const byte pin = motorPins[i];
        if (pin >= TOTAL_PINS || Firmata.getPinMode(pin) == PIN_MODE_IGNORE) {
          Firmata.sendString(F("ACCELSTEPPER_CONFIG: invalid pin"));
          return false;
        }
      }
      if ((interface & 0x01) &&
          (enablePin >= TOTAL_PINS || Firmata.getPinMode(enablePin) == PIN_MODE_IGNORE)) {
        Firmata.sendString(F("ACCELSTEPPER_CONFIG: invalid pin"));
        return false;
      }

      Firmata.setPinMode(stepOrMotorPin1, PIN_MODE_STEPPER);
      Firmata.setPinMode(directionOrMotorPin2, PIN_MODE_STEPPER);
      if (wireCount >= 3) Firmata.setPinMode(motorPin3, PIN_MODE_STEPPER);
      if (wireCount >= 4) Firmata.setPinMode(motorPin4, PIN_MODE_STEPPER);

      if (wireCount == 1) {
        stepper[id] = new AccelStepper(AccelStepper::DRIVER, stepOrMotorPin1, directionOrMotorPin2);
      } else if (wireCount == 2) {
        stepper[id] = new AccelStepper(AccelStepper::FULL2WIRE, stepOrMotorPin1, directionOrMotorPin2);
      } else if (wireCount == 3 && stepType == STEP_TYPE_WHOLE) {
        stepper[id] = new AccelStepper(AccelStepper::FULL3WIRE, stepOrMotorPin1, directionOrMotorPin2, motorPin3);
      } else if (wireCount == 3) {
        stepper[id] = new AccelStepper(AccelStepper::HALF3WIRE, stepOrMotorPin1, directionOrMotorPin2, motorPin3);
      } else if (stepType == STEP_TYPE_WHOLE) {
        stepper[id] = new AccelStepper(AccelStepper::FULL4WIRE, stepOrMotorPin1, directionOrMotorPin2, motorPin3, motorPin4, false);
      } else {
        stepper[id] = new AccelStepper(AccelStepper::HALF4WIRE, stepOrMotorPin1, directionOrMotorPin2, motorPin3, motorPin4, false);
      }
      if (!stepper[id]) {
        Firmata.sendString(F("ACCELSTEPPER_CONFIG: allocation failed"));
        return false;
      }

      // Also initializes AccelStepper's enable inversion when the optional byte is absent.
      const byte invertPins = argc == requiredArgc + 1 ? argv[requiredArgc] : 0;
      if (wireCount == 1) {
        stepper[id]->setPinsInverted(invertPins & 0x01, invertPins >> 1 & 0x01, invertPins >> 4 & 0x01);
      } else {
        stepper[id]->setPinsInverted(invertPins & 0x01, invertPins >> 1 & 0x01, invertPins >> 2 & 0x01, invertPins >> 3 & 0x01, invertPins >> 4 & 0x01);
      }

      if (interface & 0x01) {
        stepper[id]->setEnablePin(enablePin);
      }

      // Reach the requested speed on the first step when acceleration is omitted.
      stepper[id]->setMaxSpeed(2.0);
      stepper[id]->setMaxSpeed(1.0);
      stepper[id]->setAcceleration(MAX_ACCELERATION);
      numSteppers++;
      isRunning[id] = false;
      return true;
    }

    case ACCELSTEPPER_ZERO:
    case ACCELSTEPPER_STOP:
    case ACCELSTEPPER_REPORT_POSITION:
      if (id >= MAX_ACCELSTEPPERS || argc != 2) {
        Firmata.sendString(F("ACCELSTEPPER_DATA: invalid message"));
        return false;
      }
      if (stepper[id]) {
        if (stepCommand == ACCELSTEPPER_ZERO) {
          stepper[id]->setCurrentPosition(0);
        } else if (stepCommand == ACCELSTEPPER_STOP) {
          stepper[id]->stop();
          isRunning[id] = false;
          reportPosition(id, true);
        } else {
          reportPosition(id, false);
        }
      }
      return true;

    case ACCELSTEPPER_STEP:
    case ACCELSTEPPER_TO:
      if (id >= MAX_ACCELSTEPPERS || argc != 7) {
        Firmata.sendString(F("ACCELSTEPPER_DATA: invalid message"));
        return false;
      }
      if (stepper[id]) {
        const long position = decode32BitSignedInteger(argv[2], argv[3], argv[4], argv[5], argv[6]);
        if (stepCommand == ACCELSTEPPER_STEP) {
          stepper[id]->move(position);
        } else {
          stepper[id]->moveTo(position);
        }
        isRunning[id] = true;
      }
      return true;

    case ACCELSTEPPER_ENABLE:
      if (id >= MAX_ACCELSTEPPERS || argc != 3) {
        Firmata.sendString(F("ACCELSTEPPER_ENABLE: invalid message"));
        return false;
      }
      if (stepper[id]) {
        if (argv[2] == 0x00) {
          stepper[id]->disableOutputs();
        } else {
          stepper[id]->enableOutputs();
        }
      }
      return true;

    case ACCELSTEPPER_SET_ACCELERATION:
    case ACCELSTEPPER_SET_SPEED:
      if (id >= MAX_ACCELSTEPPERS || argc != 6) {
        Firmata.sendString(F("ACCELSTEPPER_DATA: invalid message"));
        return false;
      }
      if (stepper[id]) {
        const float value = decodeCustomFloat(argv[2], argv[3], argv[4], argv[5]);
        if (stepCommand == ACCELSTEPPER_SET_ACCELERATION) {
          stepper[id]->setAcceleration(value == 0.0 ? MAX_ACCELERATION : value);
        } else {
          stepper[id]->setMaxSpeed(value);
        }
      }
      return true;

    case MULTISTEPPER_CONFIG: {
      const byte groupId = id;
      if (groupId >= MAX_GROUPS || argc < 3) {
        Firmata.sendString(F("MULTISTEPPER_CONFIG: invalid message"));
        return false;
      }

      const byte requested = argc - 2;
      if ((unsigned int)groupStepperCount[groupId] + requested > MULTISTEPPER_MAX_STEPPERS) {
        Firmata.sendString(F("MULTISTEPPER_CONFIG: too many steppers"));
        return false;
      }

      for (byte i = 2; i < argc; i++) {
        const byte stepperNumber = argv[i];
        if (stepperNumber >= MAX_ACCELSTEPPERS || !stepper[stepperNumber]) {
          Firmata.sendString(F("MULTISTEPPER_CONFIG: invalid stepper"));
          return false;
        }
      }

      if (!group[groupId]) {
        group[groupId] = new MultiStepper();
        if (!group[groupId]) {
          Firmata.sendString(F("MULTISTEPPER_CONFIG: allocation failed"));
          return false;
        }
        numGroups++;
      }

      for (byte i = 2; i < argc; i++) {
        if (!group[groupId]->addStepper(*stepper[argv[i]])) {
          Firmata.sendString(F("MULTISTEPPER_CONFIG: group is full"));
          return false;
        }
        groupStepperCount[groupId]++;
      }

      groupIsRunning[groupId] = false;
      return true;
    }

    case MULTISTEPPER_TO: {
      const byte groupId = id;
      if (groupId >= MAX_GROUPS || !group[groupId] || groupStepperCount[groupId] == 0) {
        Firmata.sendString(F("MULTISTEPPER_TO: group not configured"));
        return false;
      }

      const byte count = groupStepperCount[groupId];
      const unsigned int expectedArgc = 2u + (unsigned int)count * 5u;
      if (argc != expectedArgc) {
        Firmata.sendString(F("MULTISTEPPER_TO: invalid target count"));
        return false;
      }

      long positions[MULTISTEPPER_MAX_STEPPERS];
      for (byte i = 0; i < count; i++) {
        const unsigned int offset = 2u + (unsigned int)i * 5u;
        positions[i] = decode32BitSignedInteger(argv[offset], argv[offset + 1], argv[offset + 2], argv[offset + 3], argv[offset + 4]);
      }

      group[groupId]->moveTo(positions);
      groupIsRunning[groupId] = true;
      return true;
    }

    case MULTISTEPPER_STOP: {
      const byte groupId = id;
      if (argc != 2 || groupId >= MAX_GROUPS || !group[groupId]) {
        Firmata.sendString(F("MULTISTEPPER_STOP: invalid group"));
        return false;
      }
      groupIsRunning[groupId] = false;
      reportGroupComplete(groupId);
      return true;
    }

    default:
      return false;
  }
}

/*==============================================================================
 * SETUP()
 *============================================================================*/

void AccelStepperFirmata::reset()
{
  for (byte i = 0; i < MAX_GROUPS; i++) {
    groupStepperCount[i] = 0;
    groupIsRunning[i] = false;
    if (group[i]) {
      delete group[i];
      group[i] = 0;
    }
  }
  numGroups = 0;

  for (byte i = 0; i < MAX_ACCELSTEPPERS; i++) {
    isRunning[i] = false;
    if (stepper[i]) {
      delete stepper[i];
      stepper[i] = 0;
    }
  }
  numSteppers = 0;
}

/*==============================================================================
 * Helpers
 *============================================================================*/

float AccelStepperFirmata::decodeCustomFloat(byte arg1, byte arg2, byte arg3, byte arg4)
{
  long l4 = (long)arg4;
  long significand = (long)arg1 | (long)arg2 << 7 | (long)arg3 << 14 | (l4 & 0x03) << 21;
  float exponent = (float)(((l4 >> 2) & 0x0f) - 11);
  bool sign = (bool)((l4 >> 6) & 0x01);
  float result = (float)significand;

  if (sign) {
    result *= -1;
  }

  result = result * powf(10.0, exponent);

  return result;
}

long AccelStepperFirmata::decode32BitSignedInteger(byte arg1, byte arg2, byte arg3, byte arg4, byte arg5)
{
  const uint32_t magnitude = (uint32_t)arg1 | ((uint32_t)arg2 << 7) |
      ((uint32_t)arg3 << 14) | ((uint32_t)arg4 << 21) |
      (((uint32_t)arg5 & 0x07) << 28);
  long result = (long)magnitude;

  if (arg5 & 0x08) {
    result = -result;
  }

  return result;
}

void AccelStepperFirmata::encode32BitSignedInteger(long value, byte pdata[])
{
  bool inv = false;

  if (value < 0) {
    inv = true;
    value = value * -1;
  }

  pdata[0] = value & 0x7f;
  pdata[1] = (value >> 7) & 0x7f;
  pdata[2] = (value >> 14) & 0x7f;
  pdata[3] = (value >> 21) & 0x7f;
  pdata[4] = (value >> 28) & 0x7f;

  if (inv == true) {
    pdata[4] = pdata[4] | 0x08;
  }
}

/*==============================================================================
 * LOOP()
 *============================================================================*/
void AccelStepperFirmata::report(bool elapsed)
{
  bool stepsLeft;

  if (numGroups > 0) {
    // if one or more groups exist,  update their position
    for (byte i = 0; i < MAX_GROUPS; i++) {
      if (group[i] && groupIsRunning[i] == true) {
        stepsLeft = group[i]->run();

        // send command to client application when stepping is complete
        if (stepsLeft != true) {
          groupIsRunning[i] = false;
          reportGroupComplete(i);
        }

      }
    }
  }

  if (numSteppers > 0) {
    // if one or more stepper motors are used, update their position
    for (byte i = 0; i < MAX_ACCELSTEPPERS; i++) {
      if (stepper[i] && isRunning[i] == true) {
        stepsLeft = stepper[i]->run();

        // send command to client application when stepping is complete
        if (!stepsLeft) {
          isRunning[i] = false;
          reportPosition(i, true);
        }

      }
    }
  }

}
