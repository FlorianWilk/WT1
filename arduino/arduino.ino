#include <Arduino.h>
#include "motors.h"

#define DEBUG_SERIAL

Motors motors;

void setup()
{
  //TCCR3B = bit (WGM32) | bit (WGM32) | bit (CS30);
  TCCR3B = bit(WGM32) | bit(CS32) | bit(CS30);
  Serial.begin(115200);
  Serial.println("TANK V0.1");
  motors.init();

  //Set the output to the desired starting frequency.
}

long lastprint = millis();

String input = "";

void loop()
{
  if (Serial.available() > 0)
  {
    char newInput = Serial.read(); // all other functions like readString use timedRead which uses timeouts and produce delays
    input += newInput;
    if (input.endsWith("\n"))
    {
      Serial.print("E_CMD_RECV ");
      Serial.println(input);
      switch (input.charAt(0))
      {
      case 's':
        motors.pidOn(true);
        motors.setLeftPoint = 180;
        motors.setRightPoint = 180;
        break;
      case '3':
        motors.setPIDParams(0.41, 6.5919611, 0.001);
        break;
      case '2':
        motors.setPIDParams(0.41, 5.0, 0.001);
        break;
      case '1':
        motors.setPIDParams(0.41, 3.5919611, 0.001);
        break;

      case 'x':
        motors.leftOutputSpeed = 0;
        motors.rightOutputSpeed= 0;
        motors.pidOn(false);
        break;
      case 't':
        motors.setLeftPoint = 0;
        motors.setRightPoint = 0;
        break;
      }

      input = "";
    }
  }

  motors.loop();

#ifdef DEBUG_SERIAL
  if (millis() - lastprint > 500)
  {
    Serial.print("slSpeed:");
    Serial.print(motors.lastSpeed);
    Serial.print("lSpeed:");
    Serial.print(motors.leftInSpeed);
    Serial.print(",");
    Serial.print("lSetSpeed:");
    Serial.print(motors.setLeftPoint);
    Serial.print(",");

    Serial.print("lPWM:");
    Serial.print(motors.leftOutputSpeed);
    Serial.print(",");

    Serial.print("rSpeed:");
    Serial.print(motors.rightInSpeed);
    Serial.print(",");

    Serial.print("rSetSpeed:");
    Serial.print(motors.setRightPoint);
    Serial.print(",");

    Serial.print("rPWM:");
    Serial.println(motors.rightOutputSpeed);

    lastprint = millis();
  }
#endif
}
