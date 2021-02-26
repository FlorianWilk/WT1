#include <Arduino.h>
#include "motors.h"
#include "lights.h"

#define DEBUG_SERIAL
#define BT Serial3
#define LINEAR_MAX_SPEED 2200.0
#define TURN_MAX_SPEED 1200.0

irqISR(irq1, isr1);
Motor wheel1(1, 20, 21, &irq1);
irqISR(irq2, isr2);
Motor wheel2(3, 18, 19, &irq2);
Motors motors(&wheel1, &wheel2);

Lights lights;

void initBT()
{
  BT.begin(9600);
  delay(500);
  BT.println("AT+NAMEWT1\n\r");
  BT.flush();
  delay(500);
}

void setup()
{
  //TCCR3B = bit (WGM32) | bit (WGM32) | bit (CS30);
  TCCR3B = bit(WGM32) | bit(CS32) | bit(CS30);
  Serial.begin(115200);

  lights.init();
  lights.welcome();

  Serial.println("WT1 INIT");

  Serial.print("BTSERIAL:");
  initBT();
  Serial.println("OK");

  Serial.print("MOTORS:");
  motors.init();
  motors.pidOn(false);
  Serial.println("OK");

  BT.println("WT1 INIT DONE");
}

long lastSerial;
boolean motorsDisabled = false;
boolean isMoving = false;
String input = "";

void loop()
{

  long cur = millis();

  if (Serial.available() > 0)
  {
    lastSerial = cur;
    char newInput = Serial.read(); // all other functions like readString use timedRead which uses timeouts and produce delays
    if (newInput != '#')
    { // our ping charachter, ignore this
      input += newInput;
      if (input.endsWith("\n"))
      {
        //        Serial.print("E_CMD_RECV ");
        //        Serial.println(input);

        switch (input.charAt(0))
        {
        case 's':
          motors.pidOn(true);
          int x, y;
          if (sscanf(input.c_str(), "s%d,%d", &x, &y) == 2)
          {
            double s1, s2;

            // Set Left/Right depending on the amount of y (% of LINEAR_MAX_SPEED)
            s1 = (double)y / 100.0 * LINEAR_MAX_SPEED;
            s2 = s1;

            // Add/Substract TurnSpeed to left/right depending on the amount of X, whereas maxspeed is limited to LINEAR_MAX_SPEED
            if (x < 0)
            {
              double sd = ((double)abs(x)) / 100.0 * TURN_MAX_SPEED;
              s2 = min(LINEAR_MAX_SPEED, s1 + sd);
              s1 = s2 - sd * 2;
            }
            else if (x > 0)
            {
              double sd = ((double)abs(x)) / 100.0 * TURN_MAX_SPEED;
              s1 = min(LINEAR_MAX_SPEED, s1 + sd);
              s2 = s1 - sd * 2;
            }

#ifdef DEBUG_SERIAL
            Serial.print("Speed Vector: ");
            Serial.print(x);
            Serial.print(",");
            Serial.print(y);
            Serial.print(" Wheels: ");
            Serial.print(s1);
            Serial.print(",");
            Serial.print(s2);
            Serial.println("");
#endif
            motors.setSpeedRPMDesired(s1, -s2);
          }
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
          motors.setSpeedRPMDesired(0, 0);
          motors.pidOn(false);
          break;
        case 't':
          motors.setSpeedRPMDesired(0, 0);
          break;
        }
        input = "";
      }
    }
  }

  if (cur - lastSerial > 500 || !lastSerial)
  {
    if (!motorsDisabled)
    {
      Serial.println("Disabling Motors");
      motorsDisabled = true;
      lights.setState(1);
      motors.pidOn(false);
    }
  }
  else
  {
    if (motorsDisabled)
    {
      Serial.println("Enabling Motors");
      motorsDisabled = false;
      if (isMoving)
        lights.setState(2);
      else
        lights.setState(0);
      motors.pidOn(true); // Not sure if i really want this
    }
    if (motors.isMoving() && !isMoving)
    {
      lights.setState(2);
      isMoving = true;
    }
    else if (!motors.isMoving() && isMoving)
    {
      isMoving = false;
      lights.setState(0);
    }
  }

  motors.loop();
  lights.loop();
}
