#include <Arduino.h>
#include "motors.h"

#define DEBUG_SERIAL

#define BT Serial3 

irqISR(irq1, isr1);
Motor wheel1(1, 20, 21, &irq1);
irqISR(irq2, isr2);
Motor wheel2(3, 18, 19, &irq2);
Motors motors(&wheel1, &wheel2);

void initBT(){
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

long lastprint ;

String input = "";
bool flip;
void loop()
{

  if (millis() - lastprint > 10000 && false)
  {
    Serial.print("Flipping ");
    Serial.println(flip);
    if (flip)
       motors.pidOn(true);
//      motors.setSpeedRPMDesired(2200, -2200);
    else
      motors.pidOn(false);

  //    motors.setSpeedRPMDesired(-1200, 1200);
    flip = !flip;
    lastprint=millis();
  }

  motors.loop();

  if (Serial.available() > 0)
  {
    char newInput = Serial.read(); // all other functions like readString use timedRead which uses timeouts and produce delays
    input += newInput;
    if (input.endsWith("\n"))
    {
      Serial.print("E_CMD_RECV ");
      Serial.println(input);
      /*
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
*/
      input = "";
    }
  }
}
