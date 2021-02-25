#include "motors.h"
#include <Arduino.h>
#include <AutoPID.h>
#include <PID_v1.h>

Motor::Motor(unsigned char _motorID, unsigned char _pinIRQ, unsigned char _pinIRQB,
             struct ISRVars *_isr) : motorID(_motorID), isr(_isr)
{
    debug();

    isr->pinIRQ = _pinIRQ;
    isr->pinIRQB = _pinIRQB;

    //	pinMode(isr->pinIRQ,INPUT);
    pinMode(isr->pinIRQ, INPUT_PULLUP);
    pinMode(isr->pinIRQB, INPUT_PULLUP);

    if (isr->pinIRQB != PIN_UNDEFINED)
    {
        pinMode(isr->pinIRQB, INPUT);
    }
}

bool Motor::getCurrDirection()
{
    return isr->currDirection;
}

void Motor::setPIDParams(double kp, double ki, double kd)
{
    PIDC->SetTunings(kp, ki, kd);
}

void Motor::initPID()
{
    attachInterrupt(digitalPinToInterrupt(isr->pinIRQ), isr->ISRfunc, TRIGGER);
    //    double kp = 0.11;  // powerfull (swappy)
    //   double ki = 0.9;   // tighter
    //  double kd = 0.001; //3;

    double kp = 0.11;  // powerfull (swappy)
    double ki = 1.9;   // tighter
    double kd = 0.001; //3;

    PIDC = new PID(&speedRPMInput, &speedRPMOutput, &speedRPMDesired, kp, ki, kd, DIRECT);
    PIDC->SetSampleTime(5);
    PIDC->SetMode(AUTOMATIC);
    //PIDC->SetMode(MANUAL);
    PIDC->SetOutputLimits(-255.0, 255.0);
    //setPIDParams(KP,KI,KD);
}

void Motor::init()
{
    afmotor = new AF_DCMotor(motorID);
    ;
}

void Motor::loop()
{
    speedRPMInput = SPEEDPPS2SPEEDRPM(isr->speedPPS);

    if (PIDC->GetMode() == AUTOMATIC)
    {
        PIDC->Compute();
        afmotor->run(speedRPMOutput > 0 ? FORWARD : BACKWARD);
        double out = abs(speedRPMOutput);
        afmotor->setSpeed(out > MIN_CLIP_PWM ? out : 0);
    }
}

double Motor::getSpeedOutput()
{
    return speedRPMOutput;
}

double Motor::getSpeedInput()
{
    return speedRPMInput;
}

void Motor::setSpeedRPMDesired(double speed)
{
    speedRPMDesired = speed;
}

void Motor::pidOn(boolean b)
{
    PIDC->SetMode(b ? AUTOMATIC : MANUAL);
    if (!b)
    {
        afmotor->setSpeed(0);
    }
}

Motors::Motors(Motor *_motorLeft, Motor *_motorRight)
{
    motorRight = _motorRight;
    motorLeft = _motorLeft;
}

void Motors::init()
{
    motorLeft->init();
    motorRight->init();

    motorLeft->initPID();
    motorRight->initPID();
}

void Motors::pidOn(boolean on)
{
    motorLeft->pidOn(on);
    motorRight->pidOn(on);
}

void Motors::setSpeedRPMDesired(double l, double r)
{
    motorLeft->setSpeedRPMDesired(l);
    motorRight->setSpeedRPMDesired(r);
}
void Motors::setPIDParams(double kp, double ki, double kd)
{
    motorLeft->setPIDParams(kp, ki, kd);
    motorRight->setPIDParams(kp, ki, kd);
}

void Motors::loop()
{
    long current = millis();
    motorLeft->loop();
    motorRight->loop();
    if (current - lastDebug > DEBUG_INTERVAL_MS && DEBUG_MOTORS)
    {
        Serial.print("Left:");
        Serial.print(" PWM:");
        Serial.print(motorLeft->getSpeedOutput());
        Serial.print(" Encoder:");
        Serial.print(motorLeft->getSpeedInput());
        Serial.print(" D:");
        Serial.print(motorLeft->getCurrDirection());
        Serial.print(" Right: PWM:");
        Serial.print(motorRight->getSpeedOutput());
        Serial.print(" Encoder:");
        Serial.print(motorRight->getSpeedInput());
        Serial.print(" D:");
        Serial.print(motorRight->getCurrDirection());
        Serial.println("");
        lastDebug = current;
    }
}
