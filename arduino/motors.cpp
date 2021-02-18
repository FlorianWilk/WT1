#include "motors.h"
#include <Arduino.h>

#include <AutoPID.h>

#include <PID_v1.h>

Motors *myInstance = NULL;

#define OUTPUT_MIN 0
#define OUTPUT_MAX 255

//#define TUNE
typedef void (* GenericFP)(void);
void gcb_leftA()
{
    myInstance->cb_leftA();
}
void gcb_leftB()
{
    myInstance->cb_leftB();
}
void gcb_right()
{
    myInstance->cb_right();
}

GenericFP *cab = new GenericFP[4];

Motors::Motors()
{
    myInstance = this;
}

void Motors::init()
{
    pinMode(18, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);
    pinMode(20, INPUT_PULLUP);
    pinMode(21, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(20), gcb_leftA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(21), gcb_leftB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(18), gcb_right, CHANGE);
    attachInterrupt(digitalPinToInterrupt(19), gcb_right, CHANGE);
    motor = new AF_DCMotor(1);  // MOTOR12_64KHZ);
    motor2 = new AF_DCMotor(2); //, MOTOR12_64KHZ);

    motor->run(FORWARD);
    motor->setSpeed(0);
    motor2->run(FORWARD);
    motor2->setSpeed(0);
    initPID();
}

void Motors::initPID()
{
    setLeftPoint = 0.0;
    setRightPoint = 0.0; // (25-180/190 12v 13v)

    kp = 0.41;  // powerfull (swappy)
    ki = 5.0;   // tighter
    kd = 0.001; //3;

#ifndef TUNE
    leftPID2 = new PID(&leftInSpeed, &leftOutputSpeed, &setLeftPoint, kp, ki, kd, DIRECT);
    rightPID2 = new PID(&rightInSpeed, &rightOutputSpeed, &setRightPoint, kp, ki, kd, DIRECT);
    leftPID2->SetMode(AUTOMATIC);
    rightPID2->SetMode(AUTOMATIC);
    rightPID2->SetSampleTime(200);
#else
    aTune = new PID_ATune(&rightInSpeed, &rightOutputSpeed);
    //    leftPID2->SetMode(MANUAL);
    //    rightPID2->SetMode(MANUAL);
    rightOutputSpeed = aTuneStartValue;
    aTune->SetNoiseBand(aTuneNoise);
    aTune->SetOutputStep(aTuneStep);
    aTune->SetControlType(1);
    aTune->SetLookbackSec((int)10);
#endif
}

bool stop = false;

void Motors::setPIDParams(double kp, double ki, double kd)
{
    leftPID2->SetTunings(kp, ki, kd);
    rightPID2->SetTunings(kp, ki, kd);
}

void Motors::loop()
{
    if (last_process == 0)
        last_process = millis();
    long current = millis();
    if (current - last_process > 50)
    {
        if (count_left > 0)
        {
            leftInSpeed = count_left / (float)((current - last_process)) * 60 / 56.0 / 2.0 * 60.0;
        }
        else
            leftInSpeed = 0;

        if (count_right > 0)
        {
            rightInSpeed = count_right / (float)((current - last_process)) * 60 / 56.0 / 2.0 * 60.0;
        }
        else
            rightInSpeed = 0;

        count_left = 0;
        count_right = 0;
        last_process = millis();
        leftPID2->Compute();
        rightPID2->Compute();
        setLeftSpeed(leftOutputSpeed);
        setRightSpeed(rightOutputSpeed);
    }
#ifndef TUNE

#else
    if (!stop)
    {
        byte val = (aTune->Runtime());
        if (val != 0)
        {
            Serial.println("TUNING ENDED");
            Serial.println(aTune->GetKp());
            Serial.println(aTune->GetKi());
            Serial.println(aTune->GetKd());
            stop = true;
            setRightSpeed(0);
        }
        else
        {
            setRightSpeed(rightOutputSpeed);
            //Serial.print(rightInSpeed);
            //Serial.print(":");
            //Serial.println(rightOutputSpeed);
        }
    }
#endif
}

void Motors::pidOn(boolean b){
    leftPID2->SetMode(b?AUTOMATIC:MANUAL);
    rightPID2->SetMode(b?AUTOMATIC:MANUAL);
}

void Motors::setLeftSpeed(double d)
{
    motor->setSpeed(d);
}

void Motors::setRightSpeed(double d)
{
    motor2->setSpeed(d);
}


volatile bool up;
volatile int ups=0;
void Motors::cb_leftA()
{
    count_left++;
    up=!up;
    if(up){
//        Serial.println("up");
        ups++;
//        Serial.println(ups);
    long cur=millis();
    if(lastA!=0){
        lastSpeed=(1000.0/((float)(cur-lastA)))/18.0/56.0*60.0;
    } else {
        lastSpeed=0;
    }
    lastA=cur;
    }
}


void Motors::cb_leftB()
{
    count_left++;
}


void Motors::cb_right()
{
    count_right++;
}
