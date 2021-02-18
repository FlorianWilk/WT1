#include <AFMotor.h>
#include <AutoPID.h>
#include <PID_v1.h>
class Motors
{

public:
    Motors();
    void init();
    void setLeftSpeed(double s);
    void setRightSpeed(double s);

    int count_left=0, count_right=0;

    void cb_leftA();
    void cb_leftB();
    void cb_right();

    void loop();

    double leftInSpeed=0, setLeftPoint=0, leftOutputSpeed = 0;
    double rightInSpeed=0, setRightPoint=0, rightOutputSpeed = 0;

    void setPIDParams(double kp,double ki,double kd);
    void pidOn(boolean b);
    volatile double lastSpeed;
private:

    double kp,ki,kd;

    AF_DCMotor *motor;  //(1, MOTOR12_64KHZ);
    AF_DCMotor *motor2; //(2, MOTOR12_64KHZ);

    long last_process=0;

    AutoPID *leftPID;  //(&leftInSpeed, &setLeftPoint, &leftOutputSpeed, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
    AutoPID *rightPID; //(&rightInSpeed, &setRightPoint, &rightOutputSpeed, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
    void initPID();

    PID *leftPID2;
    PID *rightPID2;

    double aTuneStep=100, aTuneNoise=1, aTuneStartValue=140;
volatile long lastA;



};
