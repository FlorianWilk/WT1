#include <AFMotor.h>
#include <AutoPID.h>
#include <PID_v1.h>

#define SEC_PER_MIN 60
#define MAX_RPM 2500    // Maximum RPM of the Motorshaft
#define MIN_CLIP_PWM 130 // Minimum Power for the Motor to move
#define MICROS_PER_SEC 1000000
#define DIR_ADVANCE HIGH
#define DIR_BACKOFF LOW
#define TRIGGER RISING
#define CPR 30.0
#define DIR_INVERSE !
#define REDUCTION_RATIO 56
#define PIN_UNDEFINED 255
#define DEBUG_MOTORS false
#define KP 0.31
#define KI 0.02
#define KD 0.00

#define DEBUG_INTERVAL_MS 1000
#define SPEEDPPS2SPEEDRPM(freq) ((double)(freq) / (double)(CPR)) * (double)(SEC_PER_MIN)

#ifdef DEBUG
#define debug()                     \
    {                               \
        if (!Serial.available())    \
            Serial.begin(Baudrate); \
        Serial.println(__func__);   \
    }
#else
#define debug() \
    {           \
    }
#endif

#define irqISR(y, x)                                                                                                                  \
    void x();                                                                                                                         \
    struct ISRVars y = {x};                                                                                                           \
    void x()                                                                                                                          \
    {                                                                                                                                 \
        static bool first_pulse = true;                                                                                               \
        y.pulseEndMicros = micros();                                                                                                  \
        if (y.pinIRQB != PIN_UNDEFINED)                                                                                               \
            y.currDirection = DIR_INVERSE(digitalRead(y.pinIRQ) ^ digitalRead(y.pinIRQB));                                            \
        else                                                                                                                          \
            y.currDirection == DIR_ADVANCE ? ++y.pulses : --y.pulses;                                                                 \
        if (first_pulse == false && y.pulseEndMicros > y.pulseStartMicros)                                                            \
        {                                                                                                                             \
            y.speedPPS = (double)(y.currDirection ? 1.0 : -1.0) * (double)(MICROS_PER_SEC / (y.pulseEndMicros - y.pulseStartMicros)); \
            /* y.accPPSS=(y.speedPPS-y.lastSpeedPPS)*y.speedPPS; */                                                                   \
        }                                                                                                                             \
        else                                                                                                                          \
            first_pulse = false;                                                                                                      \
        y.pulseStartMicros = y.pulseEndMicros;                                                                                        \
        /* y.lastSpeedPPS=y.speedPPS; */                                                                                              \
    }

struct ISRVars
{
    void (*ISRfunc)();
    //volatile unsigned long pulses;
    volatile long pulses; // 201104, direction sensitive
    volatile unsigned long pulseStartMicros;
    volatile unsigned long pulseEndMicros;
    volatile double speedPPS; // can also be negative to make the PID do the smooth transition on direction changes
    //volatile unsigned int  lastSpeedPPS;
    //volatile int accPPSS;	// acceleration, Pulse Per Sec^2
    volatile bool currDirection;
    unsigned char pinIRQB;
    unsigned char pinIRQ;
};

class Motor
{
public:
    Motor(unsigned char _motorID, unsigned char _pinIRQ, unsigned char _pinIRQB, struct ISRVars *_isr);

    struct ISRVars *isr;
    void init();
    void initPID();
    void setPIDParams(double kp, double ki, double kd);
    void loop();
    double getSpeedOutput();
    double getSpeedInput();
    double getSpeedDesired();

    boolean isPidOn();
    void setSpeedRPMDesired(double speed);
    bool getCurrDirection();
    void pidOn(boolean b);
private:
    Motor();
    unsigned char motorID;
    unsigned char pinIRQ;
    unsigned char pinIRQB;

    AF_DCMotor *afmotor;
    PID *PIDC;

    double speedRPMInput;   // RPM: Round Per Minute
    double speedRPMOutput;  // RPM
    double speedRPMDesired; // RPM
};

class Motors
{

public:
    Motors(Motor *wheelLeft, Motor *wheelRight);

    void init();
    void loop();

    void setPIDParams(double kp, double ki, double kd);
    void pidOn(boolean b);
    void setSpeedRPMDesired(double, double);

    boolean isMoving();

private:
    Motors();

    Motor *motorLeft;
    Motor *motorRight;

    long lastDebug;
    void initPID();
};
