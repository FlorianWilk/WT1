#ifndef LIGHTS_H
#define LIGHTS_H

#include <Adafruit_NeoPixel.h>

class Lights{

    public: 
        Lights();
        void init();
        void welcome();
        void loop();
        void sineFade(uint16_t c,int steps,int maxv);
        void setState(int);
    private:

        Adafruit_NeoPixel *pixels; 
        int state;

};


#endif