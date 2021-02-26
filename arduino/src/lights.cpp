#include "lights.h"
#include <Adafruit_NeoPixel.h>

#define SA(p, c)                 \
    for (int i = 0; i < 12; i++) \
    {                            \
        p->setPixelColor(i, c);  \
    }

Lights::Lights()
{
    pixels = new Adafruit_NeoPixel(12, 44, NEO_GRB + NEO_KHZ800);
    state = 0;
}

void Lights::setState(int s)
{
    state = s;
}

void Lights::init()
{
    pixels->begin();
    pixels->clear();
    pixels->show();
}

void Lights::welcome()
{
    pixels->clear();
    pixels->show();
    delay(200);
    int steps = 200;
    for (uint32_t c = 0; c < 3; c++)
    {
        for (int o = 0; o < steps; o++)
        {
            uint32_t val = (uint32_t)(100.0 * sin(PI / (double)steps * (double)o));
            uint32_t color = pixels->ColorHSV(65535 / 6, 255, val);
            SA(pixels, color);
            pixels->show();
            delay(1);
        }
        delay(200);

        //
    }
}

long lastt;

void Lights::loop()
{
    long cur = millis();

    uint16_t hue = 65535 / 6;

    switch (state)
    {
    case 0:
        hue = 65535 / 3;
        break;

    case 1:
        hue = 65535 / 6;
        break;
    case 2:
        hue = 32000;
        break;
    }
    if (cur - lastt > 50)
    {

        uint32_t color = pixels->ColorHSV(hue, 255, 1);

        long c = cur % 2000;
        int p = 0; //cur/300%12;
        uint32_t c2 = pixels->ColorHSV(hue, 255, 2);
        if (c < 60)
        {
            uint32_t val = (uint32_t)(40.0 * sin(PI / (double)60 * (double)c));
            c2 = pixels->ColorHSV(hue, 255, val);
        }
        SA(pixels, c2);
        /*        pixels->setPixelColor((0+p)%12, c2);
        pixels->setPixelColor((6+p)%12, c2);
        pixels->setPixelColor((3+p)%12, c2);
        pixels->setPixelColor((9+p)%12, c2);*/
        pixels->show();
        lastt = cur;
    }
}
