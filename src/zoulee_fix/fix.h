#ifndef __FIX_H
#define __FIX_H

enum {
	LOW = 0,
	HIGH
};

float min(float a, float b);
float max(float a, float b);
float abs(float a);
float fabs(float a);
void delayMicroseconds(int time);
void delay(int time);
float analogRead();
int digitalRead(int pin);
void digitalWrite(int pin, bool state);

#endif // __FIX_H