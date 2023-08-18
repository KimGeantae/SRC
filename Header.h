# ifndef Function_H
# define Function_H

#include "mbed.h"
#include "GP2A.h"
#include "Servo.h"
#include <string>
#include <stdlib.h>
#include "rtos.h"
#include "Thread.h"

#define psdcontroltime 50
#define maincontroltime 10
#define Alpha 0.8
//#include "Servo.h"

template <class T> T map(T x,T in_min,T in_max,T out_min,T out_max);
void sensor_read();
void sensor_print1();
void sensor_print2();
void sensor_print3();
void sensor_plus();

void psd_read();//Thread사용

void servo_set(PwmOut &rc);
void servo_move(PwmOut &rc);
void servoturn(PwmOut&rc,float deg);
void onSerialRx();

void DC_set();
void DC_move(float _PWML, float _PWMR);
void DC_follow();
void Button_start();

void tmr_move(Timer* _timer,int* _tmr , double _speedL,double _speedR);
void tmr_reset(Timer* _timer);

extern double speedL;
extern double speedR;

void test_move(Timer* __timer, int*__tmr, double __speedL,double __speedR);
void escape(Timer* __timer, double*__tmr, double __speedL,double __speedR);
void whl_move();

//이동평균필터
//float MovingAveragefilter();
//void insertintoRawArray(uint16_t value);
//void LPF_Interrupt();
#endif
