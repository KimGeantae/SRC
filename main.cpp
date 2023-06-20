//#include "C:\Users\kgt22\Mbed Programs\practice\Header.cpp"
#include "C:\Users\kgt22\Mbed Programs\practice\Header.h"

//AnalogIn ir(PA_0);
extern DigitalOut led1;
extern InterruptIn btn;

extern AnalogIn irfl;
extern AnalogIn irfr;
extern AnalogIn irbl;
extern AnalogIn irbr;
extern AnalogIn irfm;
extern AnalogIn irfmr;
extern AnalogIn irfml;
extern AnalogIn irmm;
extern AnalogIn irbmr; 
extern AnalogIn irbml;

extern AnalogIn psdfl;
extern AnalogIn psdfr;
extern AnalogIn psdm;
extern AnalogIn psdb;

extern DigitalOut DirL;
extern DigitalOut DirR;
extern PwmOut PwmL;
extern PwmOut PwmR;
extern PwmOut rcServo;

extern float ang, inc,Inc,INC;
extern char preread;
extern int count;

extern float angL;
extern float angR;

extern volatile bool gotPacket;
extern volatile float data[3];
extern uint16_t ir_val[10];
//0 fl
//1 fr
//2 bl
//3 br
//4 fm
//5 fmr
//6 fml
//7 mm
//8 bmr
//9 irbml

extern bool ir_plusval[8];
//0 fl+fr
//1 fmr + fml
//2 fl+fml
//3 fr+fmr
//4 bl+bml
//5 br+bmr
//6 all
extern bool Serial_chk;
extern bool code_start;

extern RawSerial board;
extern RawSerial pc;

extern Timer control_tmr;
extern Timer brk_tmr;
extern Timer rotate_tmr;
extern Timer tilt_tmr;
extern Timer test_tmr;
extern Timer tmove_tmr;

// int turn_escape_time = 25000;
// int back_escape_time = 100000;
extern int turn_escape_time; // 세부조정 필요!!!
extern int back_escape_time; // 세부조정 필요!!!
extern int fight_back_escape_time; // 세부조정 필요!!!
extern int rotate_escape_time; // 세부조정 필요!!!
extern int tilt_back_escape_time; // 세부조정 필요!!!
extern int turn_time;
extern int test_time;

extern uint16_t black;
extern int mode;

extern float A,B,C;
int main(){
    sensor_read();
    sensor_plus();
    DC_set();
    servo_set(rcServo);
    //rcServo.period_ms(10);
    board.attach(&onSerialRx);
    //test_tmr.start();
    servo_move(rcServo);
    btn.fall(&Button_start);
    sensor_print1();

    while(1){
        if(code_start == true){
            sensor_read();
            sensor_plus();
            sensor_print2();
            test_tmr.start();
            servo_move(rcServo);
            test_tmr.start();
            if(mode == 0){//초기 상태
                if(test_tmr.read_us()<turn_time){//90도 회전
                    speedL = -0.5;
                    speedR = 0.5;
                }

                else if(test_tmr.read_us()>=turn_time && ir_val[0]>black && ir_val[1]>black) {
                    speedL = 0.8;
                    speedR = 0.78;
                    test_tmr.stop();
                        //sensor_print2();
                }
                else if(ir_plusval[0] == true){//앞ir이 색영역을 발견하면
                    speedL = 0.6;
                    speedR = -0.8;
                }
                else if(ir_plusval[6]==true && ir_val[2]<black){
                    speedL = 0.0;
                    speedR = 0.0;
                    mode=1;
                    test_tmr.reset();
                }//*/
            }



                /*else if(mode ==1) {//원을 돌고 난 뒤 상태 (첫 번째 안) 카메라 사용
                    if(ir_plusval[0]==false) DC_follow();
                    else {
                        speedL = 0.0;
                        speedR = 0.0;
                        mode=2;
                    }
                }//*/

            else if(mode ==1) {//원을 돌고 난 뒤 상태 (두 번째 안) 카메라 사용
                if(ir_plusval[0]==false ){
                    if(data[1]<170){//가깝지 않을 때 원타기
                        speedL = 0.3;
                        speedR = 0.8;
                    }
                    if(ang >= 70 && ang <= 110 & ir_val[0]>black && ir_val[1]>black){//적이 정면에 있을 때 직진
                        speedL = 1;
                        speedR = 1;
                    }
                    else if(ir_plusval[0]==false &&ir_plusval[1]==false&&ir_plusval[2]==false&&ir_plusval[3]==false && data[1]<170){
                        DC_follow();
                    }
                    else if(ir_plusval[0]==false &&ir_plusval[1]==false&&ir_plusval[2]==false&&ir_plusval[3]==false && data[1]>170){
                        speedL = 1;
                        speedR = 1;
                    }
                }

                else if(ir_plusval[0]==true){//앞쪽 색영역 봤을 때
                    if(data[1]>170 && ir_val[5]<black && ir_val[6]<black /*&& ir_val[8]>black && ir_val[9]>black/*/){
                        speedL = 0;
                        speedR = 0;
                    }
                    else if(data[1]>170&&ir_val[5]>black && ir_val[6]>black){
                        speedL = 1;
                        speedR = 1;
                    }
                    else if(data[1]<170){
                        speedL = 0.6;
                        speedR = -0.8;
                        
                    }
                    else if(ir_plusval[6]==true && ir_val[2]<black){
                            speedL = 0.0;
                            speedR = 0.0; 
                    }
                        //mode=2;
                }
                else if(ir_plusval[7]==true){
                    mode = 19;
                }
            }//*/

            else if(mode == 19){//바퀴 4개 색영역(탈출 코드) psd 사용안함
                if(ir_plusval[7]==false){
                    mode = 1;
                }
                else if(ang<=70){
                    speedL = 0.45;
                    speedR = -0.45;
                }
                else if(ang>=110){
                    speedL = -0.45;
                    speedR = 0.45;
                }
                else if(data[1]>170 && data[0]>=70 && data[0]<=110){
                    speedL =1;
                    speedR =1;
                }
            }   

            whl_move();
        }
    
    }

}

