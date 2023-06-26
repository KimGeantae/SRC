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
extern float dis;

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
extern uint16_t psdfl_val;
extern uint16_t psdfr_val;
extern uint16_t psdm_val;
extern uint16_t psdb_val;

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
                if(ir_plusval[0]==false){
                    if(data[1]<dis && ir_val[6]<black){//가깝지 않을 때 원타기
                        speedL = 0.3;
                        speedR = 0.8;
                    }
                    if(ang >= 70 && ang <= 110){//적이 정면에 있을 때 직진
                        speedL = 1;
                        speedR = 1;
                    }
                    else if(ir_plusval[7] == true){
                        if((ang<70 || ang>110) && psdfl_val < 30){
                            //적을 쫓아가서 직진하다가 적이 옆으로 빠지고 
                            //우리가 다시 트래킹하기 전에 벽에 너무 가까운 상태여서 박히는 경우
                            mode = 21;
                        }
                        else if (data[0] == 999 && psdfl_val < 30){//카메라,  서보 오류로 적을 잃어버리거나 찾지 못해서 벽에 박히는 경우
                            mode = 22;
                        }
                        else{
                            DC_follow();
                        }

                    }
                    
                }

                else if(ir_plusval[0]==true){//앞쪽 색영역 봤을 때
                        
                    if(data[1]>dis && ir_val[5]<black && ir_val[6]<black /*&& ir_val[8]>black && ir_val[9]>black/*/){
                        speedL = 0;
                        speedR = 0;
                    }
                    else if(data[1]>dis && ir_val[5]>black && ir_val[6]>black){
                        speedL = 1;
                        speedR = 1;
                    }
                    else if(data[1]<dis){
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
                if(ir_plusval[0]==true){
                    if(psdfl_val > 30){
                        while(ir_plusval[7] == false){
                            speedL = 1;
                            speedR = 1;
                        }
                        mode = 1;
                    }
                    else{
                        speedL = -0.6; speedR = -0.6;
                    }
                }
                else if(ir_plusval[7] == false){
                    speedL = 0.5; speedR = 0.5;
                    mode = 1;
                }
            }
            else if (mode == 21){
                //적을 쫓아가서 직진하다가 적이 옆으로 빠지고 
                //우리가 다시 트래킹하기 전에 벽에 너무 가까운 상태여서 박히는 경우
                //후진을 해야하는지 실험적으로 해봐야 함
                while(ang<70 || ang>110){
                    if(ang>110){//마지막으로 적을 우측에서 봤을 경우
                       speedL= 0.2;
                       speedR= -0.2;
                    }
                   if(ang<70){//마지막으로 적을 좌측, 중앙에서 봤을 경우 / preread값이 없는 경우
                       speedL= -0.2;
                       speedR= 0.2;
                    }
                }
                speedL=0;
                speedR=0;
                mode=1;
            }

            else if(mode ==22){//카메라,  서보 오류로 적을 잃어버리거나 찾지 못해서 벽에 박히는 경우
                while (data[0] ==999){
                   if(preread == 'R'){//마지막으로 적을 우측에서 봤을 경우
                       speedL= 0.2;
                       speedR= -0.2;
                    }
                   else{//마지막으로 적을 좌측, 중앙에서 봤을 경우 / preread값이 없는 경우
                       speedL= -0.2;
                       speedR= 0.2;
                    }
                }
                speedL=0;
                speedR=0;
                mode=1;
            }


            // else if(mode ==20){//벽에 부딪힐거같은 경우

            // }
            else if(mode == 3){//함수 확인용
                DC_follow();
            }
            whl_move();
        }//if(codestart = true)
    }//while

}//main

