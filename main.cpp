//#include "C:\Users\kgt22\Mbed Programs\practice\Header.cpp"
#include "C:\Users\kgt22\Mbed Programs\practice\Header.h"
#include "C:\Users\kgt22\Mbed Programs\practice\MPU9250\MPU9250.h"

uint16_t tt;
//------------------------------------- 선언 ------------------------------------//
//------------------------------mpu9250---------------------------------------//
extern float sum;
extern uint32_t sumCount;
extern char buffer[14];
extern float tmp_angle_x, tmp_angle_y, tmp_angle_z;
extern float filltered_angle_x, filltered_angle_y, filltered_angle_z;

extern MPU9250 mpu9250; // SDA, SCL
extern Timer t;
//Serial pc(USBTX, USBRX, 115200); // tx, rx
//----------------------------------------------------------------------------//
//AnalogIn ir(PA_0);
extern DigitalOut led1;
extern InterruptIn btn;
//----------thread----------------//
extern uint64_t Now_time,Work_time,Nowm_time,Workm_time;
//extern Thread psd_th;
//--------------------------------//
extern float x_deg;
extern float y_deg;
//----------------------------------------------------------------------------//

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

extern GP2A psdfl;
extern GP2A psdfr;
extern GP2A psdm;
extern GP2A psdb;
extern float now_data;

extern DigitalOut DirL;
extern DigitalOut DirR;
extern PwmOut PwmL;
extern PwmOut PwmR;
extern PwmOut rcServo;

extern float ang, inc,Inc,INC;
extern char preread;
extern int count;
extern float dis;
extern bool color;

extern float angL;
extern float angR;

extern volatile bool gotPacket;
extern volatile float data_R[3];
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
extern double psdfl_val;
extern double psdfr_val;
extern double psdm_val;
extern double psdb_val;

extern bool Serial_chk;
extern bool code_start;

extern RawSerial board;
extern RawSerial pc;

extern Ticker control_tmr;
extern Timer brk_tmr;
extern Timer rotate_tmr;
extern Timer imu_tmr;
extern Timer test_tmr;
extern Timer tmove_tmr;
extern Timer be_tmr;//사용
extern Timer re_tmr;//사용


extern int turn_escape_time; // 세부조정 필요!!!
extern double back_escape_time; // 세부조정 필요!!!
extern int fight_back_escape_time; // 세부조정 필요!!!
extern int rotate_escape_time; // 세부조정 필요!!!
extern int tilt_back_escape_time; // 세부조정 필요!!!
extern double turn_time;
extern int escape_time;

extern uint16_t black;
extern int mode;
extern int imu_count;
extern float A,B,C,D;
Thread psd_th(osPriorityAboveNormal);
Thread imu_th(osPriorityNormal);


//------------------------------main--------------------------------------//
int main(){
    osThreadSetPriority(osThreadGetId(), osPriorityRealtime7);
    imu_th.start(&imu_read);
    psd_th.start(&psd_read);



    DC_set();
    servo_set(rcServo);
    board.attach(&onSerialRx);
    btn.fall(&Button_start);
    //float filtertime = 0.1;
    //control_tmr.attach(&LPF_Interrupt,filtertime);
    color = false;
    
    while(1){
        Nowm_time = rtos::Kernel::get_ms_count();
        //pc.printf("%11u\n",tt - Nowm_time);
        if(code_start == true){
            sensor_read();
            sensor_plus();
            servo_move(rcServo);
            //imu_read();
            //mode =1;
            pc.printf("%f   ", filltered_angle_x);
            //pc.printf(" \t");
            pc.printf(",");
            pc.printf("%f    ",filltered_angle_y);
            //pc.printf("\t");
            // pc.printf("time : %i", t.read_ms());
            pc.printf("\n");
            //sensor_print1();

            imu_tmr.start();
            //------imu값이 5도이상 체크될때------//
            if(filltered_angle_x >= x_deg){
                imu_tmr.reset();
                if(imu_tmr.read_ms()>=1500){
                    mode = 3;
                }
                else{
                    mode = 1;
                }
            }
            //-----------------------------------//
            if(mode == 0){//초기 상태
                test_tmr.start();
                double time = test_tmr.read_ms();
                if(time<turn_time){//90도 회전
                    speedL = 0.6;
                    speedR = -0.6;
                    //pc.printf("start\n");
                }

                else if(time>=turn_time && ir_val[0]>black && ir_val[1]>black) {
                    speedL = 0.5;
                    speedR = 0.53;
                    //sensor_print1();
                    //test_tmr.reset();
                    //test_tmr.stop();
                }
                else if(ir_plusval[0] == true && time>=turn_time){//앞ir이 색영역을 발견하면
                    speedL = 0.6;
                    speedR = -0.7;
                    //pc.printf("2all right");
                }
                else if(ir_plusval[6]==true && ir_val[2]<black){
                    speedL = 0.0;
                    speedR = 0.0;
                    //pc.printf("mode o done");
                    mode=1;
                    test_tmr.reset();
                }//*/
                else{
                    speedL = 0;
                    speedR = 0;
                    mode = 1;
                    //pc.printf("nope");
                }
            }

            else if(mode ==1) {//원을 돌고 난 뒤 상태 (두 번째 안) 카메라 사용
                if(ir_plusval[0]==false){
                    if(data_R[1]<dis && ir_val[6]<black){//가깝지 않을 때 원타기
                        //pc.printf("red circle\n\r");
                        speedL = 0.3;
                        speedR = 0.8;
                    }
                    else if((ang <70 || ang>110) && data_R[1]>dis){
                        DC_follow();
                    }
                    else if(data_R[1]<dis && ir_val[0]>black &&ir_val[1]>black &&ir_val[2]>black &&ir_val[3]>black &&ir_val[4]>black &&ir_val[5]>black &&ir_val[6]>black &&ir_val[7]>black &&ir_val[8]>black ){
                        if(data_R[1]<dis){
                            float delang = map<float>(abs(ang-90), 0. , 80. ,0.0 ,0.4 );
                            float dellen = map<float>(data_R[1], 0. , 250. ,0. ,0.05 );
                                if(ang>130){//전부 다 최대치로 움직이게 변경 필요
                                    A = 0.8;
                                    B = -0.8;
                                    D=0;
                                    //C =0;
                                    //pc.printf("slow right\n\r");
                                }
                                else if(ang>=50 && ang<=130){
                                    A = 0;
                                    B = 0;
                                    D = 0.59;
                                    //C= 0.3;
                                    //pc.printf("middle\n\r");
                                }
                                else if(ang<50){
                                    A=-0.8;
                                    B=0.8;
                                    D=0;
                                    //C=0;
                                    //pc.printf("slow left\n\r");
                                }
                                speedL = 0.4+ A*delang+D;
                                speedR = 0.4+ B*delang+D;
                        }
                    }
                    else if(ang >= 70 && ang <= 110){//적이 정면에 있을 때 직진
                        //pc.printf("go straight\n\r");
                        speedL = 0.99;
                        speedR = 0.99;
                    }
                    else if((ang<70 || ang>110) && (psdfl_val < 40 || psdfr_val<40)){//벽을 볼때
                        //pc.printf("side wall\n\r");
                            //적을 쫓아가서 직진하다가 적이 옆으로 빠지고 
                            //우리가 다시 트래킹하기 전에 벽에 너무 가까운 상태여서 박히는 경우
                        //while(ang<70 || ang>110){
                            if(ang<70){//마지막으로 적을 우측에서 봤을 경우
                                //pc.printf("side wall turn right\n\r");
                                speedL= -0.4;
                                speedR= 0.4;
                            }
                            else if(ang>110){//마지막으로 적을 좌측, 중앙에서 봤을 경우 / preread값이 없는 경우
                                //pc.printf("side wall turn left\n\r");
                                speedL= 0.4;
                                speedR= -0.4;
                            }
                            //DC_move(speedL, speedR);servo_move(rcServo);
                        //}
                        //mode = 21;++///////////////여기 imu때문에 수정함 while문 삭제///////////
                    }
                    else if (data_R[0] == 999 && (psdfl_val < 40 || psdfr_val < 40)){//카메라,  서보 오류로 적을 잃어버리거나 찾지 못해서 벽에 박히는 경우
                        //pc.printf("no wall\n\r");
                        if (data_R[0] ==999){//////////여기도imu때문에 수정 while -> if
                            if(preread == 'R'){//마지막으로 적을 우측에서 봤을 경우
                                //pc.printf("no wall turn right\n\r");
                                speedL= 0.4;
                                speedR= -0.4;
                            }
                            else{//마지막으로 적을 좌측, 중앙에서 봤을 경우 / preread값이 없는 경우
                                //pc.printf("no wall turn left\n\r");
                                speedL= -0.4;
                                speedR= 0.4;
                            }
                            //DC_move(speedL, speedR);; servo_move(rcServo);
                        }
                        //mode = 22;
                    }
                    else /*if(ir_plusval[7] == true)*/ DC_follow();
                    /*else if(ir_plusval[7] == true){
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
                    }//*/
                }

                else if(ir_plusval[0]==true){//앞쪽 색영역 봤을 때
                    if(data_R[1]<dis){
                        // if(psdb_val>100 && (ang<70||ang>110)){//트래킹 중 파란원에 걸쳐있을때
                        //     pc.printf("tracking blue\n\r");
                        //     blue_escape(&be_tmr,&back_escape_time,-0.5,-0.5);
                        // }
                        if (now_data<=100){//트래킹 중 빨간원에 걸칠때
                            //pc.printf("tracking red\n\r");
                            if(ir_val[2]>black){////여기도 imu때문에 수정 while->if
                                speedL=0.8;
                                speedR=-0.8;
                                //sensor_read();DC_move(speedL, speedR);servo_move(rcServo);
                            }
                            /*if(ang>=110){
                                speedL = -0.4;
                                speedR = 0.5;
                            }
                            else{
                                speedL = 0.4;
                                speedR = -0.5;
                            }//*/
                        }
                        else if (ang>70 && ang<110){
                            speedL=0;
                            speedR=0;
                        }
                    }
                    else{//상대방이 바로 앞에 있을 때 색영역을 본 경우
                        //pc.printf("blue stay\n\r");
                        if(ir_val[5]<black && ir_val[6]<black && ir_val[2]>black && ir_val[3]>black){
                               speedL = 0;
                               speedR = 0;
                        }
                        else if(ir_val[5]>black && ir_val[6]>black){
                                speedL = 0.5;
                                speedR = 0.5;
                        }
                    }
                }
                else if(ir_plusval[7]==true){
                    mode = 19;
                }
            }//*/

            else if(mode == 19){//바퀴 4개 색영역(탈출 코드) psd 사용안함
                if(now_data<60){
                    color = true;
                    escape(&be_tmr,&turn_time,-0.5,0.5);
                    if(psdfl_val < 130 || psdfr_val<130){
                        speedL = 0.5; speedR = -0.5;
                        //pc.printf("shoot\n");
                    }
                    else{
                        escape(&be_tmr,&turn_time,0.8,0.8);
                        mode = 1;
                        //pc.printf("shoot1\n");
                    }
                }
                else if(color == false && now_data >= 60){
                    escape(&re_tmr,&back_escape_time,-0.5,-0.5);
                    mode =1;
                }
            }

            else if(mode == 3){//로봇이 들렸을 때
                if(abs(filltered_angle_x) < x_deg){
                    mode = 1;
                }
                if(abs(filltered_angle_x) > 20){
                    mode = 1;
                }
                else{
                    if(ir_val[8]<black || ir_val[2]<black){//우측으로 탈출
                    speedL = -0.8;
                    speedR = 0.8;
                    }
                    else if(ir_val[8]>black || ir_val[2]>black){
                        speedL = 0.99;
                        speedR = 0.99;
                    }
                    else if(ir_val[3]<black || ir_val[7]<black){//좌측으로 탈출
                        speedL = 0.8;
                        speedR = -0.8;
                    }
                    else if(ir_val[3]>black || ir_val[7]>black){
                        speedL = 0.99;
                        speedR = 0.99;
                    }

                    if(ir_val[2]<black && ir_val[3]<black && ir_val[7]<black && ir_val[8]<black && data_R[1]<dis){
                        Rescape_move(&re_tmr,&escape_time,-0.8,-0.8);
                    }
                }
                
            }
            DC_move(speedL, speedR);
            
        }//if(codestart = true)
        //pc.printf("mainon\n");
        //pc.printf("%i",Workm_time);
        //tt = Nowm_time;
        Workm_time = rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count() + (maincontroltime-(Workm_time-Nowm_time)));
    }//while
}//main
