#include "C:\Users\kgt22\Mbed Programs\practice\Header.h"
//---------------------------define-------------------------//


// uint16_t raw_array[MASK_LENGTH] = {0,};
// uint16_t raw_array_index = 0;
//----------------------------선언--------------------------//
InterruptIn btn(BUTTON1);
DigitalOut led1(LED1);
//-------thread용------//
//Thread psd_th;
uint64_t Now_time,Work_time,Nowm_time,Workm_time;
//---------------------//

AnalogIn irfl(PA_0);
AnalogIn irfr(PA_1);
AnalogIn irbl(PC_1);
AnalogIn irbr(PB_0);
AnalogIn irfm(PB_1);
AnalogIn irfmr(PA_4);
AnalogIn irfml(PC_0);
AnalogIn irbmr(PC_5); 
AnalogIn irbml(PA_5);
//AnalogIn irbml(PA_5);

GP2A psdfl(PC_3, 20, 150, 60, 0);
GP2A psdfr(PC_4, 20, 150, 60, 0);
//GP2A psdf(PA_1,7,80,22.5,0.1606);//실험할 때 쓴 psd값
GP2A psdb(PA_7,20,150,60,0);
GP2A psdfm(PA_6,20,150,60,0);


DigitalOut DirL(PC_7);
DigitalOut DirR(PB_6);
PwmOut PwmL(PB_4);
PwmOut PwmR(PB_5);
PwmOut rcServo(PA_8);

bool Serial_chk = false;
bool code_start = false;
volatile bool gotPacket = false;
volatile float data[3];

float filter_is_first = 1u;
float prev_data;
float now_data;

uint16_t black = 20000;//ir 검정색 바닥을 봤을 때
double speedL = 0;
double speedR = 0;
int mode =0;
float ang=90.0, inc=1.0, Inc=3.0, INC=5.0;
char preread;
float angL = 110;
float angR = 70;
float A,B,C,D;
float dis = 60; //data[1] 가까움 척도(세부조정 필요)
bool color;
int timer_mode=0;

uint16_t ir_val[10];
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

//0 fl
//1 fr
//2 bl
//3 br"//5 fmr
//6 fml
bool ir_plusval[8];
//0 fl+fr
//1 fmr + fml
//2 fl+fml
//3 fr+fmr
//4 bl+bml
//5 br+bmr
//6 원돌기 걸치는 ir
//7 all
double psdfl_val;
double psdfr_val;
double psdm_val;
float psdb_val;

Ticker control_tmr;
//Timer brk_tmr;
//Timer rotate_tmr;
//Timer tilt_tmr;
Timer test_tmr;
Timer tmove_tmr;
Timer be_tmr;
Timer re_tmr;

int turn_escape_time = 300000; // 세부조정 필요!!!
double back_escape_time = 1000; // 세부조정 필요!!!
int fight_back_escape_time = 350000; // 세부조정 필요!!!
int rotate_escape_time = 3000000; // 세부조정 필요!!!
int tilt_back_escape_time = 1500000; // 세부조정 필요!!!
double turn_time = 460;
int escape_time = 600;

RawSerial board(PA_9, PA_10, 115200);
RawSerial pc(USBTX,USBRX,115200);

//-----------------------------------------------------//

void psd_read(){
    prev_data = psdb.getDistance();
        while(true){
        Now_time = rtos::Kernel::get_ms_count();
        psdb_val = psdb.getDistance();

        
        now_data = (prev_data * Alpha) + ((1 - Alpha) * psdb_val);
        prev_data = now_data;

        //pc.printf("psdon\n");
        Work_time = rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count() + (psdcontroltime-(Work_time-Now_time)));
    }
}

void sensor_read(){
        ir_val[0] = irfl.read_u16();
        ir_val[1] = irfr.read_u16();
        ir_val[2] = irbl.read_u16();    
        ir_val[3] = irbr.read_u16();
        ir_val[4] = irfm.read_u16();
        ir_val[5] = irfmr.read_u16();
        ir_val[6] = irfml.read_u16();
        //ir_val[7] = irmm.read_u16();
        ir_val[7] = irbmr.read_u16();
        ir_val[8] = irbml.read_u16();
        //ir_val[9] = irbml.read_u16();

        psdfl_val = psdfl.getDistance();
        psdfr_val = psdfr.getDistance();
        //psdm_val = psdm.read_u16();
        //psdb_val = psdb.getDistance();
}

void sensor_plus(){
    if(ir_val[0]<black && ir_val[1]<black) ir_plusval[0] = true;//앞쪽 색영역
    else ir_plusval[0] = false;
    if(ir_val[5]<black && ir_val[6]<black) ir_plusval[1] = true;
    else ir_plusval[1] = false;
    if(ir_val[0]<black && ir_val[6]<black) ir_plusval[2] = true;
    else ir_plusval[2] = false; 
    if(ir_val[1]<black && ir_val[5]<black) ir_plusval[3] = true;
    else ir_plusval[3] = false;
    if(ir_val[2]<black && ir_val[9]<black) ir_plusval[4] = true;
    else ir_plusval[4] = false;
    if(ir_val[3]<black && ir_val[8]<black) ir_plusval[5] = true;
    else ir_plusval[5] = false;
    if(ir_val[1]>black && ir_val[0]<black) ir_plusval[6] = true;
    else ir_plusval[6] = false;
    if(ir_val[0]<black && ir_val[1]<black && ir_val[2]< black && ir_val[3]< black
         ){//ir_val[4]값 빠져있음
            ir_plusval[7]=true;
        }
    else ir_plusval[7] = false;
}

void sensor_print1(){
    pc.printf("ir_val1 : | %u | %u | %u | %u | %u | %u | %u |\n", ir_val[0], ir_val[1], ir_val[2], ir_val[3], ir_val[4], ir_val[5], ir_val[6]); // 확인용 코드
    //pc.printf("ir_WhCol : | %d | %d | %d | %d | %d | %d |\n", ir_plusval[0], ir_plusval[1], ir_plusval[2], ir_plusval[3], ir_plusval[4], ir_plusval[5]); // 확인용 코드
   //pc.printf("psdfl_val : | %lf |, psdfr_val : | %lf |, psdm_val : | %lf |, psdb_val : | %lf |\n", psdfl_val, psdfr_val,psdm_val,now_data); // 확인용 코드
}

void DC_set(){
    PwmL.period_us(66);
    PwmR.period_us(66);
}

void DC_move(float _PwmL, float _PwmR){
    if(_PwmL<0) DirL = 0;
    else DirL = 1;

    if(_PwmR<0) DirR = 0;
    else DirR = 1;

    PwmL = abs(_PwmL);
    PwmR = abs(_PwmR);
}

// void tmr_move(Timer* _timer,int* _tmr , double _speedL,double _speedR){
//         _timer -> start();
//         while(_timer ->read_us() < *_tmr){
//             speedL = _speedL;
//             speedR = _speedR;
//             whl_move();        
//             }

//         _timer -> reset();
//         _timer -> stop();
       
// }
// void tmr_reset(Timer* _timer){
//     _timer -> reset();
//     _timer -> stop();
// }


//연습용
void escape(Timer* _timer ,double* _tmr, double _speedL,double _speedR){
        if(timer_mode == 0){
            _timer -> start();
            timer_mode =1;
        }
        else if(timer_mode == 1){
            _timer -> reset();
        }
        
        while(_timer -> read_ms() < *_tmr){
            speedL = _speedL;
            speedR = _speedR;
            //whl_move();
            //servo_move(rcServo);
        }
        _timer -> reset();
}


void onSerialRx(){
    static char serialInBuffer[32];
    static int data_cnt=0,buff_cnt=0;
    Serial_chk = true;
    if(board.readable()){
        //pc.printf("data= %.3f, %.3f, %.3f\n\r",data[0],data[1],data[2]);
        char byteIn=board.getc();
        if(byteIn==','){
            serialInBuffer[buff_cnt]='\0';
            data[data_cnt++]=atof(serialInBuffer);
            buff_cnt=0;
        }
        else if(byteIn=='\n'){
            serialInBuffer[buff_cnt]='\0';
            data[data_cnt]=atof(serialInBuffer);
            buff_cnt=0;
            data_cnt=0;
            gotPacket=true;
        }
        else{
            serialInBuffer[buff_cnt++]=byteIn;
        }
    }
}

template <class T> T map(T x,T in_min,T in_max,T out_min,T out_max){
    return(x -in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
void servoturn(PwmOut&rc,float deg){
    uint16_t pulseW=map<float>(deg,0.,180.,600.,2400.);
    rc.pulsewidth_us(pulseW);
}

void servo_set(PwmOut &rc){
    rcServo.period_ms(10);
    servoturn(rcServo,90);
}

float Kp= 10.0 , Ki = 1.0 , Kd= 0.1 ;
float error_;
float error_previous;

float P_control, I_control, D_control ;
float control_time = 0.004;
float PID_control;

void servo_move(PwmOut &rc){
    int count;
    if(gotPacket){
        count = count+1;
        //pc.printf("data= %.3f, %.3f, %.3f, %C \n\r",data[0],data[1],data[2],preread);
        //board.printf("data= %.3f, %.3f, %.3f\n\r",data[0],data[1],data[2]);
        gotPacket = false;

        if (count % 10 == 0){
            //board.printf("data= %.3f, %.3f, %.3f\n\r",data[0],data[1],data[2]);
            count = 0;
        }

        if (data[0] >= 0 && data[0] <= 400) error_ = abs(data[0] - 200);

        P_control = Kp*error_;
        I_control = I_control+Ki*error_*control_time;
        D_control = Kd*(error_-error_previous)/control_time;

        PID_control = P_control + I_control + D_control;
        PID_control = map<float>(PID_control, 0. , 4000. , 0. ,16. );
        if (data[0] < 160 && ang>10) {
            ang-=PID_control;
            preread ='L';
        }
        else if (data[0] > 240 && data[0]<400 && ang<170) {
            ang+=PID_control;
            preread ='R';
        }
        if (data[0] == 999 && ang>10 && preread == 'L') {
            ang-=5;
            preread ='L';
        }
        else if (data[0] == 999 && ang<170 && preread == 'R') {
            ang+=5;
            preread ='R';
        }
        servoturn(rcServo,ang);
        
        error_previous = error_;
        //pc.printf("PID = %f, error_ = %f, ang = %f\n",PID_control,error_,ang);
    }
}

void DC_follow(){
        // if(data[1]<dis){
        //     float delang = map<float>(abs(ang-90), 0. , 80. ,0.0 ,0.4 );
        //     float dellen = map<float>(data[1], 0. , 250. ,0. ,0.05 );
        //     if(ang>130){//전부 다 최대치로 움직이게 변경 필요
        //         A = 1;
        //         B = -1;
        //         D=0;
        //         //C =0;
        //         //pc.printf("slow right\n\r");
        //     }
        //     else if(ang>=50 && ang<=130){
        //         A = 0;
        //         B = 0;
        //         D = 0.59;
        //         //C= 0.3;
        //         //pc.printf("middle\n\r");
        //     }
        //     else if(ang<50){
        //         A=-1;
        //         B=1;
        //         D=0;
        //         //C=0;
        //         //pc.printf("slow left\n\r");
        //     }
        //     speedL = 0.4+ A*delang+D;
        //     speedR = 0.4+ B*delang+D;
            //speedL = 0.7 + A*delang+ A*dellen+C;
            //speedR = 0.8 + B*delang + B*dellen+(C/0.3)*0.2;
            /*if(ang>10 && ang<70){//전부 다 최대치로 움직이게 변경 필요
                A = 0.5;
                B = -0.5;
            }
            else if(ang>=70 && ang<=110){
                A = 0.5;
                B = 0.5;
            }
            else if(ang>110 && ang<170){
                A=-0.5;
                B= 0.5;
            }
            speedL = A;
            speedR = B;//*/
        //}
        if(data[1]>=dis && (ang <=50 || ang>=130)){
            if(ang<=50){
                speedL = -0.4;
                speedR = 0.4;
                //pc.printf("fast right\n\r");
            }
            else if(ang>=130){
                speedL = 0.4;
                speedR = -0.4;
                //pc.printf("fast left\n\r");
            }
        }
        else if(data[1]>=dis && ang>=50 && ang<=130){
                speedL = 0.99;
                speedR = 0.99;
                //pc.printf("fast straight\n\r");
        }//*/
            // else if((ang <70 || ang>110) && data[1]>dis){
            //     DC_follow();
            // } //DC_follow로 들어오는 조건문
}


void Button_start(){
        code_start = true;
}

// float MovingAveragefilter(){
//     int i = 0;
//     uint16_t sum = 0;

//     for(i=0;i < MASK_LENGTH;i++){
//         sum += raw_array[i];
//     }
//     return((float)sum / MASK_LENGTH);
// }

// void insertintoRawArray(uint16_t value){
//     raw_array[raw_array_index] = value;
//     raw_array_index++;

//     if(raw_array_index >= MASK_LENGTH){
//         raw_array_index =0;
//     }
// }

// void LPF_Interrupt(){
//     if(1u == filter_is_first)
//     {
//     	prev_data = psdb_val;
//         filter_is_first = 0u;
//     }
    
//     now_data = (prev_data * Alpha) + ((1 - Alpha) * psdb_val);
// }

