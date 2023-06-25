#include "C:\Users\kgt22\Mbed Programs\practice\Header.h"

InterruptIn btn(BUTTON1);
DigitalOut led1(LED1);

AnalogIn irfl(PA_0);
AnalogIn irfr(PA_1);
AnalogIn irbl(PC_1);
AnalogIn irbr(PB_0);
AnalogIn irfm(PB_1);
AnalogIn irfmr(PC_2);
AnalogIn irfml(PC_0);
//AnalogIn irmm(PA_6);
AnalogIn irbmr(PC_5); 
AnalogIn irbml(PA_5);

GP2A psdfl(PC_3, 30, 150, 60, 0);
//AnalogIn psdfl(PC_2);
AnalogIn psdfr(PC_3);
AnalogIn psdm(PC_4);
AnalogIn psdb(PC_5);

DigitalOut DirL(PC_7);
DigitalOut DirR(PB_6);
PwmOut PwmL(PB_4);
PwmOut PwmR(PB_5);
PwmOut rcServo(PA_8);

bool Serial_chk = false;
bool code_start = false;
volatile bool gotPacket = false;
volatile float data[3];

uint16_t black = 65000;//ir 검정색 바닥을 봤을 때
double speedL = 0;
double speedR = 0;
int mode =0;
float ang=90.0, inc=1.0, Inc=3.0, INC=5.0;
char preread;

float angL = 110;
float angR = 70;
float A,B,C;
float dis = 50; //data[1] 가까움 척도(세부조정 필요)


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
//3 br
//5 fmr
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
double psdb_val;

Timer control_tmr;
//Timer brk_tmr;
//Timer rotate_tmr;
//Timer tilt_tmr;
Timer test_tmr;
Timer tmove_tmr;

int turn_escape_time = 1000000; // 세부조정 필요!!!
int back_escape_time = 1000000; // 세부조정 필요!!!
int fight_back_escape_time = 350000; // 세부조정 필요!!!
int rotate_escape_time = 3000000; // 세부조정 필요!!!
int tilt_back_escape_time = 1500000; // 세부조정 필요!!!
int turn_time = 610000;
int test_time = 600000;

RawSerial board(PA_9, PA_10, 115200);
RawSerial pc(USBTX,USBRX,115200);

GP2A psdf(PA_1,7,80,22.5,0.1606);

void sensor_read(){
        ir_val[0] = irfl.read_u16();
        ir_val[1] = irfr.read_u16();
        ir_val[2] = irbl.read_u16();    
        ir_val[3] = irbr.read_u16();
        ir_val[4] = irfm.read_u16();
        ir_val[5] = irfmr.read_u16();
        ir_val[6] = irfml.read_u16();
        //ir_val[7] = irmm.read_u16();
        ir_val[8] = irbmr.read_u16();
        ir_val[9] = irbml.read_u16();

        psdfl_val = psdfl.getDistance();
        psdfr_val = psdfr.read_u16();
        psdm_val = psdm.read_u16();
        psdb_val = psdb.read_u16();
}

void sensor_plus(){
    if(ir_val[0]<black && ir_val[1]<black) ir_plusval[0] = true;
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
    if(ir_val[0]<black && ir_val[1]< black && ir_val[2]< black && ir_val[3]< black
        && ir_val[5]< black && ir_val[6]< black){//ir_val[4]값 빠져있음
            ir_plusval[7]=true;
        }
    else ir_plusval[7] = false;
}

void sensor_print1(){
    pc.printf("ir_val1 : | %u | %u | %u | %u | %u | %u | %u |\n", ir_val[0], ir_val[1], ir_val[2], ir_val[3], ir_val[4], ir_val[5], ir_val[6]); // 확인용 코드
    //pc.printf("ir_WhCol : | %d | %d | %d | %d | %d | %d |\n", ir_plusval[0], ir_plusval[1], ir_plusval[2], ir_plusval[3], ir_plusval[4], ir_plusval[5]); // 확인용 코드
    pc.printf("psdfl_val : | %lf |, psdfr_val : | %lf |, psdm_val : | %lf |, psdb_val : | %lf |\n", psdfl_val, psdfr_val,psdm_val,psdb_val); // 확인용 코드
}
void sensor_print2(){
    //pc.printf("ir_val2 : | %u | %u | %u | %u | %u |  %u |  %u |\n", ir_val[0], ir_val[1], ir_val[2], ir_val[3], ir_val[4], ir_val[5], ir_val[6]); // 확인용 코드
    //pc.printf("ir_WhCol : | %d | %d | %d | %d | %d | %d |\n", ir_plusval[0], ir_plusval[1], ir_plusval[2], ir_plusval[3], ir_plusval[4], ir_plusval[5]); // 확인용 코드
    pc.printf("psdfl_val : | %lf |, psdfr_val : | %lf |, psdm_val : | %lf |, psdb_val : | %lf |\n", psdfl_val, psdfr_val,psdm_val,psdb_val); // 확인용 코드
}
void sensor_print3(){
    pc.printf("ir_val3 :| %u | %u | %u | %u | %u |  %u |  %u |\n", ir_val[0], ir_val[1], ir_val[2], ir_val[3], ir_val[4], ir_val[5], ir_val[6]); // 확인용 코드
    //pc.printf("ir_WhCol : | %d | %d | %d | %d | %d | %d |\n", ir_plusval[0], ir_plusval[1], ir_plusval[2], ir_plusval[3], ir_plusval[4], ir_plusval[5]); // 확인용 코드
    pc.printf("psdfl_val : | %lf |, psdfr_val : | %lf |, psdm_val : | %lf |, psdb_val : | %lf |\n", psdfl_val, psdfr_val,psdm_val,psdb_val); // 확인용 코드
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

void tmr_move(Timer* _timer,int* _tmr , double _speedL,double _speedR){
        _timer -> start();
        while(_timer ->read_us() < *_tmr){
            speedL = _speedL;
            speedR = _speedR;
            whl_move();        
            }

        _timer -> reset();
        _timer -> stop();
       
}
void tmr_reset(Timer* _timer){
    _timer -> reset();
    _timer -> stop();
}


//연습용
void test_move(Timer* __timer ,int* __tmr, double __speedL,double __speedR){
    __timer -> start();
    while(__timer -> read_us() < *__tmr){
        speedL = __speedL;
        speedR = __speedR;
        whl_move();
    }


    __timer -> reset();
    __timer -> stop();
}

void whl_move(){
    //in_SerialRx_main(); // interrupt 전용

    //sensor_read();
    //sensor_cal();
    // sensor_print(); // 확인용 코드

    //servo_move(Servo);
    DC_move(speedL, speedR);

    //all_print();
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

        if (data[0] < 160){//왼쪽일 때 시그모이드 제어
            float x = map<float>(data[0], 0. , 160. ,6. ,-6. ), y = exp(x) / (exp(x) + 1.);
            inc = 5*y;
            servoturn(rcServo,ang);
            if(ang<170) ang+=inc;
            preread = 'L';
        }
            
        else if(160 <= data[0] && data[0] < 240){//중간
            preread = 'M';
        }

        else if (data[0] >= 240 && data[0]<400){//오른쪽일 때 시그모이드 제어
            float x = map<float>(data[0], 240. , 400. ,-6. ,6. ), y = exp(x) / (exp(x) + 1.);
            inc = 5*y;
            servoturn(rcServo,ang);
            if(ang>10) ang-=inc;
            preread = 'R';
        }

        else if(data[0] == 999 && preread == 'L') {//왼쪽에서 적 사라졌을 때
            servoturn(rcServo,170);
            if(ang<170) ang+=INC;
            preread = 'L';
        }

        else if(data[0] == 999 && preread == 'R'){//오른쪽에서 적 사라졌을 때
            servoturn(rcServo,ang);
            if(ang>10) ang-=INC;
            preread = 'R';
        }

        // if(ang>180.f){
        //     ang = 180.0;
        // }
        // else if(ang < 0.f){
        //     ang= 0.0;
        // }
    }
}

void DC_follow(){
        if(data[1]<dis){
            float delang = map<float>(abs(ang-90), 0. , 80. ,0.1 ,0.2 );
            float dellen = map<float>(data[1], 0. , 250. ,0. ,0.05 );
            // if(ang>10 && ang<70){//전부 다 최대치로 움직이게 변경 필요
            //     A = 1;
            //     B = -1.5;
            //     C =0;
            // }
            // else if(ang>=70 && ang<=110){
            //     A = 0;
            //     B = 0;
            //     C= 0.3;
            // }
            // else if(ang>110 && ang<170){
            //     A=-1.5;
            //     B=1;
            //     C=0;
            // }
            // speedL = 0.7 + A*delang+ A*dellen+C;
            // speedR = 0.8 + B*delang + B*dellen+(C/0.3)*0.2;
            if(ang>10 && ang<70){//전부 다 최대치로 움직이게 변경 필요
                A = 1;
                B = -1;
                C =0;
            }
            else if(ang>=70 && ang<=110){
                A = 1;
                B = 1;
                C= 0.3;
            }
            else if(ang>110 && ang<170){
                A=-1;
                B= 1;
                C=0;
            }
            speedL = A;
            speedR = B;
        }
        else if(data[1]>=dis && (ang <=70 || ang>=110)){
            if(ang<=70){
                speedL = 0.4;
                speedR = -1;
            }
            else if(ang>=110){
                speedL = -1;
                speedR = 0.4;
            }
        }
        else if(data[1]>=dis && ang>=70 && ang<=110){
                speedL =1;
                speedR = 1;
        }
        
}

void Button_start(){
        code_start = true;
}



