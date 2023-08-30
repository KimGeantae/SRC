#include "C:\Users\kgt22\Mbed Programs\practice\Header.h"
#include "C:\Users\kgt22\Mbed Programs\practice\MPU9250\MPU9250.h"
#include <time.h>
//---------------------------define-------------------------//


// uint16_t raw_array[MASK_LENGTH] = {0,};
// uint16_t raw_array_index = 0;
//----------------------------선언--------------------------//
InterruptIn btn(BUTTON1);
DigitalOut led1(LED1);
static char serialInBuffer[32];
static int data_cnt=0,buff_cnt=0;
char byteIn;

//-------thread용------//
//Thread psd_th;
uint64_t Now_time,Work_time,Nowm_time,Workm_time,Nowi_time,Worki_time;
uint16_t chek_time;
//---------------------//
float x_deg = 4;
float y_deg = 4;
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

GP2A psdfl(PC_3, 0.1, 0.8,0.23625 ,-0.297);
GP2A psdfr(PC_4, 0.07, 0.8, 0.23625 ,-0.297);
//GP2A psdf(PA_1,7,80,22.5,0.1606);//실험할 때 쓴 psd값
GP2A psdb(PA_7,30,150,60,0);
GP2A psdfm(PA_6,0.07,0.8,0.23625 ,-0.297);


DigitalOut DirL(PC_7);
DigitalOut DirR(PB_6);
PwmOut PwmL(PB_4);
PwmOut PwmR(PB_5);
PwmOut rcServo(PA_8);

bool Serial_chk = false;
bool code_start = false;
volatile bool gotPacket = false;
volatile float data_R[3];

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
int escape_mode=0;
int Rescape_mode = 0;

uint16_t ir_val[9];
//0 fl
//1 fr
//2 bl
//3 br
//4 fm
//5 fmr
//6 fml
//7 bmr
//8 bml

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
double psdb_val;

Ticker control_tmr;
//Timer brk_tmr;
//Timer rotate_tmr;
Timer imu_tmr;
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

extern Thread imu_th;

RawSerial board(PA_9, PA_10, 115200);
RawSerial pc(USBTX,USBRX,115200);

//------------------------------mpu9250---------------------------------------//
float sum = 0;
uint32_t sumCount = 0;
char buffer[14];
float tmp_angle_x, tmp_angle_y, tmp_angle_z;
float filltered_angle_x, filltered_angle_y, filltered_angle_z;
float alpha = 0.90;

MPU9250 mpu9250(D14,D15); // SDA, SCL
Timer t;

int imu_count = 0;
//Serial pc(USBTX, USBRX, 115200); // tx, rx
//----------------------------------------------------------------------------//

//-----------------------------------------------------//

void psd_read(){
    prev_data = psdb.getDistance();
        while(true){
        Now_time = rtos::Kernel::get_ms_count();
       // pc.printf("%11u\n",chek_time - Now_time);
        psdb_val = psdb.getDistance();

        now_data = (prev_data * Alpha) + ((1 - Alpha) * psdb_val);
        prev_data = now_data;

        chek_time = Now_time;
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

        psdfl_val = psdfl.getDistance()*100;
        psdfr_val = psdfr.getDistance()*100;
        psdm_val = psdfm.getDistance()*100;
        //psdb_val = psdb.getDistance();
}

void sensor_plus(){
    if(ir_val[0]<black && ir_val[1]<black) ir_plusval[0] = true;//앞쪽 색영역
    else ir_plusval[0] = false;
    if(ir_val[5]<black && ir_val[6]<black) ir_plusval[1] = true;//중간앞 ir색영역
    else ir_plusval[1] = false;
    if(ir_val[0]<black && ir_val[6]<black) ir_plusval[2] = true;//fl,fml 색영역
    else ir_plusval[2] = false; 
    if(ir_val[1]<black && ir_val[5]<black) ir_plusval[3] = true;//fr,fmr색영역
    else ir_plusval[3] = false;
    if(ir_val[2]<black && ir_val[8]<black) ir_plusval[4] = true;//bl ,bml 색영역
    else ir_plusval[4] = false;
    if(ir_val[3]<black && ir_val[7]<black) ir_plusval[5] = true;//br, bmr 색영역
    else ir_plusval[5] = false;
    if(ir_val[1]>black && ir_val[0]<black) ir_plusval[6] = true;//fr -> black, fl -> 색영역
    else ir_plusval[6] = false;
    if(ir_val[0]<black && ir_val[1]<black && ir_val[2]< black && ir_val[3]< black && ir_val[4]<black
         ){//ir_val[4]값 빠져있음
            ir_plusval[7]=true;
        }
    else ir_plusval[7] = false;
}

void sensor_print1(){
    //pc.printf("ir_val1 : | %u | %u | %u | %u | %u | %u | %u |\n", ir_val[0], ir_val[1], ir_val[2], ir_val[3], ir_val[4], ir_val[5], ir_val[6]); // 확인용 코드
    //pc.printf("ir_WhCol : | %d | %d | %d | %d | %d | %d |\n", ir_plusval[0], ir_plusval[1], ir_plusval[2], ir_plusval[3], ir_plusval[4], ir_plusval[5]); // 확인용 코드
   pc.printf("psdfl_val : | %lf |, psdfr_val : | %lf |, psdm_val : | %lf |, psdb_val : | %lf |\n", psdfl_val, psdfr_val,psdm_val,now_data); // 확인용 코드
   //pc.printf("now_data : | %lf |, psdb_val : | %lf |\n", now_data,psdb_val);
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

void Rescape_move(Timer* _timer, int*_tmr, double _speedL,double _speedR){
        if(Rescape_mode == 0){
            _timer -> start();
            Rescape_mode =1;
        }
        else if(Rescape_mode == 1){
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


//연습용
void escape(Timer* _timer ,double* _tmr, double _speedL,double _speedR){
        if(escape_mode == 0){
            _timer -> start();
            escape_mode =1;
        }
        else if(escape_mode == 1){
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
    Serial_chk = true;
    if(board.readable()){
        //pc.printf("data= %.3f, %.3f, %.3f\n\r",data[0],data[1],data[2]);
        byteIn=board.getc();
        if(byteIn==','){
            serialInBuffer[buff_cnt]='\0';
            data_R[data_cnt++]=atof(serialInBuffer);
            buff_cnt=0;
        }
        else if(byteIn=='\n'){
            serialInBuffer[buff_cnt]='\0';
            data_R[data_cnt]=atof(serialInBuffer);
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

float kp= 10.0 , ki = 1.0 , kd= 0.1 ;
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

        if (data_R[0] >= 0 && data_R[0] <= 400) error_ = abs(data_R[0] - 200);

        P_control = kp*error_;
        I_control = I_control+ki*error_*control_time;
        D_control = kd*(error_-error_previous)/control_time;

        PID_control = P_control + I_control + D_control;
        PID_control = map<float>(PID_control, 0. , 4000. , 0. ,16. );
        if (data_R[0] < 160 && ang>10) {
            ang-=PID_control;
            preread ='L';
        }
        else if (data_R[0] > 240 && data_R[0]<400 && ang<170) {
            ang+=PID_control;
            preread ='R';
        }
        if (data_R[0] == 999 && ang>10 && preread == 'L') {
            ang-=5;
            preread ='L';
        }
        else if (data_R[0] == 999 && ang<170 && preread == 'R') {
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
        if(data_R[1]>=dis && (ang <=50 || ang>=130)){
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
        else if(data_R[1]>=dis && ang>=50 && ang<=130){
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

void setup_mpu9250(){
    uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    // pc.printf("I AM 0x%x\t", whoami); pc.printf("I SHOULD BE 0x71\n\r");

    mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
    mpu9250.MPU9250SelfTest(mpu9250.SelfTest); // Start by performing self test and reporting values 
    
    mpu9250.calibrateMPU9250(mpu9250.gyroBias, mpu9250.accelBias); 
    // Calibrate gyro and accelerometers, load biases in bias registers  

    mpu9250.initMPU9250();
    // -6050확인용 mpu9250.initAK8963(mpu9250.magCalibration);

    mpu9250.getAres(); // Get accelerometer sensitivity
    mpu9250.getGres(); // Get gyro sensitivity
    // -6050확인용 mpu9250.getMres(); // Get magnetometer sensitivity

}

void imu_main(){
    //while(true){
        // If intPin goes high, all data registers have new data
        if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
        {  // On interrupt, check if data ready interrupt
            //pc.printf("imu main     ");
            mpu9250.readAccelData(mpu9250.accelCount);  // Read the x/y/z adc values   
            // Now we'll calculate the accleration value into actual g's
            mpu9250.ax = (float)mpu9250.accelCount[0]*mpu9250.aRes - mpu9250.accelBias[0];  // get actual g value, this depends on scale being set
            mpu9250.ay = (float)mpu9250.accelCount[1]*mpu9250.aRes - mpu9250.accelBias[1];   
            mpu9250.az = (float)mpu9250.accelCount[2]*mpu9250.aRes - mpu9250.accelBias[2];  
   
            mpu9250.readGyroData(mpu9250.gyroCount);  // Read the x/y/z adc values
            // Calculate the gyro value into actual degrees per second
            mpu9250.gx = (float)mpu9250.gyroCount[0]*mpu9250.gRes - mpu9250.gyroBias[0];  // get actual gyro value, this depends on scale being set
            mpu9250.gy = (float)mpu9250.gyroCount[1]*mpu9250.gRes - mpu9250.gyroBias[1];  
            mpu9250.gz = (float)mpu9250.gyroCount[2]*mpu9250.gRes - mpu9250.gyroBias[2];   
  
            // -6050확인용 mpu9250.readMagData(mpu9250.magCount);  // Read the x/y/z adc values   
            // // Calculate the magnetometer values in milliGauss
            // // Include factory calibration per data sheet and user environmental corrections
            // mpu9250.mx = (float)mpu9250.magCount[0]*mpu9250.mRes*mpu9250.magCalibration[0] - mpu9250.magbias[0];  // get actual magnetometer value, this depends on scale being set
            // mpu9250.my = (float)mpu9250.magCount[1]*mpu9250.mRes*mpu9250.magCalibration[1] - mpu9250.magbias[1];  
            // mpu9250.mz = (float)mpu9250.magCount[2]*mpu9250.mRes*mpu9250.magCalibration[2] - mpu9250.magbias[2];   
        }
        //pc.printf("%c\n",mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS));

        //pc.printf("no if\n");
        mpu9250.Now = t.read_us();
        mpu9250.deltat = (float)((mpu9250.Now - mpu9250.lastUpdate)/1000000.0f) ; 
        // set integration time by time elapsed since last filter update
        mpu9250.lastUpdate = mpu9250.Now;
    
        sum += mpu9250.deltat;
        sumCount++;
    
        // Serial print and/or display at 0.5 s rate independent of data rates
        //mpu9250.delt_t = t.read_ms() - mpu9250.count;

        mpu9250.roll = atan2(mpu9250.ay, sqrt(mpu9250.ax * mpu9250.ax + mpu9250.az * mpu9250.az)) * (180.0 / PI);
        mpu9250.pitch = atan2(-mpu9250.ax, sqrt(mpu9250.ay * mpu9250.ay + mpu9250.az * mpu9250.az)) * (180.0 / PI);

        

        //temp filter - gyro적분
        // tmp_angle_x = filltered_angle_x + mpu9250.gx * mpu9250.deltat;
        // tmp_angle_y = filltered_angle_y + mpu9250.gy * mpu9250.deltat;
        // tmp_angle_z = filltered_angle_z + mpu9250.gz * mpu9250.deltat;

        tmp_angle_x = filltered_angle_x + mpu9250.gx * 0.02;
        tmp_angle_y = filltered_angle_y + mpu9250.gy * 0.02;
        tmp_angle_z = filltered_angle_z + mpu9250.gz * 0.02;

        //alpha를 이용한 보정(상보)
        filltered_angle_x = alpha * tmp_angle_x + (1.0-alpha) * mpu9250.roll;
        filltered_angle_y = alpha * tmp_angle_y + (1.0-alpha) * mpu9250.pitch;
        // filltered_angle_z = tmp_angle_z;



        mpu9250.count = t.read_ms(); 

        if(mpu9250.count > 1<<21) {
            t.start(); // start the timer over again if ~30 minutes has passed
            mpu9250.count = 0;
            mpu9250.deltat= 0;
            //t.reset();
            mpu9250.lastUpdate = t.read_us();
        }
        sum = 0;
        sumCount = 0; 
    //}
    

}


void imu_read(){
    // imu_th.set_priority(osPriorityNormal);
    t.start();
    setup_mpu9250();
    while (true){
        // ThisThread::sleep_for(7);
        Nowi_time = rtos::Kernel::get_ms_count();
        //pc.printf("%11u ms\n", -(chek_time - Nowi_time));

        imu_main();

        //chek_time = Nowi_time;
        Worki_time = rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count() + (imu_time-(Worki_time - Nowi_time)));
    }
}
