#include "I2Cdev.h"
#include "Wire.h"
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include "adaptive.h"
#include "control.h"
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

// // --- Индексы  для углов ----
// # define YAW         0
// # define PITCH       1
// # define ROLL        2

// --- Для прерывания ---
#define INTERRUPT_PIN 2         // пин 2 для прерывания
volatile bool mpuFlag = false;  // флаг прерывания готовности

// Объявления IMU
MPU6050 mpu;

/***** ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ *******/// --> лучше делать некторых из них локальными??????
// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8);  // using pin 7 for the CE pin, and pin 8 for the CSN pin
// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
bool role = false;  // true = TX role, false = RX role

float payload_RX[5] ;//receiver from controller
float payload_TX[5] ;//send to pc

// Характеристики пропеллера (коэффициенты сил и моментов)
float kTM=51.0; 
// Плечо сил тяги (для вычисления момента силы тяги)
float Lq=0.18; //18см  - большой коптер
//float Lq=0.05; //18см  - большой коптер

// Силы тяги (4 пропеллера)
float T1=0.0,T2=0.0,T3=0.0,T4=0.0;

//-------- ПИД регулятры -------------
// заданное значение периода работы алгоритма (в мс)
int TPID;        
// период работы алгоритма (в с)
float TAU;
// сигналы управления по 4 каналам - высота, тангаж, крен, рыскание 
float uh=0.0,uY[3]={0.0},uP[3]={0.0},uR[3]={0.0};
// значение, формируемое ПИД регуляторами (моменты по 3 угловым каналам)
float Myaw, Mpitch, Mroll;
float Mroll_filter=0.0;
// коэффициенты цифрового ПИД регулятора по угловым каналам [yaw, pitch, roll]
float qP[5]={0.0},qR[5]={0.0},qY[5]={00.0};
// вектор оценок углов [yaw, pitch, roll]
float theta[3]={0.0}, omg[3]={0.0},omg_prev[3]={0.0};
float gyrox[2]={0.0}, gyroy[2]={0.0}, gyroz[2]={0.0};
// заданное значение углов Эйлера [yaw, pitch, roll]
float thetaDesired[3]={0.0}, omgDesired[3]= {0.0};
// Текущая ошибка управления по углам [yaw, pitch, roll]
float thetaErrs[3],omgErrs[3];
// Вектор ошибок по углам 
float yawErr[3]={0.0}, pitchErr[3]={0.0},rollErr[3]={0.0};
float yawRateErr[3]={0.0}, pitchRateErr[3]={0.0},rollRateErr[3]={0.0};

//--------- для работы с DMP --------------
// Для тайминга
uint32_t startMillis;
// таймер
uint32_t tmr,tmr_motion;
// адрес обращения
uint8_t IMUAddress = 0x68;
// буффер для считывания данных из DMP
uint8_t fifoBuffer[64];
// кватернион поворота
Quaternion q;
// вектор гравитационного ускорения
VectorFloat gravity;
// вектор текущих показаний ГИРО и АССЕЛ
int16_t GYR[3];
VectorInt16 ACC,ACCb,ACCi;
float DV[3], V[3], R[3];
// ----- для управления моторами ----------------
// ШИМ сигналы управления моторами
int PWM1, PWM2, PWM3, PWM4;
// параметры кривой PWM-T
//float pMotor[3]={4.513E-6,3.332E-3,-63.35E-3};
float pMotor[3]={4.03E-6,2.69E-3,-45.30E-3};

// мин макс тяги
int THROTTLE_MINIMUM = 1000;                        
int THROTTLE_MAXIMUM = 2000; 
// программная тяга (начальная)
int THROLLTE0 = 1100;  
// мотора
Servo motor_1; // Motor front right
Servo motor_2; // Motor front left
Servo motor_3; // Motor back left
Servo motor_4; // Motor back right
// флаг, для выключения двигателей 
int flag=0;
//для управления вводом с клавиатуры
int comportin=0;

//ADAPTIVECONTROL
float unn=0.0;
float vyr[5]={0.0},vwr[5]={0.0},vur[5]={0.0},cgl=0.0,an[5]={0.0}, bn[5]={0.0},pr[5]={0.0},qr[5]={0.0},pm[25]={0.0};
int mp, mq;
float alpha[3]={0.0};
float omgAD[3];
float MrollAD;
float dMroll=0.0,dMpitch=0.0,dMyaw=0.0;
/********************************************************************
********************** ПОДГОТОВКА К ЗАПУСКУ *************************
********************************************************************/
void setup() {

  // стандартная 
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);
  Serial.begin(115200);
  while (!Serial); 
  
  Serial.println("Press 1 to start!");
  while(Serial.read() != '1');
  Serial.println("Start!");

  // ---- MPU ------
  // Инициализация mpu
  Serial.println("Initialize MPU!");
  mpu.initialize();
  // There MUST be some delay between mpu.initialize() and mpu.dmpInitialize(); 
  // Typically, the delay should be longer that 5ms. Otherwise you will get a garbage sensor.
  delay(10);   
  // Инициализация DMP
  Serial.println("Initialize DMP!");
  mpu.dmpInitialize();  // в этой функции включается фильтр, задается gyro range ....
  
  // Автоматическая калибровка АССЕЛ и ГИРО

  // Serial.println("Calibrate Accels!");
  // mpu.CalibrateAccel(6); 
  // Serial.println("Calibrate Gyros!");
  // mpu.CalibrateGyro(6);
  // mpu.PrintActiveOffsets(); 


  // Ручная калибровка (ставим оффсеты из памяти)
  mpu.setXAccelOffset(226);
  mpu.setYAccelOffset(209);
  mpu.setZAccelOffset(2044);
  mpu.setXGyroOffset(88);
  mpu.setYGyroOffset(46);
  mpu.setZGyroOffset(45);



  // включить DMP
  mpu.setDMPEnabled(true);
  // инициализация прерывания
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpReady, RISING);
  

  //mpu.setIntZeroMotionEnabled(true);
  // ---- Для моторов  -------------
  // motor_1.attach(12);                                
  // motor_2.attach(11); 
  // motor_3.attach(10); 
  // motor_4.attach(9); 

  motor_1.attach(12);                                
  motor_2.attach(11);    //9 
  motor_3.attach(10);   //10
  motor_4.attach(9);  //11

  
  //Калибровка моторов ???
  //calibrateMotors();
  armMotors();

  Serial.println("SETUP: Motors calibrated");
  Init_NRF24();

  // ------ Инициализация параметров ПИД регуляторов ------------------
  // Период вызова алгоритма управления 
  TPID=8000;                  // в мкс (8мс)
  TAU= 0.01;                  // в с  (10мс)
  // вычисление коэффициентов цифровых ПИД регуляторов
  
  DIGPIDCOEFF1(qP,0.15,0.13,0.029,TAU); // Pitch  
  DIGPIDCOEFF1(qR,0.58,0.1,0.04,TAU);        //Roll
  DIGPIDCOEFF1(qY,1.42,0.001,0.001,TAU);       //Yaw

  // расчет альфа для АУ
  SetAlpha(1.0,0.71, TAU,alpha);  

  //инициализация состояния ЛА (Для БИНС)
//  V[0]=0.0;V[1]=0.0;V[2]=0.0;
//  R[0]=0.0;R[1]=0.0;R[2]=0.0;

  //инициализация адаптивного управления
  Serial.println("Init adaptive control!");
  initcntrl(vur, vyr, vwr,  an, bn, pr, qr,cgl,pm, mp, mq);

  //финиш
  Serial.println("SETUP: Finished");

  // для начала считывания времени
  startMillis = millis();   
  tmr=micros();  
  tmr_motion=millis();
}

// прерывание готовности. Поднимаем флаг
void dmpReady() {
  mpuFlag = true;
}

/********************************************************************
********************** ГЛАВНАЯ ПРОГРАММА ****************************
********************************************************************/
void loop() {  
    // Время, считанное с последнего момента получения данных с DMP
    static uint32_t DLT, TM, DTL_MOTION,flagYAW0=0;
    float DT;
    uint8_t zerotime;
    DLT=micros()-tmr;   
    TM=millis()-startMillis;
    DTL_MOTION=millis()-tmr_motion;
    Read_NRF_value();
    comportin=Serial.read();
    if (comportin=='2')
    {
      flag=1;
      Serial.println("Stop!");
    }
    
    if (comportin=='3')
    {
      flag=0;
      Serial.println("Restart!");
    }

    // При готовности DMP и превышении интервала ждания
    if (DLT>=TPID && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
        // сброс таймера  
        DT=(micros()-tmr)*1.E-6; 
        tmr = micros();           
        // получение данных с DMP (кватернион)   
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        // вычисление гравитационного ускорения
        mpu.dmpGetGravity(&gravity, &q);
        // вычисление углов
        mpu.dmpGetYawPitchRoll(theta, &q, &gravity);        
        // получение показаний ГИРО
        mpu.dmpGetGyro(GYR,fifoBuffer);
        // расчет и фильтрация угловой скорости 
        ANGLERATEFILTER(GYR, omg,DT,40.0);           
    
        // задающие сигналы (при стабилизации они равны 0)
/*        if (flagYAW0==0)
        {
            thetaDesired[YAW]=theta[YAW];
            flagYAW0=1;
        }
*/
        thetaDesired[YAW]=0.0;
        thetaDesired[PITCH]=0.0;
        thetaDesired[ROLL]=0.0*M_PI/180;
        omgDesired[YAW]=0.0;omgDesired[PITCH]=0.0;omgDesired[ROLL]=0.0;

       if (TM>2000)
          thetaDesired[PITCH]=PROGCONTROL(TM-2000); 
        // закон изменения управляемого парамета 
        
        // ПИД по угловым скоростям
        GetErrs(thetaDesired, theta, thetaErrs);
        //SignalLMT(thetaErrs,30.0);
        //SignalDEADZONE(thetaErrs,1.0);
        FindAngleRateDesired(thetaErrs, omgDesired);

//        if (TM>=10000)
//          omgDesired[ROLL]=PROGCONTROL(TM);

        GetErrs(omgDesired, omg, omgErrs);
        //SignalDEADZONE(omgErrs,5.0);
      
        vectorstore(yawRateErr,omgErrs[YAW]); 
        vectorstore(pitchRateErr,omgErrs[PITCH]); 
        vectorstore(rollRateErr,omgErrs[ROLL]); 

        // Выполнение алгоритмов ПИД регулвторов
        PID_DIGITAL(qY,yawRateErr,uY);    
        PID_DIGITAL(qP,pitchRateErr,uP);
        PID_DIGITAL(qR,rollRateErr,uR);

        // Пересчет требуемого значения моментов (с учетом физическими ограничениями)
        Myaw   = SATURATION(uY[0], 2.5, -2.5);        //рыскание
        Mpitch = SATURATION(uP[0], 2.5, -2.5);        //тангаж
        Mroll  = SATURATION(uR[0], 2.5, -2.5);        //крен

        // Для тестирования выключаем некоторые каналы
        uh=0.0; 
        Myaw=0.0;
        Mroll=0.0;
      //  Mpitch=0.0;

        //фильтрация моментов
    //    MrollAD=DigLowPassFil(Mroll_filter, uR[0], uR[1],DT,10.0);
    //    Mroll_filter=MrollAD;

        // Идентификация
    //    ANGLERATEFILTER(GYR, omgAD,DT,5.0);
    //    IDENT(omgAD[ROLL], MrollAD, vyr, vur,cgl, an, bn,pm); 

        //расстояние от ц.м. до оси вращения L=0.0253; масса дрона 1.148кг, dM0=0.025
        //dMroll=0.0253*sin(theta[ROLL])*1.148*9.81+0.025;

        //pitch: расстояние от ц.м. до оси вращения L=-0.0067; масса дрона 1.148кг, dM0=-0.049
        //dMpitch=-0.05-0.0067*sin(theta[PITCH])*1.148*9.81;
  
        dMyaw=0.0; dMpitch=-0.018; dMroll=0.0;

        // if (TM>2000)
        //    Mpitch=PROGCONTROL1(TM-2000);

        // Расчет требуемого значения сил тяги для всех моторов
        T1=0.25*uh+0.25*(Mpitch+dMpitch)/Lq+0.25*(Mroll+dMroll)/Lq+0.25*Myaw*kTM;
        T2=0.25*uh+0.25*(Mpitch+dMpitch)/Lq-0.25*(Mroll+dMroll)/Lq-0.25*Myaw*kTM;
        T3=0.25*uh-0.25*(Mpitch+dMpitch)/Lq-0.25*(Mroll+dMroll)/Lq+0.25*Myaw*kTM;
        T4=0.25*uh-0.25*(Mpitch+dMpitch)/Lq+0.25*(Mroll+dMroll)/Lq-0.25*Myaw*kTM;

        // uint32_t tm_tmp = TM % 6000;
        // if (TM<=2000)
        //     THROLLTE0=1300;
        //  if (TM>2000 && TM<6000) {
        //    THROLLTE0=1300+int((400.0/4000)*(TM-2000));
        //  }
        THROLLTE0=1300;

        // Расчет ШИМ сигналов для моторов (по кривой PWM-T)
        PWM1=T2PWM(T1,THROLLTE0,THROTTLE_MINIMUM,THROTTLE_MAXIMUM,pMotor) ;
        PWM2=T2PWM(T2,THROLLTE0,THROTTLE_MINIMUM,THROTTLE_MAXIMUM,pMotor) ;
        PWM3=T2PWM(T3,THROLLTE0,THROTTLE_MINIMUM,THROTTLE_MAXIMUM,pMotor) ;
        PWM4=T2PWM(T4,THROLLTE0,THROTTLE_MINIMUM,THROTTLE_MAXIMUM,pMotor) ;

        // Печать  
        if (flag==0)      
            PRINTRES();
            Send_NRF_value();
    }//if ( TAU>=TPID && mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    

    // if (TM>20000)
    //     flag = 1 ;
    
    //Контроль работы моторов (выключаем все после несколько секунд)
    if (abs(theta[PITCH])>=45*M_PI/180 || abs(theta[ROLL])>=45*M_PI/180 || abs(theta[YAW])>=90*M_PI/180)
    {
        flag = 1 ;
    }

    if (flag==1)
    {
      StopDrone();
    }

    // Передаем ШИМ сигналы в мотора
    motor_1.writeMicroseconds(PWM1);
    motor_2.writeMicroseconds(PWM2);
    motor_3.writeMicroseconds(PWM3);
    motor_4.writeMicroseconds(PWM4);

}// end loop

/************ ВСМОПОГАТЕЛЬНЫЕ ФУНКЦИИ ****************************/
void PRINTRES()
{
  
  int INDX=PITCH;
/*
  Serial.print(millis()); Serial.print(",");
  Serial.print(theta[INDX] * 180.0 / M_PI);Serial.print(",");
  Serial.print(omg[INDX] * 180.0 / M_PI);Serial.print(",");
  Serial.print(Mpitch); 
*/
  
  Serial.print(Mpitch); Serial.print(",");
  Serial.print(thetaDesired[INDX] * 180.0 / M_PI); Serial.print(",");
  Serial.print(theta[INDX] * 180.0 / M_PI); Serial.print(",");
  Serial.print(omgDesired[INDX] * 180.0 / M_PI); Serial.print(",");
  Serial.print(omg[INDX] * 180.0 / M_PI);//Serial.print(",");
  

/*  
  Serial.print(thetaDesired[PITCH] * 180.0 / M_PI); Serial.print(",");
  Serial.print(thetaDesired[ROLL] * 180.0 / M_PI); Serial.print(",");
  Serial.print(thetaDesired[YAW] * 180.0 / M_PI); Serial.print(",");
  Serial.print(theta[PITCH] * 180.0 / M_PI); Serial.print(",");
  Serial.print(theta[ROLL] * 180.0 / M_PI); Serial.print(",");
  Serial.print(theta[YAW] * 180.0 / M_PI); ;Serial.print(",");
  Serial.print(Mpitch); ;Serial.print(",");
  Serial.print(Mroll); ;Serial.print(",");
  Serial.print(Myaw);Serial.print(",");
  Serial.print(omg[PITCH] * 180.0 / M_PI);Serial.print(",");
  Serial.print(omg[ROLL] * 180.0 / M_PI);Serial.print(",");
  Serial.print(omg[YAW] * 180.0 / M_PI);Serial.print(",");
  Serial.print(omgDesired[PITCH] * 180.0 / M_PI);Serial.print(",");
  Serial.print(omgDesired[ROLL] * 180.0 / M_PI);Serial.print(",");
  Serial.print(omgDesired[YAW] * 180.0 / M_PI);
*/
/*
  int INDX=YAW;
  
  Serial.print(millis()); Serial.print(",");
  Serial.print(theta[INDX] * 180.0 / M_PI);Serial.print(",");
  Serial.print(omg[INDX] * 180.0 / M_PI);Serial.print(",");
  Serial.print(Myaw); ;//Serial.print(",");
*/
  Serial.println();
}

void SignalLMT(float x[], float threshold)
{
    x[0] = LMT(x[0],threshold*M_PI/180);    
    x[1] = LMT(x[1],threshold*M_PI/180);
    x[2] = LMT(x[2],threshold*M_PI/180); 
}

void SignalDEADZONE(float x[], float threshold)
{
    x[0] = DEADZONE(x[0],threshold*M_PI/180);    
    x[1] = DEADZONE(x[1],threshold*M_PI/180);
    x[2] = DEADZONE(x[2],threshold*M_PI/180); 
}

// Вычисление ошибок управления 
void GetErrs(float xd[], float x[], float dx[]) {
    dx[0]  = xd[0]   - x[0];//*180/M_PI;
    dx[1]  = xd[1]   - x[1];//*180/M_PI;
    dx[2]  = xd[2]   - x[2];//*180/M_PI;
}


// Калибровка моторов
void calibrateMotors() {
//   Serial.println("Calib motors!");
//   motor_1.writeMicroseconds(THROTTLE_MAXIMUM);
//     motor_2.writeMicroseconds(THROTTLE_MAXIMUM);
//     motor_3.writeMicroseconds(THROTTLE_MAXIMUM);
//     motor_4.writeMicroseconds(THROTTLE_MAXIMUM);
// delay(7000);
    motor_1.writeMicroseconds(THROTTLE_MINIMUM);
    motor_2.writeMicroseconds(THROTTLE_MINIMUM);
    motor_3.writeMicroseconds(THROTTLE_MINIMUM);
    motor_4.writeMicroseconds(THROTTLE_MINIMUM);
    delay(5000);
}

void armMotors() {
    motor_1.writeMicroseconds(THROTTLE_MINIMUM);
    motor_2.writeMicroseconds(THROTTLE_MINIMUM);
    motor_3.writeMicroseconds(THROTTLE_MINIMUM);
    motor_4.writeMicroseconds(THROTTLE_MINIMUM);
    delay(5000);
}

void StopDrone()
{
  PWM1=1000;
  PWM2=1000;
  PWM3=1000;
  PWM4=1000;
}

void ANGLERATEFILTER(int16_t GYR[], float omg[],float DT, float w0)
{
    static float gyrox[2]={0.0}, gyroy[2]={0.0}, gyroz[2]={0.0};
    float omgx, omgy, omgz;

    // расчет угловой скорости 
    gyrox[0] = (float)GYR[0]/32768 * 2000/180*M_PI;   //в рад/с   //roll
    gyroy[0] =-(float)GYR[1]/32768 * 2000/180*M_PI;               //pitcj
    gyroz[0] = -(float)GYR[2]/32768 * 2000/180*M_PI;              //yaw
    
    // фильтрация показаний гироскопов
    omgx=DigLowPassFil(omg[0], gyrox[0], gyrox[1],DT,w0);
    omgy=DigLowPassFil(omg[1], gyroy[0], gyroy[1],DT,w0);
    omgz=DigLowPassFil(omg[2], gyroz[0], gyroz[1],DT,w0);

    // запомнить
    gyrox[1]=gyrox[0];gyroy[1]=gyroy[0];gyroz[1]=gyroz[0];

    // вывод
    omg[ROLL]=omgx; omg[PITCH]=omgy; omg[YAW]=omgz;       
}

void ACCELERATIONFILTER(VectorInt16 *ACC, float a[],float DT, float w0)
{
    static float accx[2]={0.0}, accy[2]={0.0}, accz[2]={0.0};
    float ax, ay, az;
    

    accx[0]=ACC->x/16384.0f;
    accy[0]=ACC->y/16384.0f;
    accz[0]=ACC->z/16384.0f;

    // фильтрация показаний гироскопов
    ax=DigLowPassFil(a[0], accx[0], accx[1],DT,w0);
    ay=DigLowPassFil(a[1], accy[0], accy[1],DT,w0);
    az=DigLowPassFil(a[2], accz[0], accz[1],DT,w0);

    // запомнить
    accx[1]=accx[0];accy[1]=accy[0];accz[1]=accz[0];

    // вывод
    a[0]=ax; a[1]=ay; a[2]=az;       
      // a[0]=DEADZONE(ax, 0.05);
      // a[1]=DEADZONE(ay, 0.05);
      // a[2]=DEADZONE(az, 0.05);  
}

// Программое управление
float PROGCONTROL1(uint32_t tmr)
{  
	float thetaDesired=0.0;
  uint32_t tmr1=tmr%400;
  if (tmr1<200)
  {
      thetaDesired=-0.5;  //roll=0.1, pitch=0.05  
  }
  else  
  {
      thetaDesired=0.5;  
  }

	return thetaDesired;
}


void Init_NRF24(void){
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
  Serial.println("radio started");

  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  radio.setPayloadSize(sizeof(payload_RX));  // float datatype occupies 4 bytes
  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1
  radio.startListening();  // put radio in RX mode
}
void Read_NRF_value (void){//
    radio.startListening();
    delay(1);
    uint8_t pipe;
    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
      radio.read(&payload_RX, bytes);             // fetch payload from FIFO
      Serial.print(payload_RX[0]);Serial.print(",");  // print the payload's value
      Serial.print(payload_RX[1]);Serial.print(",");  // print the payload's value
      Serial.print(payload_RX[2]);Serial.print(",");  // print the payload's value
      Serial.print(payload_RX[3]);Serial.print(",");  // print the payload's value
      Serial.println(payload_RX[4]);  // print the payload's value
    }
    
}

void Send_NRF_value (void){//
      // This device is a TX node
    radio.stopListening();
    delay(1);
      payload_TX[0] = 1.01;          // increment float payload
      payload_TX[1] = 2.02;
      payload_TX[2] = 3.03;
      payload_TX[3] = 4.04;
      payload_TX[4] = 5.05;
    unsigned long start_timer = micros();                // start the timer
    bool report = radio.write(&payload_TX, sizeof(payload_TX));  // transmit & save the report
    unsigned long end_timer = micros();                  // end the timer

    if (report) {
      Serial.print(F("Transmission successful! "));  // payload was delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);  // print the timer result
      Serial.print(F(" us. Sent: "));
      Serial.println(payload_TX[0]);  // print payload sent
    } else {
      Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
    }
}


