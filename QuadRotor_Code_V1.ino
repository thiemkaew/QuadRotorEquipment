/*
support : Arduino 1.6.7   Arduino Due 32 bit , Gy-521, MS5611,
//เปิดใช้งานแล้ว
• Atmel SAM3X8E ARM Cortex-M3 CPU 32-bit a 84 MHz clock, ARM core microcontroller
• MPU6050 Gyro Accelerometer //I2C 400kHz nois gyro +-0.05 deg/s , acc +-0.04 m/s^2
• MS5611 Barometer//SPI
• HMC5883L Magnetometer //I2C_BYPASS ,I2C 400kHz
• GPS NEO-7N //Rx 1 Tx 1 
• ADNS3080 Optical Flow Sensor//SPI 1 MHz 
• Ultrasonic_HC-SR04
• Distance Measuring Sensor 20-150cm (SHARP GP2Y0A02YK0F)
• SMD RGB three colors LEDs module //RGB = pin 5,4,3
• ADNS 3080 Optical Flow Sensor

Quad-X
pin 9 FRONTL  M1CW        M2CCW  FRONTR pin 8
              \         / 
                \ --- /
                 |   |
                / --- \
              /         \ 
pin 6 motor_BackL  M4 CCW      M3 CW  motor_BackR  pin 7

pin 34 PWML0 = automatically triggered. Servo

//Waypoint Chart

                 ______                             ______                 ______  
         / - - - | W2 | - - - \ - - - - - - - - - - | W5 | - - - - - - - - | W8 | - - - - -\
       /         ======        \                   ========                ======           \
     /                          \                                                            \
   /                             \                                                            \
 ______                         ________                  ______                             __\____
 | W1 |                          | W4 | - - - - - - - - - | W7 | - - - - - - - - - - - - - - | W10 |
 ======                          ======                  =======                            ========
   \                                                      /    \                               /
     \                                                   /      \                             / 
      \                                          _______/        \                           /
       \                                         | W6 |           \                         /  
        \                                      /========           \                       /
         \       ______                       /                     \                     /
          \- - - | W3 | - - - - - - - - - - -/                       \      _______      / 
                 ======                                               \- - - | W9 | - - /
                                                                            =======


----------rx-----------  
A8 = PPM 8 CH
2016/03/11
 เพิ่มการทำงานของ Distance Measure กรณีถ้ามีการเปิด AUX_3 > 1700 ให้ทำงานคำนวณและเริ่มระบบใหม่
 แต่ AUX_3 < 1700 ไม่เปิดการทำงานก็ไม่ต้องทำการอ่านค่า Sensor เพื่อลดการประมวลผลด้านการทำงาน
 รวมถึง ADNS3080 เช่นเดียวกัน
2016/03/16
 เพิ่มระบบ Auto WayPoint Mode
 2 Waypoint and landing in Home
2016/03/21
 เพิ่มระบบการ Upload Waypoint จาก Labview จำนวน 3 จุดและแสดงผลการทำงาน
 เพิ่มโหมด Auto ออกเป็น 3 WayPoint
2016/03/23
 เพิ่มโหมดการทำงานของ RTH and Landing เพื่อนำไปใช้กับ FailSafe หรือ Remote ไม่สามารถเชื่อมต่อได้
2016/03/24
 เพิ่มโหมดการทำงาน Takeoff  Auto 6 WayPoint Landing
2016/04/02
 เพิ่มโหมดการทำงานโดย Takeoff จากจุด Homepoint และไป Landing ที่ Transfer point
 หลังจากนั้นทำการ Takeoff จากจุด Transfer point และกลับมา Landing ที่จุด Homepoint อีกครั้งเป็นการเสร็จขั้นตอนการทำงาน
2016/04/03
 เพิ่มระบบการคำนวนการทำงาน Shorest Path เพื่อกำหนด Waypoint ที่ระบบจะต้องทำงานอีกครั้ง
2016/04/12
 แก้ไขระบบ Waypoint with SPF Algorithm เพื่อกำหนด Waypoint ใหม่ในการหาเส้นทางที่สั้นที่สุด
 ปรับปรุงการทำงานของการ autoarmed ให้สามารถ Disarmed เมื่อสถานะ z1_hat <= 0.18 เท่านั้น
 แก้ไขโหมดของการทำงานออกเป็น 4 โหมดเหมือนเดิมแต่ทำการแก้ไขเรื่อง mode = 4 ให้สามารถ Return to Home ได้เมื่ออยู่ใน mode = 1
2016/04/16
 ใล่ระบบการทำงาน SPF ที่เกิดปัญหาเกี่ยวกับการทำงาน เมื่อมีการ Takeoff ขึ้นไปแล้ว ตัวเครื่อง Quad ไม่สามารถหยุกให้ z1_hat >= h_control ได้
 เพิ่มกระบวนการของ waypointCheck ไปที่ Altitude Controll บนระบบ
 อยู่ในระบหว่างการทดสอบการดำเนินการถ้า Complete Project ในส่วนนี้ก็เสร็จแล้ว
2016/04/24
 ทดสอบการทำงานของ waypoint_SPF ตั้งแต่ 1 - 6
 0 - Complete 6 Waypoint
 1 - Complete 1 Waypoint_SPF
 2 - Complete 2 Waypoint_SPF
 3 - Complete 3 Waypoint_SPF
 4 - Complete 4 Waypoint_SPF
 5 - Complete 5 Waypoint_SPF
 6 - ระบบยังไม่ได้ใช้มากถึง 6 เพราะยังไม่ได้มี Distance ที่สั้นที่สุด
2016/04/27
 ตรวจสอบปัญหาที่เกิดขึ้น คือเมื่อทำการ Landing จุดที่ Waypoint ปลายทางแล้วการรอคอยเวลาจะนานมากกว่า 15s ซึ่งจะทำให้ ช่วงเวลาในการทำงานนานขึ้น
 ดังนั้นตอนนี้คาดคะเนสาเหตุที่เกิดขึ้นคือ
 สาเหตุ
 ---- Arduino 1.6.8
 ---- Serial Print ใน Function  if (frameCounter % TASK_5HZ == 0)
 การแก้ไขตอนนี้
 ----(แก้ไขปัญหาไม่ได้) ปิดไปใช้ Function if (frameCounter % TASK_NoHZ == 0)
 ---- ลด Version สำหรับการ compile เพื่อให้ Quad-Rotor ทำงาน เหลือ Version 1.6.7
 ---- ลด Header ลงเพื่อ
2016/04/30
 ยังมีปัญหา Error บางจุดใน Mode Auto and RTH คือเครื่องมาจอดที่ Home Point หรือ Transfer Point เครื่องยังไม่ทำการ Disarmed
 ปรับ Error Rate ของ GPS_LAT GPS_LON
 abs(error_LAT) <= 150  เปลี่ยนเป็น abs(error_LAT) <= 150
 abs(error_LON) <= 150  เปลี่ยนเป็น abs(error_LON) <= 120
2016/09/29
 ปรับปรุงโครงสร้างการทำงานเพิ่มเติม ให้ทำงานได้ตรงตาม Scope ของโปรเจ็ค IS 
 1. เพิ่มโหมดการทำงานของ Auto Waypoint with Transfering ออกเป็น 2 Station และสามารถแยกการทำงาน โดยผ่าน Switch ของ Remote
 2. ปรับปรุงการทำงานของ Sensor เดิม MPU6050 เดิม พร้อมกับปรับ offset เพิ่มเติมให้ quad ไม่ไหลย๊วย
 3. ทดสอบการทำงานของ Source Code และ Micro Controller อีกครั้ง
 4. เพิ่มโหมดการทำงานเพิ่มเติมโดยระบุความสูงในแต่ไต่ระดับแต่ล่ะ Waypoint ได้โดยเริ่มที่ความสูงที่ h_control = 12 m เป็นต้นไป
 5. เพิ่มเติมการทำงานปิดเปิดของกล่อง Servo โดยกำหนดที่ และควบคุมการทำงานอัตโนมัติ
 // Default ปิด
    int Servo_trigger = 1850;
 // Default เปิด
    int Servo_trigger = 1020;
2016/10/23  การทดลองสุดท้าย
 1. เพิ่มเติมการแสดงผลการทำงานผ่าน LABVIEW เพิ่มเติม
   motor_FrontLf_d --> MotorControl Front Left    
   motor_FrontRf_d --> MotorControl Front Right
   motor_BackLf_d --> MotorControl Back Left
   motor_BackRf_d --> MotorCoftrol Back Right
   Ampare --> Mode Control in QuadCopter
   Distance_L --> Distance Measure Left
   Distance_R --> Distance Measute Right
   Distance_F --> Distance Measure Front
   Distance_B --> Distance Measure Back
 2. ปรับปรุงหน้าตาของโปรแกรม Labview เพื่อให้มีความเหมาะสมสำหรับตรวจสอบระบบ
   เพิ่มหน้าโปรแกรมแสดงเกี่ยวกับจุดหรือตำแหน่งที่ Quad-Rotor กำลังเดินทางอยู่บนอากาศ
   เพิ่มผังการแสดงการทำงานของ Motor ในแต่ล่ะตัว
   เพิ่มผังการแสดงการทำงานของ Distance Measure ในแต่ล่ะตัว
   เพิ่มการเก็บข้อมูลของโปรแกรม Labview
     Track GPS LAT
     Track GPS LON
     Altitude Control
     -- Motor
 3. ปรับปรุงการทำงานของระบบ Auto Waypoint ออกเป็น 3 Mode
   Mode 3 - 3  --> เป็นการทำงานของระบบ Waypoint Home <--> Transfer
   Mode 3 - 2  --> เป็นการทำงานของระบบ Waypoint Home <--> Waypoint 1 <--> Waypoint 2 <--> Transfer
   Mode 3 - 1  --> เป็นการทำงานของระบบ Waypoint Home <--> Waypoint 1 <--> Waypoint 2 <--> Waypoint 3 <--> Transfer
   ซึ่งจุดที่ได้ออกแบบนั้นอ้างอิงเกี่ยวกับพื้นที่ทางภูมิศาสตร์ของ โรงพยาบาลโอเวอร์บรุ๊คเชียงราย
 */

#include <Arduino.h>
#include "Wire_due32.h"
#include "SPI_sam.h"
#include "AP_Baro_MS5611.h"
#include "Aconfigsam3x8e.h"
#include "multi_rx_sam3x8e.h"
#include "mpu6050sam3x8e.h"
#include "ahrs_tinsam3x8e.h"
#include "GPSNEO7N_multi.h"
#include "ADNS3080.h"
#include "AnalogSource_Arduino.h"
#include "Ultrasonic04.h"
#include "Control_PPIDsam3x8e.h"
#include "Control_Autonomous_WithShortestPath.h"
#include "Control_Autonomous_System.h"
#include "motorX4sam3x8e.h"
#include "Kalman_ObserverTin.h"

////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(57600);//115200
  //Serial1.begin(57600);
  //Enable and Disable Labview 
  Serial3.begin(57600);
  Serial.print("Due32bit_GY521MPU_Control");Serial.println("\n");
  GPS_multiInt();
  pinMode(13, OUTPUT);//pinMode (30, OUTPUT);pinMode (31, OUTPUT);//pinMode (30, OUTPUT);pinMode (31, OUTPUT);//(13=A=M),(31=B=STABLEPIN),(30=C,GPS FIG LEDPIN)
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  //pinMode(Pin_Laser, OUTPUT);
  pinMode(Pin_LED_B, OUTPUT);
  pinMode(Pin_LED_G, OUTPUT);
  pinMode(Pin_LED_R, OUTPUT);
  digitalWrite(13, HIGH);
  //digitalWrite(Pin_Laser, HIGH);
  digitalWrite(Pin_LED_B, LOW);
  digitalWrite(Pin_LED_G, LOW);
  digitalWrite(Pin_LED_R, LOW);
  configureReceiver();
  motor_initialize();
  ESC_calibration();
  delay(10);
  Wire.begin();
  Wire.setClock(400000);
  delay(10);
  Wire1.begin();
  Wire1.setClock(400000);//400000
  delay(10);
  mpu6050_initialize();
  delay(30); //GYROSCOPE START-UP TIME 30 ms
  MagHMC5883Int();
  Serial.print("HMC5883_initialize");Serial.print("\n");
  delay(10);
  SPI.begin();
  //SPI.setClockDivider(SPI_CLOCK_DIV32);     // 500khz for debugging, increase later
  delay(10);
  //baro.init(MS561101BA_ADDR_CSB_LOW);
  baro.init();
  //baro.calibrate();
  delay(10);
  //installation_a3080();
  //delay(10);
  UltrasonicInt();
  delay(10);
  Serial.print("Read Sensor");Serial.println("\t");
     for(int i=0; i<100; i++) 
    {
     mpu6050_readGyroSum();
     mpu6050_readAccelSum();
     Mag5883Read();
     //temperaturetr = baro.getTemperature(MS561101BA_OSR_4096);
     //presser = baro.getPressure(MS561101BA_OSR_4096);
     baro._update(micros());
     baro.read();
     presser = baro.get_pressure();
     temperaturetr = baro.get_temperature();
     pushAvg(presser, temperaturetr);
     delay(10);
    }
    presserf = getAvg(movavg_buff, MOVAVG_SIZE);
    temperaturetrf = getAvg(movavg_buffT, MOVAVG_SIZE);
    sea_press = presserf - 0.12;// + 0.09 presser 1007.25   1003.52
    Serial.print("presser ");Serial.print(sea_press);
    Serial.print(" Temperature ");Serial.println(temperaturetrf);
    digitalWrite(Pin_LED_B, HIGH);
    digitalWrite(Pin_LED_G, HIGH);
    digitalWrite(Pin_LED_R, HIGH);
    mpu6050_Get_accel();
    mpu6050_Get_gyro();
    delay(10);
    sensor_Calibrate();//sensor.h
    ahrs_initialize();//ahrs.h
    RC_Calibrate();//"multi_rxPPM2560.h"
    setupFourthOrder();//ahrs
  delay(100);
  digitalWrite(13, LOW);
      for(uint8_t i=0;i<=10;i++){
      //GPS_NewData(); 
        while(Serial2.available())
       {
       char byteGPS1=Serial2.read(); 
       GPS_UBLOX_newFrame(byteGPS1);
       }
       GPS_LAT1 = GPS_coord[LAT]/10000000.0;// 1e-7 degrees / position as degrees (*10E7)
       GPS_LON1 = GPS_coord[LON]/10000000.0;
       if ( GPS_LAT1 == 0.000000000 && GPS_LON1 == 0.000000000 ) {
          GPS_LAT_HOME = GPS_LAT_HOME;
          GPS_LON_HOME = GPS_LON_HOME;
       }
       else {
          GPS_LAT_HOME = GPS_LAT1;
          GPS_LON_HOME = GPS_LON1;
       }
       digitalWrite(Pin_LED_B, LOW);
       delay(20);
       digitalWrite(Pin_LED_B, HIGH);
       delay(80);
    }
  //digitalWrite(Pin_Laser, LOW);
  digitalWrite(Pin_LED_B, HIGH);
  digitalWrite(Pin_LED_G, HIGH);
  digitalWrite(Pin_LED_R, HIGH);
  sensorPreviousTime = micros();
  previousTime = micros();
}
void loop() {
  while(1){
    while(Serial2.available()){ /////GPS///////////////////////////////
     char byteGPS1=Serial2.read(); 
     GPS_UBLOX_newFrame(byteGPS1);
     }//end gps  ///////////////////////////////////////
     ///Read Labview/////////// data 20 byte
     
     //Enable and Disable Labview Connnection
  if(Serial3.available() >= 20){
     byte byteHeader = Serial3.read(); 
     byte byteLabview2 = Serial3.read();
     byte byteLabview3 = Serial3.read();
     byte byteLabview4 = Serial3.read(); 
     byte byteLabview5 = Serial3.read();
     byte byteLabview6 = Serial3.read();
     byte byteLabview7 = Serial3.read();
     byte byteLabview8 = Serial3.read(); 
     byte byteLabview9 = Serial3.read();
     byte byteLabview10 = Serial3.read();
     byte byteLabview11 = Serial3.read();
     byte byteLabview12 = Serial3.read(); 
     byte byteLabview13 = Serial3.read();
     byte byteLabview14 = Serial3.read();
     byte byteLabview15 = Serial3.read();
     byte byteLabview16 = Serial3.read();
     byte byteLabview17 = Serial3.read();
     byte byteLabview18 = Serial3.read();
     byte byteLabview19 = Serial3.read();
     byte SumLabview = (byteLabview2 + byteLabview3 + byteLabview4 + byteLabview5 + byteLabview6 + byteLabview7 
     + byteLabview8 + byteLabview9 + byteLabview10 + byteLabview11 + byteLabview12 + byteLabview13 + byteLabview14
     + byteLabview15 + byteLabview16 + byteLabview17 + byteLabview18 + byteLabview19);
     if(byteHeader == 0x3B && byteLabview19 == SumLabview)
     {//59 = 0x3B
           
         int32_t Labview_LAT_Transport = (((int32_t)byteLabview5)<<24) | (((int32_t)byteLabview4)<<16) | (((int32_t)byteLabview3)<<8) | (byteLabview2);
         int32_t Labview_LON_Transport = (((int32_t)byteLabview9)<<24) | (((int32_t)byteLabview8)<<16) | (((int32_t)byteLabview7)<<8) | (byteLabview6);
         int32_t Labview_LAT_Home = (((int32_t)byteLabview13)<<24) | (((int32_t)byteLabview12)<<16) | (((int32_t)byteLabview11)<<8) | (byteLabview10);
         int32_t Labview_LON_Home = (((int32_t)byteLabview17)<<24) | (((int32_t)byteLabview16)<<16) | (((int32_t)byteLabview15)<<8) | (byteLabview14);
         int16_t set_hcontrol = (((int16_t)byteLabview19)<<8) | (byteLabview18);
         
         GPS_LAT_HOME = (float)Labview_LAT_Home/10000000.0;
         GPS_LON_HOME = (float)Labview_LON_Home/10000000.0;
         GPS_LAT_HOME_Transport = (float)Labview_LAT_Transport/10000000.0;
         GPS_LON_HOME_Transport = (float)Labview_LON_Transport/10000000.0;
         h_control = (float)set_hcontrol*100.0/32767.0;
     }
     else{
           while(Serial3.available()>0){
           char byteEnd = Serial3.read(); 
           }
     }
     }
/////////////////
    Dt_sensor = micros() - sensorPreviousTime;///////////Roop sensor 1 kHz/////////
    if(Dt_sensor <= 0){Dt_sensor = 1001;}
    if(Dt_sensor >= 1000 && gyroSamples < 4)////Collect 2 samples = 1000 us 
    {  
        sensorPreviousTime = micros();
        mpu6050_readGyroSum();
        mpu6050_readAccelSum();
    }
   Dt_roop = micros() - previousTime;// 200 Hz task loop (5 ms)  , 2500 us = 400 Hz
   if(Dt_roop <= 0){Dt_roop = 5001;}  
   if (Dt_roop >= 5000) 
    {
      previousTime = micros();
      G_Dt = Dt_roop*0.000001;
      frameCounter++;
/////get sensor////////////////////////////////////////////////////////////
      mpu6050_Get_accel();
      mpu6050_Get_gyro();
      baro._update(previousTime);
      //////////////////////////////////////////////////////////
  if (frameCounter % TASK_100HZ == 0)// 100 Hz tak
 {
   /*
        ///////////////////Filter FourthOrder ///////////////////////////////////////
    Accel[XAXIS] = AccX;
    Accel[YAXIS] = AccY;
    Accel[ZAXIS] = AccZ;
    for (int axis = XAXIS; axis <= ZAXIS; axis++) {
      filteredAccel[axis] = computeFourthOrder(Accel[axis], &fourthOrder[axis]);//"ahrs_tin.h"
    }
    AccXf = filteredAccel[XAXIS];
    AccYf = filteredAccel[YAXIS];
    AccZf = filteredAccel[ZAXIS];
    */
  //presser = baro.getPressure(MS561101BA_OSR_4096);
  //โหมด Return to Home และทำการเปิด Sensor ADNS3080 และ Distance Measure
//    if (AUX_1 > (Loiter_Sensor-20) && AUX_1 <= (Loiter_Sensor+20)) 
//    {
//        analogReadSUM();//AnalogSource_Arduino.h
//    }
    
  analogReadSUM();
  baro.read();
  presser = baro.get_pressure();
  temperaturetr = baro.get_temperature();
  pushAvg(presser, temperaturetr);
 }
////////////////Moving Average Filters///////////////////////////
      GyroXf = (GyroX + GyroX2)/2.0;
      GyroYf = (GyroY + GyroY2)/2.0;
      GyroZf = (GyroZ + GyroZ2)/2.0;
      //AccXf = (AccX + AccX2)/2.0;
      //AccYf = (AccY + AccY2)/2.0;
      //AccZf = (AccZ + AccZ2)/2.0;
      //AccX2 = AccX;AccY2 = AccY;AccZ2 = AccZ;//acc Old1
      GyroX2 = GyroX;GyroY2 = GyroY;GyroZ2 = GyroZ;//gyro Old1
////////////////Low pass filter/////////////////////////////////
    AccXf = AccXf + (AccX - AccXf)*0.12101;//0.240121 ,0.121  //Low pass filter ,smoothing factor  α := dt / (RC + dt)
    AccYf = AccYf + (AccY - AccYf)*0.12101;//0.240121 ,0.121
    AccZf = AccZf + (AccZ - AccZf)*0.12101;//0.240121 ,0.121
//////////////////////////////////////////////////////////
    ahrs_updateMARG(GyroXf, GyroYf, GyroZf, AccXf, AccYf, AccZf, c_magnetom_x, c_magnetom_y, c_magnetom_z, G_Dt);//quaternion ,direction cosine matrix ,Euler angles
//Observer kalman filter//////////////////////////////////
    Observer_kalman_filter();
//Sliding modeControl/////////////////////////////////////
    Control_PPIDRate();//"Control_Slid.h"
//////Out motor///////////
//armed = 1;
    motor_Mix();//"motor.h"
/////////////////////////
    motor_command(); 
////////end Out motor//////
  if (frameCounter % TASK_50HZ == 0)// 50 Hz tak (20 ms)
 {
  //mpu6050_Get_accel();
  computeRC();//multi_rx.h
    if (CH_THR < MINCHECK)  //////ARM and DISARM your Quadrotor///////////////
        {
            if (CH_RUD > MAXCHECK && armed == 0 && abs(ahrs_p) < 10 && abs(ahrs_r) < 10)//+- 10 deg, ARM 
            {
                armed = 1;
                digitalWrite(Pin_LED_R, LOW);
                Altitude_Ground = Altitude_baro;
                setHeading = ahrs_y;// 0 degree ,ahrs_tin.h
                //ปิดกล่องอัตโนมัติ
                Servo_trigger = 1850;
            }
            if (CH_RUD < MINCHECK && armed == 1) //DISARM
            {
                armed = 0;
                digitalWrite(Pin_LED_R, HIGH);
                Altitude_Ground = Altitude_baro;
                setHeading = ahrs_y;// 0 degree ,ahrs_tin.h
                //เปิดกล่องอัตโนมัติ
                Servo_trigger = 1020;
            }
            if (CH_RUD < MINCHECK && armed == 0 && CH_ELE > MAXCHECK) //Mag_Calibrate
            {
              Mag_Calibrate();//#include "mpu6050.h"
            }
        }//end  ARM and DISARM your helicopter/////////////// 
 }
      if (frameCounter % TASK_20HZ == 0)// 20 Hz task (50 ms)
        {
          Mag5883Read();
          UltrasonicRead();//"Ultrasonic04.h"
          Distance_Measuring_Get();
          Control_PositionHold();
          //Get_pixy();	

          //โหมด Return to Home และทำการเปิด Sensor ADNS3080 และ Distance Measure
//          if (AUX_1 > (Loiter_Sensor-20) && AUX_1 <= (Loiter_Sensor+20)) 
//          {
//            //updateOF();//AP_OPTICALFLOW_ADNS3080
//            Distance_Measuring_Get();
//            //update_positionA3080(GyroXfMO ,GyroYfMO ,1.0 ,0.0 ,z1_hat);//(float Gyroroll, float Gyropitch, float cos_yaw_x, float sin_yaw_y, float altitude)
//          }
//          else
//          {
//            //Default Distance Measure and ADNS3080
//            Distance_X = 0;
//            Distance_Y = 0;
//            //posistion_X = 0.0;
//            //posistion_Y = 0.0;
//            //surface_quality = 0;
//          }
                   
        }
        if (frameCounter % TASK_10HZ == 0)//roop TASK_10HZ
        {
            Chack_Command();
            //AutomaticTransport();
            //AutomaticTransport_Home();
            //AutoTransportOperating();
            AutoTransportOperating_SPF();
            failSafeAuto();          
        }
        if (frameCounter % TASK_5HZ == 0)//GPS_calc TASK_5HZ
        {
           presserf = getAvg(movavg_buff, MOVAVG_SIZE);
           temperaturetrf = getAvg(movavg_buffT, MOVAVG_SIZE);
           presserfF = presserfF + (presserf - presserfF)*0.8852101;//0.9852101 0.785 0.685 0.545 0.345 Low Pass Filter
           Altitude_baro = getAltitude(presserfF,temperaturetrf);//Altitude_Ground
           Altitude_barof = Altitude_baro - Altitude_Ground + Altitude_II;
           baro_vz = (z1_hat - baro_vz_old)/0.2;//G_Dt Diff Baro 5HZ=0.2 s  ,,20 Hz=0.05
           baro_vz_old = z1_hat;
           GPS_LAT1 = GPS_coord[LAT]/10000000.0;// 1e-7 degrees / position as degrees (*10E7)
           GPS_LON1 = GPS_coord[LON]/10000000.0;
           Cal_GPS();//#include "Control_PPIDsam3x8e.h"
           //Control_PositionHold();
           //GPS_distance_m_bearing(GPS_LAT1, GPS_LON1, GPS_LAT_HOME, GPS_LON_HOME, Altitude_hat);
        }
        if (frameCounter % TASK_2HZ == 0)//LED GPS
        {
          if(Status_LED_GPS == HIGH && GPS_FIX  == 1)
             {
               Status_LED_GPS = LOW;
               Counter_LED_GPS++;
               if(Counter_LED_GPS >= GPS_numSat){
               Counter_LED_GPS = 0;
               digitalWrite(Pin_LED_G, LOW);
               }
               else
               {
               digitalWrite(Pin_LED_G, HIGH);
               }
             }
             else
             {
              Status_LED_GPS = HIGH;
             }
            digitalWrite(Pin_LED_B, Status_LED_GPS);
        }//end if LED GPS
       if (frameCounter % TASK_5HZ == 0)//roop print  ,TASK_NoHZ TASK_5HZ  TASK_10HZ
        {
          //Voltage = analogRead(A5);
          //Ampere = analogRead(A6);
          Voltage = GPS_numSat + 255;
          //Ampere = Status_waypoint + 255;
          Ampere = Mode + 255;
          SendToLabview();
        }
      //if (frameCounter % TASK_5HZ == 0)//roop print  ,TASK_NoHZ TASK_5HZ  TASK_10HZ
      if (frameCounter % TASK_NoHZ == 0)//roop print  ,TASK_NoHZ TASK_5HZ  TASK_10HZ
        {
            //Serial.print(CH_THRf);Serial.print("\t");
            //Serial.print(CH_AILf);Serial.print("\t");  
            //Serial.print(CH_ELEf);Serial.print("\t");
            //Serial.print(CH_RUDf);Serial.print("\t");  
            //Serial.print(AUX_1);Serial.print("\t"); 
            //Serial.print(AUX_2);Serial.print("\t"); 
            //Serial.print(AUX_3);Serial.print("\t"); 
            //Serial.print(AUX_4);Serial.print("\t"); 
            //Serial.print(failsafeCnt);Serial.print("\t");
            
            //Serial.print(setpoint_rate_roll);Serial.print("\t");
            //Serial.print(setpoint_rate_pitch);Serial.print("\t"); 
             
            //Serial.print(MagX1);Serial.print("\t");
            //--Serial.print(MagXf);Serial.print("\t");
            //Serial.print(MagY1);Serial.print("\t");
            //--Serial.print(MagYf);Serial.print("\t");
            //Serial.print(MagZ1);Serial.print("\t");  
            //--Serial.print(MagZf);Serial.print("\t");
            
            //Serial.print(c_magnetom_x);Serial.print("\t");
            //Serial.print(c_magnetom_y);Serial.print("\t");
            //Serial.print(c_magnetom_z);Serial.print("\t"); 
            
            //Serial.print(GPS_FIX1);Serial.print("\t");
            //Serial.print(GPS_LAT1,9);Serial.print("\t"); 
            //Serial.print(GPS_LAT1f,9);Serial.print("\t");

            //Serial.print(GPS_LON1,9);Serial.print("\t");
            //Serial.print(GPS_LON1f,9);Serial.print("\t");
            //Serial.print(GPS_LON1f2,9);Serial.print("\t");
            //Serial.print(error_LAT);Serial.print("\t");
            //Serial.print(error_LON);Serial.print("\t");
            //Serial.print(GPS_speed);Serial.print("\t");//cm/s
            //Serial.print(GPS_ground_course);Serial.print("\t");//deg

            //GPS Control
            //Serial.print(target_LAT,9);Serial.print("\t");
            //Serial.print(target_LON,9);Serial.print("\t");
//            Serial.print(z1_hat);Serial.print("\t"); 
//            Serial.print(time_auto);Serial.print("\t");
//            Serial.print(time_AutoOperate);Serial.print("\t");
//            Serial.print(takeoff);Serial.print("\t");
//            Serial.print(endAuto);Serial.print("\t");
//            Serial.print(h_counter);Serial.print("\t");
//            Serial.print(Vz_Hold);Serial.print("\t");
//            Serial.print(Status_waypoint);Serial.print("\t");
//            Serial.print(" || ");
//            Serial.print(waypointCheck);Serial.print("\t");
//            Serial.print(time_auto_WPath1);Serial.print("\t");
//            Serial.print(time_AutoOperate_WPath1);Serial.print("\t");
//            Serial.print(takeoff_WPath1);Serial.print("\t");
//            Serial.print(endAuto_WPath1);Serial.print("\t");
//            Serial.print(h_counter_WPath1);Serial.print("\t");
//            Serial.print(Vz_Hold_WPath1);Serial.print("\t");
//            Serial.print(Status_waypoint_WPath1);Serial.print("\t");
//            Serial.print(" | ");            
//            Serial.print(time_auto_WPath2);Serial.print("\t");
//            Serial.print(time_AutoOperate_WPath2);Serial.print("\t");
//            Serial.print(takeoff_WPath2);Serial.print("\t");
//            Serial.print(endAuto_WPath2);Serial.print("\t");
//            Serial.print(h_counter_WPath2);Serial.print("\t");
//            Serial.print(Vz_Hold_WPath2);Serial.print("\t");
//            Serial.print(Status_waypoint_WPath2);Serial.print("\t");
//            Serial.print(" | ");
//            Serial.print(time_auto_WPath3);Serial.print("\t");
//            Serial.print(time_AutoOperate_WPath3);Serial.print("\t");
//            Serial.print(takeoff_WPath3);Serial.print("\t");
//            Serial.print(endAuto_WPath3);Serial.print("\t");
//            Serial.print(h_counter_WPath3);Serial.print("\t");
//            Serial.print(Vz_Hold_WPath3);Serial.print("\t");
//            Serial.print(Status_waypoint_WPath3);Serial.print("\t");
//            Serial.print(" | ");
//            Serial.print(time_auto_WPath4);Serial.print("\t");
//            Serial.print(time_AutoOperate_WPath4);Serial.print("\t");
//            Serial.print(takeoff_WPath4);Serial.print("\t");
//            Serial.print(endAuto_WPath4);Serial.print("\t");
//            Serial.print(h_counter_WPath4);Serial.print("\t");
//            Serial.print(Vz_Hold_WPath4);Serial.print("\t");
//            Serial.print(Status_waypoint_WPath4);Serial.print("\t");
//            Serial.print(" | ");
//            Serial.print(time_auto_WPath5);Serial.print("\t");
//            Serial.print(time_AutoOperate_WPath5);Serial.print("\t");
//            Serial.print(takeoff_WPath5);Serial.print("\t");
//            Serial.print(endAuto_WPath5);Serial.print("\t");
//            Serial.print(h_counter_WPath5);Serial.print("\t");
//            Serial.print(Vz_Hold_WPath5);Serial.print("\t");
//            Serial.print(Status_waypoint_WPath5);Serial.print("\t");
//            Serial.print(" | ");
//            Serial.print(time_auto_WPath6);Serial.print("\t");
//            Serial.print(time_AutoOperate_WPath6);Serial.print("\t");
//            Serial.print(takeoff_WPath6);Serial.print("\t");
//            Serial.print(endAuto_WPath6);Serial.print("\t");
//            Serial.print(h_counter_WPath6);Serial.print("\t");
//            Serial.print(Vz_Hold_WPath6);Serial.print("\t");
//            Serial.print(Status_waypoint_WPath6);Serial.print("\t");
            
//            Serial.print(GPS_LAT_HOME,9);Serial.print("\t");
//            Serial.print(GPS_LON_HOME,9);Serial.print("\t");
//            Serial.print(waypoint1_LAT,9);Serial.print("\t");
//            Serial.print(waypoint1_LON,9);Serial.print("\t");
//            Serial.print(waypoint2_LAT,9);Serial.print("\t");
//            Serial.print(waypoint2_LON,9);Serial.print("\t");
//            Serial.print(waypoint3_LAT,9);Serial.print("\t");
//            Serial.print(waypoint3_LON,9);Serial.print("\t");
//            Serial.print(waypoint4_LAT,9);Serial.print("\t");
//            Serial.print(waypoint4_LON,9);Serial.print("\t");
//            Serial.print(waypoint5_LAT,9);Serial.print("\t");
//            Serial.print(waypoint5_LON,9);Serial.print("\t");
//            Serial.print(waypoint6_LAT,9);Serial.print("\t");
//            Serial.print(waypoint6_LON,9);Serial.print("\t");
//            Serial.print(GPS_LAT_HOME_Transport,9);Serial.print("\t");
//            Serial.print(GPS_LON_HOME_Transport,9);Serial.print("\t");

            //Serial.print(h_control);Serial.print("\t");
            //Serial.print(h_counter);Serial.print("\t");
            //Serial.print(time_auto);Serial.print("\t");
            //Serial.print(takeoff);Serial.print("\t");
            //Serial.print(timeLanding);Serial.print("\t");
            
            //Serial.print(Status_waypoint);Serial.print("\t");
            //Serial.print(endAuto);Serial.print("\t");
            

            
            //Serial.print(_velocity_north);Serial.print("\t");
            //Serial.print(actual_speedX);Serial.print("\t");
            //Serial.print(actual_speedXf);Serial.print("\t");
            //Serial.print(vx_hat);Serial.print("\t");
            //Serial.print(_velocity_east);Serial.print("\t");
            //Serial.print(_vel_down);Serial.print("\t");
            //Serial.print(actual_speedY);Serial.print("\t");
            //Serial.print(actual_speedYf);Serial.print("\t");
            //Serial.print(vy_hat);Serial.print("\t");
            //Serial3.print(GPS_Distance);Serial3.print("\t");
            //Serial3.print(GPS_ground_course);Serial3.print("\t");
            //Serial3.print(Control_XEf);Serial3.print("\t");
            //Serial3.print(Control_YEf);Serial3.print("\t");
            //Serial3.print(Control_XBf);Serial3.print("\t");
            //Serial3.print(Control_YBf);Serial3.print("\t");
            
            //Serial.print(Control_XEf);Serial.print("\t");
            //Serial.print(Control_YEf);Serial.print("\t");
            //Serial.print(Control_XBf);Serial.print("\t");
            //Serial.print(Control_YBf);Serial.print("\t");
            
            //Serial.print(panError);Serial.print("\t");
            //Serial.print(tiltError);Serial.print("\t");
            //Serial.print(LError);Serial.print("\t");
            
            //Serial.print(posistion_X);Serial.print("\t");
            //Serial.print(posistion_Y);Serial.print("\t");
            //Serial.print(surface_quality);Serial.print("\t");
            
            //Serial.print(Distance_L);Serial.print("\t");
            //Serial.print(Distance_R);Serial.print("\t");
            //Serial.print(Distance_F);Serial.print("\t");
            //Serial.print(Distance_B);Serial.print("\t");
            //Serial.print(Distance_X);Serial.print("\t");
            //Serial.print(Distance_Y);Serial.print("\t");
            //Serial.print(Voltage);Serial.print("\t");
            //Serial.print(Ampere);Serial.print("\t");
            
            //Serial.print(TempMPU);Serial.print("\t");
            //Serial.print(temperaturetr);Serial.print("\t");
            //Serial.print(presser,3);Serial.print("\t");
            //Serial.print(presserf,3);Serial.print("\t");
            //Serial.print(presserfF,3);Serial.print("\t");
           // Serial.print(Altitude_baro);Serial.print("\t");
            //---Serial.print(Altitude_Baro_ult);Serial.print("\t");
            //Serial.print(Altitude_barof);Serial.print("\t");
            //Serial.print(Altitude_sonaf);Serial.print("\t");
            //Serial.print(z1_hat);Serial.print("\t"); 
            
            //Serial.print(Vz_Baro_ult);Serial.print("\t");
            //Serial.print(baro_vz);Serial.print("\t");
            //Serial.print(vz_sonaf);Serial.print("\t");
            //Serial.print(z2_hat);Serial.print("\t");
            //Serial.print(Altitude_sona);Serial.print("\t");
            //Serial.print(Altitude_hat);Serial.print("\t");
            //Serial.print(vz_sona*10);Serial.print("\t");
            //Serial.print(vz_hat*10);Serial.print("\t");
            //Serial.print(h_counter);Serial.print("\t");
            //Serial.print(GPS_hz);Serial.print("\t"); 

            //Serial.print(vz_hat);Serial.print("\t");
            //Serial.print(DCM10);Serial.print("\t");
            //Serial.print(DCM11);Serial.print("\t");
            //Serial.print(DCM12);Serial.print("\t");
            //Serial.print(AccX);Serial.print("\t");
            //Serial.print(AccXf);Serial.print("\t");
            //Serial.print(AccXf2);Serial.print("\t");
            //Serial.print(AccY);Serial.print("\t");  
            //Serial.print(AccYf);Serial.print("\t"); 
            //Serial.print(AccZ);Serial.print("\t");
            //Serial.print(AccZf2,3);Serial.print("\t");
            //Serial.print(AccZf3,3);Serial.print("\t");
            //Serial.print(AccZf);Serial.print("\t");       
            //Serial.print(accrX_Earth);Serial.print("\t");
            //Serial.print(accrY_Earth);Serial.print("\t");
            //Serial.print(accrZ_Earthf);Serial.print("\t");
            //Serial.print(accrZ_Earth);Serial.print("\t");
            
            //--Serial.print(accelRaw[XAXIS]);Serial.print("\t");
            //--Serial.print(accelRaw[YAXIS]);Serial.print("\t");
            //--Serial.print(accelRaw[ZAXIS]);Serial.print("\t");
            
            //Serial.print(-GyroX*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroXf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(roll_D_rate);Serial.print("\t");
            //Serial.print(GyroYf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroXfMO*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroZf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyrofY);Serial.print("\t");  
            //Serial.print(GyroZ);Serial.print("\t");  
            //Serial.print(gyroRaw[XAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[YAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[ZAXIS]);Serial.print("\t");
            
            //Serial.print(_accel9250X);Serial.print("\t"); 
            //Serial.print(_accel9250Y);Serial.print("\t");
            //Serial.print(_accel9250Z);Serial.print("\t");
            
            //Serial.print(ahrs_r);Serial.print("\t");
            //Serial.print(ahrs_p);Serial.print("\t");  
            //Serial.print(ahrs_y);Serial.print("\t");  
            //Serial3.print(ahrs_y*RAD_TO_DEG);Serial3.print("\t"); 
            //Serial.print(cos_rollcos_pitch);Serial.print("\t"); 
             
            //Serial.print(x_angle);Serial.print("\t");
            
            //Serial.print(err_pitch_rate);Serial.print("\t");
            //Serial.print(roll_I_rate);Serial.print("\t");
            
            //Serial.print(u_roll);Serial.print("\t");
            //Serial.print(u_pitch);Serial.print("\t");
            //Serial.print(u_yaw);Serial.print("\t");
            
            //Serial.print(motor_FrontL);Serial.print("\t");
            //Serial.print(motor_FrontLf);Serial.print("\t");
            //Serial.print(motor_FrontR);Serial.print("\t");
            //Serial.print(motor_FrontRf);Serial.print("\t");
            //Serial.print(motor_BackL);Serial.print("\t");
            //Serial.print(motor_BackLf);Serial.print("\t");
            //Serial.print(motor_BackR);Serial.print("\t");
            //Serial.print(motor_BackRf);Serial.print("\t");
            //Serial.print(motor_Left);Serial.print("\t");
            //Serial.print(motor_Right);Serial.print("\t");
            
            //Serial.print(GPS_numSat);Serial.print("\t");
            //Serial.print(Mode);Serial.print("\t");
            //Serial.print(gyroSamples2);Serial.print("\t");
            //Serial.print(1/G_Dt);Serial.print("\t");
            //Serial.print("Hz");
            //Serial.print(millis()/1000.0);//millis() micros()
            //Serial.print("\n"); 
        }//end roop 5 Hz //end roop NoHz 
      if (frameCounter >= TASK_1HZ) { // Reset frameCounter back to 0 after reaching 100 (1s)
            frameCounter = 0;
            //Station 1 Count
            time_auto++;
            time_auto_tranH++;
            time_AutoOperate++;
            //Station 2 Count
            time_auto_sta2++;
            time_auto_tranH_sta2++;
            time_AutoOperate_sta2++;
            //Station 3 Count
            time_auto_sta3++;
            time_auto_tranH_sta3++;
            time_AutoOperate_sta3++;
            //Return to Home Count
            time_auto_rth++;

            
            time_auto_WPath1++;
            time_auto_WPath2++;
            time_auto_WPath3++;
            time_auto_WPath4++;
            time_auto_WPath5++;
            time_auto_WPath6++;
            time_auto_tranH_WPath1++;
            time_auto_tranH_WPath2++;
            time_auto_tranH_WPath3++;
            time_auto_tranH_WPath4++;
            time_auto_tranH_WPath5++;
            time_auto_tranH_WPath6++;
            time_AutoOperate_WPath1++;
            time_AutoOperate_WPath2++;
            time_AutoOperate_WPath3++;
            time_AutoOperate_WPath4++;
            time_AutoOperate_WPath5++;
            time_AutoOperate_WPath6++;
            
            //temperaturetr = baro.getTemperature(MS561101BA_OSR_4096);
            //mpu6050_Temp_Values();
            Remote_TrimACC();//motor.h
            if(Status_LED == LOW)
             {
              Status_LED = HIGH;
              //digitalWrite(Pin_LED_G, LOW);
              }
            else
            {
            Status_LED = LOW;
            //digitalWrite(Pin_LED_G, HIGH);
            } 
            digitalWrite(13, Status_LED);//A
        }//end roop 1 Hz
    }//end roop 400 Hz
  }//end while roop
}
void SendToLabview() {
    unsigned int xAngle_d = (ahrs_r*32767/180.0) + 32767;
    unsigned int yAngle_d = (ahrs_p*32767/180.0) + 32767;
    unsigned int zAngle_d = (ahrs_y*32767/180.0) + 32767;
    unsigned int z1_hat_d = (z1_hat*32767/100.0) + 32767;
    unsigned int h_control_d = (h_control*32767/100.0) + 32767;
    unsigned long GpsDataLat = GPS_LAT1f*10000000;
    unsigned long GpsDataLon = GPS_LON1f*10000000;
    //Home
    unsigned long GpsDataLat_Home = GPS_LAT_HOME*10000000;
    unsigned long GpsDataLon_Home = GPS_LON_HOME*10000000;
    //Transport
    unsigned long GpsDataLat_Transport = GPS_LAT_HOME_Transport*10000000;
    unsigned long GpsDataLon_Transport = GPS_LON_HOME_Transport*10000000;
    //GPS Speed
    unsigned int GPS_speed_d = ((GPS_speed*0.036)*32767/100) + 32767;
//    //Distance From GPS_HOME and GPS_TRANSFER
    float GPS_HOME_distance = calc_dist(GPS_LAT_HOME,GPS_LON_HOME,GPS_LAT1f,GPS_LON1f);
    float GPS_TRANSPORT_distance = calc_dist(GPS_LAT_HOME_Transport,GPS_LON_HOME_Transport,GPS_LAT1f,GPS_LON1f);
    unsigned int GPS_HOME_d = (GPS_HOME_distance*32767/200) + 32767;
    unsigned int GPS_TRANSPORT_d = (GPS_TRANSPORT_distance*32767/200) + 32767;
//    //Distance Measure 
//    unsigned int Distance_L_d = ((Distance_L/10)*32767/100) + 32767;
//    unsigned int Distance_R_d = ((Distance_R/10)*32767/100) + 32767;
//    unsigned int Distance_F_d = ((Distance_F/10)*32767/100) + 32767;
//    unsigned int Distance_B_d = ((Distance_B/10)*32767/100) + 32767;
    //Motor 
//    unsigned int motor_FrontLf_d = ((motor_FrontLf/10)*32767/185.0) + 32767;
//    unsigned int motor_FrontRf_d = ((motor_FrontRf/10)*32767/185.0) + 32767;
//    unsigned int motor_BackLf_d = ((motor_BackLf/10)*32767/185.0) + 32767;
//    unsigned int motor_BackRf_d = ((motor_BackRf/10)*32767/185.0) + 32767;
    
    byte Header = 58; //0xFF = 255
    byte output1 = (xAngle_d & 0xFF);   
    byte output2 = ((xAngle_d >> 8) & 0xFF);
    byte output3 = (yAngle_d & 0xFF);   
    byte output4 = ((yAngle_d >> 8) & 0xFF);
    byte output5 = (zAngle_d & 0xFF);   
    byte output6 = ((zAngle_d >> 8) & 0xFF);
    byte output7 = (GpsDataLat & 0xFF);   
    byte output8 = ((GpsDataLat >> 8) & 0xFF);
    byte output9 = ((GpsDataLat >> 16) & 0xFF);
    byte output10 = ((GpsDataLat >> 24) & 0xFF);
    byte output11 = (GpsDataLon & 0xFF);   
    byte output12 = ((GpsDataLon >> 8) & 0xFF);
    byte output13 = ((GpsDataLon >> 16) & 0xFF);
    byte output14 = ((GpsDataLon >> 24) & 0xFF);
    byte output15 = (z1_hat_d & 0xFF);   
    byte output16 = ((z1_hat_d >> 8) & 0xFF);
    byte output17 = (Voltage & 0xFF);   
    byte output18 = ((Voltage >> 8) & 0xFF);
    byte output19 = (Ampere & 0xFF);   
    byte output20 = ((Ampere >> 8) & 0xFF);
    byte output21 = (h_control_d & 0xFF);   
    byte output22 = ((h_control_d >> 8) & 0xFF);
    //Motor PWM Control
//    byte output23 = (motor_FrontLf_d & 0xFF);   
//    byte output24 = ((motor_FrontLf_d >> 8) & 0xFF);
//    byte output25 = (motor_FrontRf_d & 0xFF);   
//    byte output26 = ((motor_FrontRf_d >> 8) & 0xFF);
//    byte output27 = (motor_BackLf_d & 0xFF);   
//    byte output28 = ((motor_BackLf_d >> 8) & 0xFF);
//    byte output29 = (motor_BackRf_d & 0xFF);   
//    byte output30 = ((motor_BackRf_d >> 8) & 0xFF);
    //Home
    byte output23 = (GpsDataLat_Home & 0xFF);   
    byte output24 = ((GpsDataLat_Home >> 8) & 0xFF);
    byte output25 = ((GpsDataLat_Home >> 16) & 0xFF);
    byte output26 = ((GpsDataLat_Home >> 24) & 0xFF);
    byte output27 = (GpsDataLon_Home & 0xFF);   
    byte output28 = ((GpsDataLon_Home >> 8) & 0xFF);
    byte output29 = ((GpsDataLon_Home >> 16) & 0xFF);
    byte output30 = ((GpsDataLon_Home >> 24) & 0xFF);
    //Transport
    byte output31 = (GpsDataLat_Transport & 0xFF);   
    byte output32 = ((GpsDataLat_Transport >> 8) & 0xFF);
    byte output33 = ((GpsDataLat_Transport >> 16) & 0xFF);
    byte output34 = ((GpsDataLat_Transport >> 24) & 0xFF); 
    byte output35 = (GpsDataLon_Transport & 0xFF);   
    byte output36 = ((GpsDataLon_Transport >> 8) & 0xFF);
    byte output37 = ((GpsDataLon_Transport >> 16) & 0xFF);
    byte output38 = ((GpsDataLon_Transport >> 24) & 0xFF);
    //GPS Speed
    byte output39 = (GPS_speed_d & 0xFF);   
    byte output40 = ((GPS_speed_d >> 8) & 0xFF);
    //GPS_Distance
    byte output41 = (GPS_HOME_d & 0xFF);   
    byte output42 = ((GPS_HOME_d >> 8) & 0xFF);
    byte output43 = (GPS_TRANSPORT_d & 0xFF);   
    byte output44 = ((GPS_TRANSPORT_d >> 8) & 0xFF);
//    //Distance Measure
//    byte output31 = (Distance_L_d & 0xFF);   
//    byte output32 = ((Distance_L_d >> 8) & 0xFF);
//    byte output33 = (Distance_R_d & 0xFF);   
//    byte output34 = ((Distance_R_d >> 8) & 0xFF);
//    byte output35 = (Distance_F_d & 0xFF);   
//    byte output36 = ((Distance_F_d >> 8) & 0xFF);
//    byte output37 = (Distance_B_d & 0xFF);   
//    byte output38 = ((Distance_B_d >> 8) & 0xFF);
//    //Distance From GPS_HOME and GPS_TRANSFER
//    byte output57 = (GPS_HOME_d & 0xFF);   
//    byte output58 = ((GPS_HOME_d >> 8) & 0xFF);
//    byte output59 = (GPS_TRANSPORT_d & 0xFF);   
//    byte output60 = ((GPS_TRANSPORT_d >> 8) & 0xFF);

     Serial3.write(Header);
     Serial3.write(output1);
     Serial3.write(output2);
     Serial3.write(output3);
     Serial3.write(output4);
     Serial3.write(output5);
     Serial3.write(output6);
     Serial3.write(output7);
     Serial3.write(output8);
     Serial3.write(output9);
     Serial3.write(output10);
     Serial3.write(output11);
     Serial3.write(output12);
     Serial3.write(output13);
     Serial3.write(output14);
     Serial3.write(output15);
     Serial3.write(output16);
     Serial3.write(output17);
     Serial3.write(output18);
//     Serial3.write(output19);
//     Serial3.write(output20);
//     Serial3.write(output21);
//     Serial3.write(output22);
//     Serial3.write(output23);
//     Serial3.write(output24);
//     Serial3.write(output25);
//     Serial3.write(output26);
//     Serial3.write(output27);
//     Serial3.write(output28);
//     Serial3.write(output29);
//     Serial3.write(output30);
//     Serial3.write(output31);
//     Serial3.write(output32);
//     Serial3.write(output33);
//     Serial3.write(output34);
//     Serial3.write(output35);
//     Serial3.write(output36);
//     Serial3.write(output37);
//     Serial3.write(output38);
//     Serial3.write(output39);
//     Serial3.write(output40);
//     Serial3.write(output41);
//     Serial3.write(output42);
//     Serial3.write(output43);
//     Serial3.write(output44);
//     Serial3.write(output45);
//     Serial3.write(output46);
//     Serial3.write(output47);
//     Serial3.write(output48);
//     Serial3.write(output49);
//     Serial3.write(output50);
//     Serial3.write(output51);
//     Serial3.write(output52);
//     Serial3.write(output53);
//     Serial3.write(output54);
//     Serial3.write(output55);
//     Serial3.write(output56);
//     Serial3.write(output57);
//     Serial3.write(output58);
//     Serial3.write(output59);
//     Serial3.write(output60);
     Serial3.println();
}
