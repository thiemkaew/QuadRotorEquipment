

//The type of multicopter  
//#define HEX_X
#define Quad_X
//#define Quad_P
#define CPPM

///////////////Mode///////////////////////////
#define AltHold 1245
#define RTH 1495
#define Loiter_Sensor 1695  //โหมด Return to Home และทำการเปิด Sensor ADNS3080 และ Distance Measure
#define PositionHold 1345  //Loiter mode
#define Auto 1795
#define FailSafe 980

//Default Configuration
///////////////Mode///////////////////////////
//#define AltHold 1248
//#define RTH 1503
//#define PositionHold 1398  //Loiter mode
//#define Auto 1846
//#define FailSafe 980


#define MINTHROTTLE 1064 //1090
#define MAXTHROTTLE 1750 //1850
#define MINCOMMAND 1000
#define MAXCOMMAND 1850
#define MIDRC 1500
#define MINCHECK 1100
#define MAXCHECK 1900
//Mode Control
int Mode = 0;
//Mode 0 = Stabilize
//Mode 1 = Altitude Hold
//Mode 2 = Position Hold ,Loiter
//Mode 3 = Automatic  Takeoff ,Landing
//Mode 4 = RTH and Landing using in failsafeMode
//Active Transport go home Control
int AutoTransHome = 0;
int AutoTransHome_sta2 = 0;
int AutoTransHome_sta3 = 0;
int AutoTransHome_WPath1 = 0;
int AutoTransHome_WPath2 = 0;
int AutoTransHome_WPath3 = 0;
int AutoTransHome_WPath4 = 0;
int AutoTransHome_WPath5 = 0;
int AutoTransHome_WPath6 = 0;
/////////////////////////////////////////////
// Automatic take-off and landing 
float h_control = 10.0;  //12.0 15.0 meter
//float h_control = 12.0;  //12.0 15.0 meter
//float h_control = 15.0;  //12.0 15.0 meter
//float h_control_WPath1 = 7.0;  //7.0 5.0 3.5 2.7 0.6 0.9 meter
//float h_control_WPath2 = 7.0;  //7.0 5.0 3.5 2.7 0.6 0.9 meter
//float h_control_WPath3 = 7.0;  //7.0 5.0 3.5 2.7 0.6 0.9 meter
//float h_control_WPath4 = 7.0;  //7.0 5.0 3.5 2.7 0.6 0.9 meter
//float h_control_WPath5 = 7.0;  //7.0 5.0 3.5 2.7 0.6 0.9 meter
//float h_control_WPath6 = 7.0;  //7.0 5.0 3.5 2.7 0.6 0.9 meter
//TESTING of SPF
//float h_control = 1.0;  //6.0 5.0 3.5 2.7 0.6 0.9 meter
float h_control_rth = 10.0;  //7.0 5.0 3.5 2.7 0.6 0.9 meter

//---------------------------------------------------------------------------------------------------------------------------------------------
//Waypoint Autonomous Control in Control_Autonomous_WithShortestPath.h
//Declare Value in Header
//Default Value
float nextDistance = 0.000;
float lessDistance = 0.000;
float nextDistance_tr = 0.000;
float lessDistance_tr = 0.000;
float nextDistance_waypoint = 0.000;
float nextDistance_waypoint_w3 = 0.000;
float allDistance = 0.000;
float allDistance_w3 = 0.000;
int total = 0;
int total_waypoint = 0;

//Total Waypoint Calculate
int total_W1 = 0;
int total_W2 = 0;
int total_W3 = 0;
int total_W4 = 0;

//Waypoint Declare
int WaypointAll = 4;
float WaypointCompare[4];
int waypointCheck = 0;
//---------------------------------------------------------------------------------------------------------------------------------------------

//Parameter system Quadrotor
#define m_quad 1.4     //kg //Default 1.3
#define L_quad 0.175   //m quad+=0.25   I = (1/3)mL2 = 0.02291
/////////////////////////////////////////////////////////////////////
float K_Roll = 0.76162;//motor 1400 Kv = 1.0
float K_Pitch = 0.76162;//motor 980 Kv = 1.15
float K_Yaw = 0.76162;//redcon 1400 Kv = 0.76162
float K_Altitude = 0.76162;
/////////////////////////////////
//P-PID-------------Rate
float Kp_rateRoll = 2.42;//2.42 2.78 1.18 5.28
float Ki_rateRoll = 1.12;//1.12 2.75
float Kd_rateRoll = 0.095;//0.15 0.085 0.025 - 0.045

float Kp_ratePitch = 2.42;//2.42 1.18 5.28
float Ki_ratePitch = 1.12;//1.12 2.75 0.5 - 2.8
float Kd_ratePitch = 0.095;//0.15 0.078 0.025 - 0.045

float Kp_rateYaw = 3.75;//2.75 3.75 5.75 1.75 - 3.450  350.0
float Ki_rateYaw = 1.85;//1.85 3.65  2.95
float Kd_rateYaw = 0.092;//0.095 0.035 0.065

//PID--------------Stable
float Kp_levelRoll= 4.95;//4.2 6.2 7.8 9.2 
//float Ki_levelRoll= 0.00;//0.0
//float Kd_levelRoll= 0.00;//0.0

float Kp_levelPitch= 4.95;//4.2 6.2 9.2 
//float Ki_levelPitch= 0.00;
//float Kd_levelPitch= 0.00;

float Kp_levelyaw= 5.15;//4.2 4.5

//stat feedback--------------Altitude
float Kp_altitude = 145.2;//155.2 225.2 265.2  175.0  165.0
float Ki_altitude = 1.15;//2.1 12.25 52.13 2.13 0.018 2.5,0.0
float Kd_altitude = 185.2;//195.2 185.2 250 280.5 315.5 120
float Kd2_altitude = 39.25;//35.25 18.25 22.25 42.5 12.2 1.25
float Ka_altitude = 12.5;//8.5 18.5 26 32.5 38.5 41.5 35 25 - 45

//////////////////RC//////////////////////////////////
//#define tarremote 0.025 // fast
//#define tarremote 0.062  //0.092 slow 0.12 0.02 0.08 remote 
#define tar 0.011 //0.012 0.015
float tarremote = 0.095;//0.065
//////////////////////////////////////////////////

//PID GPS////////////////////////////////////////////
float Kp_gps = 0.245;//0.15 0.101 2.101 5.101
float Ki_gps = 0.122;//0.68 2.68 0.25 0.085 0.15
float Kd_gps = 1.85;//2.85 3.35 4.35 1.05 1.9 4.3 0.35 1.35 3.35
float Kp_speed = 0.61;//0.37 0.27 0.15 0.35 0.095 min 0.15

//#define Pin_Laser 2
#define Pin_LED_B 4
#define Pin_LED_G 5
#define Pin_LED_R 3

//Level1
//GPS //สตาร์ท//////////////////////////////////////
//Home -- ลงจอด
//float GPS_LAT_HOME = 19.915006638;//19.915007
//float GPS_LON_HOME = 99.857070923; //99.857074
////Waypoint 1
//float waypoint1_LAT = 19.915044785;//19.915045
//float waypoint1_LON = 99.856544495;//99.856548
////Waypoint 2
//float waypoint2_LAT = 19.914815903;//19.914816
//float waypoint2_LON = 99.856758118;//99.856757

//Level 2 - พื้นที่แรกที่ทำการทดลองระบบ
//GPS //สตาร์ท//////////////////////////////////////
//Home -- ลงจอด
//float GPS_LAT_HOME = 19.914968;
//float GPS_LON_HOME = 99.856556; 
////Waypoint 1
//float waypoint1_LAT = 19.914937;
//float waypoint1_LON = 99.856856;
////Waypoint 2
//float waypoint2_LAT = 19.914784;
//float waypoint2_LON = 99.856853;
////Waypoint 3
//float waypoint3_LAT = 19.914753;
//float waypoint3_LON = 99.856637;
////Waypoint 4
//float waypoint4_LAT = 19.914659;
//float waypoint4_LON = 99.856828;
////Waypoint 5
//float waypoint5_LAT = 19.914864;
//float waypoint5_LON = 99.856990;
////Waypoint 6
//float GPS_LAT_HOME_Transport = 19.914883;
//float GPS_LON_HOME_Transport = 99.856715;

//Level 3 - พืนที่ทำการทดลองในปัจจุบับ
//GPS //สตาร์ท//////////////////////////////////////
//Home -- ลงจอด
//ได้ค่ามาจากโปรแกรม Google Map with LAT and LON
//GPS Tracking Waypoint Level 1
// Random Distance with เลือกเส้นทางเอง
//float GPS_LAT_HOME = 20.029859;
//float GPS_LON_HOME = 100.111262; 
////Waypoint 1
//float waypoint1_LAT = 20.029585;
//float waypoint1_LON = 100.111058;
////Waypoint 2
//float waypoint2_LAT = 20.029295;
//float waypoint2_LON = 100.110966;
////Waypoint 3
//float waypoint3_LAT = 20.029303;
//float waypoint3_LON = 100.111364;
////Waypoint 4
//float waypoint4_LAT = 20.029439;
//float waypoint4_LON = 100.111492;
////Waypoint 5
//float waypoint5_LAT = 20.029632;
//float waypoint5_LON = 100.111480;
////Waypoint 6
//float waypoint6_LAT = 20.029909;
//float waypoint6_LON = 100.111501;
////Transfer_Point
//float GPS_LAT_HOME_Transport = 20.029785;
//float GPS_LON_HOME_Transport = 100.111628;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////    เชียงราย    ///////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//----------------------------------------------------------------------------------
//GPS Tracking Waypoint Station 1
//ระยะทางที่กำหนดให้มี Limit Distance < 30.000 m
//เชียงราย
float GPS_LAT_HOME =  20.029279; //20.029288; //20.029151;
float GPS_LON_HOME =  100.111089; //100.111071; //100.110976; 
//Transfer_Point
float GPS_LAT_HOME_Transport = 20.029864; //20.0298768; //20.030097; //20.030062; //20.029931;
float GPS_LON_HOME_Transport = 100.111165; //100.1111296; //100.111292; //100.111351; //100.111065;

//Waypoint 1
float waypoint1_LAT_WDefault1 = 20.029401;
float waypoint1_LON_WDefault1 = 100.110917;
//Waypoint 2
float waypoint2_LAT_WDefault1 = 20.029624;
float waypoint2_LON_WDefault1 = 100.110983;
//-----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//GPS Tracking Waypoint Level Station 2
//ระยะทางที่กำหนดให้มี Limit Distance < 30.000 m
//Transfer_Point2
//float GPS_LAT_HOME_Transport = 20.029768;
//float GPS_LON_HOME_Transport = 100.111426;
//Waypoint 1
float waypoint1_LAT_WDefault2 = 20.029111;
float waypoint1_LON_WDefault2 = 100.111199;
//Waypoint 2
float waypoint2_LAT_WDefault2 = 20.029287;
float waypoint2_LON_WDefault2 = 100.111420;
//Waypoint 3
float waypoint3_LAT_WDefault2 = 20.029544;
float waypoint3_LON_WDefault2 = 100.111389;
//-----------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Waypoint 1
float waypoint1_LAT_WDefault = 20.029303;
float waypoint1_LON_WDefault = 100.111364;
//Waypoint 2
float waypoint2_LAT_WDefault = 20.029439;
float waypoint2_LON_WDefault = 100.111492;
//Waypoint 3
float waypoint3_LAT_WDefault = 20.029632;
float waypoint3_LON_WDefault = 100.111480;
//Waypoint 4
float waypoint4_LAT_WDefault = 20.029303;
float waypoint4_LON_WDefault = 100.111364;
//Waypoint 5
float waypoint5_LAT_WDefault = 20.029439;
float waypoint5_LON_WDefault = 100.111492;
//Waypoint 6
float waypoint6_LAT_WDefault = 20.029632;
float waypoint6_LON_WDefault = 100.111480;
//GPS in RealWorld
//Default Home 1
float GPS_LAT_SPF_HOME = 20.029845;
float GPS_LON_SPF_HOME = 100.111316;
//Home 2
//float GPS_LAT_SPF_HOME = 20.029699;
//float GPS_LON_SPF_HOME = 100.111252;
//Home 3
//float GPS_LAT_SPF_HOME = 20.029496;
//float GPS_LON_SPF_HOME = 100.111093; 
//Home 4
//float GPS_LAT_SPF_HOME = 20.029386;
//float GPS_LON_SPF_HOME = 100.111108;

//Default Transport 1
float GPS_LAT_SPF_TR = 20.029178;
float GPS_LON_SPF_TR = 100.111393;
//Transport 2
//float GPS_LAT_SPF_TR = 20.029178;
//float GPS_LON_SPF_TR = 100.111393;
//Transport 3
//float GPS_LAT_SPF_TR = 20.029356;
//float GPS_LON_SPF_TR = 100.111276;

//Default Waypoint Value
//Waypoint 1
float waypoint1_LAT = 20.029674;
float waypoint1_LON = 100.111120;
//Waypoint 2
float waypoint2_LAT = 20.029478;
float waypoint2_LON = 100.111024;
//Waypoint 3
float waypoint3_LAT = 20.029279;
float waypoint3_LON = 100.111143;
//Waypoint 4
float waypoint4_LAT = 20.029303;
float waypoint4_LON = 100.111364;
//Waypoint 5
float waypoint5_LAT = 20.029439;
float waypoint5_LON = 100.111492;
//Waypoint 6
float waypoint6_LAT = 20.029632;
float waypoint6_LON = 100.111480;
//Transfer_Point

//Distance Waypoint with Shortest Path Algorithm
//GPS_ALL_POINT_SYSTEM
//เชียงราย
float GPS_LAT_SPF_ALL[] = {20.029823,   20.029728,  20.029676,  20.029609,  20.029518,  20.029485,  20.029404,  20.029299,  20.029288,  20.029194};
float GPS_LON_SPF_ALL[] = {100.111226,  100.111376, 100.111156, 100.111267, 100.111391, 100.111158, 100.111284, 100.111384, 100.111148, 100.111277};
//WayPoint_Tracking_1
float GPS_LAT_SPF_W1[] = {20.029823,  20.029728,  20.029518,  20.029299,  20.029194};
float GPS_LON_SPF_W1[] = {100.111226, 100.111376, 100.111391, 100.111384, 100.111277};
//WayPoint_Tracking_2
float GPS_LAT_SPF_W2[] = {20.029823,  20.029728,  20.029609,  20.029404,  20.029194};
float GPS_LON_SPF_W2[] = {100.111226, 100.111376, 100.111267, 100.111284, 100.111277};
//WayPoint_Tracking_3
float GPS_LAT_SPF_W3[] = {20.029823,  20.029676,  20.029485,  20.029404,  20.029288,  20.029194};
float GPS_LON_SPF_W3[] = {100.111226, 100.111156, 100.111158, 100.111284, 100.111148, 100.111277};
//WayPoint_Tracking_4
float GPS_LAT_SPF_W4[] = {20.029823,  20.029676,  20.029485,  20.029404,  20.029194};
float GPS_LON_SPF_W4[] = {100.111226, 100.111156, 100.111158, 100.111284, 100.111277};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////    เชียงใหม่    ///////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////GPS Tracking Waypoint Level 4
////ระยะทางที่กำหนดให้มี Limit Distance < 30.000 m
////เชียงใหม่ 18.805893,98.955291
//float GPS_LAT_HOME = 18.805893;
//float GPS_LON_HOME = 98.955291; 
////Waypoint 1 18.805939,98.955350
//float waypoint1_LAT_WDefault = 18.805939;
//float waypoint1_LON_WDefault = 98.955350;
////Waypoint 2 
//float waypoint2_LAT_WDefault = 18.806018;
//float waypoint2_LON_WDefault = 98.955158;
////Waypoint 3
//float waypoint3_LAT_WDefault = 18.806075;
//float waypoint3_LON_WDefault = 98.955364;
////Waypoint 4
//float waypoint4_LAT_WDefault = 18.806150;
//float waypoint4_LON_WDefault = 98.955154;
////Waypoint 5
//float waypoint5_LAT_WDefault = 18.806213;
//float waypoint5_LON_WDefault = 98.955340;
////Waypoint 6
//float waypoint6_LAT_WDefault = 18.806335;
//float waypoint6_LON_WDefault = 98.955145;
////Transfer_Point 18.806413,98.955254
//float GPS_LAT_HOME_Transport = 18.806413;
//float GPS_LON_HOME_Transport = 98.955254;
////GPS in RealWorld
//
////Default Home 1
//float GPS_LAT_SPF_HOME = 18.805947;
//float GPS_LON_SPF_HOME = 98.955282;
////Default Transport 1
//float GPS_LAT_SPF_TR = 18.806506;
//float GPS_LON_SPF_TR = 98.955321;
//
////Default Waypoint Value
//float waypoint1_LAT = 18.805939;
//float waypoint1_LON = 98.955350;
////Waypoint 2 
//float waypoint2_LAT = 18.806018;
//float waypoint2_LON = 98.955158;
////Waypoint 3
//float waypoint3_LAT = 18.806075;
//float waypoint3_LON = 98.955364;
////Waypoint 4
//float waypoint4_LAT = 18.806150;
//float waypoint4_LON = 98.955154;
////Waypoint 5
//float waypoint5_LAT = 18.806213;
//float waypoint5_LON = 98.955340;
////Waypoint 6
//float waypoint6_LAT = 18.806335;
//float waypoint6_LON = 98.955145;
//
////Distance Waypoint with Shortest Path Algorithm
////GPS_ALL_POINT_SYSTEM
////เชียงใหม่
//
//  float GPS_LAT_SPF_ALL[] = {18.805992,   18.806068,  18.806046,  18.806158,  18.806250,  18.806258,  18.806324,  18.806378,  18.806378,  18.806473};
//  float GPS_LON_SPF_ALL[] = {98.955220,   98.955103,  98.955336,  98.955222,  98.955114,  98.955351,  98.955229,  98.955164,  98.955305,  98.955241};
//  //Waypoint_Track_1
//  float GPS_LAT_SPF_W1[] = {18.805992,  18.806068,  18.806250,  18.806378,  18.806473};
//  float GPS_LON_SPF_W1[] = {98.955220,  98.955103,  98.955114,  98.955164,  98.955241};
//  //Waypoint_Track_2
//  float GPS_LAT_SPF_W2[] = {18.805992,  18.806068,  18.806158,  18.806324,  18.806473};
//  float GPS_LON_SPF_W2[] = {98.955220,  98.955103,  98.955222,  98.955229,  98.955241};
//  //Waypoint_Track_3
//  float GPS_LAT_SPF_W3[] = {18.805992,  18.806046,  18.806258,  18.806324,  18.806378,  18.806473};
//  float GPS_LON_SPF_W3[] = {98.955220,  98.955336,  98.955351,  98.955229,  98.955305,  98.955241};
//  //Waypoint_Track_4
//  float GPS_LAT_SPF_W4[] = {18.805992,  18.806046,  18.806258,  18.806324,  18.806473};
//  float GPS_LON_SPF_W4[] = {98.955220,  98.955336,  98.955351,  98.955229,  98.955241};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Default ปิด
int Servo_trigger = 1850;
// Default เปิด
//int Servo_trigger = 1020;
////////////////////////////////////////////////////////////////////
//Accelerometer calibration constants; use the Calibrate example from print(accelRaw[XAXIS]);
// Update Value in 08/10/2016 21:04
int A_X_MIN = -4250;    //
int A_X_MAX = 3945;     //
int A_Y_MIN = -3950;    //
int A_Y_MAX = 4215;     //
int A_Z_MIN = -4350;    //
int A_Z_MAX = 3980;     //4007

// Update Value in 29/05/2016 21:04
//int A_X_MIN = -4084;    //
//int A_X_MAX = 4085;     //
//int A_Y_MIN = -3972;    //
//int A_Y_MAX = 4230;     //
//int A_Z_MIN = -4395;    //
//int A_Z_MAX = 3942;     //4007

// Update Value in 31/01/2016 21:04
//int A_X_MIN = -4225;    //
//int A_X_MAX = 3950;     //
//int A_Y_MIN = -3927;    //
//int A_Y_MAX = 4250;     //
//int A_Z_MIN = -4120;    //
//int A_Z_MAX = 4190;     //4007

////////////////////////////////////////////////////////////////////
//magnetometer calibration constants; use the Calibrate example from print(MagXf);
// the Pololu library to find the right values for your board
// Update Value in 08/10/2016 21:04
int M_X_MIN = -264;    //-490 -654  -693   -688
int M_X_MAX = 544;     //310 185   209    170
int M_Y_MIN = -359;    //-369 -319  -311   -310
int M_Y_MAX = 478;     //397 513   563    546
int M_Z_MIN = -360;    //-392 -363  -374   -377
int M_Z_MAX = 380;     //346 386   429    502

// Update Value in 29/05/2016 21:04
//int M_X_MIN = -254;    //-490 -654  -693   -688
//int M_X_MAX = 521;     //310 185   209    170
//int M_Y_MIN = -363;    //-369 -319  -311   -310
//int M_Y_MAX = 450;     //397 513   563    546
//int M_Z_MIN = -327;    //-392 -363  -374   -377
//int M_Z_MAX = 377;     //346 386   429    502

// Update Value in 31/01/2016 21:04
//int M_X_MIN = -340;    //-490 -654  -693   -688
//int M_X_MAX = 640;     //310 185   209    170
//int M_Y_MIN = -470;    //-369 -319  -311   -310
//int M_Y_MAX = 552;     //397 513   563    546
//int M_Z_MIN = -412;    //-392 -363  -374   -377
//int M_Z_MAX = 485;     //346 386   429    502

////////////////////////////////////////////////////////////////////
//Observer hz
float Altitude_Hold = 0.0;
//float Altitude_hat=0.0;//Observer hx
float vx_hat=0.0;
float vx_hat2=0.0;
float vy_hat=0.0;
float vy_hat2=0.0;
//float vz_hat=0.0;
//float vz_hat2=0.0;
//float h=0.0;
float seth=0.0;//set control
float uthrottle=0.0;
float uAltitude = 1000.0;
float accrX_Earth = 0.0;
float accrY_Earth = 0.0;
float accrZ_Earth = 0.0;
float accrX_Earthf = 0.0;
float accrY_Earthf = 0.0;
float accrZ_Earthf = 0.0;
//float vz = 0.0;
//kalman
float z1_hat = 0.0;
float z2_hat = 0.0;
float z1_hat2 = 0.0;
float z2_hat2 = 0.0;
float u_z = 0.0;
float baro_vz = 0.0;
float baro_vz_old = 0.0;
float baro_vz_old2 = 0.0;

//GPS
float GPS_LAT1 = 0.0;
float GPS_LON1 = 0.0;
float GPS_LAT1f = 0.0;
float GPS_LON1f = 0.0;
float GPS_LAT1Lf = 0.0;
float GPS_LON1Lf = 0.0;
float GPS_LAT1lead = 0.0;
float GPS_LON1lead = 0.0;
float GPS_speed = 0.0;
float actual_speedX = 0.0;
float actual_speedY = 0.0;
float actual_speedXf = 0.0;
float actual_speedYf = 0.0;
float actual_speedXold = 0.0;
float actual_speedYold = 0.0;
float _last_velocityX = 0.0;
float _last_velocityY = 0.0;
float GPS_LAT1_old = GPS_LAT_HOME;
float GPS_LON1_old = GPS_LON_HOME;
float Control_XEf = 0.0;
float Control_YEf = 0.0;
float Control_XBf = 0.0;
float Control_YBf = 0.0;
float target_LAT = 0.0;
float target_LON = 0.0;
byte currentCommand[23];
byte Read_command = 0;
float GPS_hz = 0.0;
float GPS_vz = 0.0;
float GPS_ground_course2 = 0.0;
float GPS_Distance = 0.0;
float error_LAT = 0.0;
float error_LON = 0.0;  
float GPSI_LAT = 0.0;
float GPSI_LON = 0.0;  
uint8_t GPS_filter_index = 0;
float GPS_SUM_LAT[5];
float GPS_SUM_LON[5];
bool Status_LED_GPS = HIGH;

//Automatic Takeoff and Landing
uint8_t Status_waypoint = 0;
uint8_t Status_waypoint_tranH = 0;
//Automatic Takeoff and Landing Station 2
uint8_t Status_waypoint_sta2 = 0;
uint8_t Status_waypoint_tranH_sta2 = 0;
//Automatic Takeoff and Landing Station 2
uint8_t Status_waypoint_sta3 = 0;
uint8_t Status_waypoint_tranH_sta3 = 0;

//Automatic Takeoff and Landing_WPath1
uint8_t Status_waypoint_WPath1 = 0;
uint8_t Status_waypoint_tranH_WPath1 = 0;
//Automatic Takeoff and Landing_WPath2
uint8_t Status_waypoint_WPath2 = 0;
uint8_t Status_waypoint_tranH_WPath2 = 0;
//Automatic Takeoff and Landing_WPath3
uint8_t Status_waypoint_WPath3 = 0;
uint8_t Status_waypoint_tranH_WPath3 = 0;
//Automatic Takeoff and Landing_WPath4
uint8_t Status_waypoint_WPath4 = 0;
uint8_t Status_waypoint_tranH_WPath4 = 0;
//Automatic Takeoff and Landing_WPath5
uint8_t Status_waypoint_WPath5 = 0;
uint8_t Status_waypoint_tranH_WPath5 = 0;
//Automatic Takeoff and Landing_WPath6
uint8_t Status_waypoint_WPath6 = 0;
uint8_t Status_waypoint_tranH_WPath6 = 0;


//RTH and Landing
uint8_t Status_waypoint_rth = 0;

uint8_t Counter_LED_GPS = 0;
int Voltage = 0;
int Ampere = 0;
////////time roop////////////////////////////////////////////
#define TASK_100HZ 2
#define TASK_50HZ 4
#define TASK_20HZ 10
#define TASK_10HZ 20
#define TASK_5HZ 40
#define TASK_2HZ 100
#define TASK_1HZ 200
#define TASK_NoHZ 410
//#define RAD_TO_DEG 57.295779513082320876798154814105
//#define DEG_TO_RAD 0.017453292519943295769236907684886

//direction cosine matrix (DCM)   Rotated Frame to Stationary Frame ZYX
float DCM00 = 1.0;
float DCM01 = 0.0;
float DCM02 = 0.0;
float DCM10 = 0.0;
float DCM11 = 1.0;
float DCM12 = 0.0;
float DCM20 = 0.0;
float DCM21 = 0.0;
float DCM22 = 1.0;
//float DCM23 = 1.0;
float cos_rollcos_pitch = 1.0;

// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;

uint16_t frameCounter = 0;
//Automatic Takeoff and Landing
uint8_t timeLanding = 0;
uint8_t timeOff = 0;
uint8_t timeLanding_tranH = 0;
uint8_t timeOff_tranH = 0;
uint8_t timeOff_Operate = 0;
uint8_t timeOff_Operate_tranH = 0;

//Automatic Takeoff and Landing Station 2
uint8_t timeLanding_sta2 = 0;
uint8_t timeOff_sta2 = 0;
uint8_t timeLanding_tranH_sta2 = 0;
uint8_t timeOff_tranH_sta2 = 0;
uint8_t timeOff_Operate_sta2 = 0;
uint8_t timeOff_Operate_tranH_sta2 = 0;

//Automatic Takeoff and Landing Station 2
uint8_t timeLanding_sta3 = 0;
uint8_t timeOff_sta3 = 0;
uint8_t timeLanding_tranH_sta3 = 0;
uint8_t timeOff_tranH_sta3 = 0;
uint8_t timeOff_Operate_sta3 = 0;
uint8_t timeOff_Operate_tranH_sta3 = 0;



//Automatic Takeoff and Landing_WPath1
uint8_t timeLanding_WPath1 = 0;
uint8_t timeOff_WPath1 = 0;
uint8_t timeLanding_tranH_WPath1 = 0;
uint8_t timeOff_tranH_WPath1 = 0;
uint8_t timeOff_Operate_WPath1 = 0;
uint8_t timeOff_Operate_tranH_WPath1 = 0;

//Automatic Takeoff and Landing_WPath2
uint8_t timeLanding_WPath2 = 0;
uint8_t timeOff_WPath2 = 0;
uint8_t timeLanding_tranH_WPath2 = 0;
uint8_t timeOff_tranH_WPath2 = 0;
uint8_t timeOff_Operate_WPath2 = 0;
uint8_t timeOff_Operate_tranH_WPath2 = 0;

//Automatic Takeoff and Landing_WPath3
uint8_t timeLanding_WPath3 = 0;
uint8_t timeOff_WPath3 = 0;
uint8_t timeLanding_tranH_WPath3 = 0;
uint8_t timeOff_tranH_WPath3 = 0;
uint8_t timeOff_Operate_WPath3 = 0;
uint8_t timeOff_Operate_tranH_WPath3 = 0;

//Automatic Takeoff and Landing_WPath4
uint8_t timeLanding_WPath4 = 0;
uint8_t timeOff_WPath4 = 0;
uint8_t timeLanding_tranH_WPath4 = 0;
uint8_t timeOff_tranH_WPath4 = 0;
uint8_t timeOff_Operate_WPath4 = 0;
uint8_t timeOff_Operate_tranH_WPath4 = 0;

//Automatic Takeoff and Landing_WPath5
uint8_t timeLanding_WPath5 = 0;
uint8_t timeOff_WPath5 = 0;
uint8_t timeLanding_tranH_WPath5 = 0;
uint8_t timeOff_tranH_WPath5 = 0;
uint8_t timeOff_Operate_WPath5 = 0;
uint8_t timeOff_Operate_tranH_WPath5 = 0;

//Automatic Takeoff and Landing_WPath6
uint8_t timeLanding_WPath6 = 0;
uint8_t timeOff_WPath6 = 0;
uint8_t timeLanding_tranH_WPath6 = 0;
uint8_t timeOff_tranH_WPath6 = 0;
uint8_t timeOff_Operate_WPath6 = 0;
uint8_t timeOff_Operate_tranH_WPath6 = 0;



//RTH and Landing
uint8_t timeLanding_rth = 0;
uint8_t timeOff_rth = 0;

byte armed = 0;
float G_Dt = 0.01; 
long Dt_sensor = 1000;
long Dt_roop = 10000;
bool Status_LED = LOW;
int ESC_calibra = 0;

//Baro
//MS561101BA32bit baro = MS561101BA32bit();
AP_Baro_MS5611 baro;
#define  MOVAVG_SIZE 10 //100 80 20
float movavg_buff[MOVAVG_SIZE];
float movavg_buffT[MOVAVG_SIZE];
int movavg_i=0;
float sea_press=1013.25;
float temperaturetr = 32.5;
float temperaturetrf = 32.5;
float presser=1013.25;
float presserf=1013.25;
float presserfF=1013.25;
float Altitude_baro = 0.0;
float Altitude_barof=0.0;
float Altitude_Ground = 0.0;


//RTH and Landing Mode of FailSafeMode

