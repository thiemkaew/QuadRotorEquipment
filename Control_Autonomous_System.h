// Waypoint with Path Control
/*************************************************************************
 * //Path Control in Desing เชียงราย
 *************************************************************************/
/*

      Path 1 - W1, W2 ,W5, W8, W10
      Path 2 - W1, W2, W4, W7, W10
      Path 3 - W1, W3, W6, W7, W9, W10
      Path 4 - W1, W3, W6, W7, W10
      
 * GPS_TR_LAT_W1[] = {20.029823,  20.029728,  20.029518,  20.029299,  20.029194};
 * GPS_TR_LON_W1[] = {100.111226, 100.111376, 100.111391, 100.111384, 100.111277};
 * 
 * GPS_TR_LAT_W2[] = {20.029823,  20.029728,  20.029609,  20.029404,  20.029194};
 * GPS_TR_LON_W2[] = {100.111226, 100.111376, 100.111267, 100.111284, 100.111277};
 * 
 * GPS_TR_LAT_W3[] = {20.029823,  20.029676,  20.029485,  20.029404,  20.029288,  20.029194};
 * GPS_TR_LON_W3[] = {100.111226, 100.111156, 100.111158, 100.111284, 100.111148, 100.111277};
 * 
 * GPS_TR_LAT_W4[] = {20.029823,  20.029676,  20.029485,  20.029404,  20.029194};
 * GPS_TR_LON_W4[] = {100.111226, 100.111156, 100.111158, 100.111284, 100.111277};
 * 
 *    Home_Point - 20.029923, 100.111293,
      W1 - 20.029823, 100.111226,
      W2 - 20.029728, 100.111376,
      W3 - 20.029676, 100.111156,
      W4 - 20.029609, 100.111267,
      W5 - 20.029518, 100.111391,
      W6 - 20.029485, 100.111158,
      W7 - 20.029404, 100.111284,
      W8 - 20.029299, 100.111384,
      W9 - 20.029288, 100.111148,
      W10 - 20.029194, 100.111277,
      Transfer_Point - 20.029097, 100.111271,


/*************************************************************************
 * //Path Control in Desing เชียงใหม่
 *************************************************************************/
/*

      Path 1 - W1, W2 ,W5, W8, W10
      Path 2 - W1, W2, W4, W7, W10
      Path 3 - W1, W3, W6, W7, W9, W10
      Path 4 - W1, W3, W6, W7, W10

  float GPS_LAT_SPF_ALL[] = {18.805992,   18.806068,  18.806046,  18.806158,  18.806250,  18.806258,  18.806324,  18.806378,  18.806378,  18.806473};
  float GPS_LON_SPF_ALL[] = {98.955220,   98.955103,  98.955336,  98.955222,  98.955114,  98.955351,  98.955229,  98.955164,  98.955305,  98.955241};
 
  float GPS_LAT_SPF_W1[] = {18.805992,  18.806068,  18.806250,  18.806378,  18.806473};
  float GPS_LON_SPF_W1[] = {98.955220,  98.955103,  98.955114,  98.955164,  98.955241};
 
  float GPS_LAT_SPF_W2[] = {18.805992,  18.806068,  18.806158,  18.806324,  18.806473};
  float GPS_LON_SPF_W2[] = {98.955220,  98.955103,  98.955222,  98.955229,  98.955241};
 
  float GPS_LAT_SPF_W3[] = {18.805992,  18.806046,  18.806258,  18.806324,  18.806378,  18.806473};
  float GPS_LON_SPF_W3[] = {98.955220,  98.955336,  98.955351,  98.955229,  98.955305,  98.955241};
 
  float GPS_LAT_SPF_W4[] = {18.805992,  18.806046,  18.806258,  18.806324,  18.806473};
  float GPS_LON_SPF_W4[] = {98.955220,  98.955336,  98.955351,  98.955229,  98.955241};
  
     Home_Point - 18.805947, 98.955282,
      W1 - 18.805992, 98.955220,
      W2 - 18.806068, 98.955103,
      W3 - 18.806046, 98.955336,
      W4 - 18.806158, 98.955222,
      W5 - 18.806250, 98.955114,
      W6 - 18.806258, 98.955351,
      W7 - 18.806324, 98.955229,
      W8 - 18.806378, 98.955164,
      W9 - 18.806378, 98.955305,
      W10 - 18.806473,98.955241,
      Transfer_Point - 18.806506, 98.955321,


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
 */

//Control_Path and System
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------- ตัวอย่างกระบวนการทำงานที่สำเร็จและทำงานได้อย่างสมบูรณ์ ----------------------------------------------------------------------
// Automatic Transport System
// Using Finite State Machine
// ---------------------------------------------------------------------------------------------------------------
//void AutoTransportOperating(){    
//
//  if (Mode == 3 && CH_THRf > MINCHECK)
//  {
//    //เริ่มการทำงาน โดยที่ให้ Takeoff ที่จุด Home Point แล้วไปยังจุด Transport Point โดยใช้ Waypoint เป็นตัวกำหนด
//    //-------------
//    AutomaticTransport();
//     // ถึงจุดหมายและนับเวลาการ armed == 0; หลังจากนั้น รอเวลาประมาณ 10 วินาที ให้ทำ takeoff ขึ้น
//      if(time_AutoOperate > 36 && endAuto == 0 && AutoTransHome == 0)
//        {
//          timeOff_Operate++;
//          //relay 2 s and < 12 s ( 10 s ) time-Off to Disarm and Open time Transporter
//          if(timeOff_Operate > 20 && timeOff_Operate < 200) // 2 s - Disarmed Quadrotor
//          {
//              if(z1_hat <= 0.18)
//              {
//                  armed = 0;
//                //เปิดกล่องเพื่อทำการส่ง
//                //Servo_trigger = 1020;
//              }
//          } 
//          if(timeOff_Operate > 200) //Wait for 20 - 2 s = 18 s for Active Again
//          {
//            AutoTransHome = 1;
//            armed = 1;
//            timeOff_Operate = 0;
//            // ปิดกล่องเพื่อทำการส่ง
//            // Servo_trigger = 1020;
//          } 
//        } 
//    
//    //------------- 
//    //ระบบการทำงานเชื่อมต่อกัน ระหว่าง AutomaticTransport และจะเริ่มทำงานที่
//    //-------------
//    
//    //เริ่มการทำงานเมื่อไปถึงจุด Transport แล้ว Takeoff กลับมายัง Home
//    //-------------
//    AutomaticTransport_Home();
//    //-------------
//      // ถึงจุดหมายและนับเวลาการ armed == 0;
//      if(time_AutoOperate > 72 && endAuto_tranH == 0 && AutoTransHome == 1)
//        {
//          timeOff_Operate_tranH++;
//          //relay 2 s time-Off to Disarm and Open time Transporter
//          if(timeOff_Operate_tranH > 20)
//          {
//              if(z1_hat <= 0.18)
//              {
//                  armed = 0;
//                //เปิดกล่องเพื่อทำการส่ง
//                // Servo_trigger = 1020;
//              }
//          } 
//        }  
//  }
//  else
//  {
//    time_AutoOperate = 0;
//    //time_auto_tranH = 0;
//    //time_auto = 0;
//
//    //Default Value Reset of AutomaticTransport();
//    takeoff = 0;
//    timeLanding = 0;
//    timeOff = 0;
//    time_auto = 0;
//    h_counter = 0.1;//0.0
//    h_counter_old = 0.1;
//    Vz_Hold = 0.0;
//    Status_waypoint = 0;
//    
//
//    //Default Value Reset of AutomaticTransport_Home();
//    takeoff_tranH = 0;
//    timeLanding_tranH = 0;
//    timeOff_tranH = 0;
//    time_auto_tranH = 0;
//    h_counter_tranH = 0.1;//0.0
//    h_counter_old_tranH = 0.1;
//    Vz_Hold_tranH = 0.0;
//    Status_waypoint_tranH = 0;
//    
//    AutoTransHome = 0;
//    timeOff_Operate = 0;
//    timeOff_Operate_tranH = 0;
//  } 
//}

// ---------------------------------------------------------------------------------------------------------------


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void AutoTransportOperating_SPF(){
  if (Mode == 3 && CH_THRf > MINCHECK)
  {
    //เริ่มการทำงาน โดยที่ให้ Takeoff ที่จุด Home Point แล้วไปยังจุด Transport Point โดยใช้ Waypoint เป็นตัวกำหนด
    //-------------
    // เริ่มการทำงานโดนกำหนดค่า waypoint LAT LON จากการคำนวณจาก Shortest Path
//    if(waypointCheck == 0 && AUX_3 >= 1700)
//    {
//      Waypoint_ShortestPath();
//    }
//    //--------------------------- เลือกการคำนวณหา Shortest Path บน Waypoint ที่กำหนด ---------------------------------------
//    //Waypoint = 1 point and Less of Distance
//    if (waypointCheck == 1 && AUX_3 >= 1700)
//    {
//        AutomaticTransport_WPath1();
//            if(time_AutoOperate_WPath1 > 32 && endAuto_WPath1 == 0 && AutoTransHome_WPath1 == 0)
//            {
//              timeOff_Operate_WPath1++;
//              //relay 2 s and < 12 s ( 10 s ) time-Off to Disarm and Open time Transporter
//              if(timeOff_Operate_WPath1 > 20 && timeOff_Operate_WPath1 < 200) // 2 s - Disarmed Quadrotor
//              {
//                if(z1_hat <= 0.18)
//                {
//                    armed = 0;
//                   //เปิดกล่องเพื่อทำการส่ง
//                   //Servo_trigger = 1020;
//                }
//              } 
//              if(timeOff_Operate_WPath1 > 200) //Wait for 20 - 2 s = 18 s for Active Again
//              {
//                AutoTransHome_WPath1 = 1;
//                armed = 1;
//                timeOff_Operate_WPath1 = 0;
//                // ปิดกล่องเพื่อทำการส่ง
//                // Servo_trigger = 1020;
//              }
//            }
//        AutomaticTransport_Home_WPath1();
//            if(time_AutoOperate_WPath1 > 64 && endAuto_tranH_WPath1 == 0 && AutoTransHome_WPath1 == 1)
//            {
//              timeOff_Operate_tranH_WPath1++;
//              //relay 2 s time-Off to Disarm and Open time Transporter
//              if(timeOff_Operate_tranH_WPath1 > 20)
//              {
//                if(z1_hat <= 0.18)
//                {
//                    armed = 0;
//                   //เปิดกล่องเพื่อทำการส่ง
//                   // Servo_trigger = 1020;
//                }
//              } 
//            }
//    }
//    else
//    {
////////////////////////////////////////////// _WPath1 //////////////////////////////////////////////////////////////////////////////////////
//
//    time_AutoOperate_WPath1 = 0;
//    
//    //Default Value Reset of AutomaticTransport_WPath1();
//    takeoff_WPath1 = 0;
//    timeLanding_WPath1 = 0;
//    timeOff_WPath1 = 0;
//    time_auto_WPath1 = 0;
//    h_counter_WPath1 = 0.1;//0.0
//    h_counter_old_WPath1 = 0.1;
//    Vz_Hold_WPath1 = 0.0;
//    Status_waypoint_WPath1 = 0;
//    
//
//    //Default Value Reset of AutomaticTransport_Home_WPath1();
//    takeoff_tranH_WPath1 = 0;
//    timeLanding_tranH_WPath1 = 0;
//    timeOff_tranH_WPath1 = 0;
//    time_auto_tranH_WPath1 = 0;
//    h_counter_tranH_WPath1 = 0.1;//0.0
//    h_counter_old_tranH_WPath1 = 0.1;
//    Vz_Hold_tranH_WPath1 = 0.0;
//    Status_waypoint_tranH_WPath1 = 0;
//    
//    AutoTransHome_WPath1 = 0;
//    timeOff_Operate_WPath1 = 0;
//    timeOff_Operate_tranH_WPath1 = 0;
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    }
//    //Waypoint = 2 point and Less of Distance
//    if (waypointCheck == 2 && AUX_3 >= 1700)
//    {
//        AutomaticTransport_WPath2();
//            if(time_AutoOperate_WPath2 > 40 && endAuto_WPath2 == 0 && AutoTransHome_WPath2 == 0)
//            {
//              timeOff_Operate_WPath2++;
//              //relay 2 s and < 12 s ( 10 s ) time-Off to Disarm and Open time Transporter
//              if(timeOff_Operate_WPath2 > 20 && timeOff_Operate_WPath2 < 200) // 2 s - Disarmed Quadrotor
//              {
//                if(z1_hat <= 0.18)
//                {
//                    armed = 0;
//                   //เปิดกล่องเพื่อทำการส่ง
//                   //Servo_trigger = 1020;
//                }
//              } 
//              if(timeOff_Operate_WPath2 > 200) //Wait for 20 - 2 s = 18 s for Active Again
//              {
//                AutoTransHome_WPath2 = 1;
//                armed = 1;
//                timeOff_Operate_WPath2 = 0;
//                // ปิดกล่องเพื่อทำการส่ง
//                // Servo_trigger = 1020;
//              }
//            }
//        AutomaticTransport_Home_WPath2();
//            if(time_AutoOperate_WPath2 > 80 && endAuto_tranH_WPath2 == 0 && AutoTransHome_WPath2 == 1)
//            {
//              timeOff_Operate_tranH_WPath2++;
//              //relay 2 s time-Off to Disarm and Open time Transporter
//              if(timeOff_Operate_tranH_WPath2 > 20)
//              {
//                if(z1_hat <= 0.18)
//                {
//                    armed = 0;
//                   //เปิดกล่องเพื่อทำการส่ง
//                   // Servo_trigger = 1020;
//                }
//              } 
//            }
//    }
//    else
//    {
////////////////////////////////////////////// _WPath2 //////////////////////////////////////////////////////////////////////////////////////
//
//    time_AutoOperate_WPath2 = 0;
//    
//    //Default Value Reset of AutomaticTransport_WPath2();
//    takeoff_WPath2 = 0;
//    timeLanding_WPath2 = 0;
//    timeOff_WPath2 = 0;
//    time_auto_WPath2 = 0;
//    h_counter_WPath2 = 0.1;//0.0
//    h_counter_old_WPath2 = 0.1;
//    Vz_Hold_WPath2 = 0.0;
//    Status_waypoint_WPath2 = 0;
//    
//
//    //Default Value Reset of AutomaticTransport_Home_WPath2();
//    takeoff_tranH_WPath2 = 0;
//    timeLanding_tranH_WPath2 = 0;
//    timeOff_tranH_WPath2 = 0;
//    time_auto_tranH_WPath2 = 0;
//    h_counter_tranH_WPath2 = 0.1;//0.0
//    h_counter_old_tranH_WPath2 = 0.1;
//    Vz_Hold_tranH_WPath2 = 0.0;
//    Status_waypoint_tranH_WPath2 = 0;
//    
//    AutoTransHome_WPath2 = 0;
//    timeOff_Operate_WPath2 = 0;
//    timeOff_Operate_tranH_WPath2 = 0;
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    }
//    //Waypoint = 3 point and Less of Distance
//    if (waypointCheck == 3 && AUX_3 >= 1700)
//    {
//        AutomaticTransport_WPath3();
//            if(time_AutoOperate_WPath3 > 48 && endAuto_WPath3 == 0 && AutoTransHome_WPath3 == 0)
//            {
//              timeOff_Operate_WPath3++;
//              //relay 2 s and < 12 s ( 10 s ) time-Off to Disarm and Open time Transporter
//              if(timeOff_Operate_WPath3 > 20 && timeOff_Operate_WPath3 < 200) // 2 s - Disarmed Quadrotor
//              {
//                if(z1_hat <= 0.18)
//                {
//                    armed = 0;
//                   //เปิดกล่องเพื่อทำการส่ง
//                   //Servo_trigger = 1020;
//                }
//              } 
//              if(timeOff_Operate_WPath3 > 200) //Wait for 20 - 2 s = 18 s for Active Again
//              {
//                AutoTransHome_WPath3 = 1;
//                armed = 1;
//                timeOff_Operate_WPath3 = 0;
//                // ปิดกล่องเพื่อทำการส่ง
//                // Servo_trigger = 1020;
//              }
//            }
//        AutomaticTransport_Home_WPath3();
//            if(time_AutoOperate_WPath3 > 96 && endAuto_tranH_WPath3 == 0 && AutoTransHome_WPath3 == 1)
//            {
//              timeOff_Operate_tranH_WPath3++;
//              //relay 2 s time-Off to Disarm and Open time Transporter
//              if(timeOff_Operate_tranH_WPath3 > 20)
//              {
//                if(z1_hat <= 0.18)
//                {
//                    armed = 0;
//                   //เปิดกล่องเพื่อทำการส่ง
//                   // Servo_trigger = 1020;
//                }
//              } 
//            }
//    }
//    else
//    {
////////////////////////////////////////////// _WPath3 //////////////////////////////////////////////////////////////////////////////////////
//
//    time_AutoOperate_WPath3 = 0;
//
//    //Default Value Reset of AutomaticTransport_WPath3();
//    takeoff_WPath3 = 0;
//    timeLanding_WPath3 = 0;
//    timeOff_WPath3 = 0;
//    time_auto_WPath3 = 0;
//    h_counter_WPath3 = 0.1;//0.0
//    h_counter_old_WPath3 = 0.1;
//    Vz_Hold_WPath3 = 0.0;
//    Status_waypoint_WPath3 = 0;
//    
//
//    //Default Value Reset of AutomaticTransport_Home_WPath3();
//    takeoff_tranH_WPath3 = 0;
//    timeLanding_tranH_WPath3 = 0;
//    timeOff_tranH_WPath3 = 0;
//    time_auto_tranH_WPath3 = 0;
//    h_counter_tranH_WPath3 = 0.1;//0.0
//    h_counter_old_tranH_WPath3 = 0.1;
//    Vz_Hold_tranH_WPath3 = 0.0;
//    Status_waypoint_tranH_WPath3 = 0;
//    
//    AutoTransHome_WPath3 = 0;
//    timeOff_Operate_WPath3 = 0;
//    timeOff_Operate_tranH_WPath3 = 0;
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    }
//    
//    //Waypoint = 4 point and Less of Distance
//    if (waypointCheck == 4 && AUX_3 >= 1700)
//    {
//        AutomaticTransport_WPath4();
//          if(time_AutoOperate_WPath4 > 56 && endAuto_WPath4 == 0 && AutoTransHome_WPath4 == 0)
//            {
//              timeOff_Operate_WPath4++;
//              //relay 2 s and < 12 s ( 10 s ) time-Off to Disarm and Open time Transporter
//              if(timeOff_Operate_WPath4 > 20 && timeOff_Operate_WPath4 < 200) // 2 s - Disarmed Quadrotor
//              {
//                if(z1_hat <= 0.18)
//                {
//                    armed = 0;
//                   //เปิดกล่องเพื่อทำการส่ง
//                   //Servo_trigger = 1020;
//                }
//              } 
//              if(timeOff_Operate_WPath4 > 200) //Wait for 20 - 2 s = 18 s for Active Again
//              {
//                AutoTransHome_WPath4 = 1;
//                armed = 1;
//                timeOff_Operate_WPath4 = 0;
//                // ปิดกล่องเพื่อทำการส่ง
//                // Servo_trigger = 1020;
//              } 
//            } 
//        AutomaticTransport_Home_WPath4();
//           if(time_AutoOperate_WPath4 > 112 && endAuto_tranH_WPath4 == 0 && AutoTransHome_WPath4 == 1)
//            {
//              timeOff_Operate_tranH_WPath4++;
//              //relay 2 s time-Off to Disarm and Open time Transporter
//              if(timeOff_Operate_tranH_WPath4 > 20)
//              {
//                if(z1_hat <= 0.18)
//                {
//                    armed = 0;
//                   //เปิดกล่องเพื่อทำการส่ง
//                   // Servo_trigger = 1020;
//                }
//              } 
//            }
//    }
//    else
//    {
////////////////////////////////////////////// _WPath4 //////////////////////////////////////////////////////////////////////////////////////
//
//    time_AutoOperate_WPath4 = 0;
//
//    //Default Value Reset of AutomaticTransport_WPath4();
//    takeoff_WPath4 = 0;
//    timeLanding_WPath4 = 0;
//    timeOff_WPath4 = 0;
//    time_auto_WPath4 = 0;
//    h_counter_WPath4 = 0.1;//0.0
//    h_counter_old_WPath4 = 0.1;
//    Vz_Hold_WPath4 = 0.0;
//    Status_waypoint_WPath4 = 0;
//    
//
//    //Default Value Reset of AutomaticTransport_Home_WPath4();
//    takeoff_tranH_WPath4 = 0;
//    timeLanding_tranH_WPath4 = 0;
//    timeOff_tranH_WPath4 = 0;
//    time_auto_tranH_WPath4 = 0;
//    h_counter_tranH_WPath4 = 0.1;//0.0
//    h_counter_old_tranH_WPath4 = 0.1;
//    Vz_Hold_tranH_WPath4 = 0.0;
//    Status_waypoint_tranH_WPath4 = 0;
//    
//    AutoTransHome_WPath4 = 0;
//    timeOff_Operate_WPath4 = 0;
//    timeOff_Operate_tranH_WPath4 = 0;
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    }
//    
//    //Waypoint = 5 point and Less of Distance
//    if (waypointCheck == 5 && AUX_3 >= 1700)
//    {
//        AutomaticTransport_WPath5();
//            if(time_AutoOperate_WPath5 > 64 && endAuto_WPath5 == 0 && AutoTransHome_WPath5 == 0)
//            {
//              timeOff_Operate_WPath5++;
//              //relay 2 s and < 12 s ( 10 s ) time-Off to Disarm and Open time Transporter
//              if(timeOff_Operate_WPath5 > 20 && timeOff_Operate_WPath5 < 200) // 2 s - Disarmed Quadrotor
//              {
//                if(z1_hat <= 0.18)
//                {
//                    armed = 0;
//                   //เปิดกล่องเพื่อทำการส่ง
//                   //Servo_trigger = 1020;
//                }
//              } 
//              if(timeOff_Operate_WPath5 > 200) //Wait for 20 - 2 s = 18 s for Active Again
//              {
//                AutoTransHome_WPath5 = 1;
//                armed = 1;
//                timeOff_Operate_WPath5 = 0;
//                // ปิดกล่องเพื่อทำการส่ง
//                // Servo_trigger = 1020;
//              } 
//            } 
//        AutomaticTransport_Home_WPath5();
//            if(time_AutoOperate_WPath5 > 128 && endAuto_tranH_WPath5 == 0 && AutoTransHome_WPath5 == 1)
//            {
//              timeOff_Operate_tranH_WPath5++;
//              //relay 2 s time-Off to Disarm and Open time Transporter
//              if(timeOff_Operate_tranH_WPath5 > 20)
//              {
//                if(z1_hat <= 0.18)
//                {
//                    armed = 0;
//                   //เปิดกล่องเพื่อทำการส่ง
//                   // Servo_trigger = 1020;
//                }
//              } 
//            }
//    }
//    else
//    {
////////////////////////////////////////////// _WPath5 //////////////////////////////////////////////////////////////////////////////////////
//
//    time_AutoOperate_WPath5 = 0;
//
//    //Default Value Reset of AutomaticTransport_WPath5();
//    takeoff_WPath5 = 0;
//    timeLanding_WPath5 = 0;
//    timeOff_WPath5 = 0;
//    time_auto_WPath5 = 0;
//    h_counter_WPath5 = 0.1;//0.0
//    h_counter_old_WPath5 = 0.1;
//    Vz_Hold_WPath5 = 0.0;
//    Status_waypoint_WPath5 = 0;
//    
//
//    //Default Value Reset of AutomaticTransport_Home_WPath5();
//    takeoff_tranH_WPath5 = 0;
//    timeLanding_tranH_WPath5 = 0;
//    timeOff_tranH_WPath5 = 0;
//    time_auto_tranH_WPath5 = 0;
//    h_counter_tranH_WPath5 = 0.1;//0.0
//    h_counter_old_tranH_WPath5 = 0.1;
//    Vz_Hold_tranH_WPath5 = 0.0;
//    Status_waypoint_tranH_WPath5 = 0;
//    
//    AutoTransHome_WPath5 = 0;
//    timeOff_Operate_WPath5 = 0;
//    timeOff_Operate_tranH_WPath5 = 0;
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    }
//
//    //Waypoint = 6 point and Less of Distance
//    if (waypointCheck == 6 && AUX_3 >= 1700)
//    {    
//        AutomaticTransport_WPath6();
//         // ถึงจุดหมายและนับเวลาการ armed == 0; หลังจากนั้น รอเวลาประมาณ 10 วินาที ให้ทำ takeoff ขึ้น
//          if(time_AutoOperate_WPath6 > 72 && endAuto_WPath6 == 0 && AutoTransHome_WPath6 == 0)
//            {
//              timeOff_Operate_WPath6++;
//              //relay 2 s and < 12 s ( 10 s ) time-Off to Disarm and Open time Transporter
//              if(timeOff_Operate_WPath6 > 20 && timeOff_Operate_WPath6 < 200) // 2 s - Disarmed Quadrotor
//              {
//                if(z1_hat <= 0.18)
//                {
//                    armed = 0;
//                   //เปิดกล่องเพื่อทำการส่ง
//                   //Servo_trigger = 1020;
//                }
//              } 
//              if(timeOff_Operate_WPath6 > 200) //Wait for 20 - 2 s = 18 s for Active Again
//              {
//                AutoTransHome_WPath6 = 1;
//                armed = 1;
//                timeOff_Operate_WPath6 = 0;
//                // ปิดกล่องเพื่อทำการส่ง
//                // Servo_trigger = 1020;
//              } 
//            } 
//        
//        //------------- 
//        //ระบบการทำงานเชื่อมต่อกัน ระหว่าง AutomaticTransport และจะเริ่มทำงานที่
//        //-------------
//        
//        //เริ่มการทำงานเมื่อไปถึงจุด Transport แล้ว Takeoff กลับมายัง Home
//        //-------------
//        AutomaticTransport_Home_WPath6();
//        //-------------
//          // ถึงจุดหมายและนับเวลาการ armed == 0;
//          if(time_AutoOperate_WPath6 > 144 && endAuto_tranH_WPath6 == 0 && AutoTransHome_WPath6 == 1)
//            {
//              timeOff_Operate_tranH_WPath6++;
//              //relay 2 s time-Off to Disarm and Open time Transporter
//              if(timeOff_Operate_tranH_WPath6 > 20)
//              {
//                if(z1_hat <= 0.18)
//                {
//                    armed = 0;
//                   //เปิดกล่องเพื่อทำการส่ง
//                   // Servo_trigger = 1020;
//                }
//              } 
//            }
//    }
//    else
//    {
////////////////////////////////////////////// _WPath6 //////////////////////////////////////////////////////////////////////////////////////
//
//    time_AutoOperate_WPath6 = 0;
//
//    //Default Value Reset of AutomaticTransport_WPath5();
//    takeoff_WPath6 = 0;
//    timeLanding_WPath6 = 0;
//    timeOff_WPath6 = 0;
//    time_auto_WPath6 = 0;
//    h_counter_WPath6 = 0.1;//0.0
//    h_counter_old_WPath6 = 0.1;
//    Vz_Hold_WPath6 = 0.0;
//    Status_waypoint_WPath6 = 0;
//    
//
//    //Default Value Reset of AutomaticTransport_Home_WPath5();
//    takeoff_tranH_WPath6 = 0;
//    timeLanding_tranH_WPath6 = 0;
//    timeOff_tranH_WPath6 = 0;
//    time_auto_tranH_WPath6 = 0;
//    h_counter_tranH_WPath6 = 0.1;//0.0
//    h_counter_old_tranH_WPath6 = 0.1;
//    Vz_Hold_tranH_WPath6 = 0.0;
//    Status_waypoint_tranH_WPath6 = 0;
//    
//    AutoTransHome_WPath6 = 0;
//    timeOff_Operate_WPath6 = 0;
//    timeOff_Operate_tranH_WPath6 = 0;
// }


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////   Waypoint Station 1  //////////////////////////////////////////////////////////
    if(waypointCheck == 0 && AUX_3 >= 1650)
    {
         AutomaticTransport();

          if(time_AutoOperate > 40 && endAuto == 0 && AutoTransHome == 0)
            {
              timeOff_Operate++;
              //relay 2 s and < 12 s ( 10 s ) time-Off to Disarm and Open time Transporter
              if(timeOff_Operate > 10 && timeOff_Operate < 250) // 2 s - Disarmed Quadrotor
              {
                if(z1_hat <= 0.18)
                {
                   armed = 0;
                }
              } 
              if(timeOff_Operate > 40 && timeOff_Operate < 250) // 4 s - เริ่มเปิดกล่อง
              {
                   //เปิดกล่องเพื่อทำการส่ง
                   Servo_trigger = 1020;
              }
              if(timeOff_Operate > 210 && timeOff_Operate < 250) // 21 s - ปิดกล่อง //สรุปใช้เวลาเปิดกล่องประมาณ 17 s
              {
                   //ปิดกล่องเพื่อทำการส่ง
                   Servo_trigger = 1850;
              }
              if(timeOff_Operate > 250) //Start กลับ
              {
                AutoTransHome = 1;
                armed = 1;
                timeOff_Operate = 0;
              } 
            } 
        
        //------------- 
        //ระบบการทำงานเชื่อมต่อกัน ระหว่าง AutomaticTransport และจะเริ่มทำงานที่
        //-------------
        
        //เริ่มการทำงานเมื่อไปถึงจุด Transport แล้ว Takeoff กลับมายัง Home
        //-------------
        AutomaticTransport_Home();
        //-------------

          if(time_AutoOperate > 80 && endAuto_tranH == 0 && AutoTransHome == 1)
            {
              timeOff_Operate_tranH++;
              //relay 2 s time-Off to Disarm and Open time Transporter
              if(timeOff_Operate_tranH > 10)
              {
                if(z1_hat <= 0.18)
                //if(Altitude_sonaf <= 0.18)
                {
                    armed = 0;   
                    //เปิดกล่องเพื่อทำการส่ง
                   Servo_trigger = 1020;            
                }
              } 
            }
    }
    else
    {
       time_AutoOperate = 0;
      //time_auto_tranH = 0;
      //time_auto = 0;
  
      //Default Value Reset of AutomaticTransport();
      takeoff = 0;
      timeLanding = 0;
      timeOff = 0;
      time_auto = 0;
      h_counter = 0.1;//0.0
      h_counter_old = 0.1;
      Vz_Hold = 0.0;
      Status_waypoint = 0;
      
  
      //Default Value Reset of AutomaticTransport_Home();
      takeoff_tranH = 0;
      timeLanding_tranH = 0;
      timeOff_tranH = 0;
      time_auto_tranH = 0;
      h_counter_tranH = 0.1;//0.0
      h_counter_old_tranH = 0.1;
      Vz_Hold_tranH = 0.0;
      Status_waypoint_tranH = 0;
      
      AutoTransHome = 0;
      timeOff_Operate = 0;
      timeOff_Operate_tranH = 0;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////   Waypoint Station 2  //////////////////////////////////////////////////////////
    if(waypointCheck == 0 && AUX_3 >= 1300 && AUX_3 <= 1650)
    {
         AutomaticTransport_sta2();
         
          if(time_AutoOperate_sta2 > 48 && endAuto_sta2 == 0 && AutoTransHome_sta2 == 0)
            {
              timeOff_Operate_sta2++;
              //relay 2 s and < 12 s ( 10 s ) time-Off to Disarm and Open time Transporter
              if(timeOff_Operate_sta2 > 20 && timeOff_Operate_sta2 < 250) // 2 s - Disarmed Quadrotor
              {
                if(z1_hat <= 0.18)
                {
                   armed = 0;
                }
              } 
              if(timeOff_Operate_sta2 > 40 && timeOff_Operate_sta2 < 250) // 4 s - เริ่มเปิดกล่อง
              {
                   //เปิดกล่องเพื่อทำการส่ง
                   Servo_trigger = 1020;
              }
              if(timeOff_Operate_sta2 > 210 && timeOff_Operate_sta2 < 250) // 21 s - ปิดกล่อง //สรุปใช้เวลาเปิดกล่องประมาณ 17 s
              {
                   //ปิดกล่องเพื่อทำการส่ง
                   Servo_trigger = 1850;
              }
              if(timeOff_Operate_sta2 > 250) //Start to Go Home
              {
                AutoTransHome_sta2 = 1;
                armed = 1;
                timeOff_Operate_sta2 = 0;
              } 
            } 
        
        //------------- 
        //ระบบการทำงานเชื่อมต่อกัน ระหว่าง AutomaticTransport และจะเริ่มทำงานที่
        //-------------
        
        //เริ่มการทำงานเมื่อไปถึงจุด Transport แล้ว Takeoff กลับมายัง Home
        //-------------
        AutomaticTransport_Home_sta2();
        //-------------
          if(time_AutoOperate_sta2 > 84 && endAuto_tranH_sta2 == 0 && AutoTransHome_sta2 == 1)
            {
              timeOff_Operate_tranH_sta2++;
              //relay 2 s time-Off to Disarm and Open time Transporter
              if(timeOff_Operate_tranH_sta2 > 20)
              {
                if(z1_hat <= 0.18)
                //if(Altitude_sonaf <= 0.18)
                {
                    armed = 0;
                   //เปิดกล่องเพื่อทำการส่ง
                   Servo_trigger = 1020;                  
                }
              } 
            }
    }
    else
    {
       time_AutoOperate_sta2 = 0;
      //time_auto_tranH = 0;
      //time_auto = 0;
  
      //Default Value Reset of AutomaticTransport();
      takeoff_sta2 = 0;
      timeLanding_sta2 = 0;
      timeOff_sta2 = 0;
      time_auto_sta2 = 0;
      h_counter_sta2 = 0.1;//0.0
      h_counter_old_sta2 = 0.1;
      Vz_Hold_sta2 = 0.0;
      Status_waypoint_sta2 = 0;
      
  
      //Default Value Reset of AutomaticTransport_Home();
      takeoff_tranH_sta2 = 0;
      timeLanding_tranH_sta2 = 0;
      timeOff_tranH_sta2 = 0;
      time_auto_tranH_sta2 = 0;
      h_counter_tranH_sta2 = 0.1;//0.0
      h_counter_old_tranH_sta2 = 0.1;
      Vz_Hold_tranH_sta2 = 0.0;
      Status_waypoint_tranH_sta2 = 0;
      
      AutoTransHome_sta2 = 0;
      timeOff_Operate_sta2 = 0;
      timeOff_Operate_tranH_sta2 = 0;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////   Waypoint Station 3  //////////////////////////////////////////////////////////
    if(waypointCheck == 0 && AUX_3 <= 1300)
    {
         AutomaticTransport_sta3();
         
          if(time_AutoOperate_sta3 > 24 && endAuto_sta3 == 0 && AutoTransHome_sta3 == 0)
            {
              timeOff_Operate_sta3++;
              //relay 2 s and < 12 s ( 10 s ) time-Off to Disarm and Open time Transporter
              if(timeOff_Operate_sta3 > 20 && timeOff_Operate_sta3 < 250) // 2 s - Disarmed Quadrotor
              {
                if(z1_hat <= 0.18)
                {
                   armed = 0;
                }
              } 
              if(timeOff_Operate_sta3 > 40 && timeOff_Operate_sta3 < 250) // 4 s - เริ่มเปิดกล่อง
              {
                   //เปิดกล่องเพื่อทำการส่ง
                   Servo_trigger = 1020;
              }
              if(timeOff_Operate_sta3 > 210 && timeOff_Operate_sta3 < 250) // 21 s - ปิดกล่อง //สรุปใช้เวลาเปิดกล่องประมาณ 17 s
              {
                   //ปิดกล่องเพื่อทำการส่ง
                   Servo_trigger = 1850;
              }
              if(timeOff_Operate_sta3 > 250) //Start to Go Home
              {
                AutoTransHome_sta3 = 1;
                armed = 1;
                timeOff_Operate_sta3 = 0;
              } 
            } 
        
        //------------- 
        //ระบบการทำงานเชื่อมต่อกัน ระหว่าง AutomaticTransport และจะเริ่มทำงานที่
        //-------------
        
        //เริ่มการทำงานเมื่อไปถึงจุด Transport แล้ว Takeoff กลับมายัง Home
        //-------------
        AutomaticTransport_Home_sta3();
        //-------------
          if(time_AutoOperate_sta3 > 48 && endAuto_tranH_sta3 == 0 && AutoTransHome_sta3 == 1)
            {
              timeOff_Operate_tranH_sta3++;
              //relay 2 s time-Off to Disarm and Open time Transporter
              if(timeOff_Operate_tranH_sta3 > 20)
              {
                if(z1_hat <= 0.18)
                //if(Altitude_sonaf <= 0.18)
                {
                    armed = 0;
                   //เปิดกล่องเพื่อทำการส่ง
                   Servo_trigger = 1020;                  
                }
              } 
            }
    }
    else
    {
       time_AutoOperate_sta3 = 0;
      //time_auto_tranH = 0;
      //time_auto = 0;
  
      //Default Value Reset of AutomaticTransport();
      takeoff_sta3 = 0;
      timeLanding_sta3 = 0;
      timeOff_sta3 = 0;
      time_auto_sta3 = 0;
      h_counter_sta3 = 0.1;//0.0
      h_counter_old_sta3 = 0.1;
      Vz_Hold_sta3 = 0.0;
      Status_waypoint_sta3 = 0;
      
  
      //Default Value Reset of AutomaticTransport_Home();
      takeoff_tranH_sta3 = 0;
      timeLanding_tranH_sta3 = 0;
      timeOff_tranH_sta3 = 0;
      time_auto_tranH_sta3 = 0;
      h_counter_tranH_sta3 = 0.1;//0.0
      h_counter_old_tranH_sta3 = 0.1;
      Vz_Hold_tranH_sta3 = 0.0;
      Status_waypoint_tranH_sta3 = 0;
      
      AutoTransHome_sta3 = 0;
      timeOff_Operate_sta3 = 0;
      timeOff_Operate_tranH_sta3 = 0;
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  }
  else
  {
    time_AutoOperate = 0;
    time_AutoOperate_WPath1 = 0;
    time_AutoOperate_WPath2 = 0;
    time_AutoOperate_WPath3 = 0;
    time_AutoOperate_WPath4 = 0;
    time_AutoOperate_WPath5 = 0;
    time_AutoOperate_WPath6 = 0;
    //time_auto_tranH = 0;
    //time_auto = 0;

    //Default Value Reset of AutomaticTransport();
    takeoff = 0;
    timeLanding = 0;
    timeOff = 0;
    time_auto = 0;
    h_counter = 0.1;//0.0
    h_counter_old = 0.1;
    Vz_Hold = 0.0;
    Status_waypoint = 0;
    //Default Value Reset of AutomaticTransport_Home();
    takeoff_tranH = 0;
    timeLanding_tranH = 0;
    timeOff_tranH = 0;
    time_auto_tranH = 0;
    h_counter_tranH = 0.1;//0.0
    h_counter_old_tranH = 0.1;
    Vz_Hold_tranH = 0.0;
    Status_waypoint_tranH = 0;
    AutoTransHome = 0;
    timeOff_Operate = 0;
    timeOff_Operate_tranH = 0;

    //Default Value Reset of AutomaticTransport_sta2();
    takeoff_sta2 = 0;
    timeLanding_sta2 = 0;
    timeOff_sta2 = 0;
    time_auto_sta2 = 0;
    h_counter_sta2 = 0.1;//0.0
    h_counter_old_sta2 = 0.1;
    Vz_Hold_sta2 = 0.0;
    Status_waypoint_sta2 = 0;
    //Default Value Reset of AutomaticTransport_Home_sta2();
    takeoff_tranH_sta2 = 0;
    timeLanding_tranH_sta2 = 0;
    timeOff_tranH_sta2 = 0;
    time_auto_tranH_sta2 = 0;
    h_counter_tranH_sta2 = 0.1;//0.0
    h_counter_old_tranH_sta2 = 0.1;
    Vz_Hold_tranH_sta2 = 0.0;
    Status_waypoint_tranH_sta2 = 0;
    AutoTransHome_sta2 = 0;
    timeOff_Operate_sta2 = 0;
    timeOff_Operate_tranH_sta2 = 0;

    //Default Value Reset of AutomaticTransport_sta3();
    takeoff_sta3 = 0;
    timeLanding_sta3 = 0;
    timeOff_sta3 = 0;
    time_auto_sta3 = 0;
    h_counter_sta3 = 0.1;//0.0
    h_counter_old_sta3 = 0.1;
    Vz_Hold_sta3 = 0.0;
    Status_waypoint_sta3 = 0;
    //Default Value Reset of AutomaticTransport_Home_sta3();
    takeoff_tranH_sta3 = 0;
    timeLanding_tranH_sta3 = 0;
    timeOff_tranH_sta3 = 0;
    time_auto_tranH_sta3 = 0;
    h_counter_tranH_sta3 = 0.1;//0.0
    h_counter_old_tranH_sta3 = 0.1;
    Vz_Hold_tranH_sta3 = 0.0;
    Status_waypoint_tranH_sta3 = 0;
    AutoTransHome_sta3 = 0;
    timeOff_Operate_sta3 = 0;
    timeOff_Operate_tranH_sta3 = 0;


    

    //waypointCheck_Reset
    waypointCheck = 0;
    //Waypoint1
    waypoint1_LAT = waypoint1_LAT_WDefault;
    waypoint1_LON = waypoint1_LON_WDefault;
    //Waypoint 2 
    waypoint2_LAT = waypoint2_LAT_WDefault;
    waypoint2_LON = waypoint2_LON_WDefault;
    //Waypoint 3
    waypoint3_LAT = waypoint3_LAT_WDefault;
    waypoint3_LON = waypoint3_LON_WDefault;
    //Waypoint 4
    waypoint4_LAT = waypoint4_LAT_WDefault;
    waypoint4_LON = waypoint4_LON_WDefault;
    //Waypoint 5
    waypoint5_LAT = waypoint5_LAT_WDefault;
    waypoint5_LON = waypoint5_LON_WDefault;
    //Waypoint 6
    waypoint6_LAT = waypoint6_LAT_WDefault;
    waypoint6_LON = waypoint6_LON_WDefault;

//////////////////////////////////////////// _WPath1 //////////////////////////////////////////////////////////////////////////////////////
    //Default Value Reset of AutomaticTransport_WPath1();
    takeoff_WPath1 = 0;
    timeLanding_WPath1 = 0;
    timeOff_WPath1 = 0;
    time_auto_WPath1 = 0;
    h_counter_WPath1 = 0.1;//0.0
    h_counter_old_WPath1 = 0.1;
    Vz_Hold_WPath1 = 0.0;
    Status_waypoint_WPath1 = 0;
    

    //Default Value Reset of AutomaticTransport_Home_WPath1();
    takeoff_tranH_WPath1 = 0;
    timeLanding_tranH_WPath1 = 0;
    timeOff_tranH_WPath1 = 0;
    time_auto_tranH_WPath1 = 0;
    h_counter_tranH_WPath1 = 0.1;//0.0
    h_counter_old_tranH_WPath1 = 0.1;
    Vz_Hold_tranH_WPath1 = 0.0;
    Status_waypoint_tranH_WPath1 = 0;
    
    AutoTransHome_WPath1 = 0;
    timeOff_Operate_WPath1 = 0;
    timeOff_Operate_tranH_WPath1 = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// _WPath2 //////////////////////////////////////////////////////////////////////////////////////
    //Default Value Reset of AutomaticTransport_WPath2();
    takeoff_WPath2 = 0;
    timeLanding_WPath2 = 0;
    timeOff_WPath2 = 0;
    time_auto_WPath2 = 0;
    h_counter_WPath2 = 0.1;//0.0
    h_counter_old_WPath2 = 0.1;
    Vz_Hold_WPath2 = 0.0;
    Status_waypoint_WPath2 = 0;
    

    //Default Value Reset of AutomaticTransport_Home_WPath2();
    takeoff_tranH_WPath2 = 0;
    timeLanding_tranH_WPath2 = 0;
    timeOff_tranH_WPath2 = 0;
    time_auto_tranH_WPath2 = 0;
    h_counter_tranH_WPath2 = 0.1;//0.0
    h_counter_old_tranH_WPath2 = 0.1;
    Vz_Hold_tranH_WPath2 = 0.0;
    Status_waypoint_tranH_WPath2 = 0;
    
    AutoTransHome_WPath2 = 0;
    timeOff_Operate_WPath2 = 0;
    timeOff_Operate_tranH_WPath2 = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// _WPath3 //////////////////////////////////////////////////////////////////////////////////////
    //Default Value Reset of AutomaticTransport_WPath3();
    takeoff_WPath3 = 0;
    timeLanding_WPath3 = 0;
    timeOff_WPath3 = 0;
    time_auto_WPath3 = 0;
    h_counter_WPath3 = 0.1;//0.0
    h_counter_old_WPath3 = 0.1;
    Vz_Hold_WPath3 = 0.0;
    Status_waypoint_WPath3 = 0;
    

    //Default Value Reset of AutomaticTransport_Home_WPath3();
    takeoff_tranH_WPath3 = 0;
    timeLanding_tranH_WPath3 = 0;
    timeOff_tranH_WPath3 = 0;
    time_auto_tranH_WPath3 = 0;
    h_counter_tranH_WPath3 = 0.1;//0.0
    h_counter_old_tranH_WPath3 = 0.1;
    Vz_Hold_tranH_WPath3 = 0.0;
    Status_waypoint_tranH_WPath3 = 0;
    
    AutoTransHome_WPath3 = 0;
    timeOff_Operate_WPath3 = 0;
    timeOff_Operate_tranH_WPath3 = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// _WPath4 //////////////////////////////////////////////////////////////////////////////////////
    //Default Value Reset of AutomaticTransport_WPath4();
    takeoff_WPath4 = 0;
    timeLanding_WPath4 = 0;
    timeOff_WPath4 = 0;
    time_auto_WPath4 = 0;
    h_counter_WPath4 = 0.1;//0.0
    h_counter_old_WPath4 = 0.1;
    Vz_Hold_WPath4 = 0.0;
    Status_waypoint_WPath4 = 0;
    

    //Default Value Reset of AutomaticTransport_Home_WPath4();
    takeoff_tranH_WPath4 = 0;
    timeLanding_tranH_WPath4 = 0;
    timeOff_tranH_WPath4 = 0;
    time_auto_tranH_WPath4 = 0;
    h_counter_tranH_WPath4 = 0.1;//0.0
    h_counter_old_tranH_WPath4 = 0.1;
    Vz_Hold_tranH_WPath4 = 0.0;
    Status_waypoint_tranH_WPath4 = 0;
    
    AutoTransHome_WPath4 = 0;
    timeOff_Operate_WPath4 = 0;
    timeOff_Operate_tranH_WPath4 = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// _WPath5 //////////////////////////////////////////////////////////////////////////////////////
    //Default Value Reset of AutomaticTransport_WPath5();
    takeoff_WPath5 = 0;
    timeLanding_WPath5 = 0;
    timeOff_WPath5 = 0;
    time_auto_WPath5 = 0;
    h_counter_WPath5 = 0.1;//0.0
    h_counter_old_WPath5 = 0.1;
    Vz_Hold_WPath5 = 0.0;
    Status_waypoint_WPath5 = 0;
    

    //Default Value Reset of AutomaticTransport_Home_WPath5();
    takeoff_tranH_WPath5 = 0;
    timeLanding_tranH_WPath5 = 0;
    timeOff_tranH_WPath5 = 0;
    time_auto_tranH_WPath5 = 0;
    h_counter_tranH_WPath5 = 0.1;//0.0
    h_counter_old_tranH_WPath5 = 0.1;
    Vz_Hold_tranH_WPath5 = 0.0;
    Status_waypoint_tranH_WPath5 = 0;
    
    AutoTransHome_WPath5 = 0;
    timeOff_Operate_WPath5 = 0;
    timeOff_Operate_tranH_WPath5 = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// _WPath6 //////////////////////////////////////////////////////////////////////////////////////
    //Default Value Reset of AutomaticTransport_WPath5();
    takeoff_WPath6 = 0;
    timeLanding_WPath6 = 0;
    timeOff_WPath6 = 0;
    time_auto_WPath6 = 0;
    h_counter_WPath6 = 0.1;//0.0
    h_counter_old_WPath6 = 0.1;
    Vz_Hold_WPath6 = 0.0;
    Status_waypoint_WPath6 = 0;
    

    //Default Value Reset of AutomaticTransport_Home_WPath5();
    takeoff_tranH_WPath6 = 0;
    timeLanding_tranH_WPath6 = 0;
    timeOff_tranH_WPath6 = 0;
    time_auto_tranH_WPath6 = 0;
    h_counter_tranH_WPath6 = 0.1;//0.0
    h_counter_old_tranH_WPath6 = 0.1;
    Vz_Hold_tranH_WPath6 = 0.0;
    Status_waypoint_tranH_WPath6 = 0;
    
    AutoTransHome_WPath6 = 0;
    timeOff_Operate_WPath6 = 0;
    timeOff_Operate_tranH_WPath6 = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  } 
  
}


