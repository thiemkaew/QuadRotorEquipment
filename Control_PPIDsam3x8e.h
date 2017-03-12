

float u_roll = 0;
float u_pitch = 0;
float u_yaw = 0;

float roll_I_rate = 0.0;
float roll_D_rate = 0.0;
float setpoint_rollold = 0.0;
float setpoint_rate_roll = 0.0;
float error_rollold = 0.0;
float error_rate_rollold = 0.0;
float pitch_I_rate = 0.0;
float pitch_D_rate = 0.0;
float setpoint_pitchold = 0.0;
float setpoint_rate_pitch = 0.0;
float error_pitchold = 0.0;
float error_rate_pitchold = 0.0;
float yaw_I_rate = 0.0;
float yaw_D_rate = 0.0;
float error_rate_yawold = 0.0;

//Automatic take-off and landing
float err_hz = 0.0;
int time_auto = 0;
float h_counter = 0.1;//0.08
float h_counter_old = 0.1;
float Vz_Hold = 0.0;
float hz_I = 0.0;
float hz_D_rate = 0.0;
float error_Vz_old = 0.0;
uint8_t takeoff = 0;
uint8_t endAuto = 0;

//Automatic take-off and landing 
//Auto Transport Return to Home and Landing
int time_auto_tranH = 0;
float h_counter_tranH = 0.1;//0.08
float h_counter_old_tranH = 0.1;
float Vz_Hold_tranH = 0.0;
uint8_t takeoff_tranH = 0;
uint8_t endAuto_tranH = 0;
//Automatic_Operation
int time_AutoOperate = 0;


//_sta2
int time_auto_sta2 = 0;
float h_counter_sta2 = 0.1;//0.08
float h_counter_old_sta2 = 0.1;
float Vz_Hold_sta2 = 0.0;
uint8_t takeoff_sta2 = 0;
uint8_t endAuto_sta2 = 0;
//Auto Transport Return to Home and Landing
int time_auto_tranH_sta2 = 0;
float h_counter_tranH_sta2 = 0.1;//0.08
float h_counter_old_tranH_sta2 = 0.1;
float Vz_Hold_tranH_sta2 = 0.0;
uint8_t takeoff_tranH_sta2 = 0;
uint8_t endAuto_tranH_sta2 = 0;
//Automatic_Operation
int time_AutoOperate_sta2 = 0;

//_sta3
int time_auto_sta3 = 0;
float h_counter_sta3 =  0.1;//0.08
float h_counter_old_sta3 = 0.1;
float Vz_Hold_sta3 = 0.0;
uint8_t takeoff_sta3 = 0;
uint8_t endAuto_sta3 = 0;
//Auto Transport Return to Home and Landing
int time_auto_tranH_sta3 = 0;
float h_counter_tranH_sta3 = 0.1;//0.08
float h_counter_old_tranH_sta3 = 0.1;
float Vz_Hold_tranH_sta3 = 0.0;
uint8_t takeoff_tranH_sta3 = 0;
uint8_t endAuto_tranH_sta3 = 0;
//Automatic_Operation
int time_AutoOperate_sta3 = 0;



//_WPath1
int time_auto_WPath1 = 0;
float h_counter_WPath1 = 0.1;//0.08
float h_counter_old_WPath1 = 0.1;
float Vz_Hold_WPath1 = 0.0;
uint8_t takeoff_WPath1 = 0;
uint8_t endAuto_WPath1 = 0;
//Auto Transport Return to Home and Landing
int time_auto_tranH_WPath1 = 0;
float h_counter_tranH_WPath1 = 0.1;//0.08
float h_counter_old_tranH_WPath1 = 0.1;
float Vz_Hold_tranH_WPath1 = 0.0;
uint8_t takeoff_tranH_WPath1 = 0;
uint8_t endAuto_tranH_WPath1 = 0;
//Automatic_Operation
int time_AutoOperate_WPath1 = 0;

//_WPath2
int time_auto_WPath2 = 0;
float h_counter_WPath2 = 0.1;//0.08
float h_counter_old_WPath2 = 0.1;
float Vz_Hold_WPath2 = 0.0;
uint8_t takeoff_WPath2 = 0;
uint8_t endAuto_WPath2 = 0;
//Auto Transport Return to Home and Landing
int time_auto_tranH_WPath2 = 0;
float h_counter_tranH_WPath2 = 0.1;//0.08
float h_counter_old_tranH_WPath2 = 0.1;
float Vz_Hold_tranH_WPath2 = 0.0;
uint8_t takeoff_tranH_WPath2 = 0;
uint8_t endAuto_tranH_WPath2 = 0;
//Automatic_Operation
int time_AutoOperate_WPath2 = 0;

//_WPath3
int time_auto_WPath3 = 0;
float h_counter_WPath3 = 0.1;//0.08
float h_counter_old_WPath3 = 0.1;
float Vz_Hold_WPath3 = 0.0;
uint8_t takeoff_WPath3 = 0;
uint8_t endAuto_WPath3 = 0;
//Auto Transport Return to Home and Landing
int time_auto_tranH_WPath3 = 0;
float h_counter_tranH_WPath3 = 0.1;//0.08
float h_counter_old_tranH_WPath3 = 0.1;
float Vz_Hold_tranH_WPath3 = 0.0;
uint8_t takeoff_tranH_WPath3 = 0;
uint8_t endAuto_tranH_WPath3 = 0;
//Automatic_Operation
int time_AutoOperate_WPath3 = 0;

//_WPath4
int time_auto_WPath4 = 0;
float h_counter_WPath4 = 0.1;//0.08
float h_counter_old_WPath4 = 0.1;
float Vz_Hold_WPath4 = 0.0;
uint8_t takeoff_WPath4 = 0;
uint8_t endAuto_WPath4 = 0;
//Auto Transport Return to Home and Landing
int time_auto_tranH_WPath4 = 0;
float h_counter_tranH_WPath4 = 0.1;//0.08
float h_counter_old_tranH_WPath4 = 0.1;
float Vz_Hold_tranH_WPath4 = 0.0;
uint8_t takeoff_tranH_WPath4 = 0;
uint8_t endAuto_tranH_WPath4 = 0;
//Automatic_Operation
int time_AutoOperate_WPath4 = 0;

//_WPath5
int time_auto_WPath5 = 0;
float h_counter_WPath5 = 0.1;//0.08
float h_counter_old_WPath5 = 0.1;
float Vz_Hold_WPath5 = 0.0;
uint8_t takeoff_WPath5 = 0;
uint8_t endAuto_WPath5 = 0;
//Auto Transport Return to Home and Landing
int time_auto_tranH_WPath5 = 0;
float h_counter_tranH_WPath5 = 0.1;//0.08
float h_counter_old_tranH_WPath5 = 0.1;
float Vz_Hold_tranH_WPath5 = 0.0;
uint8_t takeoff_tranH_WPath5 = 0;
uint8_t endAuto_tranH_WPath5 = 0;
//Automatic_Operation
int time_AutoOperate_WPath5 = 0;

//_WPath6
int time_auto_WPath6 = 0;
float h_counter_WPath6 = 0.1;//0.08
float h_counter_old_WPath6 = 0.1;
float Vz_Hold_WPath6 = 0.0;
uint8_t takeoff_WPath6 = 0;
uint8_t endAuto_WPath6 = 0;
//Auto Transport Return to Home and Landing
int time_auto_tranH_WPath6 = 0;
float h_counter_tranH_WPath6 = 0.1;//0.08
float h_counter_old_tranH_WPath6 = 0.1;
float Vz_Hold_tranH_WPath6 = 0.0;
uint8_t takeoff_tranH_WPath6 = 0;
uint8_t endAuto_tranH_WPath6 = 0;
//Automatic_Operation
int time_AutoOperate_WPath6 = 0;

//RTH and Landing Using in FailSafeMode
int time_auto_rth = 0;
float h_counter_rth = 0.1;//0.08
float h_counter_old_rth = 0.1;
float Vz_Hold_rth = 0.0;
uint8_t takeoff_rth = 0;
uint8_t endAuto_rth = 0;


//Including ADNS3080 and Distance Measure
float error_rate_LAT = 0.0;
float error_rate_LON = 0.0;

void Control_PPIDRate(){
// ROLL CONTROL P-PID  control  By tinnakon///////////
  float setpoint_roll = ((CH_AILf-CH_AIL_Cal)*0.085) + Control_YBf;//0.12 max +-45 deg  ////+-18  + Control_YBf
  applyDeadband(setpoint_roll, 2.5);//1.2
  setpoint_rate_roll = (0.065*setpoint_rate_roll/(0.065+G_Dt)) + ((setpoint_roll-setpoint_rollold)/(0.065+G_Dt));//Diff remote
  setpoint_rollold = setpoint_roll;
  setpoint_rate_roll = constrain(setpoint_rate_roll, -80, 80);//+-80 deg/s
  float error_roll = setpoint_roll - ahrs_r;//ahrs_r*ToDeg
  float error_rate_roll = setpoint_rate_roll + error_roll*Kp_levelRoll  - GyroXf*RAD_TO_DEG;
  roll_I_rate += error_rate_roll*Ki_rateRoll*G_Dt;
  roll_I_rate = constrain(roll_I_rate, -5, 5);//+-150
  roll_D_rate = (tar*roll_D_rate/(tar+G_Dt)) + ((error_rate_roll-error_rate_rollold)/(tar+G_Dt));
  error_rate_rollold = error_rate_roll;
  u_roll = Kp_rateRoll*error_rate_roll + roll_I_rate + Kd_rateRoll*roll_D_rate;
  u_roll = constrain(u_roll*K_Roll, -220, 220);//+-250 +-300 120
// PITCH CONTROL P-PID  control  By tinnakon///////////
  float setpoint_pitch = ((CH_ELEf-CH_ELE_Cal)*-0.085) + Control_XBf;//max +-45 deg  ////+-18 - Control_XBf
  applyDeadband(setpoint_pitch, 2.5);//1.2
  setpoint_rate_pitch = (0.065*setpoint_rate_pitch/(0.065+G_Dt)) + ((setpoint_pitch-setpoint_pitchold)/(0.065+G_Dt));//Diff remote
  setpoint_pitchold = setpoint_pitch;
  setpoint_rate_pitch = constrain(setpoint_rate_pitch, -80, 80);//+-80
  float error_pitch = setpoint_pitch - ahrs_p;//ahrs_p*RAD_TO_DEG
  float error_rate_pitch = setpoint_rate_pitch + error_pitch*Kp_levelPitch - GyroYf*RAD_TO_DEG;
  pitch_I_rate += error_rate_pitch*Ki_ratePitch*G_Dt;
  pitch_I_rate = constrain(pitch_I_rate, -5, 5);//+-150
  pitch_D_rate = (tar*pitch_D_rate/(tar+G_Dt)) + ((error_rate_pitch-error_rate_pitchold)/(tar+G_Dt));
  error_rate_pitchold = error_rate_pitch;
  u_pitch = Kp_ratePitch*error_rate_pitch + pitch_I_rate + Kd_ratePitch*pitch_D_rate;
  u_pitch = constrain(u_pitch*K_Pitch, -220, 220);//+-250 +-300 120
// YAW  CONTROL P-PID  control  By tinnakon///////////
float setpoint_rate_yaw = (CH_RUDf-CH_RUD_Cal)*0.55;//0.4 0.35
  applyDeadband(setpoint_rate_yaw, 8.5);//6.5
  if(abs(setpoint_rate_yaw) > 0.1){
   setHeading = ahrs_y;// 0 degree ,ahrs_tin.h
  }
  float error_yaw = 0.0 - Heading;
  float error_rate_yaw = setpoint_rate_yaw + error_yaw*Kp_levelyaw - GyroZf*RAD_TO_DEG;
  yaw_I_rate += error_rate_yaw*Ki_rateYaw*G_Dt;
  yaw_I_rate = constrain(yaw_I_rate, -50, 50);//+-100
  yaw_D_rate = (tar*yaw_D_rate/(tar+G_Dt)) + ((error_rate_yaw-error_rate_yawold)/(tar+G_Dt));
  error_rate_yawold = error_rate_yaw;
  u_yaw = Kp_rateYaw*error_rate_yaw + yaw_I_rate + Kd_rateYaw*yaw_D_rate;
  u_yaw = constrain(u_yaw*K_Yaw, -120, 120);//+-170 +-150
 ////Altitude//////////////////////////////////////////////////////////////////////////////////////////////////////  
  if(Mode == 1 || Mode == 2)//Altitude Hold, 
  {
    err_hz = Altitude_Hold - z1_hat;
    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
    //applyDeadband(err_hz, 0.11);//nois 0.2 m
    float error_Vz = 0.0 - z2_hat;
    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
    error_Vz_old = error_Vz;
    float error_hII = err_hz*1.25 + error_Vz;//0.55
    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -20, 20);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////// Station 1//////////////////////////////////////////////////////////////////////////////////////////////////
  
  else if(waypointCheck == 0 && Mode == 3 && AutoTransHome == 0 && AUX_3 >= 1650)//waypointCheck == 0 ระบบการทำงานรันที่ Control_Waypoint_0_Default.h
  {
    err_hz = h_counter - z1_hat;
    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
    float error_Vz = Vz_Hold - z2_hat;//Vz_Hold 
    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
    error_Vz_old = error_Vz;
    float error_hII = err_hz*1.25 + error_Vz;//0.55
    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -100, 100);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
  }
  else if(waypointCheck == 0 && Mode == 3 && AutoTransHome == 1 && AUX_3 >= 1650)//Auto Transportation Return to Home and Landing //waypointCheck == 0 ระบบการทำงานรันที่ Control_Waypoint_0_Default.h
  {
    err_hz = h_counter_tranH - z1_hat;
    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
    float error_Vz = Vz_Hold_tranH - z2_hat;//Vz_Hold_tranH 
    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
    error_Vz_old = error_Vz;
    float error_hII = err_hz*1.25 + error_Vz;//0.55
    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -100, 100);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////// Station 2//////////////////////////////////////////////////////////////////////////////////////////
  
  else if(waypointCheck == 0 && Mode == 3 && AutoTransHome_sta2 == 0 && AUX_3 >= 1300 && AUX_3 <= 1650)//waypointCheck == 0 ระบบการทำงานรันที่ Control_Waypoint_0_Default.h
  {
    err_hz = h_counter_sta2 - z1_hat;
    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
    float error_Vz = Vz_Hold_sta2 - z2_hat;//Vz_Hold 
    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
    error_Vz_old = error_Vz;
    float error_hII = err_hz*1.25 + error_Vz;//0.55
    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -100, 100);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
  }
  else if(waypointCheck == 0 && Mode == 3 && AutoTransHome_sta2 == 1 && AUX_3 >= 1300 && AUX_3 <= 1650)//Auto Transportation Return to Home and Landing //waypointCheck == 0 ระบบการทำงานรันที่ Control_Waypoint_0_Default.h
  {
    err_hz = h_counter_tranH_sta2 - z1_hat;
    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
    float error_Vz = Vz_Hold_tranH_sta2 - z2_hat;//Vz_Hold_tranH 
    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
    error_Vz_old = error_Vz;
    float error_hII = err_hz*1.25 + error_Vz;//0.55
    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -100, 100);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////// Station 3//////////////////////////////////////////////////////////////////////////////////////////
  
  else if(waypointCheck == 0 && Mode == 3 && AutoTransHome_sta3 == 0 && AUX_3 <= 1300)//waypointCheck == 0 ระบบการทำงานรันที่ Control_Waypoint_0_Default.h
  {
    err_hz = h_counter_sta3 - z1_hat;
    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
    float error_Vz = Vz_Hold_sta3 - z2_hat;//Vz_Hold 
    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
    error_Vz_old = error_Vz;
    float error_hII = err_hz*1.25 + error_Vz;//0.55
    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -100, 100);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
  }
  else if(waypointCheck == 0 && Mode == 3 && AutoTransHome_sta3 == 1 && AUX_3 <= 1300)//Auto Transportation Return to Home and Landing //waypointCheck == 0 ระบบการทำงานรันที่ Control_Waypoint_0_Default.h
  {
    err_hz = h_counter_tranH_sta3 - z1_hat;
    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
    float error_Vz = Vz_Hold_tranH_sta3 - z2_hat;//Vz_Hold_tranH 
    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
    error_Vz_old = error_Vz;
    float error_hII = err_hz*1.25 + error_Vz;//0.55
    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -100, 100);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
//  else if(waypointCheck == 1 && Mode == 3 && AutoTransHome_WPath1 == 0)//Automatic  Takeoff, Landing //waypointCheck == 1 ระบบการทำงานรันที่ Control_Waypoint_1.h
//  {
//    err_hz = h_counter_WPath1 - z1_hat;
//    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
//    float error_Vz = Vz_Hold_WPath1 - z2_hat;//Vz_Hold_WPath1
//    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
//    error_Vz_old = error_Vz;
//    float error_hII = err_hz*1.25 + error_Vz;//0.55
//    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
//    hz_I = constrain(hz_I, -100, 100);//+-200
//    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
//    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
//    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
//  }
//  else if(waypointCheck == 1 && Mode == 3 && AutoTransHome_WPath1 == 1)//Auto Transportation Return to Home and Landing //waypointCheck == 1 ระบบการทำงานรันที่ Control_Waypoint_1.h
//  {
//    err_hz = h_counter_tranH_WPath1 - z1_hat;
//    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
//    float error_Vz = Vz_Hold_tranH_WPath1 - z2_hat;//Vz_Hold_tranH_WPath1
//    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
//    error_Vz_old = error_Vz;
//    float error_hII = err_hz*1.25 + error_Vz;//0.55
//    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
//    hz_I = constrain(hz_I, -100, 100);//+-200
//    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
//    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
//    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
//  }
//  else if(waypointCheck == 2 && Mode == 3 && AutoTransHome_WPath2 == 0)//Automatic  Takeoff, Landing //waypointCheck == 2 ระบบการทำงานรันที่ Control_Waypoint_2.h
//  {
//    err_hz = h_counter_WPath2 - z1_hat;
//    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
//    float error_Vz = Vz_Hold_WPath2 - z2_hat;//Vz_Hold_WPath2 
//    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
//    error_Vz_old = error_Vz;
//    float error_hII = err_hz*1.25 + error_Vz;//0.55
//    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
//    hz_I = constrain(hz_I, -100, 100);//+-200
//    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
//    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
//    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
//  }
//  else if(waypointCheck == 2 && Mode == 3 && AutoTransHome_WPath2 == 1)//Auto Transportation Return to Home and Landing //waypointCheck == 2 ระบบการทำงานรันที่ Control_Waypoint_2.h
//  {
//    err_hz = h_counter_tranH_WPath2 - z1_hat;
//    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
//    float error_Vz = Vz_Hold_tranH_WPath2 - z2_hat;//Vz_Hold_tranH_WPath2
//    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
//    error_Vz_old = error_Vz;
//    float error_hII = err_hz*1.25 + error_Vz;//0.55
//    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
//    hz_I = constrain(hz_I, -100, 100);//+-200
//    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
//    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
//    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
//  }
//  else if(waypointCheck == 3 && Mode == 3 && AutoTransHome_WPath3 == 0)//Automatic  Takeoff, Landing //waypointCheck == 3 ระบบการทำงานรันที่ Control_Waypoint_3.h
//  {
//    err_hz = h_counter_WPath3 - z1_hat;
//    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
//    float error_Vz = Vz_Hold_WPath3 - z2_hat;//Vz_Hold_WPath3
//    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
//    error_Vz_old = error_Vz;
//    float error_hII = err_hz*1.25 + error_Vz;//0.55
//    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
//    hz_I = constrain(hz_I, -100, 100);//+-200
//    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
//    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
//    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
//  }
//  else if(waypointCheck == 3 && Mode == 3 && AutoTransHome_WPath3 == 1)//Auto Transportation Return to Home and Landing //waypointCheck == 3 ระบบการทำงานรันที่ Control_Waypoint_3.h
//  {
//    err_hz = h_counter_tranH_WPath3 - z1_hat;
//    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
//    float error_Vz = Vz_Hold_tranH_WPath3 - z2_hat;//Vz_Hold_tranH_WPath3
//    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
//    error_Vz_old = error_Vz;
//    float error_hII = err_hz*1.25 + error_Vz;//0.55
//    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
//    hz_I = constrain(hz_I, -100, 100);//+-200
//    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
//    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
//    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
//  }
//  else if(waypointCheck == 4 && Mode == 3 && AutoTransHome_WPath4 == 0)//Automatic  Takeoff, Landing //waypointCheck == 4 ระบบการทำงานรันที่ Control_Waypoint_4.h
//  {
//    err_hz = h_counter_WPath4 - z1_hat;
//    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
//    float error_Vz = Vz_Hold_WPath4 - z2_hat;//Vz_Hold_WPath4
//    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
//    error_Vz_old = error_Vz;
//    float error_hII = err_hz*1.25 + error_Vz;//0.55
//    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
//    hz_I = constrain(hz_I, -100, 100);//+-200
//    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
//    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
//    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
//  }
//  else if(waypointCheck == 4 && Mode == 3 && AutoTransHome_WPath4 == 1)//Auto Transportation Return to Home and Landing //waypointCheck == 4 ระบบการทำงานรันที่ Control_Waypoint_4.h
//  {
//    err_hz = h_counter_tranH_WPath4 - z1_hat;
//    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
//    float error_Vz = Vz_Hold_tranH_WPath4 - z2_hat;//Vz_Hold_tranH_WPath4
//    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
//    error_Vz_old = error_Vz;
//    float error_hII = err_hz*1.25 + error_Vz;//0.55
//    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
//    hz_I = constrain(hz_I, -100, 100);//+-200
//    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
//    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
//    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
//  }
//  else if(waypointCheck == 5 && Mode == 3 && AutoTransHome_WPath5 == 0)//Automatic  Takeoff, Landing //waypointCheck == 5 ระบบการทำงานรันที่ Control_Waypoint_5.h
//  {
//    err_hz = h_counter_WPath5 - z1_hat;
//    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
//    float error_Vz = Vz_Hold_WPath5 - z2_hat;//Vz_Hold_WPath5
//    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
//    error_Vz_old = error_Vz;
//    float error_hII = err_hz*1.25 + error_Vz;//0.55
//    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
//    hz_I = constrain(hz_I, -100, 100);//+-200
//    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
//    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
//    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
//  }
//  else if(waypointCheck == 5 && Mode == 3 && AutoTransHome_WPath5 == 1)//Auto Transportation Return to Home and Landing //waypointCheck == 5 ระบบการทำงานรันที่ Control_Waypoint_5.h
//  {
//    err_hz = h_counter_tranH_WPath5 - z1_hat;
//    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
//    float error_Vz = Vz_Hold_tranH_WPath5 - z2_hat;//Vz_Hold_tranH_WPath5
//    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
//    error_Vz_old = error_Vz;
//    float error_hII = err_hz*1.25 + error_Vz;//0.55
//    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
//    hz_I = constrain(hz_I, -100, 100);//+-200
//    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
//    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
//    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
//  }
//  else if(waypointCheck == 6 && Mode == 3 && AutoTransHome_WPath6 == 0)//Automatic  Takeoff, Landing //waypointCheck == 6 ระบบการทำงานรันที่ Control_Waypoint_6.h
//  {
//    err_hz = h_counter_WPath6 - z1_hat;
//    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
//    float error_Vz = Vz_Hold_WPath6 - z2_hat;//Vz_Hold_WPath6 
//    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
//    error_Vz_old = error_Vz;
//    float error_hII = err_hz*1.25 + error_Vz;//0.55
//    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
//    hz_I = constrain(hz_I, -100, 100);//+-200
//    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
//    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
//    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
//  }
//  else if(waypointCheck == 6 && Mode == 3 && AutoTransHome_WPath6 == 1)//Auto Transportation Return to Home and Landing //waypointCheck == 6 ระบบการทำงานรันที่ Control_Waypoint_6.h
//  {
//    err_hz = h_counter_tranH_WPath6 - z1_hat;
//    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
//    float error_Vz = Vz_Hold_tranH_WPath6 - z2_hat;//Vz_Hold_tranH_WPath6
//    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
//    error_Vz_old = error_Vz;
//    float error_hII = err_hz*1.25 + error_Vz;//0.55
//    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
//    hz_I = constrain(hz_I, -100, 100);//+-200
//    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
//    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
//    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
//  }
  else if(Mode == 4)//RTH and Landing
  {
    err_hz = h_counter_rth - z1_hat;
    err_hz = constrain(err_hz, -1.0, 1.0);//+-2 m
    float error_Vz = Vz_Hold_rth - z2_hat;//Vz_Hold_rth
    hz_D_rate = (tar*hz_D_rate/(tar+G_Dt)) + ((error_Vz - error_Vz_old)/(tar+G_Dt));// D Controller
    error_Vz_old = error_Vz;
    float error_hII = err_hz*1.25 + error_Vz;//0.55
    hz_I = hz_I + (Ki_altitude*error_hII*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -100, 100);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I + (error_Vz*Kd_altitude) + (hz_D_rate*Kd2_altitude) - (accrZ_Earth*Ka_altitude);//state feedback
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
  }
  else
  {
    hz_I = 0.0;
    uthrottle = 0.0;
    Altitude_Hold = z1_hat;
  }//end Altitude Hold
  uthrottle = constrain(uthrottle*K_Altitude, -200, 200);//+-120 +-150
  uAltitude = CH_THRf + uthrottle;//m*g = 10.8 N = 
  uAltitude = constrain(uAltitude, MINCOMMAND, MAXTHROTTLE);
}
void Chack_Command(){
   if(AUX_1 <= (AltHold-20))//Stabilize 
  {
    Mode = 0;
  }
  /////////////////////////////////////////////////////////////////////////////////////////
  //Altitude Hold, 
   if(AUX_1 > (AltHold-20) && AUX_1 <= (AltHold+20) && AUX_4 <= 1300)
  {
    Mode = 1;
  }
  //////////////////////////
  // RTH and Landing
  if (AUX_1 > (AltHold-20) && AUX_1 <= (AltHold+20) && AUX_4 >= 1700) 
  {
      Mode = 4;
  }
  if (AUX_4 >= 1700) 
  {
      Mode = 4;
  }
  /////////////////////////////////////////////////////////////////////////////////////////
//   if(AUX_1 > (PositionHold-20) && AUX_1 <= (PositionHold+20))//Position Hold and Loieter
//  {
//    Mode = 2;
//  }  
  if(AUX_1 > (Auto-20))//Automatic  Takeoff and Landing with Mix SPF Path
  {
    Mode = 3;
  }
  //Return to Home
  if(AUX_1 > (RTH-20) && AUX_1 <= (RTH+20))//RTH and Loieter
  {
   Mode = 2;
   target_LAT = GPS_LAT_HOME;//GPS_LAT_Hold
   target_LON = GPS_LON_HOME;//GPS_LON_Hold
  }
  //Position Hold and Change Point when CH_ALT and CH_ELEf > 20
  if(AUX_1 > (PositionHold-20) && AUX_1 <= (PositionHold+20))
  {
    Mode = 2;
    //change (CH_AILf-CH_AIL_Cal)  (CH_ELEf-CH_ELE_Cal)
    if(abs(CH_AILf-CH_AIL_Cal) > 20 || abs(CH_ELEf-CH_ELE_Cal) > 20)
    {
     target_LAT = GPS_LAT1f;//change GPS_LAT_Hold
     target_LON = GPS_LON1f;//change GPS_LON_Hold
    }
  }
   if(AUX_2 <= 1300)//Set Home Point
  {
    GPS_LAT_HOME = GPS_LAT1f;
    GPS_LON_HOME = GPS_LON1f;
    GPS_LAT_SPF_HOME = GPS_LAT1f;
    GPS_LON_SPF_HOME = GPS_LON1f;
    digitalWrite(Pin_LED_G, HIGH);
    digitalWrite(Pin_LED_R, LOW);
  }
   if(AUX_2 >= 1700)//Set Transport Point
  {
    GPS_LAT_HOME_Transport = GPS_LAT1f;
    GPS_LON_HOME_Transport = GPS_LON1f;
    GPS_LAT_SPF_TR = GPS_LAT1f;
    GPS_LON_SPF_TR = GPS_LON1f;
    digitalWrite(Pin_LED_R, HIGH);
    digitalWrite(Pin_LED_G, LOW);
  }
//  if(AUX_3 >= 1700){
//      Servo_trigger = 1850;
//  }
//  else if(AUX_3 <= 1300) {
//      Servo_trigger = 1020;
//  }
//  else {
//    timeOff_armed = 0;
//  }
}//end Chack_Command()


///////////////////////////////////////////
//FailSafeAuto and Landing
///////////////////////////////////////////
// ---------------------------------------------------------------------------------------------------------------
void failSafeAuto() {
  ////mode 4
  if(Mode == 4 && CH_THRf > MINCHECK && armed == 1)
  {
    //Takeoof and Checking
    if(time_auto_rth < 2)
    {
      takeoff_rth = 1;
      h_counter_rth = z1_hat;
      endAuto_rth = 1;
      Vz_Hold_rth = 0.0;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ GPS_LAT_HOME, GPS_LON_HOME
     if(time_auto_rth > 2 && endAuto_rth == 1 && Status_waypoint_rth == 0)//waypoint1
    {
      target_LAT = GPS_LAT_HOME;
      target_LON = GPS_LON_HOME;
      Status_waypoint_rth = 1;
    }
    //Home Point and Landing
    if(time_auto_rth > 10 && abs(error_LAT) <= 150 && abs(error_LON) <= 150  && endAuto_rth == 1 && Status_waypoint_rth == 1)//50 10 Landing and position hold mode
    {
      timeLanding_rth++;
      if(timeLanding_rth >= 20)//relay 3 s Landing
      {
       takeoff_rth = 0;
      }
    }
  }
  //กรณีที่ไม่เข้า Mode == 4
  else//(Mode == 4 && CH_THR > MINCHECK && armed == 1)
  {
    takeoff_rth = 0;
    timeLanding_rth = 0;
    timeOff_rth = 0;
    time_auto_rth = 0;
    h_counter_rth = 0.1;//0.0
    h_counter_old_rth = 0.1;
    Vz_Hold_rth = 0.0;
    Status_waypoint_rth = 0;
  } 
////////////////
      //Function Counter การ Landing
      if(takeoff_rth == 0 && endAuto_rth == 1)//landing
      {
        h_counter_rth = h_counter_rth - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_rth = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_rth = 0.0;
         }
        if(z1_hat <= 0.18)
        {
         endAuto_rth = 0;
        }
      }
////////////////////////////////////
      // ถึงจุดหมายและนับเวลาการ armed == 0;
      if(time_auto_rth > 18 && endAuto_rth == 0) //15 s End waypoint quadrotor
        {
          timeOff_rth++;
          if(timeOff_rth > 10)//relay 2 s time-Off
          {
            if(z1_hat <= 0.18)
            {
              armed = 0;
            }
          } 
        }  
///////////////////////////////////////////////////////////
}
// ---------------------------------------------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
void Control_PositionHold(){
  if(Mode == 2 || Mode == 3 || Mode == 4 && GPS_FIX  == 1){
          error_LAT = (target_LAT - GPS_LAT1f)*6371000.0; // X Error cm
          error_LON = (target_LON - GPS_LON1f)*6371000.0;// Y Error   cm
          //Level 1 - error_LAT, error_LON
          error_LAT = constrain(error_LAT,-500,500);//200 = +-2 m
          error_LON = constrain(error_LON,-500,500);
          
          float target_speedLAT = error_LAT*Kp_speed;//P Control Velocity GPS
          float target_speedLON = error_LON*Kp_speed;//P Control Velocity GPS
          //Level 1 - target_speedLAT, target_speedLON
          target_speedLAT = constrain(target_speedLAT,-300,300);//+-100 cm/s = 1m/s
          target_speedLON = constrain(target_speedLON,-300,300);


          //AUX_1 > (Loiter_Sensor-20) && AUX_1 <= (Loiter_Sensor+20) เปิดการทำงาน ADNS3080
          //if (AUX_1 > (Loiter_Sensor-20) && AUX_1 <= (Loiter_Sensor+20) && z1_hat <= 0.6) { // 1 m work with Optical Flow
              //error_rate_LAT = 0.0 - posistion_Y;
              //error_rate_LON = 0.0 - posistion_X;
          //}
          //else {
              error_rate_LAT = target_speedLAT - vx_hat;
              error_rate_LON = target_speedLON - vy_hat;
          //}

          //Level 1 - error_rate_LAT, error_rate_LON
          error_rate_LAT = constrain(error_rate_LAT,-300,300);//+-200 cm/s
          error_rate_LON = constrain(error_rate_LON,-300,300);
          
          GPSI_LAT = GPSI_LAT + (error_rate_LAT*Ki_gps*0.05);//5Hz=0.2 ,, 20 Hz = 0.05
          GPSI_LON = GPSI_LON + (error_rate_LON*Ki_gps*0.05);  
          GPSI_LAT = constrain(GPSI_LAT,-200,200);//win speed +-200 cm/s
          GPSI_LON = constrain(GPSI_LON,-200,200);//250
          //Control_XEf = error_rate_LAT*Kd_gps;//P Control speed 
          //Control_YEf = error_rate_LON*Kd_gps;

          Control_XEf = error_LAT*Kp_gps + error_rate_LAT*Kd_gps + GPSI_LAT;//PID Control speed 
          Control_YEf = error_LON*Kp_gps + error_rate_LON*Kd_gps + GPSI_LON;  
          Control_XEf = constrain(Control_XEf,-800,800);//PWM 1000 - 1900
          Control_YEf = constrain(Control_YEf,-800,800);
          //The desired roll and pitch angles by tinnakon 
          float urolldesir = (Control_YEf*m_quad)/uAltitude;//uAltitude/2 = 1000 - 1900
          float upitchdesir = (Control_XEf*m_quad*-1.0)/uAltitude;//*-1
          urolldesir = constrain(urolldesir,-0.7,0.7);//+-0.7 = +-44.427
          upitchdesir = constrain(upitchdesir,-0.7,0.7);
          float temp_YBf = asin(urolldesir)*RAD_TO_DEG;//Control Earth Frame
          float temp_XBf = asin(upitchdesir)*RAD_TO_DEG;//Control Earth Frame

          //AUX_1 > (Loiter_Sensor-20) && AUX_1 <= (Loiter_Sensor+20) เปิดการทำงาน Distance Measure
//          if (AUX_1 > (Loiter_Sensor-20) && AUX_1 <= (Loiter_Sensor+20)) {
//            //Add Distance Measure Control in Position Hold
//            Control_XBf = (DCM00*temp_XBf) + (DCM01*temp_YBf) + Distance_X*0.08;//Control Body Frame use Rotation matrix
//            Control_YBf = (DCM10*temp_XBf) + (DCM11*temp_YBf) + Distance_Y*0.08;//Control Body Frame use Rotation matrix
//          }
//          else {
//            Control_XBf = (DCM00*temp_XBf) + (DCM01*temp_YBf);//Control Body Frame use Rotation matrix
//            Control_YBf = (DCM10*temp_XBf) + (DCM11*temp_YBf);//Control Body Frame use Rotation matrix
//          }

          Control_XBf = (DCM00*temp_XBf) + (DCM01*temp_YBf);//Control Body Frame use Rotation matrix
          Control_YBf = (DCM10*temp_XBf) + (DCM11*temp_YBf);//Control Body Frame use Rotation matrix
//          Control_XBf = (DCM00*temp_XBf) + (DCM01*temp_YBf) + Distance_X*0.08;//Control Body Frame use Rotation matrix
//          Control_YBf = (DCM10*temp_XBf) + (DCM11*temp_YBf) + Distance_Y*0.08;//Control Body Frame use Rotation matrix
          
          Control_XBf = constrain(Control_XBf, -20, 20);//+-20 deg
          Control_YBf = constrain(Control_YBf, -20, 20);//+-20 deg
          
          //The desired roll and pitch angles by paper Modeling and Backstepping-based Nonlinear Control 
          //Ashfaq Ahman Mian 2008 (eq.25 and eq.26)
          //float urolldesir = ((Control_XEf*m_quad*sin(ahrs_y))/uAltitude) - ((Control_YEf*m_quad*cos_yaw)/uAltitude);
          //float upitchdesir = ((Control_XEf*m_quad)/(uAltitude*cos_roll*cos_yaw)) - ((sin(ahrs_r)*sin(ahrs_y))/(cos_roll*cos_yaw));
          //urolldesir = constrain(urolldesir,-0.7,0.7);//+-0.7 = +-44.427
          //upitchdesir = constrain(upitchdesir,-0.7,0.7);
          //Control_YBf = asin(urolldesir)*RAD_TO_DEG;//Control roll eq.25 
          //Control_XBf = asin(upitchdesir)*RAD_TO_DEG*-1.0;//Control roll eq.26
          //Control_XBf = constrain(Control_XBf, -20, 20);//+-20 +- 44
          //Control_YBf = constrain(Control_YBf, -20, 20);
  }
  else{
      Control_XBf = 0.0;
      Control_YBf = 0.0;
      GPSI_LAT = 0.0;
      GPSI_LON = 0.0;
      target_LAT = GPS_LAT1f;//GPS_LAT_Hold
      target_LON = GPS_LON1f;//GPS_LON_Hold
  }
}//end  Control_PositionHold()
/////////////////////////////////////////////////////////////////////////
void Cal_GPS(){
  if(GPS_FIX  == 1){
 //Apply moving average filter to GPS data
      GPS_filter_index = (GPS_filter_index+1) % 4;// 4 moving average
      GPS_SUM_LAT[GPS_filter_index] = GPS_LAT1;
      GPS_SUM_LON[GPS_filter_index] = GPS_LON1;
   float sum1=0.0;
   float sum2=0.0;
  for(int i=0;i<4;i++)
  {
    sum1 += GPS_SUM_LAT[i];
    sum2 += GPS_SUM_LON[i];
  }
   GPS_LAT1f = sum1/4.0;
   GPS_LON1f = sum2/4.0;
  }
  else{
    GPS_numSat = 0;
  }
/* ///////////////////////////////////////
     //Diff speed
     actual_speedX = (GPS_LAT1f - GPS_LAT1_old)*6371000.0/0.3;//5 Hz = 0.2 ,cm/s  10000000.0 ,R = 6371000.0
     actual_speedY = (GPS_LON1f - GPS_LON1_old)*637100.0/0.3;//cm/s
     //actual_speedX = constrain(actual_speedX, -400, 400);//+-400 cm/s
     //actual_speedY = constrain(actual_speedY, -400, 400);//+-400 cm/s  
     GPS_LAT1_old = GPS_LAT1f;
     GPS_LON1_old = GPS_LON1f;
     actual_speedXf = (actual_speedX + actual_speedXold)/2.0;//Moving Average Filters/
     actual_speedYf = (actual_speedY + actual_speedYold)/2.0;//Moving Average Filters/
     actual_speedXold = actual_speedX;
     actual_speedYold = actual_speedY;
  
/////////////LeadFilter GPS/////////////////////////////////
    float lag_in_seconds = 0.85;//1.0 0.5
    float accel_contribution = (actual_speedXf - _last_velocityX) * lag_in_seconds * lag_in_seconds;
    float vel_contribution = actual_speedXf * lag_in_seconds;
    _last_velocityX = actual_speedXf;    // store velocity for next iteration
    GPS_LAT1lead = GPS_LAT1f  + vel_contribution + accel_contribution;
    float accel_contributio = (actual_speedYf - _last_velocityY) * lag_in_seconds * lag_in_seconds;
    float vel_contributio = actual_speedYf * lag_in_seconds;
    _last_velocityY = actual_speedYf;    // store velocity for next iteration
    GPS_LON1lead = GPS_LON1f  + vel_contributio + accel_contributio;
    */
}
