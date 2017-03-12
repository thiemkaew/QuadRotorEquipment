// Waypoint with Path Control
/*************************************************************************
 * //Path Control in Desing
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


//GPS Distance Calculator
/*************************************************************************
 * //Function to calculate the distance between two waypoints
 *************************************************************************/
float calc_dist(float flat1, float flon1, float flat2, float flon2)
{
      float dist_calc=0;
      float dist_calc2=0;
      float diflat=0;
      float diflon=0;
      
      //I've to spplit all the calculation in several steps. If i try to do it in a single line the arduino will explode.
      diflat=radians(flat2-flat1);
      flat1=radians(flat1);
      flat2=radians(flat2);
      diflon=radians((flon2)-(flon1));
      
      dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
      dist_calc2= cos(flat1);
      dist_calc2*=cos(flat2);
      dist_calc2*=sin(diflon/2.0);
      dist_calc2*=sin(diflon/2.0);
      dist_calc +=dist_calc2;
      
      dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
      
      dist_calc*=6371000.0; //Converting to meters
      //Serial.println(dist_calc);
      return dist_calc;
}

//---------------------------------------------------------------------------------------------------------------------

void Waypoint_ShortestPath() {
    nextDistance = 0.000;
    lessDistance = 0.000;
    nextDistance_tr = 0.000;
    lessDistance_tr = 0.000;
    nextDistance_waypoint = 0.000;
    nextDistance_waypoint_w3 = 0.000;
    allDistance = 0.000;
    allDistance_w3 = 0.000;
    int Waypointindex = 0;
    int lessIndex = 0;
    int lessIndex_tr = 0;
  
    //errorWaypoint Detect
    int errorWaypointBefore_W1 = 0;
    int errorWaypointBefore_W2 = 0;
    int errorWaypointBefore_W3 = 0;
    int errorWaypointBefore_W4 = 0;
    int errorWaypointAfter_W1 = 0;
    int errorWaypointAfter_W2 = 0;
    int errorWaypointAfter_W3 = 0;
    int errorWaypointAfter_W4 = 0; 
  
    //Waypoint_Reset
    float nextDistance_W1 = 0.000;
    float nextDistance_W2 = 0.000;
    float nextDistance_W3 = 0.000;
    float nextDistance_W4 = 0.000;
    float allDistance_W1 = 0.000;
    float allDistance_W2 = 0.000;
    float allDistance_W3 = 0.000;
    float allDistance_W4 = 0.000;
  
    //Distance for Chooseup about Waypoint
    float WaypointlessDistance = 0;
    int WaypointLast_W1 = 0;
    int WaypointLast_W2 = 0;
    int WaypointLast_W3 = 0;
    int WaypointLast_W4 = 0;



    total = ((sizeof(GPS_LAT_SPF_ALL)/4 - 1) + (sizeof(GPS_LON_SPF_ALL)/4 - 1))/2;
  //Serial.print(total);
  for(int i = 0; i <= total; i++)
  {
      nextDistance = calc_dist(GPS_LAT_SPF_HOME,GPS_LON_SPF_HOME,GPS_LAT_SPF_ALL[i],GPS_LON_SPF_ALL[i]);
      //Serial.print(nextDistance,3); Serial.print("\t");
      if(i == 0)
      {
        lessDistance = nextDistance;
      }
      //Home
      if( lessDistance > nextDistance ) 
      {
        lessDistance = nextDistance;
        lessIndex = i;
      }
  }

  for(int i = 0; i <= total; i++)
  {
      nextDistance_tr = calc_dist(GPS_LAT_SPF_TR,GPS_LON_SPF_TR,GPS_LAT_SPF_ALL[i],GPS_LON_SPF_ALL[i]);
      //Serial.print(nextDistance_tr,3); Serial.print("\t");
      if(i == 0)
      {
        lessDistance_tr = nextDistance_tr;
      }
      //Transport
      if( lessDistance_tr > nextDistance_tr ) 
      {
        lessDistance_tr = nextDistance_tr;
        lessIndex_tr = i;
      } 
  }
//  Serial.print(" | "); Serial.print(lessDistance,3); Serial.print("  ");
//  Serial.print(lessIndex); Serial.print("  ");
//  Serial.print(GPS_LAT_SPF_ALL[lessIndex],6); Serial.print("  ");
//  Serial.print(GPS_LON_SPF_ALL[lessIndex],6); Serial.print(" || "); 
//  Serial.print(lessDistance_tr,3); Serial.print("  ");
//  Serial.print(lessIndex_tr); Serial.print("  ");
//  Serial.print(GPS_LAT_SPF_ALL[lessIndex_tr],6); Serial.print("  ");
//  Serial.print(GPS_LON_SPF_ALL[lessIndex_tr],6); Serial.print(" || ");

//------------------------------------------------------------------------------------------------------------------------------
  //////////////////////////////////////
  //Distance Total Waypoint 1
  //////////////////////////////////////
  total_W1 = ((sizeof(GPS_LAT_SPF_W1)/4 - 1) + (sizeof(GPS_LON_SPF_W1)/4 - 1))/2;
  for(int i = 0; i <= total_W1; i++)
  {
    if(GPS_LAT_SPF_ALL[lessIndex] == GPS_LAT_SPF_W1[i] && GPS_LON_SPF_ALL[lessIndex] == GPS_LON_SPF_W1[i])
    {
       errorWaypointBefore_W1 = i;
    }
    if(GPS_LAT_SPF_ALL[lessIndex_tr] == GPS_LAT_SPF_W1[i] && GPS_LON_SPF_ALL[lessIndex_tr] == GPS_LON_SPF_W1[i])
    {
       errorWaypointAfter_W1 = total_W1 - i;
       WaypointLast_W1 = i;
    }
  }
  //Serial.print(errorWaypointBefore_W1);  Serial.print("  "); Serial.print(errorWaypointAfter_W1);
  for(int i = 0 + errorWaypointBefore_W1 ; i < total_W1 - errorWaypointAfter_W1; i++)
  {
      nextDistance_W1 = calc_dist(GPS_LAT_SPF_W1[i],GPS_LON_SPF_W1[i],GPS_LAT_SPF_W1[i+1],GPS_LON_SPF_W1[i+1]);
      allDistance_W1 = allDistance_W1 + nextDistance_W1;
      //Serial.print(nextDistance_W1,3);  Serial.print("\t");
  }
  //Serial.print(" | "); Serial.print(allDistance_W1,3); Serial.print(" || "); Serial.print("\t");

//------------------------------------------------------------------------------------------------------------------------------
  //////////////////////////////////////
  //Distance Total Waypoint 2
  //////////////////////////////////////
  total_W2 = ((sizeof(GPS_LAT_SPF_W2)/4 - 1) + (sizeof(GPS_LON_SPF_W2)/4 - 1))/2;
  for(int i = 0; i <= total_W2; i++)
  {
    if(GPS_LAT_SPF_ALL[lessIndex] == GPS_LAT_SPF_W2[i] && GPS_LON_SPF_ALL[lessIndex] == GPS_LON_SPF_W2[i])
    {
       errorWaypointBefore_W2 = i;
    }
    if(GPS_LAT_SPF_ALL[lessIndex_tr] == GPS_LAT_SPF_W2[i] && GPS_LON_SPF_ALL[lessIndex_tr] == GPS_LON_SPF_W2[i])
    {
       errorWaypointAfter_W2 = total_W2 - i;
       WaypointLast_W2 = i;
    }
  }
  //Serial.print(errorWaypointBefore_W2);  Serial.print("  "); Serial.print(errorWaypointAfter_W2);
  for(int i = 0 + errorWaypointBefore_W2 ; i < total_W2 - errorWaypointAfter_W2; i++)
  {
      nextDistance_W2 = calc_dist(GPS_LAT_SPF_W2[i],GPS_LON_SPF_W2[i],GPS_LAT_SPF_W2[i+1],GPS_LON_SPF_W2[i+1]);
      allDistance_W2 = allDistance_W2 + nextDistance_W2;
      //Serial.print(nextDistance_W1,3);  Serial.print("\t");
  }
  //Serial.print(" | "); Serial.print(allDistance_W2,3); Serial.print(" || "); Serial.print("\t");

//------------------------------------------------------------------------------------------------------------------------------
  //////////////////////////////////////
  //Distance Total Waypoint 3
  //////////////////////////////////////
  total_W3 = ((sizeof(GPS_LAT_SPF_W3)/4 - 1) + (sizeof(GPS_LON_SPF_W3)/4 - 1))/2;
  for(int i = 0; i <= total_W3; i++)
  {
    if(GPS_LAT_SPF_ALL[lessIndex] == GPS_LAT_SPF_W3[i] && GPS_LON_SPF_ALL[lessIndex] == GPS_LON_SPF_W3[i])
    {
       errorWaypointBefore_W3 = i;
    }
    if(GPS_LAT_SPF_ALL[lessIndex_tr] == GPS_LAT_SPF_W3[i] && GPS_LON_SPF_ALL[lessIndex_tr] == GPS_LON_SPF_W3[i])
    {
       errorWaypointAfter_W3 = total_W3 - i;
       WaypointLast_W3 = i;
    }
  }
  //Serial.print(errorWaypointBefore_W3);  Serial.print("  "); Serial.print(errorWaypointAfter_W3);
  for(int i = 0 + errorWaypointBefore_W3 ; i < total_W3 - errorWaypointAfter_W3; i++)
  {
      nextDistance_W3 = calc_dist(GPS_LAT_SPF_W3[i],GPS_LON_SPF_W3[i],GPS_LAT_SPF_W3[i+1],GPS_LON_SPF_W3[i+1]);
      allDistance_W3 = allDistance_W3 + nextDistance_W3;
      //Serial.print(nextDistance_W1,3);  Serial.print("\t");
  }
  //Serial.print(" | "); Serial.print(allDistance_W3,3); Serial.print(" || "); Serial.print("\t");

//------------------------------------------------------------------------------------------------------------------------------
  //////////////////////////////////////
  //Distance Total Waypoint 4
  //////////////////////////////////////
  total_W4 = ((sizeof(GPS_LAT_SPF_W4)/4 - 1) + (sizeof(GPS_LON_SPF_W4)/4 - 1))/2;
  for(int i = 0; i <= total_W4; i++)
  {
    if(GPS_LAT_SPF_ALL[lessIndex] == GPS_LAT_SPF_W4[i] && GPS_LON_SPF_ALL[lessIndex] == GPS_LON_SPF_W4[i])
    {
       errorWaypointBefore_W4 = i;
    }
    if(GPS_LAT_SPF_ALL[lessIndex_tr] == GPS_LAT_SPF_W4[i] && GPS_LON_SPF_ALL[lessIndex_tr] == GPS_LON_SPF_W4[i])
    {
       errorWaypointAfter_W4 = total_W4 - i;
       WaypointLast_W4 = i;
    }
  }
  //Serial.print(errorWaypointBefore_W4);  Serial.print("  "); Serial.print(errorWaypointAfter_W4);
  for(int i = 0 + errorWaypointBefore_W4 ; i < total_W4 - errorWaypointAfter_W4; i++)
  {
      nextDistance_W4 = calc_dist(GPS_LAT_SPF_W4[i],GPS_LON_SPF_W4[i],GPS_LAT_SPF_W4[i+1],GPS_LON_SPF_W4[i+1]);
      allDistance_W4 = allDistance_W4 + nextDistance_W4;
      //Serial.print(nextDistance_W1,3);  Serial.print("\t");
  }
  //Serial.print(" | "); Serial.print(allDistance_W4,3); Serial.print(" || "); Serial.print("\t");

//------------------------------------------------------------------------------------------------------------------------------
// Compare AllDistance to Choose best less Distance
  for(int i = 0; i < WaypointAll; i++)
  {
    if( i == 0 ) 
    {
       WaypointCompare[i] = allDistance_W1;
       WaypointlessDistance = WaypointCompare[i];
    }
    else if ( i == 1 ) 
    {
       WaypointCompare[i] = allDistance_W2;
    }
    else if ( i == 2 ) 
    {
       WaypointCompare[i] = allDistance_W3;
    }
    else if ( i == 3 ) 
    {
       WaypointCompare[i] = allDistance_W4;
    }
    if( WaypointlessDistance > WaypointCompare[i])
    {
      WaypointlessDistance = WaypointCompare[i];
      Waypointindex = i;
    }
  }
      if(Waypointindex == 0)
      {
        //Value for Next Calculate
          for(int i = 0 + errorWaypointBefore_W1; i <= total_W1 - errorWaypointAfter_W1; i++)
          {
              if( i == errorWaypointBefore_W1 )
              {
                waypoint1_LAT = GPS_LAT_SPF_W1[errorWaypointBefore_W1];
                waypoint1_LON = GPS_LON_SPF_W1[errorWaypointBefore_W1];
              }
              else if( i == errorWaypointBefore_W1 + 1)
              {
                waypoint2_LAT = GPS_LAT_SPF_W1[errorWaypointBefore_W1+1];
                waypoint2_LON = GPS_LON_SPF_W1[errorWaypointBefore_W1+1];
              }
              else if( i == errorWaypointBefore_W1 + 2)
              {
                waypoint3_LAT = GPS_LAT_SPF_W1[errorWaypointBefore_W1+2];
                waypoint3_LON = GPS_LON_SPF_W1[errorWaypointBefore_W1+2];
              }
              else if( i == errorWaypointBefore_W1 + 3)
              {
                waypoint4_LAT = GPS_LAT_SPF_W1[errorWaypointBefore_W1+3];
                waypoint4_LON = GPS_LON_SPF_W1[errorWaypointBefore_W1+3];
              }
              else if( i == errorWaypointBefore_W1 + 4)
              {
                waypoint5_LAT = GPS_LAT_SPF_W1[errorWaypointBefore_W1+4];
                waypoint5_LON = GPS_LON_SPF_W1[errorWaypointBefore_W1+4];
              }
              else if( i == errorWaypointBefore_W1 + 5)
              {
                waypoint6_LAT = GPS_LAT_SPF_W1[errorWaypointBefore_W1+5];
                waypoint6_LON = GPS_LON_SPF_W1[errorWaypointBefore_W1+5];
              }
          }
          waypointCheck = (total_W1 - errorWaypointAfter_W1) - errorWaypointBefore_W1 + 1;
      } 
      else if(Waypointindex == 1){
        //Value for Next Calculate
          for(int i = 0 + errorWaypointBefore_W2; i <= total_W2 - errorWaypointAfter_W2; i++)
          {
              if( i == errorWaypointBefore_W2)
              {
                waypoint1_LAT = GPS_LAT_SPF_W2[errorWaypointBefore_W2];
                waypoint1_LON = GPS_LON_SPF_W2[errorWaypointBefore_W2];
              }
              else if( i == errorWaypointBefore_W2 + 1)
              {
                waypoint2_LAT = GPS_LAT_SPF_W2[errorWaypointBefore_W2+1];
                waypoint2_LON = GPS_LON_SPF_W2[errorWaypointBefore_W2+1];
              }
              else if( i == errorWaypointBefore_W2 + 2)
              {
                waypoint3_LAT = GPS_LAT_SPF_W2[errorWaypointBefore_W2+2];
                waypoint3_LON = GPS_LON_SPF_W2[errorWaypointBefore_W2+2];
              }
              else if( i == errorWaypointBefore_W2 + 3)
              {
                waypoint4_LAT = GPS_LAT_SPF_W2[errorWaypointBefore_W2+3];
                waypoint4_LON = GPS_LON_SPF_W2[errorWaypointBefore_W2+3];
              }
              else if( i == errorWaypointBefore_W2 + 4)
              {
                waypoint5_LAT = GPS_LAT_SPF_W2[errorWaypointBefore_W2+4];
                waypoint5_LON = GPS_LON_SPF_W2[errorWaypointBefore_W2+4];
              }
              else if( i == errorWaypointBefore_W2 + 5)
              {
                waypoint6_LAT = GPS_LAT_SPF_W2[errorWaypointBefore_W2+5];
                waypoint6_LON = GPS_LON_SPF_W2[errorWaypointBefore_W2+5];
              }
          }
          waypointCheck = (total_W2 - errorWaypointAfter_W2) - errorWaypointBefore_W2 + 1;
      }
      else if(Waypointindex == 2){
        //Value for Next Calculate
          for(int i = 0 + errorWaypointBefore_W3; i <= total_W3 - errorWaypointAfter_W3; i++)
          {
              if( i == errorWaypointBefore_W3)
              {
                waypoint1_LAT = GPS_LAT_SPF_W3[errorWaypointBefore_W3];
                waypoint1_LON = GPS_LON_SPF_W3[errorWaypointBefore_W3];
              }
              else if( i == errorWaypointBefore_W3 + 1)
              {
                waypoint2_LAT = GPS_LAT_SPF_W3[errorWaypointBefore_W3+1];
                waypoint2_LON = GPS_LON_SPF_W3[errorWaypointBefore_W3+1];
              }
              else if( i == errorWaypointBefore_W3 + 2)
              {
                waypoint3_LAT = GPS_LAT_SPF_W3[errorWaypointBefore_W3+2];
                waypoint3_LON = GPS_LON_SPF_W3[errorWaypointBefore_W3+2];
              }
              else if( i == errorWaypointBefore_W3 + 3)
              {
                waypoint4_LAT = GPS_LAT_SPF_W3[errorWaypointBefore_W3+3];
                waypoint4_LON = GPS_LON_SPF_W3[errorWaypointBefore_W3+3];
              }
              else if( i == errorWaypointBefore_W3 + 4)
              {
                waypoint5_LAT = GPS_LAT_SPF_W3[errorWaypointBefore_W3+4];
                waypoint5_LON = GPS_LON_SPF_W3[errorWaypointBefore_W3+4];
              }
              else if( i == errorWaypointBefore_W3 + 5)
              {
                waypoint6_LAT = GPS_LAT_SPF_W3[errorWaypointBefore_W3+5];
                waypoint6_LON = GPS_LON_SPF_W3[errorWaypointBefore_W3+5];
              }
          }
          waypointCheck = (total_W3 - errorWaypointAfter_W3) - errorWaypointBefore_W3 + 1;
      } 
      else if(Waypointindex == 3){
        //Value for Next Calculate
          for(int i = 0 + errorWaypointBefore_W4; i <= total_W4 - errorWaypointAfter_W4; i++)
          {
              if( i == errorWaypointBefore_W4)
              {
                waypoint1_LAT = GPS_LAT_SPF_W4[errorWaypointBefore_W4];
                waypoint1_LON = GPS_LON_SPF_W4[errorWaypointBefore_W4];
              }
              else if( i == errorWaypointBefore_W4 + 1)
              {
                waypoint2_LAT = GPS_LAT_SPF_W4[errorWaypointBefore_W4+1];
                waypoint2_LON = GPS_LON_SPF_W4[errorWaypointBefore_W4+1];
              }
              else if( i == errorWaypointBefore_W4 + 2)
              {
                waypoint3_LAT = GPS_LAT_SPF_W4[errorWaypointBefore_W4+2];
                waypoint3_LON = GPS_LON_SPF_W4[errorWaypointBefore_W4+2];
              }
              else if( i == errorWaypointBefore_W4 + 3)
              {
                waypoint4_LAT = GPS_LAT_SPF_W4[errorWaypointBefore_W4+3];
                waypoint4_LON = GPS_LON_SPF_W4[errorWaypointBefore_W4+3];
              }
              else if( i == errorWaypointBefore_W4 + 4)
              {
                waypoint5_LAT = GPS_LAT_SPF_W4[errorWaypointBefore_W4+4];
                waypoint5_LON = GPS_LON_SPF_W4[errorWaypointBefore_W4+4];
              }
              else if( i == errorWaypointBefore_W4 + 5)
              {
                waypoint6_LAT = GPS_LAT_SPF_W4[errorWaypointBefore_W4+5];
                waypoint6_LON = GPS_LON_SPF_W4[errorWaypointBefore_W4+5];
              }
          }
          waypointCheck = (total_W4 - errorWaypointAfter_W4) - errorWaypointBefore_W4 + 1;
      }
      
//      Serial.print(" | "); Serial.print(WaypointlessDistance, 3); Serial.print(" | ");
//      Serial.print(" | "); Serial.print(Waypointindex); Serial.print(" | ");
//      Serial.print(" | "); Serial.print(waypointCheck); Serial.print(" || ");
//      Serial.print("  "); Serial.print(waypoint1_LAT,6); Serial.print("  "); 
//      Serial.print("  "); Serial.print(waypoint1_LON,6); Serial.print("  "); 
//      Serial.print("  "); Serial.print(waypoint2_LAT,6); Serial.print("  "); 
//      Serial.print("  "); Serial.print(waypoint2_LON,6); Serial.print("  "); 
//      Serial.print("  "); Serial.print(waypoint3_LAT,6); Serial.print("  "); 
//      Serial.print("  "); Serial.print(waypoint3_LON,6); Serial.print("  "); 
//      Serial.print("  "); Serial.print(waypoint4_LAT,6); Serial.print("  "); 
//      Serial.print("  "); Serial.print(waypoint4_LON,6); Serial.print("  "); 
//      Serial.print("  "); Serial.print(waypoint5_LAT,6); Serial.print("  "); 
//      Serial.print("  "); Serial.print(waypoint5_LON,6); Serial.print("  "); 
//      Serial.print("  "); Serial.print(waypoint6_LAT,6); Serial.print("  "); 
//      Serial.print("  "); Serial.print(waypoint6_LON,6); Serial.print("  "); 
//      Serial.println();
}


//Control_Path and System
//Automatic Takeoff and Landing in TransportArea
// ---------------------------------------------------------------------------------------------------------------
// Waypoint with Path Control
/*************************************************************************
 * //Path Control in Desing
 *************************************************************************/
/*
     Takeoff......... --> waypoint1 --> waypoint2 --> waypoint3 --> waypoint4 --> waypoint5 --> waypoint6 --> | ....Landing..... |
     Home                                                                                                     | Transport_Point  |
                                                                                                              | Transport_Point  |
                                                                                                              | ....Takeoff..... |
     Landing......... <-- waypoint1 <-- waypoint2 <-- waypoint3 <-- waypoint4 <-- waypoint5 <-- waypoint6 <-----------------------
     Home
 */

////--------------------------------------------------------------------------------------------------------------------------------------------------------
//// เปิดระบบการทำงานใน State 2 Takeoff ที่ HomePoint และ ไปที่ตำแหน่ง Waypoint 5 จุด หลังจากนั้นลงไปที่ GPS_LAT, GPS_LON
//void AutomaticTransport(){
// //Altitude control and 1 waypoint navigation
//  ////mode 3
//  if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome == 0)
//  //if(Mode == 3 && CH_THRf > MINCHECK && AutoTransHome == 0)
//  {
//    //Takeoff Checking
//    if(time_auto < 2)
//    {//Check time < 5
//      takeoff = 1;
//    }
//    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ waypoint1_LAT, waypoint1_LON
//     if(time_auto > 8 && z1_hat >= h_control && endAuto == 1 && Status_waypoint == 0)//waypoint1
//    {
//      target_LAT = waypoint1_LAT_WDefault;
//      target_LON = waypoint1_LON_WDefault;
//      Status_waypoint = 1;
//    }
//    // เคลื่อนที่ QuadRotor ไปที่ waypoint2_LAT, waypoint2_LON
//     if(time_auto > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto == 1 && Status_waypoint == 1)//50 10 Landing and position hold mode
//    {
//      target_LAT = waypoint2_LAT_WDefault;
//      target_LON = waypoint2_LON_WDefault;
//      Status_waypoint = 2;
//    }
//   // เคลื่อนที่ QuadRotor ไปที่ waypoint3_LAT, waypoint3_LON
//    if(time_auto > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto == 1 && Status_waypoint == 2)//50 10 Landing and position hold mode
//    {
//      target_LAT = waypoint3_LAT_WDefault;
//      target_LON = waypoint3_LON_WDefault;
//      Status_waypoint = 3;
//    }
//    // เคลื่อนที่ QuadRotor ไปที่ waypoint4_LAT, waypoint4_LON
//    if(time_auto > 32 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto == 1 && Status_waypoint == 3)//50 10 Landing and position hold mode
//    {
//      target_LAT = waypoint4_LAT_WDefault;
//      target_LON = waypoint4_LON_WDefault;
//      Status_waypoint = 4;
//    }
//    // เคลื่อนที่ QuadRotor ไปที่ waypoint5_LAT, waypoint5_LON
//    if(time_auto > 40 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto == 1 && Status_waypoint == 4)//50 10 Landing and position hold mode
//    {
//      target_LAT = waypoint5_LAT_WDefault;
//      target_LON = waypoint5_LON_WDefault;
//      Status_waypoint = 5;
//    }
//    // เคลื่อนที่ QuadRotor ไปที่ waypoint6_LAT, waypoint6_LON
//    if(time_auto > 48 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto == 1 && Status_waypoint == 5)//50 10 Landing and position hold mode
//    {
//      target_LAT = waypoint6_LAT_WDefault;
//      target_LON = waypoint6_LON_WDefault;
//      Status_waypoint = 6;
//    }
//    // เคลื่อนที่ QuadRotor ไปยัง Transport Point
//    if(time_auto > 56 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto == 1 && Status_waypoint == 6)//50 10 Landing and position hold mode
//    {
//      target_LAT = GPS_LAT_HOME_Transport;
//      target_LON = GPS_LON_HOME_Transport;
//      //target_LAT = GPS_LAT_HOME;
//      //target_LON = GPS_LON_HOME;
//      Status_waypoint = 7;
//    }
//    //Transport Point and Landing
//    if(time_auto > 64 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto == 1 && Status_waypoint == 7)//50 10 Landing and position hold mode
//    {
//      timeLanding++;
//      if(timeLanding >= 20)//relay 2 s Landing
//      {
//       takeoff = 0;
//      }
//    }   
//  }
//  //กรณีที่ไม่เข้า Mode == 3
//  else//(Mode == 3 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 0)
//  {
//    takeoff = 0;
//    timeLanding = 0;
//    timeOff = 0;
//    time_auto = 0;
//    h_counter = 0.1;//0.0
//    h_counter_old = 0.1;
//    Vz_Hold = 0.0;
//    Status_waypoint = 0;
//  } 
//// Takeoff and Landing Level 2
////////////////////////////////////////////////////////////////////// 
//      //Function Counter การ Takeoff
//      if(h_counter < h_control && takeoff == 1 && AutoTransHome == 0)//take-off
//      {
//        endAuto = 1;
//        h_counter = h_counter + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
//        Vz_Hold = 0.82;//(m/s) roop 10 Hz
//        //h_counter_old = h_counter;
//      }
//      else{
//        Vz_Hold = 0.0;
//      }
// ////////////////
//      //Function Counter การ Landing
//      if(takeoff == 0 && endAuto == 1 && AutoTransHome == 0)//landing
//      {
//        h_counter = h_counter - 0.059;//0.023 ramp input hz  landing
//        Vz_Hold = -0.59;//(m/s) roop 10 Hz
//         if(z1_hat <= 0.6){
//           Vz_Hold = 0.0;
//         }
//        //h_counter_old = h_counter;
//        if(z1_hat <= 0.18)
//        {
//         endAuto = 0;
//        }
//      }
//////////////////////////////////////
/////////////////////////////////////////////////////////////  
//}
//// ---------------------------------------------------------------------------------------------------------------
//
//// Automatic Transport to Home
//// ---------------------------------------------------------------------------------------------------------------
//void AutomaticTransport_Home() {
//   if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome == 1)
//  {
//    //Takeoff Checking
//    if(time_auto_tranH < 2)
//    {//Check time < 5
//      takeoff_tranH = 1;
//    }
//    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ GPS_LAT_HOME, GPS_LON_HOME
//     if(time_auto_tranH > 8 && z1_hat >= h_control && endAuto_tranH == 1 && Status_waypoint_tranH == 0)//waypoint1
//    {
//      target_LAT = waypoint6_LAT_WDefault;
//      target_LON = waypoint6_LON_WDefault;
//      Status_waypoint_tranH = 1;
//    }
//    if(time_auto_tranH > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH == 1 && Status_waypoint_tranH == 1)//waypoint5
//    {
//      target_LAT = waypoint5_LAT_WDefault;
//      target_LON = waypoint5_LON_WDefault;
//      Status_waypoint_tranH = 2;
//    }
//    if(time_auto_tranH > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH == 1 && Status_waypoint_tranH == 2)//waypoint5
//    {
//      target_LAT = waypoint4_LAT_WDefault;
//      target_LON = waypoint4_LON_WDefault;
//      Status_waypoint_tranH = 3;
//    }
//    if(time_auto_tranH > 32 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH == 1 && Status_waypoint_tranH == 3)//waypoint5
//    {
//      target_LAT = waypoint3_LAT_WDefault;
//      target_LON = waypoint3_LON_WDefault;
//      Status_waypoint_tranH = 4;
//    }
//    if(time_auto_tranH > 40 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH == 1 && Status_waypoint_tranH == 4)//waypoint5
//    {
//      target_LAT = waypoint2_LAT_WDefault;
//      target_LON = waypoint2_LON_WDefault;
//      Status_waypoint_tranH = 5;
//    }
//    if(time_auto_tranH > 48 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH == 1 && Status_waypoint_tranH == 5)//waypoint5
//    {
//      target_LAT = waypoint1_LAT_WDefault;
//      target_LON = waypoint1_LON_WDefault;
//      Status_waypoint_tranH = 6;
//    }
//    if(time_auto_tranH > 56 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH == 1 && Status_waypoint_tranH == 6)//waypoint5
//    {
//      target_LAT = GPS_LAT_HOME;
//      target_LON = GPS_LON_HOME;
//      Status_waypoint_tranH = 7;
//    }
//    //Home Point and Landing
//    if(time_auto_tranH > 64 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH == 1 && Status_waypoint_tranH == 7)//50 10 Landing and position hold mode
//    {
//      timeLanding_tranH++;
//      if(timeLanding_tranH >= 20)//relay 2 s Landing
//      {
//       takeoff_tranH = 0;
//      }
//    }
//  }
//  //กรณีที่ไม่เข้า AutoTransHome = 1;
//  else//(Mode = 3 && AutoTransHome == 1 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 1)
//  {
//    takeoff_tranH = 0;
//    timeLanding_tranH = 0;
//    timeOff_tranH = 0;
//    time_auto_tranH = 0;
//    h_counter_tranH = 0.1;//0.0
//    h_counter_old_tranH = 0.1;
//    Vz_Hold_tranH = 0.0;
//    Status_waypoint_tranH = 0;
//    AutoTransHome = 0;
//  } 
//// Takeoff and Landing Level 2
////////////////////////////////////////////////////////////////////// 
//      //Function Counter การ Takeoff
//      if(h_counter_tranH < h_control && takeoff_tranH == 1 && AutoTransHome == 1)//take-off
//      {
//        endAuto_tranH = 1;
//        h_counter_tranH = h_counter_tranH + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
//        Vz_Hold_tranH = 0.82;//(m/s) roop 10 Hz
//        //h_counter_old = h_counter;
//      }
//      else{
//        Vz_Hold_tranH = 0.0;
//      }
// ////////////////
//      //Function Counter การ Landing
//      if(takeoff_tranH == 0 && endAuto_tranH == 1 && AutoTransHome == 1)//landing
//      {
//        h_counter_tranH = h_counter_tranH - 0.059;//0.023 ramp input hz  landing
//        Vz_Hold_tranH = -0.59;//(m/s) roop 10 Hz
//         if(z1_hat <= 0.6){
//           Vz_Hold_tranH = 0.0;
//         }
//        //h_counter_old = h_counter;
//        if(z1_hat <= 0.18)
//        {
//         endAuto_tranH = 0;
//        }
//      }
//////////////////////////////////////
/////////////////////////////////////////////////////////////
//}
//// ---------------------------------------------------------------------------------------------------------------



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------------------------------------
// เปิดระบบการทำงานใน State 2 Takeoff ที่ HomePoint และ ไปที่ตำแหน่ง Waypoint 5 จุด หลังจากนั้นลงไปที่ GPS_LAT, GPS_LON
void AutomaticTransport(){
 //Altitude control and 1 waypoint navigation
  ////mode 3
  if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome == 0)
  //if(Mode == 3 && CH_THRf > MINCHECK && AutoTransHome == 0)
  {
    //Takeoff Checking
    if(time_auto < 2)
    {//Check time < 5
      takeoff = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ waypoint1_LAT, waypoint1_LON
     if(time_auto > 8 && z1_hat >= h_control && endAuto == 1 && Status_waypoint == 0)//waypoint1
    {
      target_LAT = waypoint1_LAT_WDefault1;
      target_LON = waypoint1_LON_WDefault1;
      Status_waypoint = 1;
    }
    if(time_auto > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto == 1 && Status_waypoint == 1)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint2_LAT_WDefault1;
      target_LON = waypoint2_LON_WDefault1;
      Status_waypoint = 2;
    }  
    if(time_auto > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto == 1 && Status_waypoint == 2)//50 10 Landing and position hold mode
    {
      target_LAT = GPS_LAT_HOME_Transport;
      target_LON = GPS_LON_HOME_Transport;
      Status_waypoint = 3;
    }  
    //Transport Point and Landing
    if(time_auto > 32 && abs(error_LAT) <= 200 && abs(error_LON) <= 200 && endAuto == 1 && Status_waypoint == 3)//50 10 Landing and position hold mode
    {
      timeLanding++;
      if(timeLanding >= 20)//relay 2 s Landing
      {
       takeoff = 0;
      }
    }   
  }
  //กรณีที่ไม่เข้า Mode == 3
  else//(Mode == 3 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 0)
  {
    takeoff = 0;
    timeLanding = 0;
    timeOff = 0;
    time_auto = 0;
    h_counter = 0.1;//0.0
    h_counter_old = 0.1;
    Vz_Hold = 0.0;
    Status_waypoint = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter < h_control && takeoff == 1 && AutoTransHome == 0)//take-off
      {
        endAuto = 1;
        h_counter = h_counter + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff == 0 && endAuto == 1 && AutoTransHome == 0)//landing
      {
        h_counter = h_counter - 0.059;//0.023 ramp input hz  landing
        Vz_Hold = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto = 0;
        }
      }
////////////////////////////////////
///////////////////////////////////////////////////////////  
}
// ---------------------------------------------------------------------------------------------------------------

// Automatic Transport to Home
// ---------------------------------------------------------------------------------------------------------------
void AutomaticTransport_Home() {
   if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome == 1)
  {
    //Takeoff Checking
    if(time_auto_tranH < 2)
    {//Check time < 5
      takeoff_tranH = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ GPS_LAT_HOME, GPS_LON_HOME
     if(time_auto_tranH > 8 && z1_hat >= h_control && endAuto_tranH == 1 && Status_waypoint_tranH == 0)//waypoint1
    {
      target_LAT = waypoint2_LAT_WDefault1;
      target_LON = waypoint2_LON_WDefault1;
      Status_waypoint_tranH = 1;
    }
    if(time_auto_tranH > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH == 1 && Status_waypoint_tranH == 1)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint1_LAT_WDefault1;
      target_LON = waypoint1_LON_WDefault1;
      Status_waypoint_tranH = 2;
    }
    if(time_auto_tranH > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH == 1 && Status_waypoint_tranH == 2)//50 10 Landing and position hold mode
    {
      target_LAT = GPS_LAT_HOME;
      target_LON = GPS_LON_HOME;
      Status_waypoint_tranH = 3;
    }
    //Home Point and Landing
    if(time_auto_tranH > 32 && abs(error_LAT) <= 200 && abs(error_LON) <= 200 && endAuto_tranH == 1 && Status_waypoint_tranH == 3)//50 10 Landing and position hold mode
    {
      timeLanding_tranH++;
      if(timeLanding_tranH >= 20)//relay 2 s Landing
      {
       takeoff_tranH = 0;
      }
    }
  }
  //กรณีที่ไม่เข้า AutoTransHome = 1;
  else//(Mode = 3 && AutoTransHome == 1 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 1)
  {
    takeoff_tranH = 0;
    timeLanding_tranH = 0;
    timeOff_tranH = 0;
    time_auto_tranH = 0;
    h_counter_tranH = 0.1;//0.0
    h_counter_old_tranH = 0.1;
    Vz_Hold_tranH = 0.0;
    Status_waypoint_tranH = 0;
    AutoTransHome = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_tranH < h_control && takeoff_tranH == 1 && AutoTransHome == 1)//take-off
      {
        endAuto_tranH = 1;
        h_counter_tranH = h_counter_tranH + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_tranH = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_tranH = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_tranH == 0 && endAuto_tranH == 1 && AutoTransHome == 1)//landing
      {
        h_counter_tranH = h_counter_tranH - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_tranH = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
         //if(Altitude_sonaf <= 0.6){
           Vz_Hold_tranH = 0.0;
         }
        //h_counter_old = h_counter; //Altitude_sonaf
        if(z1_hat <= 0.18)
        //if(Altitude_sonaf <= 0.18)
        {
         endAuto_tranH = 0;
        }
      }
////////////////////////////////////
///////////////////////////////////////////////////////////
}
// ---------------------------------------------------------------------------------------------------------------

//Station 2
void AutomaticTransport_sta2(){
 //Altitude control and 1 waypoint navigation
  ////mode 3
  if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_sta2 == 0)
  //if(Mode == 3 && CH_THRf > MINCHECK && AutoTransHome == 0)
  {
    //Takeoff Checking
    if(time_auto_sta2 < 2)
    {//Check time < 5
      takeoff_sta2 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ waypoint1_LAT, waypoint1_LON
     if(time_auto_sta2 > 8 && z1_hat >= h_control && endAuto_sta2 == 1 && Status_waypoint_sta2 == 0)//waypoint1
    {
      target_LAT = waypoint1_LAT_WDefault2;
      target_LON = waypoint1_LON_WDefault2;
      Status_waypoint_sta2 = 1;
    }
    if(time_auto_sta2 > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_sta2 == 1 && Status_waypoint_sta2 == 1)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint2_LAT_WDefault2;
      target_LON = waypoint2_LON_WDefault2;
      Status_waypoint_sta2 = 2;
    }  
    if(time_auto_sta2 > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_sta2 == 1 && Status_waypoint_sta2 == 2)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint3_LAT_WDefault2;
      target_LON = waypoint3_LON_WDefault2;
      Status_waypoint_sta2 = 3;
    }
    if(time_auto_sta2 > 32 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_sta2 == 1 && Status_waypoint_sta2 == 3)//50 10 Landing and position hold mode
    {
      target_LAT = GPS_LAT_HOME_Transport;
      target_LON = GPS_LON_HOME_Transport;
      Status_waypoint_sta2 = 4;
    }    
    //Transport Point and Landing
    if(time_auto_sta2 > 40 && abs(error_LAT) <= 200 && abs(error_LON) <= 200 && endAuto_sta2 == 1 && Status_waypoint_sta2 == 4)//50 10 Landing and position hold mode
    {
      timeLanding_sta2++;
      if(timeLanding_sta2 >= 20)//relay 2 s Landing
      {
       takeoff_sta2 = 0;
      }
    }   
  }
  //กรณีที่ไม่เข้า Mode == 3
  else//(Mode == 3 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 0)
  {
    takeoff_sta2 = 0;
    timeLanding_sta2 = 0;
    timeOff_sta2 = 0;
    time_auto_sta2 = 0;
    h_counter_sta2 = 0.1;//0.0
    h_counter_old_sta2 = 0.1;
    Vz_Hold_sta2 = 0.0;
    Status_waypoint_sta2 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_sta2 < h_control && takeoff_sta2 == 1 && AutoTransHome_sta2 == 0)//take-off
      {
        endAuto_sta2 = 1;
        h_counter_sta2 = h_counter_sta2 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_sta2 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_sta2 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_sta2 == 0 && endAuto_sta2 == 1 && AutoTransHome_sta2 == 0)//landing
      {
        h_counter_sta2 = h_counter_sta2 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_sta2 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
         //if(Altitude_sonaf <= 0.6){
           Vz_Hold_sta2 = 0.0;
         }
        //h_counter_old = h_counter; //Altitude_sonaf
        if(z1_hat <= 0.18)
        //if(Altitude_sonaf <= 0.18)
        {
         endAuto_sta2 = 0;
        }
      }
////////////////////////////////////
///////////////////////////////////////////////////////////  
}
// ---------------------------------------------------------------------------------------------------------------

// Automatic Transport to Home
// ---------------------------------------------------------------------------------------------------------------
void AutomaticTransport_Home_sta2() {
   if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_sta2 == 1)
  {
    //Takeoff Checking
    if(time_auto_tranH_sta2 < 2)
    {//Check time < 5
      takeoff_tranH_sta2 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ GPS_LAT_HOME, GPS_LON_HOME
     if(time_auto_tranH_sta2 > 8 && z1_hat >= h_control && endAuto_tranH_sta2 == 1 && Status_waypoint_tranH_sta2 == 0)//waypoint1
    {
      target_LAT = waypoint3_LAT_WDefault2;
      target_LON = waypoint3_LON_WDefault2;
      Status_waypoint_tranH_sta2 = 1;
    }
    if(time_auto_tranH_sta2 > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_sta2 == 1 && Status_waypoint_tranH_sta2 == 1)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint2_LAT_WDefault2;
      target_LON = waypoint2_LON_WDefault2;
      Status_waypoint_tranH_sta2 = 2;
    }
    if(time_auto_tranH_sta2 > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_sta2 == 1 && Status_waypoint_tranH_sta2 == 2)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint1_LAT_WDefault2;
      target_LON = waypoint1_LAT_WDefault2;
      Status_waypoint_tranH_sta2 = 3;
    }
    if(time_auto_tranH_sta2 > 32 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_sta2 == 1 && Status_waypoint_tranH_sta2 == 3)//50 10 Landing and position hold mode
    {
      target_LAT = GPS_LAT_HOME;
      target_LON = GPS_LON_HOME;
      Status_waypoint_tranH_sta2 = 4;
    }
    //Home Point and Landing
    if(time_auto_tranH_sta2 > 40 && abs(error_LAT) <= 200 && abs(error_LON) <= 200 && endAuto_tranH_sta2 == 1 && Status_waypoint_tranH_sta2 == 4)//50 10 Landing and position hold mode
    {
      timeLanding_tranH_sta2++;
      if(timeLanding_tranH_sta2 >= 20)//relay 2 s Landing
      {
       takeoff_tranH_sta2 = 0;
      }
    }
  }
  //กรณีที่ไม่เข้า AutoTransHome = 1;
  else//(Mode = 3 && AutoTransHome == 1 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 1)
  {
    takeoff_tranH_sta2 = 0;
    timeLanding_tranH_sta2 = 0;
    timeOff_tranH_sta2 = 0;
    time_auto_tranH_sta2 = 0;
    h_counter_tranH_sta2 = 0.1;//0.0
    h_counter_old_tranH_sta2 = 0.1;
    Vz_Hold_tranH_sta2 = 0.0;
    Status_waypoint_tranH_sta2 = 0;
    AutoTransHome_sta2 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_tranH_sta2 < h_control && takeoff_tranH_sta2 == 1 && AutoTransHome_sta2 == 1)//take-off
      {
        endAuto_tranH_sta2 = 1;
        h_counter_tranH_sta2 = h_counter_tranH_sta2 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_tranH_sta2 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_tranH_sta2 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_tranH_sta2 == 0 && endAuto_tranH_sta2 == 1 && AutoTransHome_sta2 == 1)//landing
      {
        h_counter_tranH_sta2 = h_counter_tranH_sta2 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_tranH_sta2 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_tranH_sta2 = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto_tranH_sta2 = 0;
        }
      }
////////////////////////////////////
///////////////////////////////////////////////////////////
}
// ---------------------------------------------------------------------------------------------------------------

//Station 3
void AutomaticTransport_sta3(){
 //Altitude control and 1 waypoint navigation
  ////mode 3
  if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_sta3 == 0)
  //if(Mode == 3 && CH_THRf > MINCHECK && AutoTransHome == 0)
  {
    //Takeoff Checking
    if(time_auto_sta3 < 2)
    {//Check time < 5
      takeoff_sta3 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ waypoint1_LAT, waypoint1_LON
     if(time_auto_sta3 > 8 && z1_hat >= h_control && endAuto_sta3 == 1 && Status_waypoint_sta3 == 0)//waypoint1
    {
      target_LAT = GPS_LAT_HOME_Transport;
      target_LON = GPS_LON_HOME_Transport;
      Status_waypoint_sta3 = 1;
    }
    //Transport Point and Landing
    if(time_auto_sta3 > 16 && abs(error_LAT) <= 200 && abs(error_LON) <= 200 && endAuto_sta3 == 1 && Status_waypoint_sta3 == 1)//50 10 Landing and position hold mode
    {
      timeLanding_sta3++;
      if(timeLanding_sta3 >= 20)//relay 2 s Landing
      {
       takeoff_sta3 = 0;
      }
    }   
  }
  //กรณีที่ไม่เข้า Mode == 3
  else//(Mode == 3 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 0)
  {
    takeoff_sta3 = 0;
    timeLanding_sta3 = 0;
    timeOff_sta3 = 0;
    time_auto_sta3 = 0;
    h_counter_sta3 = 0.1;//0.0
    h_counter_old_sta3 = 0.1;
    Vz_Hold_sta3 = 0.0;
    Status_waypoint_sta3 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_sta3 < h_control && takeoff_sta3 == 1 && AutoTransHome_sta3 == 0)//take-off
      {
        endAuto_sta3 = 1;
        h_counter_sta3 = h_counter_sta3 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_sta3 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_sta3 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_sta3 == 0 && endAuto_sta3 == 1 && AutoTransHome_sta3 == 0)//landing
      {
        h_counter_sta3 = h_counter_sta3 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_sta3 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
         //if(Altitude_sonaf <= 0.6){
           Vz_Hold_sta3 = 0.0;
         }
        //h_counter_old = h_counter; //Altitude_sonaf
        if(z1_hat <= 0.18)
        //if(Altitude_sonaf <= 0.18)
        {
         endAuto_sta3 = 0;
        }
      }
////////////////////////////////////
///////////////////////////////////////////////////////////  
}
// ---------------------------------------------------------------------------------------------------------------

// Automatic Transport to Home
// ---------------------------------------------------------------------------------------------------------------
void AutomaticTransport_Home_sta3() {
   if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_sta3 == 1)
  {
    //Takeoff Checking
    if(time_auto_tranH_sta3 < 2)
    {//Check time < 5
      takeoff_tranH_sta3 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ GPS_LAT_HOME, GPS_LON_HOME
     if(time_auto_tranH_sta3 > 8 && z1_hat >= h_control && endAuto_tranH_sta3 == 1 && Status_waypoint_tranH_sta3 == 0)//waypoint1
    {
      target_LAT = GPS_LAT_HOME;
      target_LON = GPS_LON_HOME;
      Status_waypoint_tranH_sta3 = 1;
    }
    //Home Point and Landing
    if(time_auto_tranH_sta3 > 16 && abs(error_LAT) <= 200 && abs(error_LON) <= 200 && endAuto_tranH_sta3 == 1 && Status_waypoint_tranH_sta3 == 1)//50 10 Landing and position hold mode
    {
      timeLanding_tranH_sta3++;
      if(timeLanding_tranH_sta3 >= 20)//relay 2 s Landing
      {
       takeoff_tranH_sta3 = 0;
      }
    }
  }
  //กรณีที่ไม่เข้า AutoTransHome = 1;
  else//(Mode = 3 && AutoTransHome == 1 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 1)
  {
    takeoff_tranH_sta3 = 0;
    timeLanding_tranH_sta3 = 0;
    timeOff_tranH_sta3 = 0;
    time_auto_tranH_sta3 = 0;
    h_counter_tranH_sta3 = 0.1;//0.0
    h_counter_old_tranH_sta3 = 0.1;
    Vz_Hold_tranH_sta3 = 0.0;
    Status_waypoint_tranH_sta3 = 0;
    AutoTransHome_sta3 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_tranH_sta3 < h_control && takeoff_tranH_sta3 == 1 && AutoTransHome_sta3 == 1)//take-off
      {
        endAuto_tranH_sta3 = 1;
        h_counter_tranH_sta3 = h_counter_tranH_sta3 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_tranH_sta3 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_tranH_sta3 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_tranH_sta3 == 0 && endAuto_tranH_sta3 == 1 && AutoTransHome_sta3 == 1)//landing
      {
        h_counter_tranH_sta3 = h_counter_tranH_sta3 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_tranH_sta3 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_tranH_sta3 = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto_tranH_sta3 = 0;
        }
      }
////////////////////////////////////
///////////////////////////////////////////////////////////
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////
//Quad_Rotor Control 1 WayPoint
//////////////////////////////////////////////////

//Automatic Takeoff and Landing in TransportArea
// ---------------------------------------------------------------------------------------------------------------
// เปิดระบบการทำงานใน State 2 Takeoff ที่ HomePoint และ ไปที่ตำแหน่ง Waypoint 5 จุด หลังจากนั้นลงไปที่ GPS_LAT, GPS_LON
void AutomaticTransport_WPath1(){
 //Altitude control and 1 waypoint navigation
  ////mode 3
  if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_WPath1 == 0)
  //if(Mode == 3 && CH_THRf > MINCHECK && AutoTransHome_WPath1 == 0)
  {
    //Takeoff Checking
    if(time_auto_WPath1 < 2)
    {//Check time < 5
      takeoff_WPath1 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ waypoint1_LAT, waypoint1_LON
     if(time_auto_WPath1 > 8 && z1_hat >= h_control && endAuto_WPath1 == 1 && Status_waypoint_WPath1 == 0)//waypoint1
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint_WPath1 = 1;
    }
    // เคลื่อนที่ QuadRotor ไปที่ waypoint2_LAT, waypoint2_LON
     if(time_auto_WPath1 > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath1 == 1 && Status_waypoint_WPath1 == 1)//50 10 Landing and position hold mode
    {
      target_LAT = GPS_LAT_SPF_TR;
      target_LON = GPS_LON_SPF_TR;
      Status_waypoint_WPath1 = 2;
    }
    // เคลื่อนที่ QuadRotor ไปยัง Transport Point
    //Transport Point and Landing
    if(time_auto_WPath1 > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath1 == 1 && Status_waypoint_WPath1 == 2)//50 10 Landing and position hold mode
    {
      timeLanding_WPath1++;
      if(timeLanding_WPath1 >= 20)//relay 2 s Landing
      {
       takeoff_WPath1 = 0;
      }
    }   
  }
  //กรณีที่ไม่เข้า Mode == 3
  else//(Mode == 3 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 0)
  {
    takeoff_WPath1 = 0;
    timeLanding_WPath1 = 0;
    timeOff_WPath1 = 0;
    time_auto_WPath1 = 0;
    h_counter_WPath1 = 0.1;//0.0
    h_counter_old_WPath1 = 0.1;
    Vz_Hold_WPath1 = 0.0;
    Status_waypoint_WPath1 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_WPath1 < h_control && takeoff_WPath1 == 1 && AutoTransHome_WPath1 == 0)//take-off
      {
        endAuto_WPath1 = 1;
        h_counter_WPath1 = h_counter_WPath1 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_WPath1 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_WPath1 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_WPath1 == 0 && endAuto_WPath1 == 1 && AutoTransHome_WPath1 == 0)//landing
      {
        h_counter_WPath1 = h_counter_WPath1 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_WPath1 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_WPath1 = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto_WPath1 = 0;
        }
      }
///////////////////////////////////////////////////////////  
}
// ---------------------------------------------------------------------------------------------------------------


// Automatic Transport to Home
// ---------------------------------------------------------------------------------------------------------------
void AutomaticTransport_Home_WPath1() {
   if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_WPath1 == 1)
  {
    //Takeoff Checking
    if(time_auto_tranH_WPath1 < 2)
    {//Check time < 5
      takeoff_tranH_WPath1 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ GPS_LAT_HOME, GPS_LON_HOME
     if(time_auto_tranH_WPath1 > 8 && z1_hat >= h_control && endAuto_tranH_WPath1 == 1 && Status_waypoint_tranH_WPath1 == 0)//waypoint1
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint_tranH_WPath1 = 1;
    }
    if(time_auto_tranH_WPath1 > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath1 == 1 && Status_waypoint_tranH_WPath1 == 1)//waypoint5
    {
      target_LAT = GPS_LAT_SPF_HOME;
      target_LON = GPS_LON_SPF_HOME;
      Status_waypoint_tranH_WPath1 = 2;
    }
    //Home Point and Landing
    if(time_auto_tranH_WPath1 > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath1 == 1 && Status_waypoint_tranH_WPath1 == 2)//50 10 Landing and position hold mode
    {
      timeLanding_tranH_WPath1++;
      if(timeLanding_tranH_WPath1 >= 20)//relay 2 s Landing
      {
       takeoff_tranH_WPath1 = 0;
      }
    }
  }
  //กรณีที่ไม่เข้า AutoTransHome = 1;
  else//(Mode = 3 && AutoTransHome_WPath1 == 1 && CH_THR > MINCHECK && armed == 1 && AutoTransHome_WPath1 == 1)
  {
    takeoff_tranH_WPath1 = 0;
    timeLanding_tranH_WPath1 = 0;
    timeOff_tranH_WPath1 = 0;
    time_auto_tranH_WPath1 = 0;
    h_counter_tranH_WPath1 = 0.1;//0.0
    h_counter_old_tranH_WPath1 = 0.1;
    Vz_Hold_tranH_WPath1 = 0.0;
    Status_waypoint_tranH_WPath1 = 0;
    AutoTransHome_WPath1 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_tranH_WPath1 < h_control && takeoff_tranH_WPath1 == 1 && AutoTransHome_WPath1 == 1)//take-off
      {
        endAuto_tranH_WPath1 = 1;
        h_counter_tranH_WPath1 = h_counter_tranH_WPath1 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_tranH_WPath1 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_tranH_WPath1 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_tranH_WPath1 == 0 && endAuto_tranH_WPath1 == 1 && AutoTransHome_WPath1 == 1)//landing
      {
        h_counter_tranH_WPath1 = h_counter_tranH_WPath1 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_tranH_WPath1 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_tranH_WPath1 = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto_tranH_WPath1 = 0;
        }
      }
///////////////////////////////////////////////////////////
}
// ---------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////
//Quad_Rotor Control 2 WayPoint
//////////////////////////////////////////////////

//Automatic Takeoff and Landing in TransportArea
// ---------------------------------------------------------------------------------------------------------------
// เปิดระบบการทำงานใน State 2 Takeoff ที่ HomePoint และ ไปที่ตำแหน่ง Waypoint 5 จุด หลังจากนั้นลงไปที่ GPS_LAT, GPS_LON
void AutomaticTransport_WPath2(){
 //Altitude control and 1 waypoint navigation
  ////mode 3
  if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_WPath2 == 0)
  //if(Mode == 3 && CH_THRf > MINCHECK && AutoTransHome_WPath2 == 0)
  {
    //Takeoff Checking
    if(time_auto_WPath2 < 2)
    {//Check time < 5
      takeoff_WPath2 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ waypoint1_LAT, waypoint1_LON
     if(time_auto_WPath2 > 8 && z1_hat >= h_control && endAuto_WPath2 == 1 && Status_waypoint_WPath2 == 0)//waypoint1
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint_WPath2 = 1;
    }
    // เคลื่อนที่ QuadRotor ไปที่ waypoint2_LAT, waypoint2_LON
     if(time_auto_WPath2 > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath2 == 1 && Status_waypoint_WPath2 == 1)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint2_LAT;
      target_LON = waypoint2_LON;
      Status_waypoint_WPath2 = 2;
    }
   // เคลื่อนที่ QuadRotor ไปที่ waypoint3_LAT, waypoint3_LON
    if(time_auto_WPath2 > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath2 == 1 && Status_waypoint_WPath2 == 2)//50 10 Landing and position hold mode
    {
      target_LAT = GPS_LAT_SPF_TR;
      target_LON = GPS_LON_SPF_TR;
      Status_waypoint_WPath2 = 3;
    }
    // เคลื่อนที่ QuadRotor ไปยัง Transport Point
    //Transport Point and Landing
    if(time_auto_WPath2 > 32 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath2 == 1 && Status_waypoint_WPath2 == 3)//50 10 Landing and position hold mode
    {
      timeLanding_WPath2++;
      if(timeLanding_WPath2 >= 20)//relay 2 s Landing
      {
       takeoff_WPath2 = 0;
      }
    }   
  }
  //กรณีที่ไม่เข้า Mode == 3
  else//(Mode == 3 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 0)
  {
    takeoff_WPath2 = 0;
    timeLanding_WPath2 = 0;
    timeOff_WPath2 = 0;
    time_auto_WPath2 = 0;
    h_counter_WPath2 = 0.1;//0.0
    h_counter_old_WPath2 = 0.1;
    Vz_Hold_WPath2 = 0.0;
    Status_waypoint_WPath2 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_WPath2 < h_control && takeoff_WPath2 == 1 && AutoTransHome_WPath2 == 0)//take-off
      {
        endAuto_WPath2 = 1;
        h_counter_WPath2 = h_counter_WPath2 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_WPath2 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_WPath2 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_WPath2 == 0 && endAuto_WPath2 == 1 && AutoTransHome_WPath2 == 0)//landing
      {
        h_counter_WPath2 = h_counter_WPath2 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_WPath2 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_WPath2 = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto_WPath2 = 0;
        }
      }
///////////////////////////////////////////////////////////  
}
// ---------------------------------------------------------------------------------------------------------------


// Automatic Transport to Home
// ---------------------------------------------------------------------------------------------------------------
void AutomaticTransport_Home_WPath2() {
   if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_WPath2 == 1)
  {
    //Takeoff Checking
    if(time_auto_tranH_WPath2 < 2)
    {//Check time < 5
      takeoff_tranH_WPath2 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ GPS_LAT_HOME, GPS_LON_HOME
     if(time_auto_tranH_WPath2 > 8 && z1_hat >= h_control && endAuto_tranH_WPath2 == 1 && Status_waypoint_tranH_WPath2 == 0)//waypoint1
    {
      target_LAT = waypoint2_LAT;
      target_LON = waypoint2_LON;
      Status_waypoint_tranH_WPath2 = 1;
    }
    if(time_auto_tranH_WPath2 > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath2 == 1 && Status_waypoint_tranH_WPath2 == 1)//waypoint5
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint_tranH_WPath2 = 2;
    }
    if(time_auto_tranH_WPath2 > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath2 == 1 && Status_waypoint_tranH_WPath2 == 2)//waypoint5
    {
      target_LAT = GPS_LAT_SPF_HOME;
      target_LON = GPS_LON_SPF_HOME;
      Status_waypoint_tranH_WPath2 = 3;
    }
    //Home Point and Landing
    if(time_auto_tranH_WPath2 > 32 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath2 == 1 && Status_waypoint_tranH_WPath2 == 3)//50 10 Landing and position hold mode
    {
      timeLanding_tranH_WPath2++;
      if(timeLanding_tranH_WPath2 >= 20)//relay 2 s Landing
      {
       takeoff_tranH_WPath2 = 0;
      }
    }
  }
  //กรณีที่ไม่เข้า AutoTransHome = 1;
  else//(Mode = 3 && AutoTransHome == 1 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 1)
  {
    takeoff_tranH_WPath2 = 0;
    timeLanding_tranH_WPath2 = 0;
    timeOff_tranH_WPath2 = 0;
    time_auto_tranH_WPath2 = 0;
    h_counter_tranH_WPath2 = 0.1;//0.0
    h_counter_old_tranH_WPath2 = 0.1;
    Vz_Hold_tranH_WPath2 = 0.0;
    Status_waypoint_tranH_WPath2 = 0;
    AutoTransHome_WPath2 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_tranH_WPath2 < h_control && takeoff_tranH_WPath2 == 1 && AutoTransHome_WPath2 == 1)//take-off
      {
        endAuto_tranH_WPath2 = 1;
        h_counter_tranH_WPath2 = h_counter_tranH_WPath2 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_tranH_WPath2 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_tranH_WPath2 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_tranH_WPath2 == 0 && endAuto_tranH_WPath2 == 1 && AutoTransHome_WPath2 == 1)//landing
      {
        h_counter_tranH_WPath2 = h_counter_tranH_WPath2 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_tranH_WPath2 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_tranH_WPath2 = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto_tranH_WPath2 = 0;
        }
      }
///////////////////////////////////////////////////////////
}
// ---------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////
//Quad_Rotor Control 3 WayPoint
//////////////////////////////////////////////////

//Automatic Takeoff and Landing in TransportArea
// ---------------------------------------------------------------------------------------------------------------
// เปิดระบบการทำงานใน State 2 Takeoff ที่ HomePoint และ ไปที่ตำแหน่ง Waypoint 5 จุด หลังจากนั้นลงไปที่ GPS_LAT, GPS_LON
void AutomaticTransport_WPath3(){
 //Altitude control and 1 waypoint navigation
  ////mode 3
  if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_WPath3 == 0)
  //if(Mode == 3 && CH_THRf > MINCHECK && AutoTransHome_WPath3 == 0)
  {
    //Takeoff Checking
    if(time_auto_WPath3 < 2)
    {//Check time < 5
      takeoff_WPath3 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ waypoint1_LAT, waypoint1_LON
     if(time_auto_WPath3 > 8 && z1_hat >= h_control && endAuto_WPath3 == 1 && Status_waypoint_WPath3 == 0)//waypoint1
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint_WPath3 = 1;
    }
    // เคลื่อนที่ QuadRotor ไปที่ waypoint2_LAT, waypoint2_LON
     if(time_auto_WPath3 > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath3 == 1 && Status_waypoint_WPath3 == 1)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint2_LAT;
      target_LON = waypoint2_LON;
      Status_waypoint_WPath3 = 2;
    }
   // เคลื่อนที่ QuadRotor ไปที่ waypoint3_LAT, waypoint3_LON
    if(time_auto_WPath3 > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath3 == 1 && Status_waypoint_WPath3 == 2)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint3_LAT;
      target_LON = waypoint3_LON;
      Status_waypoint_WPath3 = 3;
    }
    // เคลื่อนที่ QuadRotor ไปที่ waypoint4_LAT, waypoint4_LON
    if(time_auto_WPath3 > 32 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath3 == 1 && Status_waypoint_WPath3 == 3)//50 10 Landing and position hold mode
    {
      target_LAT = GPS_LAT_SPF_TR;
      target_LON = GPS_LON_SPF_TR;
      Status_waypoint_WPath3 = 4;
    }
    // เคลื่อนที่ QuadRotor ไปยัง Transport Point
    //Transport Point and Landing
    if(time_auto_WPath3 > 40 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath3 == 1 && Status_waypoint_WPath3 == 4)//50 10 Landing and position hold mode
    {
      timeLanding_WPath3++;
      if(timeLanding_WPath3 >= 20)//relay 2 s Landing
      {
       takeoff_WPath3 = 0;
      }
    }   
  }
  //กรณีที่ไม่เข้า Mode == 3
  else//(Mode == 3 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 0)
  {
    takeoff_WPath3 = 0;
    timeLanding_WPath3 = 0;
    timeOff_WPath3 = 0;
    time_auto_WPath3 = 0;
    h_counter_WPath3 = 0.1;//0.0
    h_counter_old_WPath3 = 0.1;
    Vz_Hold_WPath3 = 0.0;
    Status_waypoint_WPath3 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_WPath3 < h_control && takeoff_WPath3 == 1 && AutoTransHome_WPath3 == 0)//take-off
      {
        endAuto_WPath3 = 1;
        h_counter_WPath3 = h_counter_WPath3 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_WPath3 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_WPath3 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_WPath3 == 0 && endAuto_WPath3 == 1 && AutoTransHome_WPath3 == 0)//landing
      {
        h_counter_WPath3 = h_counter_WPath3 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_WPath3 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_WPath3 = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto_WPath3 = 0;
        }
      }
///////////////////////////////////////////////////////////  
}
// ---------------------------------------------------------------------------------------------------------------


// Automatic Transport to Home
// ---------------------------------------------------------------------------------------------------------------
void AutomaticTransport_Home_WPath3() {
   if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_WPath3 == 1)
  {
    //Takeoff Checking
    if(time_auto_tranH_WPath3 < 2)
    {//Check time < 5
      takeoff_tranH_WPath3 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ GPS_LAT_HOME, GPS_LON_HOME
     if(time_auto_tranH_WPath3 > 8 && z1_hat >= h_control && endAuto_tranH_WPath3 == 1 && Status_waypoint_tranH_WPath3 == 0)//waypoint1
    {
      target_LAT = waypoint3_LAT;
      target_LON = waypoint3_LON;
      Status_waypoint_tranH_WPath3 = 1;
    }
    if(time_auto_tranH_WPath3 > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath3 == 1 && Status_waypoint_tranH_WPath3 == 1)//waypoint5
    {
      target_LAT = waypoint2_LAT;
      target_LON = waypoint2_LON;
      Status_waypoint_tranH_WPath3 = 2;
    }
    if(time_auto_tranH_WPath3 > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath3 == 1 && Status_waypoint_tranH_WPath3 == 2)//waypoint5
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint_tranH_WPath3 = 3;
    }
    if(time_auto_tranH_WPath3 > 32 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath3 == 1 && Status_waypoint_tranH_WPath3 == 3)//waypoint5
    {
      target_LAT = GPS_LAT_SPF_HOME;
      target_LON = GPS_LON_SPF_HOME;
      Status_waypoint_tranH_WPath3 = 4;
    }
    //Home Point and Landing
    if(time_auto_tranH_WPath3 > 40 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath3 == 1 && Status_waypoint_tranH_WPath3 == 4)//50 10 Landing and position hold mode
    {
      timeLanding_tranH_WPath3++;
      if(timeLanding_tranH_WPath3 >= 20)//relay 2 s Landing
      {
       takeoff_tranH_WPath3 = 0;
      }
    }
  }
  //กรณีที่ไม่เข้า AutoTransHome = 1;
  else//(Mode = 3 && AutoTransHome == 1 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 1)
  {
    takeoff_tranH_WPath3 = 0;
    timeLanding_tranH_WPath3 = 0;
    timeOff_tranH_WPath3 = 0;
    time_auto_tranH_WPath3 = 0;
    h_counter_tranH_WPath3 = 0.1;//0.0
    h_counter_old_tranH_WPath3 = 0.1;
    Vz_Hold_tranH_WPath3 = 0.0;
    Status_waypoint_tranH_WPath3 = 0;
    AutoTransHome_WPath3 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_tranH_WPath3 < h_control && takeoff_tranH_WPath3 == 1 && AutoTransHome_WPath3 == 1)//take-off
      {
        endAuto_tranH_WPath3 = 1;
        h_counter_tranH_WPath3 = h_counter_tranH_WPath3 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_tranH_WPath3 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_tranH_WPath3 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_tranH_WPath3 == 0 && endAuto_tranH_WPath3 == 1 && AutoTransHome_WPath3 == 1)//landing
      {
        h_counter_tranH_WPath3 = h_counter_tranH_WPath3 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_tranH_WPath3 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_tranH_WPath3 = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto_tranH_WPath3 = 0;
        }
      }
///////////////////////////////////////////////////////////
}
// ---------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////
//Quad_Rotor Control 4 WayPoint
//////////////////////////////////////////////////

//Automatic Takeoff and Landing in TransportArea
// ---------------------------------------------------------------------------------------------------------------
// เปิดระบบการทำงานใน State 2 Takeoff ที่ HomePoint และ ไปที่ตำแหน่ง Waypoint 5 จุด หลังจากนั้นลงไปที่ GPS_LAT, GPS_LON
void AutomaticTransport_WPath4(){
 //Altitude control and 1 waypoint navigation
  ////mode 3
  if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_WPath4 == 0)
  //if(Mode == 3 && CH_THRf > MINCHECK && AutoTransHome_WPath4 == 0)
  {
    //Takeoff Checking
    if(time_auto_WPath4 < 2)
    {//Check time < 5
      takeoff_WPath4 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ waypoint1_LAT, waypoint1_LON
     if(time_auto_WPath4 > 8 && z1_hat >= h_control && endAuto_WPath4 == 1 && Status_waypoint_WPath4 == 0)//waypoint1
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint_WPath4 = 1;
    }
    // เคลื่อนที่ QuadRotor ไปที่ waypoint2_LAT, waypoint2_LON
     if(time_auto_WPath4 > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath4 == 1 && Status_waypoint_WPath4 == 1)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint2_LAT;
      target_LON = waypoint2_LON;
      Status_waypoint_WPath4 = 2;
    }
   // เคลื่อนที่ QuadRotor ไปที่ waypoint3_LAT, waypoint3_LON
    if(time_auto_WPath4 > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath4 == 1 && Status_waypoint_WPath4 == 2)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint3_LAT;
      target_LON = waypoint3_LON;
      Status_waypoint_WPath4 = 3;
    }
    // เคลื่อนที่ QuadRotor ไปที่ waypoint4_LAT, waypoint4_LON
    if(time_auto_WPath4 > 32 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath4 == 1 && Status_waypoint_WPath4 == 3)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint4_LAT;
      target_LON = waypoint4_LON;
      Status_waypoint_WPath4 = 4;
    }
    // เคลื่อนที่ QuadRotor ไปที่ waypoint5_LAT, waypoint5_LON
    if(time_auto_WPath4 > 40 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath4 == 1 && Status_waypoint_WPath4 == 4)//50 10 Landing and position hold mode
    {
      target_LAT = GPS_LAT_SPF_TR;
      target_LON = GPS_LON_SPF_TR;
      Status_waypoint_WPath4 = 5;
    }
    // เคลื่อนที่ QuadRotor ไปยัง Transport Point
    //Transport Point and Landing
    if(time_auto_WPath4 > 48 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath4 == 1 && Status_waypoint_WPath4 == 5)//50 10 Landing and position hold mode
    {
      timeLanding_WPath4++;
      if(timeLanding_WPath4 >= 20)//relay 2 s Landing
      {
       takeoff_WPath4 = 0;
      }
    }   
  }
  //กรณีที่ไม่เข้า Mode == 3
  else//(Mode == 3 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 0)
  {
    takeoff_WPath4 = 0;
    timeLanding_WPath4 = 0;
    timeOff_WPath4 = 0;
    time_auto_WPath4 = 0;
    h_counter_WPath4 = 0.1;//0.0
    h_counter_old_WPath4 = 0.1;
    Vz_Hold_WPath4 = 0.0;
    Status_waypoint_WPath4 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_WPath4 < h_control && takeoff_WPath4 == 1 && AutoTransHome_WPath4 == 0)//take-off
      {
        endAuto_WPath4 = 1;
        h_counter_WPath4 = h_counter_WPath4 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_WPath4 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_WPath4 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_WPath4 == 0 && endAuto_WPath4 == 1 && AutoTransHome_WPath4 == 0)//landing
      {
        h_counter_WPath4 = h_counter_WPath4 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_WPath4 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_WPath4 = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto_WPath4 = 0;
        }
      }
///////////////////////////////////////////////////////////  
}
// ---------------------------------------------------------------------------------------------------------------


// Automatic Transport to Home
// ---------------------------------------------------------------------------------------------------------------
void AutomaticTransport_Home_WPath4() {
   if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_WPath4 == 1)
  {
    //Takeoff Checking
    if(time_auto_tranH_WPath4 < 2)
    {//Check time < 5
      takeoff_tranH_WPath4 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ GPS_LAT_HOME, GPS_LON_HOME
     if(time_auto_tranH_WPath4 > 8 && z1_hat >= h_control && endAuto_tranH_WPath4 == 1 && Status_waypoint_tranH_WPath4 == 0)//waypoint1
    {
      target_LAT = waypoint4_LAT;
      target_LON = waypoint4_LON;
      Status_waypoint_tranH_WPath4 = 1;
    }
    if(time_auto_tranH_WPath4 > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath4 == 1 && Status_waypoint_tranH_WPath4 == 1)//waypoint5
    {
      target_LAT = waypoint3_LAT;
      target_LON = waypoint3_LON;
      Status_waypoint_tranH_WPath4 = 2;
    }
    if(time_auto_tranH_WPath4 > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath4 == 1 && Status_waypoint_tranH_WPath4 == 2)//waypoint5
    {
      target_LAT = waypoint2_LAT;
      target_LON = waypoint2_LON;
      Status_waypoint_tranH_WPath4 = 3;
    }
    if(time_auto_tranH_WPath4 > 32 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath4 == 1 && Status_waypoint_tranH_WPath4 == 3)//waypoint5
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint_tranH_WPath4 = 4;
    }
    if(time_auto_tranH_WPath4 > 40 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath4 == 1 && Status_waypoint_tranH_WPath4 == 4)//waypoint5
    {
      target_LAT = GPS_LAT_SPF_HOME;
      target_LON = GPS_LON_SPF_HOME;
      Status_waypoint_tranH_WPath4 = 5;
    }
    //Home Point and Landing
    if(time_auto_tranH_WPath4 > 48 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath4 == 1 && Status_waypoint_tranH_WPath4 == 5)//50 10 Landing and position hold mode
    {
      timeLanding_tranH_WPath4++;
      if(timeLanding_tranH_WPath4 >= 20)//relay 2 s Landing
      {
       takeoff_tranH_WPath4 = 0;
      }
    }
  }
  //กรณีที่ไม่เข้า AutoTransHome = 1;
  else//(Mode = 3 && AutoTransHome == 1 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 1)
  {
    takeoff_tranH_WPath4 = 0;
    timeLanding_tranH_WPath4 = 0;
    timeOff_tranH_WPath4 = 0;
    time_auto_tranH_WPath4 = 0;
    h_counter_tranH_WPath4 = 0.1;//0.0
    h_counter_old_tranH_WPath4 = 0.1;
    Vz_Hold_tranH_WPath4 = 0.0;
    Status_waypoint_tranH_WPath4 = 0;
    AutoTransHome_WPath4 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_tranH_WPath4 < h_control && takeoff_tranH_WPath4 == 1 && AutoTransHome_WPath4 == 1)//take-off
      {
        endAuto_tranH_WPath4 = 1;
        h_counter_tranH_WPath4 = h_counter_tranH_WPath4 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_tranH_WPath4 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_tranH_WPath4 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_tranH_WPath4 == 0 && endAuto_tranH_WPath4 == 1 && AutoTransHome_WPath4 == 1)//landing
      {
        h_counter_tranH_WPath4 = h_counter_tranH_WPath4 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_tranH_WPath4 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_tranH_WPath4 = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto_tranH_WPath4 = 0;
        }
      }
///////////////////////////////////////////////////////////
}
// ---------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////
//Quad_Rotor Control 5 WayPoint
//////////////////////////////////////////////////

//Automatic Takeoff and Landing in TransportArea
// ---------------------------------------------------------------------------------------------------------------
// เปิดระบบการทำงานใน State 2 Takeoff ที่ HomePoint และ ไปที่ตำแหน่ง Waypoint 5 จุด หลังจากนั้นลงไปที่ GPS_LAT, GPS_LON
void AutomaticTransport_WPath5(){
 //Altitude control and 1 waypoint navigation
  ////mode 3
  if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_WPath5 == 0)
  //if(Mode == 3 && CH_THRf > MINCHECK && AutoTransHome_WPath5 == 0)
  {
    //Takeoff Checking
    if(time_auto_WPath5 < 2)
    {//Check time < 5
      takeoff_WPath5 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ waypoint1_LAT, waypoint1_LON
     if(time_auto_WPath5 > 8 && z1_hat >= h_control && endAuto_WPath5 == 1 && Status_waypoint_WPath5 == 0)//waypoint1
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint_WPath5 = 1;
    }
    // เคลื่อนที่ QuadRotor ไปที่ waypoint2_LAT, waypoint2_LON
     if(time_auto_WPath5 > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath5 == 1 && Status_waypoint_WPath5 == 1)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint2_LAT;
      target_LON = waypoint2_LON;
      Status_waypoint_WPath5 = 2;
    }
   // เคลื่อนที่ QuadRotor ไปที่ waypoint3_LAT, waypoint3_LON
    if(time_auto_WPath5 > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath5 == 1 && Status_waypoint_WPath5 == 2)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint3_LAT;
      target_LON = waypoint3_LON;
      Status_waypoint_WPath5 = 3;
    }
    // เคลื่อนที่ QuadRotor ไปที่ waypoint4_LAT, waypoint4_LON
    if(time_auto_WPath5 > 32 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath5 == 1 && Status_waypoint_WPath5 == 3)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint4_LAT;
      target_LON = waypoint4_LON;
      Status_waypoint_WPath5 = 4;
    }
    // เคลื่อนที่ QuadRotor ไปที่ waypoint5_LAT, waypoint5_LON
    if(time_auto_WPath5 > 40 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath5 == 1 && Status_waypoint_WPath5 == 4)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint5_LAT;
      target_LON = waypoint5_LON;
      Status_waypoint_WPath5 = 5;
    }
    // เคลื่อนที่ QuadRotor ไปยัง Transport Point
    if(time_auto_WPath5 > 48 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath5 == 1 && Status_waypoint_WPath5 == 5)//50 10 Landing and position hold mode
    {
      target_LAT = GPS_LAT_SPF_TR;
      target_LON = GPS_LON_SPF_TR;
      Status_waypoint_WPath5 = 6;
    }
    //Transport Point and Landing
    if(time_auto_WPath5 > 56 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath5 == 1 && Status_waypoint_WPath5 == 6)//50 10 Landing and position hold mode
    {
      timeLanding_WPath5++;
      if(timeLanding_WPath5 >= 20)//relay 2 s Landing
      {
       takeoff_WPath5 = 0;
      }
    }   
  }
  //กรณีที่ไม่เข้า Mode == 3
  else//(Mode == 3 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 0)
  {
    takeoff_WPath5 = 0;
    timeLanding_WPath5 = 0;
    timeOff_WPath5 = 0;
    time_auto_WPath5 = 0;
    h_counter_WPath5 = 0.1;//0.0
    h_counter_old_WPath5 = 0.1;
    Vz_Hold_WPath5 = 0.0;
    Status_waypoint_WPath5 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_WPath5 < h_control && takeoff_WPath5 == 1 && AutoTransHome_WPath5 == 0)//take-off
      {
        endAuto_WPath5 = 1;
        h_counter_WPath5 = h_counter_WPath5 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_WPath5 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_WPath5 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_WPath5 == 0 && endAuto_WPath5 == 1 && AutoTransHome_WPath5 == 0)//landing
      {
        h_counter_WPath5 = h_counter_WPath5 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_WPath5 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_WPath5 = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto_WPath5 = 0;
        }
      }
///////////////////////////////////////////////////////////  
}
// ---------------------------------------------------------------------------------------------------------------


// Automatic Transport to Home
// ---------------------------------------------------------------------------------------------------------------
void AutomaticTransport_Home_WPath5() {
   if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_WPath5 == 1)
  {
    //Takeoff Checking
    if(time_auto_tranH_WPath5 < 2)
    {//Check time < 5
      takeoff_tranH_WPath5 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ GPS_LAT_HOME, GPS_LON_HOME
     if(time_auto_tranH_WPath5 > 8 && z1_hat >= h_control && endAuto_tranH_WPath5 == 1 && Status_waypoint_tranH_WPath5 == 0)//waypoint1
    {
      target_LAT = waypoint5_LAT;
      target_LON = waypoint5_LON;
      Status_waypoint_tranH_WPath5 = 1;
    }
    if(time_auto_tranH_WPath5 > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath5 == 1 && Status_waypoint_tranH_WPath5 == 1)//waypoint5
    {
      target_LAT = waypoint4_LAT;
      target_LON = waypoint4_LON;
      Status_waypoint_tranH_WPath5 = 2;
    }
    if(time_auto_tranH_WPath5 > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath5 == 1 && Status_waypoint_tranH_WPath5 == 2)//waypoint5
    {
      target_LAT = waypoint3_LAT;
      target_LON = waypoint3_LON;
      Status_waypoint_tranH_WPath5 = 3;
    }
    if(time_auto_tranH_WPath5 > 32 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath5 == 1 && Status_waypoint_tranH_WPath5 == 3)//waypoint5
    {
      target_LAT = waypoint2_LAT;
      target_LON = waypoint2_LON;
      Status_waypoint_tranH_WPath5 = 4;
    }
    if(time_auto_tranH_WPath5 > 40 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath5 == 1 && Status_waypoint_tranH_WPath5 == 4)//waypoint5
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint_tranH_WPath5 = 5;
    }
    if(time_auto_tranH_WPath5 > 48 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath5 == 1 && Status_waypoint_tranH_WPath5 == 5)//waypoint5
    {
      target_LAT = GPS_LAT_SPF_HOME;
      target_LON = GPS_LON_SPF_HOME;
      Status_waypoint_tranH_WPath5 = 6;
    }
    //Home Point and Landing
    if(time_auto_tranH_WPath5 > 56 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath5 == 1 && Status_waypoint_tranH_WPath5 == 6)//50 10 Landing and position hold mode
    {
      timeLanding_tranH_WPath5++;
      if(timeLanding_tranH_WPath5 >= 20)//relay 2 s Landing
      {
       takeoff_tranH_WPath5 = 0;
      }
    }
  }
  //กรณีที่ไม่เข้า AutoTransHome = 1;
  else//(Mode = 3 && AutoTransHome == 1 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 1)
  {
    takeoff_tranH_WPath5 = 0;
    timeLanding_tranH_WPath5 = 0;
    timeOff_tranH_WPath5 = 0;
    time_auto_tranH_WPath5 = 0;
    h_counter_tranH_WPath5 = 0.1;//0.0
    h_counter_old_tranH_WPath5 = 0.1;
    Vz_Hold_tranH_WPath5 = 0.0;
    Status_waypoint_tranH_WPath5 = 0;
    AutoTransHome_WPath5 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_tranH_WPath5 < h_control && takeoff_tranH_WPath5 == 1 && AutoTransHome_WPath5 == 1)//take-off
      {
        endAuto_tranH_WPath5 = 1;
        h_counter_tranH_WPath5 = h_counter_tranH_WPath5 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_tranH_WPath5 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_tranH_WPath5 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_tranH_WPath5 == 0 && endAuto_tranH_WPath5 == 1 && AutoTransHome_WPath5 == 1)//landing
      {
        h_counter_tranH_WPath5 = h_counter_tranH_WPath5 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_tranH_WPath5 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_tranH_WPath5 = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto_tranH_WPath5 = 0;
        }
      }
///////////////////////////////////////////////////////////
}
// ---------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////
//Quad_Rotor Control 6 WayPoint
//////////////////////////////////////////////////

//Automatic Takeoff and Landing in TransportArea
// ---------------------------------------------------------------------------------------------------------------
// เปิดระบบการทำงานใน State 2 Takeoff ที่ HomePoint และ ไปที่ตำแหน่ง Waypoint 6 จุด หลังจากนั้นลงไปที่ GPS_LAT, GPS_LON
void AutomaticTransport_WPath6(){
 //Altitude control and 1 waypoint navigation
  ////mode 3
  if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_WPath6 == 0)
  //if(Mode == 3 && CH_THRf > MINCHECK && AutoTransHome_WPath6 == 0)
  {
    //Takeoff Checking
    if(time_auto_WPath6 < 2)
    {//Check time < 5
      takeoff_WPath6 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ waypoint1_LAT, waypoint1_LON
     if(time_auto_WPath6 > 8 && z1_hat >= h_control && endAuto_WPath6 == 1 && Status_waypoint_WPath6 == 0)//waypoint1
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint_WPath6 = 1;
    }
    // เคลื่อนที่ QuadRotor ไปที่ waypoint2_LAT, waypoint2_LON
     if(time_auto_WPath6 > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath6 == 1 && Status_waypoint_WPath6 == 1)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint2_LAT;
      target_LON = waypoint2_LON;
      Status_waypoint_WPath6 = 2;
    }
   // เคลื่อนที่ QuadRotor ไปที่ waypoint3_LAT, waypoint3_LON
     if(time_auto_WPath6 > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath6 == 1 && Status_waypoint_WPath6 == 2)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint3_LAT;
      target_LON = waypoint3_LON;
      Status_waypoint_WPath6 = 3;
    }
    // เคลื่อนที่ QuadRotor ไปที่ waypoint4_LAT, waypoint4_LON
     if(time_auto_WPath6 > 32 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath6 == 1 && Status_waypoint_WPath6 == 3)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint4_LAT;
      target_LON = waypoint4_LON;
      Status_waypoint_WPath6 = 4;
    }
    // เคลื่อนที่ QuadRotor ไปที่ waypoint5_LAT, waypoint5_LON
    if(time_auto_WPath6 > 40 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath6 == 1 && Status_waypoint_WPath6 == 4)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint5_LAT;
      target_LON = waypoint5_LON;
      Status_waypoint_WPath6 = 5;
    }
    // เคลื่อนที่ QuadRotor ไปที่ waypoint6_LAT, waypoint6_LON
    if(time_auto_WPath6 > 48 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath6 == 1 && Status_waypoint_WPath6 == 5)//50 10 Landing and position hold mode
    {
      target_LAT = waypoint6_LAT;
      target_LON = waypoint6_LON;
      Status_waypoint_WPath6 = 6;
    }
    // เคลื่อนที่ QuadRotor ไปยัง Transport Point
    if(time_auto_WPath6 > 56 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath6 == 1 && Status_waypoint_WPath6 == 6)//50 10 Landing and position hold mode
    {
      target_LAT = GPS_LAT_SPF_TR;
      target_LON = GPS_LON_SPF_TR;
      Status_waypoint_WPath6 = 7;
    }
    //Transport Point and Landing
    if(time_auto_WPath6 > 64 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_WPath6 == 1 && Status_waypoint_WPath6 == 7)//50 10 Landing and position hold mode
    {
      timeLanding_WPath6++;
      if(timeLanding_WPath6 >= 20)//relay 2 s Landing
      {
       takeoff_WPath6 = 0;
      }
    }   
  }
  //กรณีที่ไม่เข้า Mode == 3
  else//(Mode == 3 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 0)
  {
    takeoff_WPath6 = 0;
    timeLanding_WPath6 = 0;
    timeOff_WPath6 = 0;
    time_auto_WPath6 = 0;
    h_counter_WPath6 = 0.1;//0.0
    h_counter_old_WPath6 = 0.1;
    Vz_Hold_WPath6 = 0.0;
    Status_waypoint_WPath6 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_WPath6 < h_control && takeoff_WPath6 == 1 && AutoTransHome_WPath6 == 0)//take-off
      {
        endAuto_WPath6 = 1;
        h_counter_WPath6 = h_counter_WPath6 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_WPath6 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_WPath6 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_WPath6 == 0 && endAuto_WPath6 == 1 && AutoTransHome_WPath6 == 0)//landing
      {
        h_counter_WPath6 = h_counter_WPath6 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_WPath6 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_WPath6 = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto_WPath6 = 0;
        }
      }
///////////////////////////////////////////////////////////  
}
// ---------------------------------------------------------------------------------------------------------------


// Automatic Transport to Home
// ---------------------------------------------------------------------------------------------------------------
void AutomaticTransport_Home_WPath6() {
   if(Mode == 3 && CH_THRf > MINCHECK && armed == 1 && AutoTransHome_WPath6 == 1)
  {
    //Takeoff Checking
    if(time_auto_tranH_WPath6 < 2)
    {//Check time < 5
      takeoff_tranH_WPath6 = 1;
    }
    // Wait z1_hat > h_control และเคลื่อนที่ QuadRotor ไปที่ GPS_LAT_HOME, GPS_LON_HOME
     if(time_auto_tranH_WPath6 > 8 && z1_hat >= h_control && endAuto_tranH_WPath6 == 1 && Status_waypoint_tranH_WPath6 == 0)//waypoint1
    {
      target_LAT = waypoint6_LAT;
      target_LON = waypoint6_LON;
      Status_waypoint_tranH_WPath6 = 1;
    }
    if(time_auto_tranH_WPath6 > 16 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath6 == 1 && Status_waypoint_tranH_WPath6 == 1)//waypoint5
    {
      target_LAT = waypoint5_LAT;
      target_LON = waypoint5_LON;
      Status_waypoint_tranH_WPath6 = 2;
    }
    if(time_auto_tranH_WPath6 > 24 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath6 == 1 && Status_waypoint_tranH_WPath6 == 2)//waypoint5
    {
      target_LAT = waypoint4_LAT;
      target_LON = waypoint4_LON;
      Status_waypoint_tranH_WPath6 = 3;
    }
    if(time_auto_tranH_WPath6 > 32 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath6 == 1 && Status_waypoint_tranH_WPath6 == 3)//waypoint5
    {
      target_LAT = waypoint3_LAT;
      target_LON = waypoint3_LON;
      Status_waypoint_tranH_WPath6 = 4;
    }
    if(time_auto_tranH_WPath6 > 40 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath6 == 1 && Status_waypoint_tranH_WPath6 == 4)//waypoint5
    {
      target_LAT = waypoint2_LAT;
      target_LON = waypoint2_LON;
      Status_waypoint_tranH_WPath6 = 5;
    }
    if(time_auto_tranH_WPath6 > 48 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath6 == 1 && Status_waypoint_tranH_WPath6 == 5)//waypoint5
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint_tranH_WPath6 = 6;
    }
    if(time_auto_tranH_WPath6 > 56 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath6 == 1 && Status_waypoint_tranH_WPath6 == 6)//waypoint5
    {
      target_LAT = GPS_LAT_SPF_HOME;
      target_LON = GPS_LON_SPF_HOME;
      Status_waypoint_tranH_WPath6 = 7;
    }
    //Home Point and Landing
    if(time_auto_tranH_WPath6 > 64 && abs(error_LAT) <= 150 && abs(error_LON) <= 150 && endAuto_tranH_WPath6 == 1 && Status_waypoint_tranH_WPath6 == 7)//50 10 Landing and position hold mode
    {
      timeLanding_tranH_WPath6++;
      if(timeLanding_tranH_WPath6 >= 20)//relay 2 s Landing
      {
       takeoff_tranH_WPath6 = 0;
      }
    }
  }
  //กรณีที่ไม่เข้า AutoTransHome = 1;
  else//(Mode = 3 && AutoTransHome == 1 && CH_THR > MINCHECK && armed == 1 && AutoTransHome == 1)
  {
    takeoff_tranH_WPath6 = 0;
    timeLanding_tranH_WPath6 = 0;
    timeOff_tranH_WPath6 = 0;
    time_auto_tranH_WPath6 = 0;
    h_counter_tranH_WPath6 = 0.1;//0.0
    h_counter_old_tranH_WPath6 = 0.1;
    Vz_Hold_tranH_WPath6 = 0.0;
    Status_waypoint_tranH_WPath6 = 0;
    AutoTransHome_WPath6 = 0;
  } 
// Takeoff and Landing Level 2
//////////////////////////////////////////////////////////////////// 
      //Function Counter การ Takeoff
      if(h_counter_tranH_WPath6 < h_control && takeoff_tranH_WPath6 == 1 && AutoTransHome_WPath6 == 1)//take-off
      {
        endAuto_tranH_WPath6 = 1;
        h_counter_tranH_WPath6 = h_counter_tranH_WPath6 + 0.082;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold_tranH_WPath6 = 0.82;//(m/s) roop 10 Hz
        //h_counter_old = h_counter;
      }
      else{
        Vz_Hold_tranH_WPath6 = 0.0;
      }
 ////////////////
      //Function Counter การ Landing
      if(takeoff_tranH_WPath6 == 0 && endAuto_tranH_WPath6 == 1 && AutoTransHome_WPath6 == 1)//landing
      {
        h_counter_tranH_WPath6 = h_counter_tranH_WPath6 - 0.059;//0.023 ramp input hz  landing
        Vz_Hold_tranH_WPath6 = -0.59;//(m/s) roop 10 Hz
         if(z1_hat <= 0.6){
           Vz_Hold_tranH_WPath6 = 0.0;
         }
        //h_counter_old = h_counter;
        if(z1_hat <= 0.18)
        {
         endAuto_tranH_WPath6 = 0;
        }
      }
///////////////////////////////////////////////////////////
}
// ---------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
