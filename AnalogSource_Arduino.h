

//int AD0_V = 0;
int AD1_L = 0;
int AD2_R = 0;
int AD3_F = 0;
int AD4_B = 0;

//int AD0_SUM = 0;
int AD1_SUM = 0;
int AD2_SUM = 0;
int AD3_SUM = 0;
int AD4_SUM = 0;

int Distance_L = 0;
int Distance_R = 0;
int Distance_F = 0;
int Distance_B = 0;
//float V_Bat = 0;

int Distance_X = 0;
int Distance_Y = 0;

uint8_t ADCSamples;
// increase this if we need more than 5 analog sources
//#define MAX_PIN_SOURCES 5

void Distance_Measuring_Sensor(){
  //AD0_V = analogRead(A0);
  AD1_L = analogRead(A1);
  AD2_R = analogRead(A2);
  AD3_F = analogRead(A4);
  AD4_B = analogRead(A3);
}
void analogReadSUM(){
    Distance_Measuring_Sensor();
    //AD0_SUM += AD0_V;
    AD1_SUM += AD1_L; 
    AD2_SUM += AD2_R;
    AD3_SUM += AD3_F;
    AD4_SUM += AD4_B;  
    ADCSamples++;
}
void Distance_Measuring_Get()
{
    // Calculate average
    //V_Bat = (AD0_SUM / ADCSamples)*0.01409;//867 = 0.01409 ,889 = 0.01374;
    Distance_L = (AD1_SUM / ADCSamples);
    Distance_R = (AD2_SUM / ADCSamples);
    Distance_F = (AD3_SUM / ADCSamples);     
    Distance_B = (AD4_SUM / ADCSamples);

  applyDeadband(Distance_L, 160);//190 = 1 m
  applyDeadband(Distance_R, 160);
  applyDeadband(Distance_F, 160);
  applyDeadband(Distance_B, 160);
  Distance_Y = Distance_L - Distance_R;
  Distance_X = Distance_B - Distance_F;
  
    // Reset SUM variables
    //AD0_SUM = 0;
    AD1_SUM = 0;
    AD2_SUM = 0;
    AD3_SUM = 0;
    AD4_SUM = 0;
    ADCSamples = 0;   
}
