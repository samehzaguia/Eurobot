#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(11,12);
#include "math.h"
#define address 0x80
#define ticks_per_cm 159
#define Speed 8000
#define Accel 6000
#define Decel 6000
#define TX_pin 18
#define RX_pin 19
//#define ticks_per_revolution 12160
#define ticks_per_revolution 13300
RoboClaw roboclaw(&serial,10000);
char next_state='T';
bool done=false;
bool allow_load_data=true;
bool allow_turn=true;
bool allow_straight=false;

bool direction_inverted=false;

uint8_t number_of_cups=0;
double goblet_x[]= {10,10,40,40,40,40,51,51,80,80,108,108,120,120,120,120,165,165,165,165,195.5,195.5,195.5,195.5};
double goblet_y[]= {67,233,30,95,205,270,45,255,110,190,45,255,30,127,173,270,106.5,133.5,166.5,193.5,100.5,139.5,160.5,199.5};

uint8_t position_number=0;
uint8_t number_of_positions=0;
//double x_goal[] = {80,40,108,108};
//double y_goal[] = {190,205,205,260};
double x_goal[] = {141,147,176.5,141,108,108};
double y_goal[] = {130,130,130,130,205,260};
double x_Goal =0;
double y_Goal =0;
double my_phi_goal=0;

double my_angle_goal=0;
double my_distance_goal=0;

uint32_t right_ticks=0;
uint32_t left_ticks=0;
uint32_t previous_right_ticks=0;
uint32_t previous_left_ticks=0;
uint32_t delta_right_ticks=0;
uint32_t delta_left_ticks=0;
uint32_t odometry_previous_time=0;
uint32_t monitor_previous_time=0;
double right_distance=0;
double left_distance=0;
double central_distance=0;
double x=93;
double y=279.5;
double phi=-PI/(double)2;
uint32_t my_angle_ticks=0;
uint32_t my_distance_ticks=0;
char turn_direction='N';
uint8_t M1_Buffer=0;//M1_number_of_unachieved_instruction=0;
uint8_t M2_Buffer=0;//M2_number_of_unachieved_instruction=0;
uint8_t M1_Buffer_old=0;//M1_number_of_unachieved_instruction_old=0;
uint8_t M2_Buffer_old=0;//M2_number_of_unachieved_instruction_old=0;
int RX=0;
//**********************************************************my functions********************************************************************//

void odometry (void);
uint32_t distance_to_ticks(double distance);
uint32_t angle_to_ticks(double angle);
double get_phi_goal(double Xg,double Yg);
double get_distance_goal(double Xg,double Yg);
void get_ticks(void);
double get_angle_goal(double phi_goal);

//**********************************************************my functions********************************************************************//
void setup() {
  pinMode(TX_pin,OUTPUT);
  pinMode(RX_pin,INPUT);  
  Serial.begin(57600);
  roboclaw.begin(38400);
  roboclaw.ResetEncoders(address);
  number_of_positions=sizeof(x_goal)/sizeof(double); // calculer le nombre des positions
  number_of_cups=sizeof(goblet_x)/sizeof(double); // calculer le nombre des goblets
  }
void get_ticks(void)
{
  uint8_t status1,status2;
  bool valid1,valid2;
  right_ticks = roboclaw.ReadEncM1(address, &status1, &valid1);
  left_ticks = roboclaw.ReadEncM2(address, &status2, &valid2);
  }
  
uint32_t distance_to_ticks(double distance){
  return(distance*ticks_per_cm);
}
uint32_t angle_to_ticks(double angle){
  double ticks_number=(angle/(double)(2*PI))*ticks_per_revolution;
  return(ticks_number);
}
double get_phi_goal(double Xg,double Yg)
{
    return(atan2((Yg-y),(Xg-x)));
}
double get_distance_goal(double Xg,double Yg){
  return(sqrt(sq(Xg-x)+sq(Yg-y)));
}

double get_angle_goal(double phi_goal){
  double angle=0;
  if (((x_Goal-x)>=0)&&((y_Goal-y)>=0)){
    if (phi>=0){
      if (phi_goal>=phi){
      turn_direction='L';
      angle=phi_goal-phi;
      }
      else if (phi_goal<=phi){
      turn_direction='R';
      angle=phi-phi_goal;
      }
    }
    else if (phi<=0){
      if(phi_goal-phi<=PI){
       turn_direction='L';
      angle=phi_goal-phi;       
      }
      else if(phi_goal-phi>=PI){
       turn_direction='R';
      angle=2*PI-(phi_goal-phi);          
      }
    }
  }
  else if (((x_Goal-x)>=0)&&((y_Goal-y)<=0)){
    if (phi>=0){
      if (phi-phi_goal<=PI){
      turn_direction='R';
      angle=phi-phi_goal;
      }
      else if (phi-phi_goal>=PI){
      turn_direction='L';
      angle=2*PI-(phi-phi_goal);
      }
    }
    else if (phi<=0){
      if(phi>=phi_goal){
       turn_direction='R';
      angle=phi-phi_goal;       
      }
      else if(phi<=phi_goal){
       turn_direction='L';
      angle=phi_goal-phi;          
      }
    }
  }
   else if (((x_Goal-x)<=0)&&((y_Goal-y)>=0)){
    if (phi<=0){
      if (phi_goal-phi<=PI){
      turn_direction='L';
      angle=phi_goal-phi;
      }
      else if (phi_goal-phi>=PI){
      turn_direction='R';
      angle=2*PI-(phi_goal-phi);
      }
    }
    else if (phi>=0){
      if(phi_goal>=phi){
       turn_direction='L';
      angle=phi_goal-phi;       
      }
      else if(phi_goal<=phi){
       turn_direction='R';
      angle=phi-phi_goal;          
      }
    }
  }
   else if (((x_Goal-x)<=0)&&((y_Goal-y)<=0)){
    if (phi<=0){
      if (phi>=phi_goal){
      turn_direction='R';
      angle=phi-phi_goal;
      }
      else if (phi<=phi_goal){
      turn_direction='L';
      angle=phi_goal-phi;
      }
    }
    else if (phi>=0){
      if(phi-phi_goal<=PI){
       turn_direction='R';
      angle=phi-phi_goal;       
      }
      else if(phi-phi_goal>=PI){
       turn_direction='L';
      angle=2*PI-(phi-phi_goal);          
      }
    }
  }
  return(angle);
}
/******************************************************************my loop**************************************************/
/******************************************************************my loop**************************************************/
/******************************************************************my loop**************************************************/

void loop() {
  
  if(allow_load_data){
    if((position_number==3)||(position_number==4))
    {
      if(phi>=0)
      {
        phi-=PI;
      }
      else
      {
        phi+=PI;
      }
      direction_inverted=!direction_inverted;
    }
    if((position_number==1)||(position_number==2)||(position_number==3)||(position_number==6))
    {
       //communication avec STM32
      digitalWrite(TX_pin,HIGH);
      while(!RX)
      {
        RX=digitalRead(RX_pin);
      }
      RX=0;
      digitalWrite(TX_pin,LOW);
      //communication avec STM32 
    }
    if(position_number<number_of_positions){
      x_Goal=x_goal[position_number];
      y_Goal=y_goal[position_number];
      my_phi_goal=get_phi_goal(x_Goal,y_Goal);
      for(int i=0; i<number_of_cups;i++)
      {
        if ((goblet_x[i]==x_Goal)&&(goblet_y[i]==y_Goal)){
          x_Goal=x_Goal-16*cos(my_phi_goal);
          y_Goal=y_Goal-16*sin(my_phi_goal);
        }
      }       
      my_distance_goal=get_distance_goal(x_Goal,y_Goal);
      my_angle_goal=get_angle_goal(my_phi_goal);
      my_distance_ticks= distance_to_ticks(my_distance_goal);
      my_angle_ticks= angle_to_ticks(my_angle_goal);
      allow_load_data=false;        
    }
    else{
      allow_load_data=false;
      allow_turn=false;
      allow_straight=false;
      done=true;
      //5edma zeyda
      for(int i=0;i<2;i++){
        my_distance_ticks=ticks_per_cm*15;
        roboclaw.SpeedAccelDeccelPositionM1M2(address,Accel,Speed,Decel,-my_distance_ticks,Accel,Speed,Decel,-my_distance_ticks,1);
        roboclaw.ReadBuffers(address,M1_Buffer,M2_Buffer);
        while(!((M1_Buffer==128)&& (M1_Buffer==128))){
          roboclaw.ReadBuffers(address,M1_Buffer,M2_Buffer);
        }
        roboclaw.ResetEncoders(address);
        delay(200);
        digitalWrite(TX_pin,HIGH);
        while(!RX)
        {
          RX=digitalRead(RX_pin);
        }
        RX=0;
        digitalWrite(TX_pin,LOW);
      }
       //5edma zeyda
    }
  }
  if(allow_turn){
    if(turn_direction=='R'){
      roboclaw.SpeedAccelDeccelPositionM1M2(address,Accel,Speed,Decel,-my_angle_ticks,Accel,Speed,Decel,my_angle_ticks,1);
    }
    else if(turn_direction=='L'){
      roboclaw.SpeedAccelDeccelPositionM1M2(address,Accel,Speed,Decel,my_angle_ticks,Accel,Speed,Decel,-my_angle_ticks,1);
    }
    allow_turn=false;
    delay(10);        
  }
  else if(allow_straight){
    if(direction_inverted)
    {
      roboclaw.SpeedAccelDeccelPositionM1M2(address,Accel,Speed,Decel,-my_distance_ticks,Accel,Speed,Decel,-my_distance_ticks,1);
    }
    else
    {
      roboclaw.SpeedAccelDeccelPositionM1M2(address,Accel,Speed,Decel,my_distance_ticks,Accel,Speed,Decel,my_distance_ticks,1);
    }
    allow_straight=false;
    delay(10);
  }
  
  roboclaw.ReadBuffers(address,M1_Buffer,M2_Buffer);
  if((M1_Buffer==128)&& (M1_Buffer==128) && (!done)){
    if(next_state=='T') // turn
    {
      next_state='S';
      allow_straight=true;
    }
    else if(next_state=='S')  // straight
    {
      position_number++;
      /*mise a jour des coordonnées*/
      x=x_Goal;
      y=y_Goal;
      phi=my_phi_goal;
      /*mise a jour des coordonnées*/
      allow_load_data=true;
      next_state='T';
      allow_turn=true;
    }
    roboclaw.ResetEncoders(address);
    delay(500);
  }
}
