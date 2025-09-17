#include "Arduino.h"
#include "Ext_Var.h"

//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(14, 15); // RX, TX
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(38, 39); // RX, TX

void PID_RUN();
Ticker PID_EXE(PID_RUN, 400, 0);

void three_min();
Ticker three_min_tick(three_min, 180000, 1);

void three_min()
{ 
  if(oil_temp_flag==1)
  {
      once_step=1;
      once_flag_for_min=1;
//      mySerial.println("1 min completed................................................ in ticker");
  }

//   if(flask_temp_flag==1)
//   {
//       once_step_flask=1;
//   }
}
//------------------------------------------------------PID------------------------------------------------------------------------------
double PID_SETPOINT, PID_INPUT, PID_OUTPUT,PID_SETPOINT_final;
double PID_SETPOINT_FLASK, PID_INPUT_FLASK, PID_OUTPUT_FLASK,PID_SETPOINT_FLASK_final;

//**************24 August 2nd reading***************************
double Kp=28, Ki=0.015, Kd=35, Ki_saved;        
double consKp = 18, consKi = 0.0025, consKd = 60;  

double Kp_FLASK=15, Ki_FLASK=0.05, Kd_FLASK=35; 
double consKp_FLASK = 13, consKi_FLASK = 0.02, consKd_FLASK = 30; //fWindowSize2 = 1000  

//**************18 August 2nd reading***************************
//double Kp=30, Ki=0.01, Kd=40, Ki_saved;        
//double consKp = 20, consKi = 0.002, consKd = 80;  
//
//double Kp_FLASK=10, Ki_FLASK=0.05, Kd_FLASK=35; 
//double consKp_FLASK = 15, consKi_FLASK = 0.015, consKd_FLASK = 30; //fWindowSize2 = 1000  

 //**************18 August*************************** // 5 degree overshoot & 6 degree undershoot
//double Kp=30, Ki=0.01, Kd=40, Ki_saved;        
//double consKp = 20, consKi = 0.002, consKd = 80;  
//
//double Kp_FLASK=10, Ki_FLASK=0.05, Kd_FLASK=35; 
//double consKp_FLASK = 10, consKi_FLASK = 0.007, consKd_FLASK = 55; //fWindowSize2 = 1500 

// //**************14 August*************************** // 6 deg undershoot & overshoot happen with 2 point calibration
//double Kp=30, Ki=0.01, Kd=40, Ki_saved;        
//double consKp = 20, consKi = 0.002, consKd = 80;  
//
//double Kp_FLASK=20, Ki_FLASK=0.03, Kd_FLASK=35; 
//double consKp_FLASK = 15, consKi_FLASK = 0.002, consKd_FLASK = 75; //fWindowSize2 = 1500 

//**************12 August*************************** // 5 degree overshoot for 70 degree after 3 hr
//double Kp=27, Ki=0.01, Kd=50, Ki_saved;        
//double consKp = 22, consKi = 0.002, consKd = 90;  
//
//double Kp_FLASK=25, Ki_FLASK=0.03, Kd_FLASK=35; 
//double consKp_FLASK = 20, consKi_FLASK = 0.004, consKd_FLASK = 85; //fWindowSize2 = 1400 

//**************1 August***************************
//double Kp=25, Ki=0.01, Kd=40, Ki_saved;        
//double consKp = 20, consKi = 0.002, consKd = 80;  
//
//double Kp_FLASK=23, Ki_FLASK=0.03, Kd_FLASK=35; 
//double consKp_FLASK = 15, consKi_FLASK = 0.004, consKd_FLASK = 75; //fWindowSize2 = 1400

//double Kp_FLASK=23, Ki_FLASK=0.03, Kd_FLASK=35; 
//double consKp_FLASK = 13, consKi_FLASK = 0.002, consKd_FLASK = 80; //fWindowSize2 = 1500
//**************31july***************************
//double Kp_FLASK=20, Ki_FLASK=0.01, Kd_FLASK=40; 
//double consKp_FLASK = 13, consKi_FLASK = 0.002, consKd_FLASK = 80; //fWindowSize2 = 1800

/******************22 May ***Windowsize = 500***************/
//double Kp=2, Ki=0.0014, Kd=3000, Ki_saved;        //--SP=70, ambient = 35 Reach time to reach 70: 1 hr, oversboot 3 degree i.e. 73 degree within 1 hr,30 min
//double Kp_FLASK=2, Ki_FLASK=0.0014, Kd_FLASK=3000, Ki_saved;

/******************19 May ***Windowsize = 500***************/
//double Kp=2, Ki=0.0015, Kd=2500, Ki_saved;  
//double Kp=3, Ki=0.0014, Kd=3000, Ki_saved;       //--SP=70, ambient = 36 Reach time to reach 70: 1 hr, oversboot 3 degree i.e. 73 degree within 1 hr,30 min
                                   
/******************18 May ***Windowsize = 500***************/
//double Kp=3, Ki=0.0013, Kd=2500, Ki_saved;       //--SP=70, ambient = 36 Reach time to reach 70: 1 hr, oversboot 3 degree i.e. 73 degree within 1 hr,30 min

/******************17 May ***Windowsize = 500***************/
//double Kp=5, Ki=0.0015, Kd=2000, Ki_saved;       //--SP=70, ambient = 41 Reach time to reach 70: 45 min, oversboot 6 degree i.e. 76 degree within 1 hr, 15 min

/******************16 May *** Windowsize = 1000***************/
 //double Kp=10, Ki=0.0025, Kd=900, Ki_saved;      //--SP=70, ambient = 36 Reach time to reach 70: 25 min, oversboot 15 degree i.e. 85degree within 1 hr

/******************15 May *** Windowsize = 1000***************/
 
//double Kp=10, Ki=0.008, Kd=700,  Ki_saved;  //10 degree ontroller
//double Kp=15, Ki=0.0010, Kd=500, Ki_saved;  //7 degree ontroller
//double Kp=20, Ki=0.0015, Kd=250, Ki_saved;
//double Kp=35, Ki=0.0025, Kd=100, Ki_saved;
//double consKp = 150, consKi = 0.5, consKd = 400;     // for internal probe values p 200
//------------------------------------------------------PID------------------------------------------------------------------------------
//int WINDOW_SIZE = 1000;
int WINDOW_SIZE = 800;
unsigned long WINDOW_START_TIME;


PID myPID(&PID_INPUT, &PID_OUTPUT, &PID_SETPOINT, Kp, Ki, Kd, DIRECT);
PID1 myPID_FLASK(&PID_INPUT_FLASK, &PID_OUTPUT_FLASK, &PID_SETPOINT_FLASK, Kp_FLASK, Ki_FLASK, Kd_FLASK, DIRECT);
//------------------------------------------------------------------------------------------------------------------------------------

bool flask_temp_flag=0;
bool oil_temp_flag=1; 
bool once_swap=0;
float ext_probe_current_temp=0;
int Probe_Swap_value=0;
int fWindowSize2 = 1000,fWindowSize = 200;
unsigned long fwindowStartTime,fwindowStartTime2;
bool flask_temp_flag_setup=1;
extern float TEMP_DEG_ktype=0;
bool once_step=1;
bool once_step_flask=1;
bool Setpoint_end_flag=0;
bool once_flag_for_min=1;

Lclass::Lclass()
{}

void Lclass:: Setup()
{
//  mySerial.begin(9600);
  
  myPID.SetOutputLimits(0, WINDOW_SIZE);
  WINDOW_START_TIME = millis();
  myPID.SetMode(AUTOMATIC);
  myPID_FLASK.SetMode1(AUTOMATIC);
}

void PID_RUN()
{
  if(oil_temp_flag==1)
  {
//      mySerial.println("PID_RUN-----1-----oil_temp_flag");
   myPID.Compute(); 
  }
   if(flask_temp_flag==1)
  { 
//    mySerial.println("PID_RUN-----2-----flask_temp_flag");
    myPID_FLASK.Compute1(); 
  }
}

void Lclass ::PID_callingg()
{
  if(LONG_PRESS)  
  { PID_INPUT = TEMP_DEG_RAW_ktype; 
//   mySerial.println("1st stastement "); 
//   mySerial.print(TEMP_DEG_RAW_ktype);
  }
  else
  {
      if(oil_temp_flag==1)
      { 
          PID_INPUT = TEMP_DEG_ktype; 
          PID_SETPOINT_final=(int(0.1*SET_TEMP_DEG)+SET_TEMP_DEG);
//  mySerial.print("PID_INPUT:  ---------- "); mySerial.println(PID_INPUT);
// mySerial.print("PID_SETPOINT_final:  ---------- "); mySerial.println(PID_SETPOINT_final);
          if(once_step==1)
          {  
            if(SET_TEMP_DEG<=80) 
                PID_SETPOINT=TEMP_DEG_ktype+15;
            else if (SET_TEMP_DEG >80 && SET_TEMP_DEG <=110)
                PID_SETPOINT=TEMP_DEG_ktype+22;
            else if (SET_TEMP_DEG >110 && SET_TEMP_DEG <=140)
                PID_SETPOINT=TEMP_DEG_ktype+27;
            else if (SET_TEMP_DEG >140 && SET_TEMP_DEG <=180)
                PID_SETPOINT=TEMP_DEG_ktype+32;
            else
                PID_SETPOINT=TEMP_DEG_ktype+52;
//            PID_SETPOINT=TEMP_DEG_ktype+15;  //23 aug change.....75 min taken to reach 70 degree 5 deg undershoot & overshoo
//          PID_SETPOINT=TEMP_DEG_ktype+12;  //22 aug change.....100 min taken to reach 70 degree
//          PID_SETPOINT=TEMP_DEG_ktype+5;             //21 aug .....135 min taken to reach 70 degree 3 deg undershoot & overshoot
//                mySerial.println(PID_SETPOINT);
                once_step=0;
          }
          if(TEMP_DEG_ktype>PID_SETPOINT-2 && TEMP_DEG_ktype<PID_SETPOINT+2 && TEMP_DEG_ktype<=PID_SETPOINT_final && once_flag_for_min)
          {
//             mySerial.println("-----------------------inside hold for 3min loop-------------"); 
//            for(int tim=0;tim>=1000;tim++)
//            {
//              delay(100);
//            }
               three_min_tick.start();  
              once_flag_for_min=0;
//            PID_SETPOINT=TEMP_DEG_ktype+5;
          }
    
         
//          PID_SETPOINT=(int(0.1*SET_TEMP_DEG)+SET_TEMP_DEG);
      } 
     
      if(flask_temp_flag==1)
      { 


         PID_INPUT_FLASK = TEMP_DEG;//temperature_read;....external probe
         PID_SETPOINT_FLASK=SET_TEMP_DEG;
        
//        PID_INPUT_FLASK = TEMP_DEG;//temperature_read;....external probe
//        PID_SETPOINT_FLASK_final=SET_TEMP_DEG;
//        if(once_step_flask==1)
//          { 
//                PID_SETPOINT_FLASK=TEMP_DEG+2;
//                once_step_flask=0;
//          }
//          if(TEMP_DEG>=PID_SETPOINT_FLASK_final-3 )
//          {
//            Setpoint_end_flag=1;
//            PID_SETPOINT_FLASK=SET_TEMP_DEG;
//            
//          }
//          if(TEMP_DEG>=PID_SETPOINT_FLASK-2 && TEMP_DEG<=PID_SETPOINT_FLASK+2 && Setpoint_end_flag==0) //TEMP_DEG<=PID_SETPOINT_FLASK_final-3
//          { 
//              three_min_tick.start();  
////            for(int tim=0;tim>=1000;tim++)
////            {
////              delay(100);
////            }
////            once_step_flask=1;
////            PID_SETPOINT=TEMP_DEG_ktype+5;
//          }
//          
          

      

        
//        mySerial.print("TEMP_DEG_ktype:  ----------flask "); mySerial.println(TEMP_DEG_ktype);
//        mySerial.print("TEMP_DEG_RAW_ktype:  ----------flask "); mySerial.println(TEMP_DEG_RAW_ktype);
//       mySerial.print("TEMP_DEG:  ----------flask "); mySerial.println(TEMP_DEG);
      }

 Probe_Swap_value=0.07*SET_TEMP_DEG;
 
// Probe_Swap_value=0.1*SET_TEMP_DEG;
 
  ext_probe_current_temp=TEMP_DEG;   //....external probe
  
  if(((SET_TEMP_DEG-ext_probe_current_temp)<=Probe_Swap_value) && once_swap==0)
  {
    flask_temp_flag=1;
    oil_temp_flag=0; 
    once_swap=1;
  }
 }
  
  if(TEMP_DEG_ktype<=260)     //TEMP_DEG_RAW_Internal <= 260)
  { 
   if(oil_temp_flag==1)
   {  
      if((PID_SETPOINT - PID_INPUT) > 9)
      {
        myPID.SetTunings(Kp, Ki, Kd);
      }                    
  else
     {
     myPID.SetTunings(consKp, consKi, consKd); 
     
      }
    
 
    myPID.SetOutputLimits(0,WINDOW_SIZE-500);
  

  if (millis() - WINDOW_START_TIME > WINDOW_SIZE)
  { 
    WINDOW_START_TIME += WINDOW_SIZE;
  }
  
    if (PID_OUTPUT< millis() - WINDOW_START_TIME) 
   digitalWrite(HEATER, LOW); // LOW);
    else 
    {
        if(TEMP_DEG_ktype >= (PID_SETPOINT + 0.1))  //---internal probe
          digitalWrite(HEATER, LOW); //LOW);
        else
        {
          digitalWrite(HEATER, HIGH); //HIGH);
        }
    }
    } 
    if(flask_temp_flag==1)
   {   
      if(flask_temp_flag_setup==1)
      {
        fwindowStartTime = millis();
        fwindowStartTime2 = millis();
         myPID_FLASK.SetOutputLimits1(0, fWindowSize2-400);
        flask_temp_flag_setup=0;
      }


      if((PID_SETPOINT_FLASK - PID_INPUT_FLASK) > 6)   //8
      {
        myPID_FLASK.SetTunings1(Kp_FLASK, Ki_FLASK, Kd_FLASK); 
      }
      else
      {
        myPID_FLASK.SetTunings1(consKp_FLASK, consKi_FLASK, consKd_FLASK); 
      }

    myPID_FLASK.SetOutputLimits1(0, fWindowSize2-500);
 

  if (millis() - fwindowStartTime2 > fWindowSize2)
  {
    fwindowStartTime2 += fWindowSize2;
  }
  
    if (PID_OUTPUT_FLASK < millis() - fwindowStartTime2) 
      digitalWrite(HEATER, LOW);// LOW);
    else 
    {
        if(TEMP_DEG >= (PID_SETPOINT_FLASK + 0.1))
         digitalWrite(HEATER, LOW); //LOW);
        else
        {
         digitalWrite(HEATER, HIGH); //HIGH);
        }
      }
    }
  }
   else digitalWrite(HEATER, LOW);
}






//************************************************************************//
//  if(!LONG_PRESS)    PID_INPUT = TEMP_DEG;
//  else               PID_INPUT = TEMP_DEG_RAW;
//  
//  if((PID_SETPOINT - PID_INPUT) > 5)   
//     myPID.SetTunings(Kp, Ki, Kd);                    
//  else
//     myPID.SetTunings(consKp, consKi, consKd); 
     
//  if (millis() - WINDOW_START_TIME > WINDOW_SIZE)
//          WINDOW_START_TIME += WINDOW_SIZE;
//
//
//          
// 
//  if (PID_OUTPUT < millis() - WINDOW_START_TIME)      digitalWrite(HEATER, LOW);
//  else 
//  {
//    if(PID_INPUT >= (float(PID_SETPOINT) + 0.1))      digitalWrite(HEATER, LOW);
//    else                                              digitalWrite(HEATER, HIGH);
//  }
//}

void Lclass :: PID_Start()
{
   PID_EXE.start(); 
}

void Lclass :: PID_Stop()
{
   PID_EXE.stop();  
}

void Lclass :: PID_Tick_Update()
{
  PID_EXE.update();
  three_min_tick.update();
}

Lclass Pid = Lclass();
