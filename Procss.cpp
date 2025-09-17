#include "Ext_var.h"
bool RPM_vary_flagg=0;
//
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(38, 39); // RX, TX

//bool COUNTDOWN_START=0;
Pclass::Pclass()
{}

int Prv_RPM=0;
bool RPM_assign_flag=0;
//bool f1=1;
//bool Once_RPM_prv=1;

void RPM_Read();
Ticker rpm_r(RPM_Read, 1000, 0);

void RPM_vary_Inprocess();
Ticker RPM_vary(RPM_vary_Inprocess, 5000, 1);
void RPM_vary_Inprocess()
{
  RPM_vary_flagg=0;
  Counter_Display_Flag=0;
//  f1=1;
   
} 

void RPM_Read()
{
  set_value = myval;
}

//void Alternate_Display();
//Ticker Alter_Disp(Alternate_Display, 2000, 0);

void Temperature_Serial();
Ticker Temp_Serial(Temperature_Serial, 1500, 0);

//**************************************************************LONGPRESS*************************************************************************
float ERROR_CAL, CAL2_CALCULATED_LOW_CAL_VALUE, CAL2_CALCULATED_HIGH_CAL_VALUE ;
float CAL2_RAW_LOW_VALUE = 0.00, CAL2_RAW_HIGH_VALUE = 0.00, RAW_LOW_VALUE = 0.00, RAW_HIGH_VALUE = 0.00, MASTER_LOW = 0.00, MASTER_HIGH = 0.00;
float PID_SETPOINT_To_Show;
bool RST_BUTT = 0, TIME_SOCK_OR_TOTAL = 0;
bool CAL2_START = 0;
bool BLINK_STATS = 0, EXECUTE_PID_LOOP = 0;
double  ERROR_CAL1;
long CAL2_RAW_LOW_VALUE_CONV, CAL2_RAW_HIGH_VALUE_CONV, MASTER_LOW_CONV, MASTER_HIGH_CONV;
bool STIR_VALUE_SET = 0, DEGREE_VALUE_SET, START_CAL_VALUE = 0;
int  SAFE_VALUE_SET_DEG = 0, SAFE_VALUE_SET_FAH = 0, ERROR_VALUE_SET = 0;
int  LOW_CAL_VALUE_SET = 30, HIGH_CAL_VALUE_SET = 0, CAL_SELECT_VALUE = 0;
int  CAL2_LOW1_VALUE_SET = 0, CAL2_LOW2_VALUE_SET = 0, CAL2_HIGH1_VALUE_SET = 0, CAL2_HIGH2_VALUE_SET = 0;
int  CAL1_VALUE_SAVED_DEG1 = 0, CAL1_VALUE_SAVED_DEG2 = 0, CAL1_VALUE_SAVED_FAH = 0;
int  SELECTION_PARAMETER = 1;
//********************************************************SINGLEPRESS***************************************
float SET_TEMP_DEG = 0.00;
int SET_TEMP_FAH = 0, SET_TEMP = 0, SET_HOUR = 0, SET_MIN = 0, SET_MINUTE = 0;
int i = 0, j = 0,  REMAIN_MIN = 0;
int STOP_PROCESS = 0;
double DISP_INC;
//********************************************************LEVEL_DETECTION***************************************
bool CHECK_RATE_OF_HEATING = 0;
float TEMP_VALUE_INITIAL, TEMP_VALUE_FINAL ;
unsigned long START_COUNT_VALUE, STOP_COUNT_VALUE ;
float RATE_OF_HEATING = 0;
//-----------------------------------------------------------------------------------------------------------------------------------
//int INTERATION=0, TOLERANCE_VALUE=0;
//unsigned int FOR_INTERATION=0;
int VALUE_FOR_BLINK = 25, TOTAL_VALUE_FOR_BLINK = 50;
float PREVIOUS_TEMP_COOLING_RATE = 0.00, CURRENT_TEMP_COOLING_RATE = 0.00;
float COOLING_RATE = 0.00, COOLOING_RATE_COMP_VAL = 0.04;
unsigned long  PREVIOUS_COOLING_RATE_TIME = 0 , CURRENT_COOLING_RATE_TIME = 0, CAL_OF_10_MIN = 0, TO_CHECK_REPETATION_WITHIN_TIME = 0;
int COOLING_CYCLE_COUNT = 0;
bool FILL_BATH_ERROR_NOT_CLEAR = 0;
bool FOR_BUZZ = 0;
bool Process_Stop;
bool Process_Interrupt_flag;
//------------------------------------------------------------------------------------------------------------------------------------
int MIN_SET_TEMP = 0, MIN_SET_FAH = 0;
bool IN_ERROR = 0, REPETATION_FLAG = 0;
unsigned short  TIMER_LED_BLINK = 0;
uint8_t Blinking_Freq = 0;
//------------------------------------------------------------------------------------------------------------------------------
int z = 0;
int  currentMillis = 0;
//bool Alternate_Display_Flag = 0;
unsigned int Counter_Display_Flag = 0;
bool Inprocess_click=0;

int set_value = 0, val = 0, val_final = 0, RPM = 0;
//float TEMP_DEG_ktype=0;


void Pclass:: Setup()
{
//  mySerial.begin(9600);

  pinMode(BUZZER, OUTPUT);
  pinMode(HEATER, OUTPUT);
  pinMode(PUMP, OUTPUT);

  digitalWrite(PUMP, LOW ); // LOW
  digitalWrite(BUZZER, LOW);
  digitalWrite(HEATER, LOW);

  pinMode(HEATER_STATUS_LED, OUTPUT);
  pinMode(CURRENT_PARA_LED, OUTPUT);
  pinMode(SET_PARA_LED, OUTPUT);
  pinMode(ALARM_LED, OUTPUT);
//  pinMode(A4, INPUT);  
//  attachInterrupt(A4, RPM_vary_func, CHANGE);  
  

  ALL_LED_OFF();
}

void Pclass :: SET_SHORT_PRESS_FUNCTION()
{
  //    set_value= analogRead(analog_pot);
  switch (ENTER_BUT)
  {
    case 0 :                                              //------------------------------Home Page
      DOT1_1 =  DOT1_0 = DOT1_2 = DOT2_2 = DOT2_0 = 0;
      REP = REP1 = IN_ERROR = FOR_REP = FOR_REP2 = 0;
      PUMP_FLAG = CAL2_START = Blinking_Freq = 0;
      PID_SETPOINT = STOP_PROCESS = EXECUTE_PID_LOOP = RATE_OF_HEATING = 0;
      Mode_Button_Single_Press_In_Settings = REPETATION_FLAG = 0;
      CHECK_RATE_OF_HEATING = 1;
      Motor_On_Off_disp_flag=0;
//       STIR_VALUE_SET=0;

      oil_temp_flag = 1;
      flask_temp_flag = 0;
      Probe_Swap_value = 0;
      flask_temp_flag_setup = 1;
      once_swap = 0;

      once_step=1;
      once_flag_for_min=1;
      once_step_flask=1;
      Setpoint_end_flag=0;
      PID_SETPOINT_final=0;
      PID_SETPOINT_FLASK_final=0;

      Tick.BUZZ_TICK_STOP();
      digitalWrite(PUMP, LOW ); //LOW
      Tick.LED_Off_Call();
      digitalWrite(HEATER, LOW);
      Pid.PID_Stop();
      ALL_LED_OFF();
      if (Process_Interrupt_flag && Proceess_Resume_Enable_Flag)
        //------------------------------Process auto resume if auto process enable
      {
        if (z < 3)                                       //------------------------------flash for 3 times
        {
          if (i < VALUE_FOR_BLINK + 50)                  //------------------------------display blank
          {
            digitalWrite(BUZZER, HIGH);
            led.DISPLAY_DIGIT1(24, 24, 24, 24) ;
            led.DISPLAY_DIGIT2(24, 24, 24, 24) ;
          }
          else                                           //------------------------------display auto Pros
          {
            digitalWrite(BUZZER, LOW);
            led.DISPLAY_DIGIT2(11, 17, 23, 12);
            led.DISPLAY_DIGIT1(12, 16, 17, 16);
          }
          if (i > TOTAL_VALUE_FOR_BLINK + 100) {
            z ++;
            i = 0;
          }
        }
        else  ENTER_BUT = 5;                           //------------------------------Process directly start
        digitalWrite(TIMER_LED1, LOW);
      }
      else
      {
//        if (!DEGREE_VALUE_SET)          //------------------------------display temperature in degree
//        {
          //                if(TEMP_DEG>99.9)  {
          led.BREAK_NUMBER2(TEMP_DEG * 10);  DOT2_2 = 1;
          //                  }
          //                else               { led.BREAK_NUMBER2(TEMP_DEG*100); DOT2_1=1; }
          //                led.BREAK_NUMBER2(TEMP_DEG*10);
          digits2[3] = 10;           //------------------------------display C
//        }
//        else                           //------------------------------display temperature in Fehranite
//        {
//          led.BREAK_NUMBER2(TEMP_FAH * 10);
//          digits2[3] = 21;           //------------------------------display F
//          DOT2_1 = 0;
//        }
        led.BREAK_NUMBER1(0000);
        digitalWrite(TIMER_LED1, HIGH);
      }

      break;
    case 1 :                                              //------------------------------set temperature
      DOT1_1 = 0;
      DOT2_1 = 0;
      digitalWrite(SET_PARA_LED, LOW);                //------------------------------glow set LED
      digitalWrite(TIMER_LED1, HIGH);
      if (i < VALUE_FOR_BLINK)                        //------------------------------display 3 digits blank while editing
      {
        if (!BLINK_STATS) {
          digits2[0] = 24;
          digits2[1] = 24;
          digits2[2] = 24;
          DOT2_1 = 0;
        }
        else i = VALUE_FOR_BLINK;
      }
      else
      {
        DOT2_1 = 0;
        DOT2_2 = 1;
        led.BREAK_NUMBER2((SET_VALUE[ENTER_BUT] * 10)); // + (SET_VALUE[ENTER_BUT +1]*10));
        //------------------------------display set temperature
      }
      if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
      digits2[3] = 10;
      led.BREAK_NUMBER1((SET_VALUE[ENTER_BUT + 2] * 100) + SET_VALUE[ENTER_BUT + 3]);
      //------------------------------set time
      break;
    case 2 :
      led.BREAK_NUMBER1((SET_VALUE[ENTER_BUT + 1] * 100) + SET_VALUE[ENTER_BUT + 2]);
      //------------------------------display set temperature
      if (!DEGREE_VALUE_SET)
      {
        led.BREAK_NUMBER2((SET_VALUE[1] * 100) + (SET_VALUE[ENTER_BUT] * 10));
        digits2[3] = 10;
        DOT2_1 = 1;
        DOT2_2 = 1;
        if (i < VALUE_FOR_BLINK)
        {
          if (!BLINK_STATS) digits2[2] = 24;
          else i = VALUE_FOR_BLINK;
        }
      }
      else
      {
        led.BREAK_NUMBER2(SET_VALUE[22] * 10);
        digits2[3] = 21;
        DOT2_1 = 0;
        if (i < VALUE_FOR_BLINK)
        {
          if (!BLINK_STATS  ) {
            digits2[0] = 24;
            digits2[1] = 24;
            digits2[2] = 24;
          }
          else i = VALUE_FOR_BLINK;
        }
      }
      if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
      break;
    case 3 :
      if (!DEGREE_VALUE_SET)
      {
        led.BREAK_NUMBER2((SET_VALUE[1] * 10)) ; //0) + (SET_VALUE[2]*10));
        digits2[3] = 10;
        DOT2_1 = 0;
        DOT2_2 = 1;
      }
      else
      {
        led.BREAK_NUMBER2(SET_VALUE[22] * 10);
        digits2[3] = 21;
        DOT2_1 = 0;
      }
      if (i < VALUE_FOR_BLINK)
      {
        if (!BLINK_STATS)    {
          digits1[0] = 24;
          digits1[1] = 24;
          DOT1_1 = 0;
        }
        else i = VALUE_FOR_BLINK;
      }
      else
      {
        DOT1_1 = 0;
        led.BREAK_NUMBER1((SET_VALUE[ENTER_BUT] * 100) + SET_VALUE[ENTER_BUT + 1]);
      }
      if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
      break;
    case 4 :
      DOT1_1 = 0;
      DOT2_2 = 1;
      if (!DEGREE_VALUE_SET)
      {
        led.BREAK_NUMBER2((SET_VALUE[1] * 10)) ; //0) + (SET_VALUE[2]*10));
        digits2[3] = 10;
        DOT2_1 = 0;
      }
      else
      {
        led.BREAK_NUMBER2(SET_VALUE[22] * 10);
        digits2[3] = 21;
        DOT2_1 = 0;
      }
      led.BREAK_NUMBER1( (SET_VALUE[ENTER_BUT - 1] * 100) + SET_VALUE[ENTER_BUT] );
      if (i < VALUE_FOR_BLINK)
      {
        if (!BLINK_STATS)    {
          digits1[2] = 24;
          digits1[3] = 24;
        }
        else i = VALUE_FOR_BLINK;
      }
      if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
      REP1 = 0;
      PUMP_FLAG = 1;
      SET_MIN = (SET_VALUE[3] * 60) + SET_VALUE[4];
      break;
    case 5:
      //          set_value= analogRead(analog_pot);
      SET_TEMP_DEG = (SET_VALUE[1]) ; // + (SET_VALUE[2] / 10.0));
      SET_TEMP_FAH =  SET_VALUE[22];
      SET_HOUR = SET_VALUE[3];
      SET_MINUTE = SET_VALUE[4];
      SET_MIN = (SET_HOUR * 60) + SET_MINUTE;

      Process_Interrupt_flag = 1;
      Eeprom.Process_Set_Values_Store_On_Eeprom();
      Eeprom.Process_Time_Set_On_Eeprom();
      //          if(!FILL_BATH_ERROR_NOT_CLEAR)
      //          {
      //
      //            if( (!DEGREE_VALUE_SET &&  SET_TEMP_DEG >= (TEMP_DEG + 10)) || (DEGREE_VALUE_SET && SET_TEMP_FAH>= (TEMP_FAH +15)) ) { FILL_BATH_ERROR_NOT_CLEAR=1; }
      //            else                                                                                                                 { FILL_BATH_ERROR_NOT_CLEAR=1; }
      //          }
      //          else if(FILL_BATH_ERROR_NOT_CLEAR)
      //          {
      //            RATE_OF_HEATING =0;
      //            CHECK_RATE_OF_HEATING=1; //1
      //            FILL_BATH_ERROR_NOT_CLEAR=0;
      //          }
      //          else
      //          {
      //            CHECK_RATE_OF_HEATING=0;
      //            digitalWrite(HEATER, HIGH);
      //            TEMP_VALUE_INITIAL = TEMP_DEG_RAW;
      //            START_COUNT_VALUE = millis();
      //          }
      //          if(!DEGREE_VALUE_SET)
      //          {
      PID_SETPOINT_To_Show = SET_TEMP_DEG;
      PID_SETPOINT = SET_TEMP_DEG;
      //          }
      //          else
      //          {
      //            PID_SETPOINT_To_Show = SET_TEMP_FAH;
      //            PID_SETPOINT = (PID_SETPOINT_To_Show - 32) / 1.8;    //-------------------------------------------convert setpoint in degree
      //          }
     // Alter_Disp.start();
      Counter_Display_Flag = 0;
      Temp_Serial.start();
      rpm_r.start();
      EXECUTE_PID_LOOP = 0;
      WINDOW_START_TIME = millis();
      while (SET_MIN > 0  )
      { 

       if(!ProbeDetectFlag)      
       { 
        Butn.Probe_Error(); 
       }
 
        Inprocess_click=1;
        rpm_r.update();
        RPM_vary.update();
        Pid.PID_Tick_Update();
        Temp_Serial.update();
        digitalWrite(SET_PARA_LED, HIGH);
        digitalWrite(HEATER_STATUS_LED, LOW);
        Temp.RTD_TEMP_CONV();
        led.DISP_UPDATE();
        //            Eeprom.Save_Pump_Data();
        if (STIR_VALUE_SET) digitalWrite(PUMP, HIGH);
        else               digitalWrite(PUMP, LOW);
        

          //*********************************************19 july 23**************************************

   //    with strirrer rod rpm readings
     if(myval>=460)
         digitalWrite(PUMP, LOW); // for above 1000 rmp, motor should be off

        val_final = map(set_value, 0, 688, 0, 1560);

        if (val_final<70)   //100
        {
          RPM = 0;
        }
        if (val_final >70 && val_final <= 300 ) // -20
        {
          RPM = val_final + 85;
        }

        else if (val_final >300 && val_final <= 500 ) //-20
        {
          RPM = val_final + 55
          ;
        }

        else if (val_final > 500 && val_final <= 900) //-55
        {
          //RPM = val_final + 35;
           RPM = val_final+20 ;
        }
        else if (val_final > 900 && val_final <= 1100)
        {
          RPM = val_final -20;
        }
        else if (1100 < val_final && val_final <= 1500)
        {
          RPM = val_final -25; //
        }
        else if( val_final>1501)
        {
          RPM = val_final; //
        }
        //RPM=val;
//        mySerial.print("val_final: ");
//        mySerial.println(val_final);
//        mySerial.print("RPM: ");
//        mySerial.println(RPM); 
//         mySerial.print("myval: ");
//        mySerial.println(myval); 
        //**********************************8 Aug 23**********************************************************

        if( ((RPM-Prv_RPM)>=50||(Prv_RPM-RPM)>=50)&&myval<460)  //RPM!=Prv_RPM  //&& f1==1
        {
          RPM_vary.start();
          RPM_vary_flagg=1;
          Counter_Display_Flag=2;
          RPM_assign_flag=1;
        }
        
        if(RPM_assign_flag==1)
        {
          Prv_RPM=RPM;
          RPM_assign_flag=0;
        }
        //*********************************************19 july 23**************************************

  /*     without strirrer rod

        val_final = map(set_value, 0, 688, 0, 1560);

        if (val_final<70)
        {
          RPM = 0;
        }
        if (val_final >70 && val_final <= 300 ) //
        {
          RPM = val_final + 105;
        }

        else if (val_final >300 && val_final <= 500 ) //
        {
          RPM = val_final + 95
          ;
        }

        else if (val_final > 500 && val_final <= 900)
        {
          RPM = val_final + 75;
        }
        else if (val_final > 900 && val_final <= 1100)
        {
          RPM = val_final + 60;
        }
        else if (1100 < val_final && val_final <= 1500)
        {
          RPM = val_final + 50; //
        }
        else if( val_final>1501)
        {
          RPM = val_final; //
        }
        //RPM=val;
        //mySerial.println("val");
        // mySerial.println(val);
//        mySerial.println("RPM");
//        mySerial.println(RPM); */
        //********************************************************************************************
//   mySerial.println(Counter_Display_Flag);
 if (!EXECUTE_PID_LOOP) {
          Pid.PID_Start();
          EXECUTE_PID_LOOP = 1;
        }
        Pid.PID_callingg();
       
        if (!TIME_SOCK_OR_TOTAL)                     COUNTDOWN_START = 1;
        else
        {
          if (!DEGREE_VALUE_SET)
          {
            if ( TEMP_DEG >= PID_SETPOINT_To_Show )        COUNTDOWN_START = 1;
            else                                          COUNTDOWN_START = 0;
          }
          else
          {
            if ( TEMP_FAH >= PID_SETPOINT_To_Show )        COUNTDOWN_START = 1;
            else                                          COUNTDOWN_START = 0;
          }
        }

        if (TEMP_DEG >= PID_SETPOINT_To_Show )
        {
          Tick.BUZZ_INTERVAL2(400);
          Tick.BUZZ_INTERVAL1(800);
          if (REP == 0)
          {
            REPETATION_FLAG = 1;
            Tick.BUZZ_TICK_START();
            REP++;
          }
          if (FOR_REP == 2)
          {
            Tick.BUZZ_TICK_STOP();
            REPETATION_FLAG = 0;
          }
        }

        SET_HOUR = SET_MIN / 60;
        REMAIN_MIN = SET_MIN % 60 ;
        DISP_INC++;

        DOT1_1 = 0;

        if (RATE_OF_HEATING >= 1 )
        {
          DOT1_1 = 0;
          DOT2_1 = 0;
          IN_ERROR = 1;
          REPETATION_FLAG = 0;
          ALL_LED_OFF();
          digitalWrite(TIMER_LED2, LOW);
          led.DISPLAY_DIGIT2(21, 25, 26, 26) ;
          led.DISPLAY_DIGIT1(14, 13, 16, 30) ;
          Tick.BUZZ_INTERVAL2(1000);
          Tick.BUZZ_INTERVAL1(2000);
          if (REP == 0)
          {
            Tick.BUZZ_TICK_START();
            REP++;
          }
        }
        else
        { 
        
          if (Pump_enable_flag)
          {
//              mySerial.println("pump function");
            DOT1_1 = 0;
            DOT2_1 = 0;
            DOT1_0 = 0;
            DOT2_2 = 0;
            ALL_LED_OFF();
//            Alternate_Display_Flag = 0;
            digitalWrite(TIMER_LED1, LOW);
            led.DISPLAY_DIGIT2(12, 16, 25, 17) ;
            if (! Motor_On_Off_disp_flag)     led.DISPLAY_DIGIT1(0, 21, 21, 24);
            else                  led.DISPLAY_DIGIT1(0, 20, 24, 24);
          }
          else
          {  
//              mySerial.println(Counter_Display_Flag);
//            TIMER_LED_BLINK++;
//            if (TIMER_LED_BLINK >= 0 && TIMER_LED_BLINK < 2200)                    digitalWrite(TIMER_LED1, HIGH);
//            else  if (TIMER_LED_BLINK >= 2200 && TIMER_LED_BLINK < 4400)           digitalWrite(TIMER_LED1, LOW);
//            else                                                              TIMER_LED_BLINK = 0;
//
//            if (IN_ERROR)  Tick.BUZZ_TICK_STOP();

//            led.BREAK_NUMBER1( (SET_HOUR * 100) + REMAIN_MIN );
            //Alter_Disp.update();
            
          if( Counter_Display_Flag==0)
            {
//            mySerial.println("in Counter_Display_Flag==      0");
              digitalWrite(CURRENT_PARA_LED, LOW);
              digitalWrite(SET_PARA_LED, HIGH);
              TIMER_LED_BLINK++;
            if (TIMER_LED_BLINK >= 0 && TIMER_LED_BLINK < 2200)                    digitalWrite(TIMER_LED1, HIGH);
            else  if (TIMER_LED_BLINK >= 2200 && TIMER_LED_BLINK < 4400)           digitalWrite(TIMER_LED1, LOW);
            else                                                              TIMER_LED_BLINK = 0;

            if (IN_ERROR)  Tick.BUZZ_TICK_STOP();
              led.BREAK_NUMBER1( (SET_HOUR * 100) + REMAIN_MIN );
              if (!DEGREE_VALUE_SET)
              {
                if (TEMP_DEG > 99.9) {
                  led.BREAK_NUMBER2(TEMP_DEG * 10);
                  DOT2_1 = 0;
                }
                else              {
                  led.BREAK_NUMBER2(TEMP_DEG * 100);
                  DOT2_1 = 1;
                }
                led.BREAK_NUMBER2(TEMP_DEG * 10);
                DOT2_1 = 0;
                DOT2_2 = 1;
                digits2[3] = 10;
              }
              else
              {
                led.BREAK_NUMBER2(TEMP_FAH * 10);
                digits2[3] = 21;
              }
            }
            else if( Counter_Display_Flag==1) 
            {
//            mySerial.println("in Counter_Display_Flag==             1");
              digitalWrite(CURRENT_PARA_LED, HIGH);
              digitalWrite(SET_PARA_LED, LOW);
              TIMER_LED_BLINK++;
            if (TIMER_LED_BLINK >= 0 && TIMER_LED_BLINK < 2200)                    digitalWrite(TIMER_LED1, HIGH);
            else  if (TIMER_LED_BLINK >= 2200 && TIMER_LED_BLINK < 4400)           digitalWrite(TIMER_LED1, LOW);
            else                                                              TIMER_LED_BLINK = 0;

            if (IN_ERROR)  Tick.BUZZ_TICK_STOP();
              led.BREAK_NUMBER1( (SET_HOUR * 100) + REMAIN_MIN );
              if (!DEGREE_VALUE_SET)
              {
                                     led.BREAK_NUMBER2(PID_SETPOINT_To_Show*10);  //(SET_TEMP_DEG*100);
                                     digits2[3] = 10;
                DOT2_1 = 0;
                DOT2_2 = 1;             
               }
              else
              {
                led.BREAK_NUMBER2(PID_SETPOINT_To_Show * 10);
                digits2[3] = 21;
              }
            }
             else if( Counter_Display_Flag==2 || RPM_vary_flagg==1) 
             {
  
                 if((STIR_VALUE_SET==1)&&myval<460)
                {
                 led.BREAK_NUMBER1(RPM) ;
                  digitalWrite(TIMER_LED1, LOW);  
                }
  
               if (!DEGREE_VALUE_SET)
              {
                                     led.BREAK_NUMBER2(PID_SETPOINT_To_Show*10);  //(SET_TEMP_DEG*100);
                                     digits2[3] = 10;
                DOT2_1 = 0;
                DOT2_2 = 1;             
               }
              else
              {
                led.BREAK_NUMBER2(PID_SETPOINT_To_Show * 10);
                digits2[3] = 21;
              }          
           }      
          }
         }
        } 
      ALL_LED_OFF();
      Tick.LED_On_Call();
      Process_Interrupt_flag = 0;
      Eeprom.Process_Time_Set_On_Eeprom();
     // Alter_Disp.stop();
      Temp_Serial.stop();
      while (Blinking_Freq < 3)
      {
        Tick.BUZZ_INTERVAL(500);
        Pid.PID_Stop();
        led.DISP_UPDATE();
      }
      ENTER_BUT = 0;
      break;
    default:
      break;
  }
}

void Pclass :: SET_LONG_PRESS_FUNCTION()
{
  digitalWrite(TIMER_LED1, LOW);
  if (Mode_Button_Single_Press_In_Settings)
  {
    switch (SELECTION_PARAMETER)
    {
      case Stir_ON_OFF :
        DOT1_1 = 0;
        DOT2_1 = 0;
        DOT1_0 = 0;
        DOT2_2 = 0;
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(12, 16, 25, 17) ;
        if (i < VALUE_FOR_BLINK)    led.DISPLAY_DIGIT1(24, 24, 24, 24) ;
        else
        {
          if (!SET_LONG_VALUE[SELECTION_PARAMETER])   led.DISPLAY_DIGIT1(0, 21, 21, 24);
          else                                       led.DISPLAY_DIGIT1(0, 20, 24, 24);
        }
        if (i > TOTAL_VALUE_FOR_BLINK)    i = 0;
        break;
      case Unit_C_F :                                          //unit                                                                          //stir
        ALL_LED_OFF();
        DOT1_1 = 0;
        DOT2_1 = 0;
        led.DISPLAY_DIGIT2(28, 20, 25, 16) ;
        //                i++;
        if (i < VALUE_FOR_BLINK)    led.DISPLAY_DIGIT1(24, 24, 24, 24) ;
        else
        {
          if (!SET_LONG_VALUE[SELECTION_PARAMETER])      led.DISPLAY_DIGIT1(29, 10, 24, 24);
          else                                          led.DISPLAY_DIGIT1(29, 21, 24, 24);
        }
        if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
        break;

      case Safe_Value :                                                       //safe
        ALL_LED_OFF();
        DOT1_1 = 0;
        led.DISPLAY_DIGIT2(12, 13, 21, 15) ;
        //i++;
        if (i < VALUE_FOR_BLINK)
        {
          if (!BLINK_STATS)    led.DISPLAY_DIGIT1(24, 24, 24, 24) ;
          else i = VALUE_FOR_BLINK;
        }
        else
        {
          if (!DEGREE_VALUE_SET)
          {
            led.BREAK_NUMBER1(SET_LONG_VALUE[SELECTION_PARAMETER] * 10);
            //                    digits1[0] = 24;
          }
          else
            led.BREAK_NUMBER1(SET_LONG_VALUE[Safe_In_Fah] * 10);
        }
        if (!DEGREE_VALUE_SET)   digits1[3] = 10;
        else                    digits1[3] = 21;
        if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
        break;
      case Time_Mode:                                              //time selection
        DOT1_0 = 0;
        DOT1_1 = 0;
        DOT2_1 = 0;
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(16, 25, 18, 15) ;
        //i++;
        if (i < VALUE_FOR_BLINK)
        {
          if (!BLINK_STATS)  led.DISPLAY_DIGIT1(24, 24, 24, 24) ;
          else i = VALUE_FOR_BLINK;
        }
        else
        {
          if (!SET_LONG_VALUE[SELECTION_PARAMETER]) led.DISPLAY_DIGIT1(16, 23, 16, 31);
          else                                     led.DISPLAY_DIGIT1(12, 23, 13, 10);
        }
        if (i > TOTAL_VALUE_FOR_BLINK)    i = 0;
        break;
      case Auto_Resume_Enable:
        ALL_LED_OFF();
        //             led.DISPLAY_DIGIT2(17, 15, 12, 18) ;
        led.DISPLAY_DIGIT2(11, 17, 23, 12) ; //Pros
        if (i < VALUE_FOR_BLINK)
        {
          if (!BLINK_STATS)  led.DISPLAY_DIGIT1(24, 24, 24, 24) ;
          else i = VALUE_FOR_BLINK;
        }
        else
        {
          if (!SET_LONG_VALUE[SELECTION_PARAMETER])  led.DISPLAY_DIGIT1(32, 25, 12, 14); //disb
          else                                      led.DISPLAY_DIGIT1(15, 20, 14, 26); //enbl
        }
        if (i > TOTAL_VALUE_FOR_BLINK)    i = 0;
        break;
      case Calibration_Selection:                                                                      //calb
        DOT1_1 = 0;
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(10, 13, 26, 14) ;
        if (i < VALUE_FOR_BLINK)    led.DISPLAY_DIGIT1(24, 24, 24, 24) ;
        else
        {
          if (SET_LONG_VALUE[SELECTION_PARAMETER])   led.DISPLAY_DIGIT1(10, 13, 26, 2) ;
          else                                      led.DISPLAY_DIGIT1(10, 13, 26, 1) ;
        }
        if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
        break;

      case Cal1_Decimal  :                                                            //cal1 -L
        DOT1_0 = 0;
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(10, 13, 26, 1) ;
        if (i < VALUE_FOR_BLINK)
        {
          if (!BLINK_STATS)    {
            digits1[0] = 24;
            digits1[1] = 24;
          }
          else i = VALUE_FOR_BLINK;
        }
        else  led.BREAK_NUMBER1((SET_LONG_VALUE[Cal1_Decimal] * 100) + ( SET_LONG_VALUE[Cal1_Point] * 10));
        digits1[3] = 10;
        if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
        break;

      case Cal1_Point :
        if (!DEGREE_VALUE_SET)
        {
          if (i < VALUE_FOR_BLINK)
          {
            if (!BLINK_STATS)    digits1[2] = 24;
            else i = VALUE_FOR_BLINK;
          }
          else   led.BREAK_NUMBER1((SET_LONG_VALUE[Cal1_Decimal] * 100) + ( SET_LONG_VALUE[Cal1_Point] * 10));
          digits1[3] = 10;
        }
        else
        {
          if (i < VALUE_FOR_BLINK)
          {
            if (!BLINK_STATS)    led.DISPLAY_DIGIT1(24, 24, 24, 24) ;
            else i = VALUE_FOR_BLINK;
          }
          else        led.BREAK_NUMBER1(SET_LONG_VALUE[Cal1_In_Fah] * 10);
          digits1[3] = 21;
        }
        if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
        break;
      case Cal2LowTemp_Set_Display :                                                                      //cal2-L
        DOT1_0 = 1;
        DOT1_1 = 0;
        ALL_LED_OFF();
        CAL2_START = 1;
        led.DISPLAY_DIGIT2(10, 13, 26, 2) ;
        if (i < VALUE_FOR_BLINK)
        {
          if (!BLINK_STATS) led.DISPLAY_DIGIT1(24, 24, 24, 24) ;
          else i = VALUE_FOR_BLINK;
        }
        else
        {
          led.BREAK_NUMBER1(SET_LONG_VALUE[Cal2LowTemp_Set_Display] * 10);
        }
        digits1[0] = 26;
        digits1[3] = 10;
        if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
//        mySerial.print("low value set-------------------------1 ");
//        mySerial.println(val_final);
        break;
      case Cal2HighTemp_Set_Display  :                                                                //cal2-H
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(10, 13, 26, 2) ;
        if (i < VALUE_FOR_BLINK)
        {
          if (!BLINK_STATS)   led.DISPLAY_DIGIT1(24, 24, 24, 24) ;
          else i = VALUE_FOR_BLINK;
        }
        else
        {
          led.BREAK_NUMBER1(SET_LONG_VALUE[Cal2HighTemp_Set_Display] ); //* 10
        }
        digits1[0] = 30;
//        digits1[3] = 10; ;
        if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
//        mySerial.print("high value set-------------------------2 ");
        break;

      case Cal2LowDecimalTemp_Enter_Heat:                       //Inprocess-Entering value
        ALL_LED_OFF();
        DOT1_1 = 1;
        DOT2_2 = 0;
        led.BREAK_NUMBER2(LOW_CAL_VALUE_SET);
        digits2[0] = 26;
        digits2[1] = 23;
        //i++;
        if (i < VALUE_FOR_BLINK)
        {
          if (!BLINK_STATS)  {
            digits1[0] = 24;
            digits1[1] = 24;
          }
          else i = VALUE_FOR_BLINK;
        }
        else
        {
          led.BREAK_NUMBER1((SET_LONG_VALUE[SELECTION_PARAMETER] * 100) + (SET_LONG_VALUE[SELECTION_PARAMETER + 1] * 10));
          digits1[3] = 10;
        }
        if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
        break;

      case Cal2LowPointTemp_Enter_Maintain :                                                           //Inprocess-Entering value decimal
        ALL_LED_OFF();
//        mySerial.print("Maintain-------------------------4 ");
        DOT2_2 = 0;
        led.BREAK_NUMBER2(LOW_CAL_VALUE_SET);
        digits2[0] = 26;
        digits2[1] = 23;
        //i++;
        if (i < VALUE_FOR_BLINK)
        {
          if (!BLINK_STATS)  digits1[2] = 24;
          else i = VALUE_FOR_BLINK;
        }
        else
        {
          led.BREAK_NUMBER1((SET_LONG_VALUE[SELECTION_PARAMETER - 1] * 100) + (SET_LONG_VALUE[SELECTION_PARAMETER] * 10));
        }
        digits1[3] = 10;
        if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
        CAL2_CALCULATED_LOW_CAL_VALUE  = ((SET_LONG_VALUE[Cal2LowDecimalTemp_Enter_Heat] ) + (SET_LONG_VALUE[Cal2LowPointTemp_Enter_Maintain] / 10.0));   //f;
        break;
      case Cal2HighDecimalTemp_Enter_Heat :                                                          //Inprocess-Entering value High
        ALL_LED_OFF();
        led.BREAK_NUMBER2(HIGH_CAL_VALUE_SET);
        digits2[0] = 30;
        digits2[1] = 25;
        DOT1_1 = 1;
        //i++;
        if (i < VALUE_FOR_BLINK)
        {
          if (!BLINK_STATS)  {
            digits1[0] = 24;
            digits1[1] = 24;
          }
          else i = VALUE_FOR_BLINK;
        }
        else
        {
          led.BREAK_NUMBER1((SET_LONG_VALUE[SELECTION_PARAMETER] * 100) + (SET_LONG_VALUE[SELECTION_PARAMETER + 1] * 10));
        }
        digits1[3] = 10;
        if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
        break;
      case Cal2HighPointTemp_Enter_Maintain :                                                            //Inprocess-Entering value High
        ALL_LED_OFF();
        led.BREAK_NUMBER2(HIGH_CAL_VALUE_SET);
        digits2[0] = 30;
        digits2[1] = 25;
        DOT1_1 = 1;
        //i++;
        if (i < VALUE_FOR_BLINK)
        {
          if (!BLINK_STATS)  digits1[2] = 24;
          else i = VALUE_FOR_BLINK;
        }
        else
        {
          led.BREAK_NUMBER1((SET_LONG_VALUE[SELECTION_PARAMETER - 1] * 100) + (SET_LONG_VALUE[SELECTION_PARAMETER] * 10));
        }
        digits1[3] = 10;
        if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;

        CAL2_CALCULATED_HIGH_CAL_VALUE = ((SET_LONG_VALUE[SELECTION_PARAMETER - 1]) + (SET_LONG_VALUE[SELECTION_PARAMETER] / 10.0)); //f;
        break;
      case Cal2_Complete_Display:
        Service_Menu = 0;
        LONG_PRESS = 0;
        break;
      case FactoryAndSure_Reset:
        DOT1_0 = 0;
        DOT1_1 = 0;
        DOT2_1 = 0;
        DOT2_2 = 0;
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(12, 28, 17, 15) ;
        led.DISPLAY_DIGIT1(17, 15, 12, 16) ;
        currentMillis = 0;
        break;
      case ResetDone:
        DOT1_0 = 0;
        DOT1_1 = 0;
        DOT2_1 = 0;
        DOT2_2 = 0;
        ALL_LED_OFF();
        //               Eeprom.EEPROM_CLEAR();
        if (currentMillis >= 0 && currentMillis <= 30)
        {
          digitalWrite(BUZZER, HIGH);
          led.DISPLAY_DIGIT2(22, 22, 22, 22) ;
          led.DISPLAY_DIGIT1(22, 22, 22, 22) ;
        }
        else if (currentMillis > 40  &&  currentMillis <= 170)
        {
          digitalWrite(BUZZER, LOW);
          led.DISPLAY_DIGIT2(17, 15, 12, 16)  ;  //rest
          led.DISPLAY_DIGIT1(32, 23, 20, 15) ;   //done
        }
        else  if (currentMillis > 170)
        {
          Eeprom.EEPROM_CLEAR();
        }
        break;
      default:
        break;

    }
  }
  else
  {
    switch (SELECTION_PARAMETER)
    {
      case Stir_ON_OFF  ://stir
        DOT1_1 = 0;
        DOT2_1 = 0;
        DOT1_0 = 0;
        DOT2_2 = 0;
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(12, 16, 25, 17) ;
        if (!STIR_VALUE_SET)   led.DISPLAY_DIGIT1(0, 21, 21, 24);
        else                  led.DISPLAY_DIGIT1(0, 20, 24, 24);
        break;
      case Unit_C_F  :  //unit
        ALL_LED_OFF();
        DOT1_1 = 0;
        DOT2_1 = 0;
        led.DISPLAY_DIGIT2(28, 20, 25, 16) ;
        if (!DEGREE_VALUE_SET)   led.DISPLAY_DIGIT1(29, 10, 24, 24);
        else                    led.DISPLAY_DIGIT1(29, 21, 24, 24);
        break;
      case Safe_Value  :  //safe
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(12, 13, 21, 15) ;
        if (!DEGREE_VALUE_SET)
        {
          led.BREAK_NUMBER1(SAFE_VALUE_SET_DEG * 10);
          digits1[3] = 10;
          //                   digits1[0] = 24;
        }
        else
        {
          DOT1_1 = 0;
          led.BREAK_NUMBER1(SAFE_VALUE_SET_FAH * 10);
          digits1[3] = 21;
        }
        break;
      case Time_Mode :   //time selection
        DOT1_0 = 0;
        DOT1_1 = 0;
        DOT2_1 = 0;
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(16, 25, 18, 15) ;
        if (!TIME_SOCK_OR_TOTAL) led.DISPLAY_DIGIT1(16, 23, 16, 31);
        else                    led.DISPLAY_DIGIT1(12, 23, 13, 10);
        break;
      case Auto_Resume_Enable:
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(13, 34, 16, 23) ;  //Auto
        led.DISPLAY_DIGIT1(11, 17, 23, 12) ; //Pros
        break;
      case Calibration_Selection: //calb
        DOT1_1 = 0;
        DOT2_1 = 0;
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(10, 13, 26, 14) ;
        if (CAL_SELECT_VALUE)   led.DISPLAY_DIGIT1(10, 13, 26, 2) ;
        else                   led.DISPLAY_DIGIT1(10, 13, 26, 1) ;
        break;
      case Cal1_Decimal  :                           //cal1
        DOT1_0 = 0;
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(10, 13, 26, 1) ;
        if (!DEGREE_VALUE_SET)
        {
          if (TEMP_DEG > 99.9)  {
            led.BREAK_NUMBER1(TEMP_DEG * 10);
            DOT1_1 = 0;
          }
          else               {
            led.BREAK_NUMBER1(TEMP_DEG * 100);
            DOT1_1 = 1;
          }
          digits1[3] = 10;
        }
        else
        {
          DOT1_1 = 0;
          led.BREAK_NUMBER1(TEMP_FAH_RAW * 10);
          digits1[3] = 21;
        }
        break;
      case Cal1_Point  : //cal1
        DOT1_0 = 0;
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(10, 13, 26, 1) ;
        if (!DEGREE_VALUE_SET)
        {
          //                  led.BREAK_NUMBER1(TEMP_DEG_RAW *100);
          if (TEMP_DEG > 99.9)  {
            led.BREAK_NUMBER1(TEMP_DEG * 10);
            DOT1_1 = 0;
          }
          else               {
            led.BREAK_NUMBER1(TEMP_DEG * 100);
            DOT1_1 = 1;
          }
          digits1[3] = 10;
        }
        else
        {
          DOT1_1 = 0;
          led.BREAK_NUMBER1(TEMP_FAH_RAW * 10);
          digits1[3] = 21;
        }
        break;
      case Cal2LowTemp_Set_Display :                      //cal2-L                                                                        // CAL1
        DOT1_0 = 1;
        DOT1_1 = 0;
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(10, 13, 26, 2) ;
        led.BREAK_NUMBER1(LOW_CAL_VALUE_SET * 10);
        digits1[0] = 26;
        digits1[3] = 10;
        break;
      case Cal2HighTemp_Set_Display  :                       //cal2-H
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(10, 13, 26, 2) ;
        led.BREAK_NUMBER1(HIGH_CAL_VALUE_SET * 10);
        digits1[0] = 30;
        digits1[3] = 10;
        break;
      case Cal2LowDecimalTemp_Enter_Heat:                        //cal2-strt  process heat
        DOT1_0 = 0;
        DOT2_1 = 1;
        DOT1_1 = 0;
         DOT2_2 = 0;
         
        ALL_LED_OFF();
        Pid.PID_Start();
        PID_SETPOINT = SET_LONG_VALUE[Cal2LowTemp_Set_Display];
        while ((PID_INPUT < PID_SETPOINT) && RST_BUTT)
        { 
//    
//           mySerial.print("PID_INPUT");      mySerial.println(PID_INPUT);
//           mySerial.print("PID_SETPOINT");   mySerial.println(PID_SETPOINT);
          Temp.RTD_TEMP_CONV();
          PID_SETPOINT = SET_LONG_VALUE[Cal2LowTemp_Set_Display];
          //                EXECUTE_PID_LOOP=1;
          Pid.PID_callingg();
          digitalWrite(HEATER_STATUS_LED, LOW);
          led.DISP_UPDATE();
          if (i < VALUE_FOR_BLINK)    led.DISPLAY_DIGIT1(24, 24, 24, 24) ;
          else led.DISPLAY_DIGIT1(30, 15, 13, 16) ; //HEAT
          if (i > TOTAL_VALUE_FOR_BLINK)    i = 0;
          led.BREAK_NUMBER2(LOW_CAL_VALUE_SET);
          digits2[0] = 26;
          digits2[1] = 23;
        }
        SET_MIN = 30;
        //             currentMillis1=0;
        COUNTDOWN_START = 1;
        SELECTION_PARAMETER++;
//        mySerial.println(SELECTION_PARAMETER);
        break;
      case Cal2LowPointTemp_Enter_Maintain:
        while ((!Mode_Button_Single_Press_In_Settings) && RST_BUTT  && COUNTDOWN_START)
        {
          Temp.RTD_TEMP_CONV();
          DOT2_2 = 0;
          PID_SETPOINT = SET_LONG_VALUE[Cal2LowTemp_Set_Display];
          led.DISP_UPDATE();
          //                EXECUTE_PID_LOOP=1;
          Pid.PID_callingg();
          digitalWrite(ALARM_LED, LOW);
          digitalWrite(HEATER_STATUS_LED, HIGH);
          led.BREAK_NUMBER2(LOW_CAL_VALUE_SET);
          digits2[0] = 26;
          digits2[1] = 23;
          DOT2_1 = 1;
          if (i < VALUE_FOR_BLINK)
          {
            led.DISPLAY_DIGIT1(24, 24, 24, 24) ;
            digitalWrite(BUZZER, LOW);
            DOT1_1 = 0;
          }
          else
          {
            digitalWrite(BUZZER, HIGH);
            if (TEMP_DEG_RAW_ktype > 99.9)  {
              led.BREAK_NUMBER1(TEMP_DEG_RAW_ktype * 10);
              DOT1_1 = 0;
            }
            else               {
              led.BREAK_NUMBER1(TEMP_DEG_RAW_ktype * 100);
              DOT1_1 = 1;
            }
            DOT1_1 = 1;
            digits1[3] = 10;
          }
          if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
        }
        if (COUNTDOWN_START)
          SELECTION_PARAMETER = Cal2LowDecimalTemp_Enter_Heat;
        else
        {
          SELECTION_PARAMETER = Cal2HighDecimalTemp_Enter_Heat;
          Mode_Button_Single_Press_In_Settings = 0;
        }
        break;
      case Cal2HighDecimalTemp_Enter_Heat :                                               //cal2-strt
        DOT1_0 = 0;
//               mySerial.println("while  Cal2HighDecimalTemp_Enter_Heat-------------------------while");
        DOT1_1 = 0;
        DOT2_1 = 0;
        PID_SETPOINT = SET_LONG_VALUE[Cal2HighTemp_Set_Display];
        while ((PID_INPUT < PID_SETPOINT) && RST_BUTT  )
        {
          Temp.RTD_TEMP_CONV();
          PID_SETPOINT = SET_LONG_VALUE[Cal2HighTemp_Set_Display];
          //                EXECUTE_PID_LOOP=1;
          Pid.PID_callingg();
          led.DISP_UPDATE();
          digitalWrite(HEATER_STATUS_LED, LOW);
    
          if (i < VALUE_FOR_BLINK)    led.DISPLAY_DIGIT1(24, 24, 24, 24) ;
          else                        led.DISPLAY_DIGIT1(30, 15, 13, 16) ;
          if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
          led.BREAK_NUMBER2(HIGH_CAL_VALUE_SET);
          DOT2_0 = 1;
          digits2[0] = 30;
         // digits2[1] = 25;
        }
        //            currentMillis1=0;
        SET_MIN = 30;
        COUNTDOWN_START = 1;
        SELECTION_PARAMETER++;
        break;
      case Cal2HighPointTemp_Enter_Maintain :                                                // maintain temp till user interupt for entering temp
        while ((!Mode_Button_Single_Press_In_Settings) && RST_BUTT && COUNTDOWN_START )
        {
          Temp.RTD_TEMP_CONV();
          PID_SETPOINT = SET_LONG_VALUE[Cal2HighTemp_Set_Display];
          //                EXECUTE_PID_LOOP=1;
          Pid.PID_callingg();
          led.DISP_UPDATE();
          digitalWrite(ALARM_LED, LOW);
          digitalWrite(HEATER_STATUS_LED, HIGH);
          led.BREAK_NUMBER2(HIGH_CAL_VALUE_SET);
          digits2[0] = 30;
         // digits2[1] = 25;
          DOT2_0 = 1;
          //                  if(currentMillis1>0 && currentMillis1<100 )  digitalWrite(BUZZER, HIGH);
          //                  else if(currentMillis1>=100 && currentMillis1<200 )  digitalWrite(BUZZER, LOW);
          //                  if (currentMillis1>200) currentMillis1=0;
          if (i < VALUE_FOR_BLINK)
          {
            led.DISPLAY_DIGIT1(24, 24, 24, 24) ;
            digitalWrite(BUZZER, LOW);
          }
          else
          {
            digitalWrite(BUZZER, HIGH);
            if (TEMP_DEG_RAW_ktype > 99.9)  {
              led.BREAK_NUMBER1(TEMP_DEG_RAW_ktype ); //* 10
              DOT1_1 = 0;
            }
            else               {
              led.BREAK_NUMBER1(TEMP_DEG_RAW_ktype);
//              DOT2_2 = 1;
            }
//            digits1[3] = 10;
            //                  DOT1_1=1;
          }
          if (i > TOTAL_VALUE_FOR_BLINK)   i = 0;
        }
        PID_SETPOINT = 0;
        //             EXECUTE_PID_LOOP=0;
        if (COUNTDOWN_START)                    SELECTION_PARAMETER = Cal2HighDecimalTemp_Enter_Heat;
        else
        {
          LONG_PRESS = 0;
          Mode_Button_Single_Press_In_Settings = 0;
        }
        break;
      case Cal2_Complete_Display :
        DOT2_1 = 0;
        DOT1_1 = 0;
        if (i < TOTAL_VALUE_FOR_BLINK + 150)
        {
          led.DISPLAY_DIGIT2(10, 13, 26, 14) ;
          led.DISPLAY_DIGIT1(12, 13, 28, 15) ;
        }
        else
        {
          Service_Menu = 0;
          Mode_Button_Single_Press_In_Settings = 0;
          LONG_PRESS = 0;
        }
        break;
      case FactoryAndSure_Reset:
        DOT1_0 = 0;
        DOT1_1 = 0;
        DOT2_1 = 0;
        DOT2_2 = 0;
        ALL_LED_OFF();
        led.DISPLAY_DIGIT2(21, 13, 10, 16) ;  //fact
        led.DISPLAY_DIGIT1(17, 15, 12, 16) ;
        break;
      default:
        break;

    }
  }

}

void Temperature_Serial()
{
  Serial1.println(TEMP_DEG);
}



void Pclass :: ALL_LED_OFF()
{
  digitalWrite(ALARM_LED, HIGH);
  digitalWrite(SET_PARA_LED, HIGH);
  digitalWrite(CURRENT_PARA_LED, HIGH);
  digitalWrite(HEATER_STATUS_LED, HIGH);
}

Pclass Procss = Pclass();
