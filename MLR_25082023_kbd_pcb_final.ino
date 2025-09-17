
#include "Arduino.h"
#include "Ext_var.h"
//
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(38, 39); // RX, TX

//long data=0;

void Probe_Error();

void VerDisp();
Ticker VersionDisplay(VerDisp, 1000, 1); 
//*************************************************Buuton**********************************************************************************
OneButton button1(MODE_KEY,  true);
OneButton button2(UP_KEY,    true);
OneButton button3(DOWN_KEY,  true);
OneButton button4(RST_KEY,   true);

int8_t ENTER_BUT = 0, Mode_Button_Single_Press_In_Settings = 0, LONG_PRESS = 0, INC5 = 0, TIME_WASTE_FLAG=0;
 
bool PUMP_FLAG=0;
int FOR_REP=0, FOR_REP2=0,  REP=0, REP1=0;
bool StopVersionDisplay=1;
int myval=0;
bool but1 = 0, Service_Menu=0;
unsigned long start_tt =0;
bool Pump_enable_flag=0;
bool Motor_On_Off_disp_flag=0;

void setup()
{
//  mySerial.begin(9600);
//  Serial1.begin(9600);
  Butn.SETUP();
  led.SETUP();  
  Pid.Setup();
  Procss.Setup();
  Temp.Setup();
  Eeprom.Setup(); 
  PrbDetect.SetupProbeDetection();
  
  pinMode(MODE_KEY, INPUT_PULLUP);
  pinMode(UP_KEY,   INPUT_PULLUP);
  pinMode(DOWN_KEY, INPUT_PULLUP);
  pinMode(RST_KEY,  INPUT_PULLUP);

  button1.attachClick(MODE_CLICK);                       //single press Mode button
  button1.attachLongPressStart(MODE_LONG_PRESS_START);   //long press Mode button
  
  button2.attachClick(INC_CLICK);                        //inc button single press
  button2.attachDuringLongPress(INC_LONGPRESS);          //inc button when long press
  button3.attachLongPressStop(INC_LONGPRESS_STOP);       //inc button when long press stop
  
  button3.attachClick(DEC_CLICK);                        //decrement button single press
  button3.attachDuringLongPress(DEC_LONGPRESS);          //decrement button when long press
  button3.attachLongPressStop(DEC_LONGPRESS_STOP);       //decrementec button when long press
  
  button4.attachClick(RESET_CLICK);                       //single press Reset button
  button4.attachLongPressStart(RESET_LONG_PRESS_START);   //long press Reset button

//  Eeprom.Setup(); 
  VersionDisplay.start();          //-----------------------To start version display  
}

void loop()
{ 
//  Serial1.println(TEMP_DEG_RAW_Internal);
  
  led.DISP_UPDATE();                       //----------------------display write update
  Temp.RTD_TEMP_CONV();  
  if(StopVersionDisplay)                   //----------------------version display
  {  
    VersionDisplay.update();  
    Tick.BUZZ_INTERVAL(25);                //---------------------Buzzz on
    led.DISPLAY_DIGIT2(34, 24, 24, 24) ;   //----------------------V
    led.DISPLAY_DIGIT1(24, 2, 0, 0) ;      //----------------------2.00
    DOT1_1=1;                              //----------------------dot on                       
  }
  else 
  {
    VersionDisplay.stop(); 
//     if(TEMP_DEG_RAW >=113)     Probe_Error();                 //-----------------------Temp error detection loop
    if(!ProbeDetectFlag)       Butn.Probe_Error();                 //-----------------------Temp error detection loop
    else
    { 
      but_check();
      if (LONG_PRESS )    Procss.SET_LONG_PRESS_FUNCTION();  //-----------------------Settings
      else                Procss.SET_SHORT_PRESS_FUNCTION(); //-----------------------To set Process
    } 
  } 
}

void VerDisp()
{
  StopVersionDisplay=0;        //-----------------------To stop version display  
}

void Bclass:: Probe_Error()
{
   DOT1_1= DOT1_0=DOT1_2= DOT2_1= DOT2_2= DOT2_0=0;  
   ENTER_BUT =  LONG_PRESS =  PID_SETPOINT = Mode_Button_Single_Press_In_Settings=0;
   digitalWrite(TIMER_LED1, LOW);            // -------------------------- : off
   digitalWrite(TIMER_LED2, LOW);            // -------------------------- : off
   led.DISPLAY_DIGIT2(11, 17, 23, 14) ;      // -------------------------- : PrbE
   led.DISPLAY_DIGIT1(15, 17, 23, 17) ;      // -------------------------- : Erpr
//   Tick.BUZZ_INTERVAL2(1000);                // -------------------------- : 1sec on
//   Tick.BUZZ_INTERVAL1(2000);                // -------------------------- : 1sec off
//   if(REP == 0)                              // -------------------------- : To start tick only oncec
//   {
//     Tick.BUZZ_TICK1_Start();                  
//     REP++;
// mySerial.print("in probe error loop: ");
// mySerial.println(ProbeDetectFlag);

//    }
}

void MODE_CLICK()
//{ Serial1.println("in mode key");
{
  if(Inprocess_click==1)
  { 

//    mySerial.println("mode single press after mode long prees");

    if(Pump_enable_flag==1)  //inprocess mode click after long press to set the stir On/OFF
    {
      if(Motor_On_Off_disp_flag==1)
            STIR_VALUE_SET=1;
      else if(Motor_On_Off_disp_flag==0)
            STIR_VALUE_SET=0;
            
     Counter_Display_Flag=0;
     Pump_enable_flag=0;
     Eeprom.Save_Pump_Data();    
    }
    else
    {
//    Serial1.println("Counter_Display_Flag");
//     Serial1.println(Counter_Display_Flag);
      Counter_Display_Flag++; 
      if(Counter_Display_Flag>2)
         Counter_Display_Flag=0; 
    }
//         mySerial.println(Counter_Display_Flag);
  }
  
  BLINK_STATS=0;                                                        //-----------------------To blink start parameter
  if (!LONG_PRESS )                                                     //-----------------------no longpress (out of settings)
  {
    i = 0;
//    if(!ENTER_BUT && !DEGREE_VALUE_SET)                               //-----------------------at home page and selection of parameter in degree
//    {    
//       int CONV3 = (TEMP_DEG * 10);
//       MIN_SET_TEMP =((TEMP_DEG + ((CONV3 % 10) / 10.0)) +10); //7    //-----------------------Level sensing start minimu limit of temp
//    }         
    if (ENTER_BUT == 4 && SET_MIN == 0 ) ENTER_BUT = 4;                 //-----------------------when selected time is zero, process should not start
    else if(ENTER_BUT == 1)                             //-----------------------at home page and selection of parameter in fah  
    {
//       MIN_SET_FAH =   TEMP_FAH + 15 ; //12                           //-----------------------Level sensing start minimu limit of temp
       ENTER_BUT = 3;
    }
    else   ENTER_BUT++;                                                 //-----------------------single press increment variable for homepage
       
  }
  else                                                                  //-----------------------in settings         
  {
    if (!Mode_Button_Single_Press_In_Settings)                          //-----------------------not press mode button (while scrolling Parameter Passed to Array)
    {                            //---------------------------------- to show saved value on display while scrolling
//      SET_LONG_VALUE[Stir_ON_OFF] = STIR_VALUE_SET;
//      Counter_Display_Flag++; 
//      if(Counter_Display_Flag>2)
//         Counter_Display_Flag=0;
      SET_LONG_VALUE[Unit_C_F]    = DEGREE_VALUE_SET;
      SET_LONG_VALUE[Auto_Resume_Enable]    = Proceess_Resume_Enable_Flag;
      
      if (!DEGREE_VALUE_SET)    SET_LONG_VALUE[Safe_Value] = SAFE_VALUE_SET_DEG;
      else                      SET_LONG_VALUE[Safe_In_Fah] = SAFE_VALUE_SET_FAH;
      
      SET_LONG_VALUE[Calibration_Selection] = CAL_SELECT_VALUE;
      // -------------160823
//      if (!DEGREE_VALUE_SET)
//      {
//        if(TEMP_DEG_RAW<100)
//        {
          SET_LONG_VALUE[Cal1_Decimal] = TEMP_DEG; //TEMP_DEG;                    //CAL1_VALUE_SAVED_DEG1;
          TEMPERARY_SAVED_VARIABLE_FOR_CAL1_DEG =  TEMP_DEG_RAW * 10;
          int CONV_TEMP = (TEMP_DEG * 10);
          SET_LONG_VALUE[Cal1_Point] = ( CONV_TEMP  % 10) ;        //CAL1_VALUE_SAVED_DEG2 ;
//        }
//        else
//        {
//          SET_LONG_VALUE[Cal1_Decimal]=99; 
//          SET_LONG_VALUE[Cal1_Point] = 9;
//        }
//      }
//      else
//      {
//        SET_LONG_VALUE[Cal1_In_Fah] =  TEMP_FAH_RAW ;  // SET_LONG_VALUE[9]  =CAL1_VALUE_SAVED_FAH;
//        TEMPERARY_SAVED_VARIABLE_FOR_CAL1_FAH = TEMP_FAH_RAW;
//      }
      SET_LONG_VALUE[Cal2LowTemp_Set_Display]  = LOW_CAL_VALUE_SET ;
      SET_LONG_VALUE[Cal2LowDecimalTemp_Enter_Heat]  =  TEMP_DEG_RAW_ktype; //TEMP_DEG ; //---------------16082023
      
      int CONV = (TEMP_DEG_RAW_ktype * 10);
      SET_LONG_VALUE[Cal2LowPointTemp_Enter_Maintain]  =  CONV % 10 ;
      
       //---------------16082023      
      if(SELECTION_PARAMETER == Cal2LowPointTemp_Enter_Maintain )      TEMP_CAL2_RAW_LOW_VALUE = TEMP_DEG_RAW_ktype;
      if(SELECTION_PARAMETER == Cal2HighPointTemp_Enter_Maintain)      TEMP_CAL2_RAW_HIGH_VALUE = TEMP_DEG_RAW_ktype;

//      if(TEMP_DEG_RAW<100)
//      {
        SET_LONG_VALUE[Cal2HighDecimalTemp_Enter_Heat] = TEMP_DEG_RAW_ktype;   //TEMP_DEG;
        int CONV2 = (TEMP_DEG_RAW_ktype * 10);
        SET_LONG_VALUE[Cal2HighPointTemp_Enter_Maintain]  = ( CONV2 % 10 ) ;
//      }
//      else
//      {
//          SET_LONG_VALUE[Cal2HighDecimalTemp_Enter_Heat]  = 99; 
//          SET_LONG_VALUE[Cal2HighPointTemp_Enter_Maintain] = 9;
//      }            

 //---------------

    
      SET_LONG_VALUE[Time_Mode] = TIME_SOCK_OR_TOTAL; //13
      if(SELECTION_PARAMETER == FactoryAndSure_Reset )  currentMillis=0;
      
      if (!DEGREE_VALUE_SET && SELECTION_PARAMETER == Cal1_Point)   SELECTION_PARAMETER--;
      Mode_Button_Single_Press_In_Settings = 1;         //-----------------------press mode button so allow editing of parameters
    }
    else
    {
      switch (SELECTION_PARAMETER)              // Saved parameter value
      {
        case Stir_ON_OFF :
          STIR_VALUE_SET = SET_LONG_VALUE[Stir_ON_OFF];
          EEPROM.write(0, STIR_VALUE_SET);
          break;
        case Unit_C_F :
          DEGREE_VALUE_SET = SET_LONG_VALUE[Unit_C_F];
          EEPROM.write(1, DEGREE_VALUE_SET);
          break;
        case Safe_Value :
          if (!DEGREE_VALUE_SET) 
          {
            SAFE_VALUE_SET_DEG = SET_LONG_VALUE[Safe_Value];
            EEPROM.put(150, SAFE_VALUE_SET_DEG);
          }
          else            
          {
            SAFE_VALUE_SET_FAH = SET_LONG_VALUE[Safe_In_Fah];
            EEPROM.put(3, SAFE_VALUE_SET_FAH);
          }
          break;
        case Auto_Resume_Enable :
             Proceess_Resume_Enable_Flag = SET_LONG_VALUE[Auto_Resume_Enable];
             EEPROM.write(15, Proceess_Resume_Enable_Flag);
        
        case Calibration_Selection :
          CAL_SELECT_VALUE = SET_LONG_VALUE[Calibration_Selection];
          EEPROM.write(5, CAL_SELECT_VALUE);
          break;
        case Cal1_Point :
          if (!DEGREE_VALUE_SET)
          {
            CAL1_VALUE_SAVED_DEG1 = SET_LONG_VALUE[Cal1_Decimal]; // EEPROM.write(6, CAL1_VALUE_SAVED_DEG1);
            CAL1_VALUE_SAVED_DEG2 = SET_LONG_VALUE[Cal1_Point];  //EEPROM.write(7, CAL1_VALUE_SAVED_DEG2);
            ERROR_CAL = ((( CAL1_VALUE_SAVED_DEG1 + (  CAL1_VALUE_SAVED_DEG2 / 10.0)) * 10) - TEMPERARY_SAVED_VARIABLE_FOR_CAL1_DEG) / 10; //f; 
             ERROR_SET_IN_DEG_FLAG=1;  ERROR_SET_IN_FAH_FLAG=0;
          }
          else
          {
            CAL1_VALUE_SAVED_FAH = SET_LONG_VALUE[Cal1_In_Fah];  //EEPROM.put(8, CAL1_VALUE_SAVED_FAH);
            ERROR_CAL = SET_LONG_VALUE[20] -  TEMPERARY_SAVED_VARIABLE_FOR_CAL1_FAH;   
            ERROR_SET_IN_DEG_FLAG=0; ERROR_SET_IN_FAH_FLAG=1;
          }   
          if(ERROR_CAL<0)  { SIGN=1;  ERROR_CAL1 = (( ERROR_CAL *10 )* (-1));}
          else             { SIGN=0;  ERROR_CAL1 = ERROR_CAL *10;}
          EEPROM.put(70, ERROR_CAL1);  
          EEPROM.write(7, SIGN);      
          EEPROM.write(8, ERROR_SET_IN_DEG_FLAG);
          EEPROM.write(9, ERROR_SET_IN_FAH_FLAG);         
          break;
        case Cal2LowTemp_Set_Display :
          LOW_CAL_VALUE_SET   =  SET_LONG_VALUE[Cal2LowTemp_Set_Display];
          SET_LONG_VALUE[Cal2HighTemp_Set_Display]   = (SET_LONG_VALUE[Cal2LowTemp_Set_Display] + 10);
          break;
        case Cal2HighTemp_Set_Display :
             HIGH_CAL_VALUE_SET  = SET_LONG_VALUE[Cal2HighTemp_Set_Display];
             TEMP_CAL2_RAW_LOW_VALUE = TEMP_DEG_RAW_ktype;
             break;
        case Cal2LowPointTemp_Enter_Maintain : 
             TEMP_CAL2_RAW_HIGH_VALUE = TEMP_DEG_RAW_ktype;
             break;
        case Cal2HighPointTemp_Enter_Maintain :
              CAL2_RAW_LOW_VALUE_CONV  = ( TEMP_CAL2_RAW_LOW_VALUE*10);
              CAL2_RAW_HIGH_VALUE_CONV =( TEMP_CAL2_RAW_HIGH_VALUE *10);              
              EEPROM.put(80, CAL2_RAW_LOW_VALUE_CONV);
              EEPROM.put(90, CAL2_RAW_HIGH_VALUE_CONV);
              MASTER_LOW_CONV  = (CAL2_CALCULATED_LOW_CAL_VALUE *10);
              MASTER_HIGH_CONV = (CAL2_CALCULATED_HIGH_CAL_VALUE *10); //serial
              EEPROM.put(100, MASTER_LOW_CONV);
              EEPROM.put(110, MASTER_HIGH_CONV);
//              mySerial.print("CAL2_RAW_LOW_VALUE_CONV:  "); mySerial.println(CAL2_RAW_LOW_VALUE_CONV);
//              mySerial.print("CAL2_RAW_HIGH_VALUE_CONV:  "); mySerial.println(CAL2_RAW_HIGH_VALUE_CONV);
//              mySerial.print("MASTER_LOW_CONV:  "); mySerial.println(MASTER_LOW_CONV);
//              mySerial.print("MASTER_HIGH_CONV:  "); mySerial.println(MASTER_HIGH_CONV);
//              mySerial.print("TEMP_DEG_RAW_ktype:  "); mySerial.println(TEMP_DEG_RAW_ktype);
//              mySerial.print("TEMP_DEG_ktype:  "); mySerial.println(TEMP_DEG_ktype);
    
          break;               
        case Time_Mode :
               TIME_SOCK_OR_TOTAL = SET_LONG_VALUE[Time_Mode];
               EEPROM.put(65, TIME_SOCK_OR_TOTAL);
              break;        
        default :
          break;
      }
       
       
      if ((!DEGREE_VALUE_SET && SELECTION_PARAMETER == Cal1_Decimal)  ||
            SELECTION_PARAMETER == Cal2LowTemp_Set_Display || 
            SELECTION_PARAMETER == Cal2LowDecimalTemp_Enter_Heat  || 
            SELECTION_PARAMETER == Cal2HighDecimalTemp_Enter_Heat ||  
            SELECTION_PARAMETER == FactoryAndSure_Reset ) 
      {
        Mode_Button_Single_Press_In_Settings = 1;
        SELECTION_PARAMETER++;
      }
      else if (SELECTION_PARAMETER == Cal2HighTemp_Set_Display  ||
               SELECTION_PARAMETER == Cal2LowPointTemp_Enter_Maintain || 
               SELECTION_PARAMETER == Cal2HighPointTemp_Enter_Maintain)        
      {
        Mode_Button_Single_Press_In_Settings = 0;
        SELECTION_PARAMETER++;
      }
      else        Mode_Button_Single_Press_In_Settings = 0;

    }
    i =VALUE_FOR_BLINK +1;
  }
  
   if ( ENTER_BUT >= 5) 
   {
      ENTER_BUT = 5; 
//      if(PUMP_FLAG)   PUMP_FLAG=0;           //--------------------To edit stir on off option in between process
//      else            PUMP_FLAG =1;
//      Eeprom.Save_Pump_Data();
   }
   if(ENTER_BUT >= 5 && REP1<1) { Tick.BUZZ_INTERVAL(1000);  REP1++; }  //long pree
   else                           Tick.BUZZ_INTERVAL(50);               //short pree
}

void MODE_LONG_PRESS_START()
{ 
  if(Inprocess_click==1)
  {  
//     mySerial.println("mode long prees");
      Pump_enable_flag=1;
  }
  if (!ENTER_BUT)
  {
    if (!LONG_PRESS )      //------------------at home page 
    {
      i = 350;
      SELECTION_PARAMETER = Stir_ON_OFF;
      LONG_PRESS = 1;      //------------------go to menu
      RST_BUTT = 1;
    }
  }
 Tick.BUZZ_INTERVAL(200); 
}

void Bclass :: Button_Tick_Update()
{
  button1.tick();
  button2.tick();
  button3.tick();
  button4.tick();  
}

void INC_CLICK()
{ 
  if(Pump_enable_flag==1 && Inprocess_click==1)
  {
       Motor_On_Off_disp_flag=!Motor_On_Off_disp_flag;
//       if(STIR_VALUE_SET)  
//        STIR_VALUE_SET=0;
//      else 
//        STIR_VALUE_SET=1;
  }
  
  if (LONG_PRESS && !Mode_Button_Single_Press_In_Settings )
  {    
    if(!Service_Menu)
    {
       SELECTION_PARAMETER--;                              //------------------- scroll down menu
      if (SELECTION_PARAMETER < Stir_ON_OFF)    
          SELECTION_PARAMETER = Auto_Resume_Enable;
    }
    else
    {
      if ( SELECTION_PARAMETER == Cal1_Point ) 
                              SELECTION_PARAMETER = Calibration_Selection;
      else if (SELECTION_PARAMETER == Cal2LowDecimalTemp_Enter_Heat ) 
                                SELECTION_PARAMETER = Cal2LowTemp_Set_Display;
      else if(SELECTION_PARAMETER == FactoryAndSure_Reset)   
                                SELECTION_PARAMETER = Cal2LowTemp_Set_Display;
      else                      SELECTION_PARAMETER--;      
       if (SELECTION_PARAMETER < Calibration_Selection)
       SELECTION_PARAMETER = FactoryAndSure_Reset;      
    }
  }  
  if (!LONG_PRESS  || (LONG_PRESS && Mode_Button_Single_Press_In_Settings ))    Butn.SET_CHANGE_UP();
}

void INC_LONGPRESS()
{
  if(!but1)
  {
    BLINK_STATS=1;
    INC5++;
    if (INC5 >= 25)
    {
      Butn.SET_CHANGE_UP();
      INC5 = 0;
    }
  }
}

void INC_LONGPRESS_STOP()
{
  BLINK_STATS=0;  
}

void DEC_CLICK()
{  
   if(Pump_enable_flag==1 && Inprocess_click==1)
  {
          Motor_On_Off_disp_flag=!Motor_On_Off_disp_flag;
//       if(Motor_On_Off_disp_flag)  
//        STIR_VALUE_SET=0;
//      else 
//        STIR_VALUE_SET=1;
  }
  
  if (LONG_PRESS && !Mode_Button_Single_Press_In_Settings )
  {
    if(!Service_Menu){
        SELECTION_PARAMETER++;
        if (SELECTION_PARAMETER > Auto_Resume_Enable)  
            SELECTION_PARAMETER = Stir_ON_OFF;
    }
    else 
    {
      if (SELECTION_PARAMETER == Calibration_Selection )     
                   SELECTION_PARAMETER = Cal1_Point;
      else if (SELECTION_PARAMETER == Cal2LowTemp_Set_Display  &&  CAL2_START )      
                      SELECTION_PARAMETER = Cal2LowDecimalTemp_Enter_Heat;
      else if (CAL2_START == 0 && SELECTION_PARAMETER >= Cal2LowTemp_Set_Display && SELECTION_PARAMETER <= Cal2_Complete_Display  )  
                       SELECTION_PARAMETER = FactoryAndSure_Reset;        
      else if(SELECTION_PARAMETER >=  FactoryAndSure_Reset)            
              SELECTION_PARAMETER = Calibration_Selection; //0; 
      else                                                 
             SELECTION_PARAMETER++;
      if (SELECTION_PARAMETER > FactoryAndSure_Reset)  
            SELECTION_PARAMETER = Calibration_Selection;
    }
  }
  if (!LONG_PRESS  || (LONG_PRESS && Mode_Button_Single_Press_In_Settings ))       Butn.SET_CHANGE_DOWN();
}

void DEC_LONGPRESS()
{ 
  if(!but1)
  {
    BLINK_STATS=1;
    INC5++;
    if (INC5 >= 25)
    {
      Butn.SET_CHANGE_DOWN();
      INC5 = 0;
    }
  }
}

void DEC_LONGPRESS_STOP()
{
  BLINK_STATS=0;
}

void but_check()//--------------------------------------------------------------------------UP DOWN Key Long Press detection
{
  if (digitalRead(UP_KEY) == LOW && digitalRead(DOWN_KEY) == LOW && but1 == 0) //----------Check for button press
  {
    start_tt = millis();
    but1 = 1;
  }
  else if (digitalRead(UP_KEY) == LOW && digitalRead(DOWN_KEY) == LOW && but1 == 1) //--------Check for button is continously
  {
    if ((millis() - start_tt) >= 3000)                                              //-------------------after 3 sec completion
    {
//      lock_set();//------------Go to lock settings mode.
     Tick.BUZZ_INTERVAL(500);  
      if (!ENTER_BUT && !LONG_PRESS )     
      {
          i = 350;
          LONG_PRESS = 1; 
          RST_BUTT = 1;
          Service_Menu=1;
          SELECTION_PARAMETER = Calibration_Selection;
      }
    }
  }
  else if (but1 == 1)
  {
    if (digitalRead(UP_KEY) == HIGH || digitalRead(DOWN_KEY) == HIGH)
    {
      but1 = 0;
    }
  }
}

void RESET_CLICK()
{
   if (!DEGREE_VALUE_SET && SELECTION_PARAMETER == Cal1_Point  || 
        SELECTION_PARAMETER == Cal2LowPointTemp_Enter_Maintain || 
        SELECTION_PARAMETER == Cal2HighPointTemp_Enter_Maintain ) 
        SELECTION_PARAMETER--;
   else
   {
      if (SELECTION_PARAMETER == Cal2LowDecimalTemp_Enter_Heat    || 
          SELECTION_PARAMETER == Cal2HighDecimalTemp_Enter_Heat )   
          Mode_Button_Single_Press_In_Settings =1;
      else
      {
        if(Mode_Button_Single_Press_In_Settings) Mode_Button_Single_Press_In_Settings=0;
      }
   }
  if (!LONG_PRESS )
  {
    if(DEGREE_VALUE_SET && ENTER_BUT==2)   ENTER_BUT=0;
    else if(ENTER_BUT == 3)                ENTER_BUT=1;
    else if (ENTER_BUT >= 1 && ENTER_BUT <= 4)  ENTER_BUT--;
   if ( ENTER_BUT == 5   &&  RATE_OF_HEATING >= 1  && CHECK_RATE_OF_HEATING == 1 )
   {
     RATE_OF_HEATING =0;
     CHECK_RATE_OF_HEATING =1;
   }
  }
   Tick.BUZZ_INTERVAL(50); 
}

void RESET_LONG_PRESS_START()
{
  if (!LONG_PRESS )   {
  if(!ENTER_BUT)
  {
    RST_BUTT = 0;    
    LONG_PRESS = 0;
    SELECTION_PARAMETER = Unit_C_F;// 0;
  }
  else
  {
      ENTER_BUT = 0;
      SET_MIN = 0;
//      Process_Interrupt_flag=0;
  }
  }
  if (LONG_PRESS )
  {
    if(Service_Menu) Service_Menu=0;
    switch (SELECTION_PARAMETER)
    {
      case Stir_ON_OFF :
        STIR_VALUE_SET  = SET_LONG_VALUE[Stir_ON_OFF];
        SET_VALUE[5]=SET_LONG_VALUE[Stir_ON_OFF];
        break;
      case Unit_C_F :
        DEGREE_VALUE_SET = SET_LONG_VALUE[Unit_C_F];
        break;
      case Safe_Value :
        if (!DEGREE_VALUE_SET)     SAFE_VALUE_SET_DEG = SET_LONG_VALUE[Safe_Value];
        else                       SAFE_VALUE_SET_FAH = SET_LONG_VALUE[Safe_In_Fah];
        break;
      case Calibration_Selection :
        CAL_SELECT_VALUE = SET_LONG_VALUE[Calibration_Selection];
        break;
      case Auto_Resume_Enable :
          Proceess_Resume_Enable_Flag = SET_LONG_VALUE[Auto_Resume_Enable];
        break;
      case Cal1_Point :
        if (!DEGREE_VALUE_SET)
        {
          CAL1_VALUE_SAVED_DEG1 = SET_LONG_VALUE[Cal1_Decimal];
          CAL1_VALUE_SAVED_DEG2 = SET_LONG_VALUE[Cal1_Point];
        }
        else      CAL1_VALUE_SAVED_FAH = SET_LONG_VALUE[Cal1_In_Fah];
        break;
      case Cal2LowTemp_Set_Display :      
        LOW_CAL_VALUE_SET   = SET_LONG_VALUE[Cal2LowTemp_Set_Display];
        break;
      case Cal2HighTemp_Set_Display :
        SET_LONG_VALUE[Cal2HighTemp_Set_Display]   = (SET_LONG_VALUE[Cal2LowTemp_Set_Display] + 10);
        HIGH_CAL_VALUE_SET  = SET_LONG_VALUE[Cal2HighTemp_Set_Display];
        break;
      case Cal2LowDecimalTemp_Enter_Heat :
        CAL2_LOW1_VALUE_SET  =  SET_LONG_VALUE[Cal2LowDecimalTemp_Enter_Heat] ;
        break;
      case Cal2LowPointTemp_Enter_Maintain :
        CAL2_LOW2_VALUE_SET  =  SET_LONG_VALUE[Cal2LowPointTemp_Enter_Maintain];
        break;
      case Cal2HighDecimalTemp_Enter_Heat :
        CAL2_HIGH1_VALUE_SET = SET_LONG_VALUE[Cal2HighDecimalTemp_Enter_Heat];
        break;
      case Cal2HighPointTemp_Enter_Maintain :
        CAL2_HIGH2_VALUE_SET = SET_LONG_VALUE[Cal2HighPointTemp_Enter_Maintain];
        break;
      case Time_Mode :
        TIME_SOCK_OR_TOTAL =SET_LONG_VALUE[Time_Mode];
        break;
      default :
        break;
    }
    LONG_PRESS = 0;
  }
 Tick.BUZZ_INTERVAL(200); 
}
