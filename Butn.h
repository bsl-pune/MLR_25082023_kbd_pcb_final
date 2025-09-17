#ifndef Butn_h
#define Butn_h

#define HEATER_STATUS_LED 27
#define CURRENT_PARA_LED  28
#define SET_PARA_LED      29
#define ALARM_LED         2  //old in wlc pcb 26  
//#define analog_pot        A4//26 //alarm pin used for analog_pot in new pcb
#define BUZZER            21 //old in wlc pcb 16
#define MODE_KEY          35   
#define UP_KEY            37  
#define DOWN_KEY          36
#define RST_KEY           34  
#define HEATER            24
#define PUMP              25 //DRV_Enable
#define TIMER_LED1        12
#define TIMER_LED2        13


//24-HEATER //25 -PUMP//
//#define SAFETY_RTD        22  

class Bclass
{
   public :
   Bclass();
   void SETUP();
   void TIME_CAL();  
   void SET_CHANGE_UP();
   void SET_CHANGE_DOWN();
   void Button_Tick_Update();

   void Probe_Error();
};
extern Bclass Butn;

#endif
