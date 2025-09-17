#ifndef Procss_h
#define Procss_h

class Pclass
{
   public :
   Pclass();
   void Setup();
   void ALL_LED_OFF();
   void SET_LONG_PRESS_FUNCTION();
   void SET_SHORT_PRESS_FUNCTION();
};

enum 
{
  Stir_ON_OFF, 
  Unit_C_F,      
  Safe_Value,    
  Time_Mode,   
  Auto_Resume_Enable, 
  Calibration_Selection,
  Cal1_Decimal,
  Cal1_Point,    
  Cal2LowTemp_Set_Display, //--------------100
  Cal2HighTemp_Set_Display ,//-------------200
  Cal2LowDecimalTemp_Enter_Heat,  //------------120
  Cal2LowPointTemp_Enter_Maintain,   //---------0.9
  Cal2HighDecimalTemp_Enter_Heat,//--------------99
  Cal2HighPointTemp_Enter_Maintain,//------------0.9
  Cal2_Complete_Display,  
  FactoryAndSure_Reset,
  ResetDone,
  Safe_In_Fah ,   
  Cal1_In_Fah    
}LongPressSelection;

extern Pclass Procss;
#endif
