#ifndef Eeprom_h
#define Eeprom_h

class Rclass
{
   public :
   Rclass();
   void Setup();
   void EEPROM_CLEAR();
   void Save_Pump_Data();
   void Process_Set_Values_Store_On_Eeprom();
   void Process_Time_Set_On_Eeprom();
};

extern Rclass Eeprom;
#endif
