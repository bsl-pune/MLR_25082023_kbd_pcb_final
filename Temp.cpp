#include "Ext_var.h"
#include <SPI.h>
//
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(38, 39); // RX, TX

#define MISO 15
#define SCLK 16
#define CS1 17
#define CS2 33

byte spiread(void); 
float Selection_Of_Thermocouple(uint8_t ChipSelectionPin);

void READ_THERMOCOUPLE();
Ticker TEMP_READING(READ_THERMOCOUPLE, 2000, 0);


//************************************************Thermocouple***********************************************
//MAX6675 thermocouple(SCK_PIN, CS_PIN, SO_PIN);
//-----------------------------------------------------------------------------------------------------------
//*********************************************temp**********************************************
float TEMP_DEG_RAW2_ktype, TEMP_DEG_RAW1_ktype; //TEMP_DEG_RAW_ktype = 25.5
float TEMP_DEG_RAW_ktype=0;
//float TEMP_DEG = 25.5;
//bool SIGN=0;

//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(38, 39); // RX, TX

//bool COUNTDOWN_START=0;   
Mclass::Mclass()
{}

void RTD_TEMP_CAL();
Ticker TEMP_UPDATE(RTD_TEMP_CAL, 125, 0); //100 to 150

void RTD_TEMP_CAL_Internal();
Ticker TEMP_UPDATE_Internal(RTD_TEMP_CAL_Internal, 50, 0); //100 to 150
//*********************************************temp**********************************************
float TEMP_DEG_RAW = 0.0, TEMP_FAH_RAW = 0.0;
float TEMP_FAH, TEMP_DEG = 0.0;

float TEMP_DEG_RAW_Internal = 0.0, TEMP_FAH_RAW_Internal = 0.0;
float TEMP_FAH_Internal, TEMP_DEG_Internal = 0.0;

//------------------------------------------------------------------------------------------------
#define Rb 100.0
#define Ra 1500.0
#define Rc 1500.0
#define Vs 5

#define Vb (Rb*Vs)/(Rb+Ra)
#define Alpha  0.00385
//float R1 = 100.0; 
//float R2 = 4700.0; 
//float R3 = 4700.0;
//#define Vb_Internal (R1*Vs)/(R1+R2)
float Vin=0, Rt=0, Rx, TEMP_UPDATETD;
float  TotalVout = 0, AvgVout = 0;
int avrage_val = 1, sample;


float Vin_Internal=0, Rt_Internal=0, Rx_Internal, TEMP_UPDATETD_Internal;
float  TotalVout_Internal = 0, AvgVout_Internal = 0;
int avrage_val_Internal = 1, sample_Internal;

//------------------------------------------------------------------------------------------------
float  TEMPERARY_SAVED_VARIABLE_FOR_CAL1_FAH, TEMP_CAL2_RAW_LOW_VALUE, TEMP_CAL2_RAW_HIGH_VALUE; 
int TEMPERARY_SAVED_VARIABLE_FOR_CAL1_DEG;
//***********************************************************************************************
bool ERROR_SET_IN_DEG_FLAG=0, ERROR_SET_IN_FAH_FLAG=0;
float TEMP_DEG_RAW2, TEMP_FAH_RAW2;
//***********************************************************************************************

void Mclass:: Setup()
{  
//  mySerial.begin(9600);
  TEMP_UPDATE.start();
  TEMP_UPDATE_Internal.start();
  SPI.swap(2);

  pinMode(MISO, OUTPUT);
  pinMode(SCLK, OUTPUT);
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);

  digitalWrite(CS1, HIGH);
  digitalWrite(CS2, HIGH);
  
  TEMP_READING.start();
}

void RTD_TEMP_CAL() //external probe
{
    sample++;
    AvgVout = analogRead(A1);
    // myval = analogRead(A4);
    TotalVout  += AvgVout ;  // Vout;
    if(sample >= avrage_val)
    {
      TotalVout /= avrage_val;
      float Vout = ((TotalVout * Vs) / 1023.0);                                      // Voltage conversion
      Vin = Vout / 10;                                                               // Gain
      Rt = ((Vb + Vin) * Rc) / (Vs - (Vb + Vin));   //   Va = Vb + Vin 
      TEMP_DEG = ((Rt / Rb) - 1) / (Alpha);         // alpha is temp coefficient for paltinum //EXT
//      TEMP_FAH_RAW = (( TEMP_DEG_RAW * 1.8 ) + 32);
      
//mySerial.print("External :");
//mySerial.println(TEMP_DEG_RAW);

      sample = 0;   AvgVout = 0;   
    }
    avrage_val = 50;
}

//-----------------------------------------Thermocouple-------------------------------//

float Selection_Of_Thermocouple(uint8_t ChipSelectionPin) //(uint8_t )
{
  uint16_t v;

  digitalWrite(ChipSelectionPin, LOW);
  delayMicroseconds(10);

  v = spiread();
  v <<= 8;
  v |= spiread();

  digitalWrite(ChipSelectionPin, HIGH);

  if (v & 0x4) 
  {
    // uh oh, no thermocouple attached!
    return -1;
    // return -100;
  }

  v >>= 3;

  return v * 0.25;
   

}

//void Mclass:: Temp_Tick_Update()
//{    
//  TEMP_READING.update();
////  TEMP_Conversion.update();
//}

void READ_THERMOCOUPLE()
{
    TEMP_DEG_RAW1_ktype  = Selection_Of_Thermocouple(CS1); 
//     mySerial.print("TEMP_DEG_RAW1_ktype :");
//mySerial.println(TEMP_DEG_RAW1_ktype);  
    TEMP_DEG_RAW2_ktype= Selection_Of_Thermocouple(CS2);
// mySerial.print("TEMP_DEG_ktype:  ---------- "); mySerial.println(TEMP_DEG_ktype);
// mySerial.print("TEMP_DEG_RAW_ktype:  ---------- "); mySerial.println(TEMP_DEG_RAW_ktype);
// mySerial.print("TEMP_DEG:  ---------- "); mySerial.println(TEMP_DEG); 
// mySerial.print("PID_SETPOINT:oil  ---------- "); mySerial.println(PID_SETPOINT); 
    
//    Serial2.println(TEMP_DEG_RAW1);
    if(TEMP_DEG_RAW1_ktype< 0 && TEMP_DEG_RAW1_ktype > 999  || TEMP_DEG_RAW2_ktype <0 && TEMP_DEG_RAW2_ktype >999 )
    {
      TEMP_DEG_RAW1_ktype=0;
      TEMP_DEG_RAW2_ktype =0;
    }
}

byte spiread(void) 
{
  int i;
  byte d = 0;

  for (i = 7; i >= 0; i--) 
  {
    digitalWrite(SCLK, LOW);
    delayMicroseconds(10);
    if (digitalRead(MISO))
    {
      // set the bit to 0 no matter what
      d |= (1 << i);
    }
    digitalWrite(SCLK, HIGH);
    delayMicroseconds(10);
  }
  return d;
}


void Mclass :: RTD_TEMP_CONV()    //-------------------------Pt100 Temp read
{ 
//    mySerial.println(CAL_SELECT_VALUE);
    if(TEMP_DEG_RAW1_ktype == 0) TEMP_DEG_RAW_ktype = TEMP_DEG_RAW2_ktype;
    else                         TEMP_DEG_RAW_ktype = TEMP_DEG_RAW1_ktype;
    
   
    
    if (!CAL_SELECT_VALUE)
    {
      
        if(SIGN) {ERROR_CAL =(ERROR_CAL1) * -1;}
        else     {ERROR_CAL =(ERROR_CAL1) ;}
    
        TEMP_DEG_ktype  =   TEMP_DEG_RAW_ktype   + ERROR_CAL;
    }
    else
    {   
//        mySerial.println("111111111111111111");
//        CAL2_RAW_LOW_VALUE = (CAL2_RAW_LOW_VALUE_CONV / 10.0);
//        CAL2_RAW_HIGH_VALUE = (CAL2_RAW_HIGH_VALUE_CONV  / 10.0);
//        MASTER_LOW =  (MASTER_LOW_CONV  / 10.0);
//        MASTER_HIGH = (MASTER_HIGH_CONV  / 10.0);

        CAL2_RAW_LOW_VALUE = (CAL2_RAW_LOW_VALUE_CONV);
        CAL2_RAW_HIGH_VALUE = (CAL2_RAW_HIGH_VALUE_CONV);
        MASTER_LOW =  (MASTER_LOW_CONV);
        MASTER_HIGH = (MASTER_HIGH_CONV);
        
        TEMP_DEG_ktype = mapFloat(TEMP_DEG_RAW_ktype, CAL2_RAW_LOW_VALUE, CAL2_RAW_HIGH_VALUE, MASTER_LOW,  MASTER_HIGH);
        
//        mySerial.print("CAL2_RAW_LOW_VALUE"); mySerial.println(CAL2_RAW_LOW_VALUE);
//        mySerial.print("CAL2_RAW_HIGH_VALUE"); mySerial.println(CAL2_RAW_HIGH_VALUE);
//        mySerial.print("MASTER_LOW"); mySerial.println(MASTER_LOW);
//        mySerial.print("MASTER_HIGH"); mySerial.println(MASTER_HIGH);
//        mySerial.print("TEMP_DEG_RAW_ktype"); mySerial.println(TEMP_DEG_RAW_ktype);
//        mySerial.print("TEMP_DEG_ktype"); mySerial.println(TEMP_DEG_RAW_ktype);
   }
}

void RTD_TEMP_CAL_Internal()
{
  
    sample_Internal++;
    AvgVout_Internal = analogRead(A0);
   myval = analogRead(A4);
    TotalVout_Internal  += AvgVout_Internal ;  // Vout;
    if(sample_Internal >= avrage_val_Internal)
    {
      TotalVout_Internal /= avrage_val_Internal;
      float Vout_Internal = ((TotalVout_Internal * Vs) / 1023.0);                                  // Voltage conversion
      Vin_Internal = Vout_Internal / 10;                                                           // Gain
//      Rt_Internal = ((Vb_Internal + Vin_Internal) * R3) / (Vs - (Vb_Internal + Vin_Internal));     //   Va = Vb + Vin 
      Rt_Internal = ((Vb + Vin_Internal) * Rc) / (Vs - (Vb + Vin_Internal));
//      TEMP_DEG_RAW_Internal = ((Rt / R2) - 1) / (Alpha);                                           // alpha is temp coefficient for paltinum //INT
      TEMP_DEG_RAW_Internal = ((Rt_Internal / Rb) - 1) / (Alpha);    //Rt_Internal
      TEMP_FAH_RAW_Internal = ((TEMP_DEG_RAW * 1.8) + 32);
//      mySerial.print("Internal :");
//mySerial.println(TEMP_DEG_RAW_Internal);
      sample_Internal = 0;   AvgVout_Internal = 0;   
    }
   avrage_val_Internal = 50;
}


void Mclass :: Rtd_Temp_Update()  //-------------------------Pt100 Temp ticker update
{  
  TEMP_UPDATE.update(); 
  TEMP_UPDATE_Internal.update(); 
   TEMP_READING.update();
}

Mclass Temp = Mclass();
