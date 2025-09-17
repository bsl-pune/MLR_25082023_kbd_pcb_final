#include "Ext_Var.h"
void Probe_Detection();

bool ProbeDetectFlag=0;
ProbeDetect::ProbeDetect()
{}

void ProbeDetect :: SetupProbeDetection() 
{  
  pinMode(PROBE_DETECTION, INPUT_PULLUP);  
  attachInterrupt(PROBE_DETECTION, Probe_Detection, CHANGE);  
  Probe_Detection();
}

void Probe_Detection()
{
  // Ticks.Buzz_Call(1000);
   if(digitalRead(PROBE_DETECTION) == 0)
   {
     ProbeDetectFlag=1;
   }
   else
   {
     ProbeDetectFlag=0;
   }
}

ProbeDetect PrbDetect = ProbeDetect();
