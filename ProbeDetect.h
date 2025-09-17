#ifndef ProbeDetect_h
#define ProbeDetect_h 

#define PROBE_DETECTION   1//   21 used for buzzer

class ProbeDetect
{
   public :
   ProbeDetect();
   void SetupProbeDetection();
};

extern ProbeDetect PrbDetect;
#endif
