#include "NComRxC.h"
#include <string>

extern "C" 
{
    NComRxC* CreateNComRxC();
    void DisposeNCoMRxC(NComRxC* nrxPtr);
    bool UpdatePacket(NComRxC* nrxPtr, unsigned char inputChar);
    double GetMeasurement(NComRxC* nrxPtr, const char* inputMeasName);
}