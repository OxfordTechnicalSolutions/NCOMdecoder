#include "NComRxC.h"
#include <string>

extern "C" 
{
    NComRxC* CreateNComRxC();
    void DisposeNCoMRxC();
    bool UpdatePacket(NComRxC* nrxPtr, unsigned char inputChar);
    double GetMeasurement(NComRxC* nrxPtr, std::string measName);
}