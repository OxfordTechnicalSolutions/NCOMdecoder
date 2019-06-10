#include "NComRxCWrapper.hpp"
#include <stdio.h>
#include <string>

NComRxC* CreateNComRxC()
{
    // Create NCom decoder and check
	NComRxC* nrx_ptr = NComCreateNComRxC();
    return nrx_ptr;
}

void DisposeNCoMRxC(NComRxC* nrxPtr)
{
    NComDestroyNComRxC(nrxPtr);
}

bool UpdatePacket(NComRxC* nrxPtr, unsigned char inputChar)
{
    return (NComNewChar(nrxPtr, inputChar) == COM_NEW_UPDATE) ? true : false; 
}
  
double GetMeasurement(NComRxC* nrxPtr, std::string measName)
{
	// Print the measurments listed in the header
	if (nrxPtr->mIsTimeValid)
	{
		if (measName.compare("Lat") == 0) { if(nrxPtr->mIsLatValid) return (nrxPtr->mLat);} 
		if (measName.compare("Lon") == 0) { if(nrxPtr->mIsLonValid) return (nrxPtr->mLon);} 
		if (measName.compare("Alt") == 0) { if(nrxPtr->mIsAltValid) return (nrxPtr->mAlt);} 
		if (measName.compare("Ax") == 0) { if(nrxPtr->mIsAxValid) return (nrxPtr->mAx);} 
		if (measName.compare("Ay") == 0) { if(nrxPtr->mIsAyValid) return (nrxPtr->mAy);} 
		if (measName.compare("Az") == 0) { if(nrxPtr->mIsAzValid) return (nrxPtr->mAz);} 
		if (measName.compare("InsNavMode)") == 0) { if(nrxPtr->mIsInsNavModeValid) return (nrxPtr->mInsNavMode);} 
		if (measName.compare("SerialNumber") == 0) { if(nrxPtr->mIsSerialNumberValid) return (nrxPtr->mSerialNumber);} 
		if (measName.compare("GpsPosMode") == 0) { if(nrxPtr->mIsGpsPosModeValid) return (nrxPtr->mGpsPosMode);} 
		if (measName.compare("GpsVelMode") == 0) { if(nrxPtr->mIsGpsVelModeValid) return (nrxPtr->mGpsVelMode);} 
		if (measName.compare("GpsAttMode") == 0) { if(nrxPtr->mIsGpsAttModeValid) return (nrxPtr->mGpsAttMode);} 
    }
	return 0;
}
