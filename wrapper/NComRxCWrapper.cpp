#include "NComRxCWrapper.hpp"
#include <stdio.h>
#include <string>
#include <iostream>
#include <bitset>

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
  
double GetMeasurement(NComRxC* nrxPtr, const char* inputMeasName)
{
	std::string measName(inputMeasName);

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
		if (measName.compare("mGpsNumObs") == 0) { if(nrxPtr->mIsGpsNumObsValid) return (nrxPtr->mGpsNumObs);} 
		if (measName.compare("mTime") == 0) { if(nrxPtr->mIsTimeValid) return (nrxPtr->mTime);} 
		if (measName.compare("mCmdChars") == 0) { if(nrxPtr->mIsCmdCharsValid) return (nrxPtr->mCmdChars);} 
		if (measName.compare("mCmdCharsSkipped") == 0) { if(nrxPtr->mIsCmdCharsSkippedValid) return (nrxPtr->mCmdCharsSkipped);} 
		if (measName.compare("mCmdPkts") == 0) { if(nrxPtr->mIsCmdPktsValid) return (nrxPtr->mCmdPkts);} 
		if (measName.compare("mCmdErrors") == 0) { if(nrxPtr->mIsCmdErrorsValid) return (nrxPtr->mCmdErrors);} 

    }
	return 0;
}
