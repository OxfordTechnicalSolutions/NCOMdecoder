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

	if (measName.compare("IsTimeValid") == 0)  				{return (nrxPtr->mIsTimeValid);} 
	if (measName.compare("Time") == 0) 	    				{return (nrxPtr->mTime);} 

	if (measName.compare("IsLatValid") == 0) 				{ return (nrxPtr->mIsLatValid);}
	if (measName.compare("Lat") == 0) 						{ return (nrxPtr->mLat);} 

	if (measName.compare("IsLonValid") == 0) 				{ return (nrxPtr->mIsLonValid);}
	if (measName.compare("Lon") == 0) 						{ return (nrxPtr->mLon);} 

	if (measName.compare("IsAltValid") == 0) 				{ return (nrxPtr->mIsAltValid);}
	if (measName.compare("Alt") == 0) 						{ return (nrxPtr->mAlt);} 

	if (measName.compare("IsAxValid")  == 0) 				{ return (nrxPtr->mIsAxValid) ;} 
	if (measName.compare("Ax") == 0)  						{ return (nrxPtr->mAx);} 

	if (measName.compare("IsAyValid")  == 0) 				{ return (nrxPtr->mIsAyValid) ;} 
	if (measName.compare("Ay") == 0)  						{ return (nrxPtr->mAy);} 

	if (measName.compare("IsAzValid")  == 0) 				{ return (nrxPtr->mIsAzValid) ;} 
	if (measName.compare("Az") == 0)  						{ return (nrxPtr->mAz);} 

	if (measName.compare("IsInsNavModeValid") == 0)			{ return (nrxPtr->mIsInsNavModeValid);}
	if (measName.compare("InsNavMode)") == 0) 				{ return (nrxPtr->mInsNavMode);} 

	if (measName.compare("IsSerialNumberValid") == 0)  		{ return (nrxPtr->mIsSerialNumberValid);}
	if (measName.compare("SerialNumber") == 0)  			{ return (nrxPtr->mSerialNumber);} 

	if (measName.compare("IsGpsPosModeValid") == 0) 		{ return (nrxPtr->mIsGpsPosModeValid);}
	if (measName.compare("GpsPosMode") == 0) 				{ return (nrxPtr->mGpsPosMode);} 

	if (measName.compare("IsGpsVelModeValid") == 0) 		{ return (nrxPtr->mIsGpsVelModeValid);}
	if (measName.compare("GpsVelMode") == 0) 				{ return (nrxPtr->mGpsVelMode);} 

	if (measName.compare("IsGpsAttModeValid") == 0) 		{ return (nrxPtr->mIsGpsAttModeValid);}
	if (measName.compare("GpsAttMode") == 0) 				{ return (nrxPtr->mGpsAttMode);} 

	if (measName.compare("IsGpsNumObsValid") == 0)			{ return (nrxPtr->mIsGpsNumObsValid);}
	if (measName.compare("GpsNumObs") == 0)					{ return (nrxPtr->mGpsNumObs);} 

	if (measName.compare("IsCmdCharsValid") == 0)			{ return (nrxPtr->mIsCmdCharsValid);}
	if (measName.compare("CmdChars") == 0) 					{ return (nrxPtr->mCmdChars);} 

	if (measName.compare("IsCmdCharsSkippedValid") == 0)   	{ return (nrxPtr->mIsCmdCharsSkippedValid);} 
	if (measName.compare("CmdCharsSkipped") == 0)  			{ return (nrxPtr->mCmdCharsSkipped);} 
	
	if (measName.compare("IsCmdPktsValid") == 0) 		 	{ return (nrxPtr->mIsCmdPktsValid);}
	if (measName.compare("CmdPkts") == 0) 		 			{ return (nrxPtr->mCmdPkts);} 

	if (measName.compare("IsCmdErrorsValid") == 0) 			{ return (nrxPtr->mIsCmdErrorsValid);}
	if (measName.compare("CmdErrors") == 0) 				{ return (nrxPtr->mCmdErrors);} 

	else return 0;

}
