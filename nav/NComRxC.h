//============================================================================================================
//!
//! The software is protected by copyright of Oxford Technical Solutions at oxts.com.
//! © 2008 - 2017, Oxford Technical Solutions Ltd.
//! Unauthorised use, copying or distribution is not permitted.
//! 
//! Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
//! associated documentation files (the "Software"), to deal in the Software without restriction, including 
//! without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
//! copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the 
//! following conditions:
//!
//! All copies or substantial portions of the software must reproduce the above copyright notices, this list 
//! of conditions and the following disclaimer in the software documentation and/or other materials provided
//! with the distribution.
//!
//! The software is provided by the copyright holders "as is" without any warranty of any kind, express or 
//! implied, including, but not limited to, warranties of merchantability or fitness for a particular purpose.
//! In no event shall the copyright holders be liable for any direct, indirect, incidental, special,
//! exemplary, or consequential damages however caused and on any liability, whether in contract, strict
//! liability, or tort (including negligence or otherwise) arising in any way out of the use of this software.
//!
//!
//! \file NComRxC.h
//!
//! \brief NCom C decoder header.
//!
//============================================================================================================


#ifndef NCOMRXC_H
#define NCOMRXC_H


#define NCOMRXC_DEV_ID "130325"  //!< Development Identification.


//============================================================================================================
// Provide multiple compiler support for standard integer types for 8, 16, 32, and 64 bit quantities.
//
// Wish to use standard integer types for 8, 16, 32, and 64 bit quantities. Not all compilers support these
// types. Our aim is to use (u)intN_t for low level byte conversions and counters, i.e. places where the size
// of the type is very important. In more general code the regular types are used, where it is assumed that
// {char, short, int, long long} <--> {8, 16, 32, 64} bit types for both 32 and 64 bit x86 systems.
//
// For x86 systems long can either be 32 or 64 bits so its (direct) use is avoided. The definitions below
// should be modified according to compiler, but with preference to include cstdint for C++ or stdint.h for
// C if provided.
//
// It seems that MSVC does not support the usage of 8 bit integers in printf.

#ifndef __STDC_LIMIT_MACROS
#define __STDC_LIMIT_MACROS
#endif

#ifndef __STDC_FORMAT_MACROS
#define	__STDC_FORMAT_MACROS
#endif

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif


#if(_MSC_VER)
#if(__cplusplus)
#include <climits>
#else
#include <limits.h>
#endif
#if(_MSC_VER < 1300)
	typedef signed   char     int8_t;
	typedef signed   short    int16_t;
	typedef signed   int      int32_t;
	typedef signed   __int64  int64_t;
	typedef unsigned char     uint8_t;
	typedef unsigned short    uint16_t;
	typedef unsigned int      uint32_t;
	typedef unsigned __int64  uint64_t;
#elif(_MSC_VER < 1600)
	typedef signed   __int8   int8_t;          //!< Standard signed integer basic type of 8 bits.
	typedef signed   __int16  int16_t;         //!< Standard signed integer basic type of 16 bits.
	typedef signed   __int32  int32_t;         //!< Standard signed integer basic type of 32 bits.
	typedef signed   __int64  int64_t;         //!< Standard signed integer basic type of 64 bits.
	typedef unsigned __int8   uint8_t;         //!< Standard unsigned integer basic type of 8 bits.
	typedef unsigned __int16  uint16_t;        //!< Standard unsigned integer basic type of 16 bits.
	typedef unsigned __int32  uint32_t;        //!< Standard unsigned integer basic type of 32 bits.
	typedef unsigned __int64  uint64_t;        //!< Standard unsigned integer basic type of 64 bits.
	#define INT8_MIN      ((int8_t)  _I8_MIN)  //!< Limit macro for the most negative value of the standard signed integer type of 8 bits.
	#define INT16_MIN     ((int16_t) _I16_MIN) //!< Limit macro for the most negative value of the standard signed integer type of 16 bits.
	#define INT32_MIN     ((int32_t) _I32_MIN) //!< Limit macro for the most negative value of the standard signed integer type of 32 bits.
	#define INT64_MIN     ((int64_t) _I64_MIN) //!< Limit macro for the most negative value of the standard signed integer type of 64 bits.
	#define INT8_MAX      (_I8_MAX)            //!< Limit macro for the most positive value of the standard signed integer type of 8 bits.
	#define INT16_MAX     (_I16_MAX)           //!< Limit macro for the most positive value of the standard signed integer type of 16 bits.
	#define INT32_MAX     (_I32_MAX)           //!< Limit macro for the most positive value of the standard signed integer type of 32 bits.
	#define INT64_MAX     (_I64_MAX)           //!< Limit macro for the most positive value of the standard signed integer type of 64 bits.
	#define UINT8_MAX     (_UI8_MAX)           //!< Limit macro for the most positive value of the standard unsigned integer type of 8 bits.
	#define UINT16_MAX    (_UI16_MAX)          //!< Limit macro for the most positive value of the standard unsigned integer type of 16 bits.
	#define UINT32_MAX    (_UI32_MAX)          //!< Limit macro for the most positive value of the standard unsigned integer type of 32 bits.
	#define UINT64_MAX    (_UI64_MAX)          //!< Limit macro for the most positive value of the standard unsigned integer type of 64 bits.
	#define INT8_C(val)   val##i8              //!< Literal constant macro for the standard signed integer type of 8 bits.
	#define INT16_C(val)  val##i16             //!< Literal constant macro for the standard signed integer type of 16 bits.
	#define INT32_C(val)  val##i32             //!< Literal constant macro for the standard signed integer type of 32 bits.
	#define INT64_C(val)  val##i64             //!< Literal constant macro for the standard signed integer type of 64 bits.
	#define UINT8_C(val)  val##ui8             //!< Literal constant macro for the standard unsigned integer type of 8 bits.
	#define UINT16_C(val) val##ui16            //!< Literal constant macro for the standard unsigned integer type of 16 bits.
	#define UINT32_C(val) val##ui32            //!< Literal constant macro for the standard unsigned integer type of 32 bits.
	#define UINT64_C(val) val##ui64            //!< Literal constant macro for the standard unsigned integer type of 64 bits.
#else
#if(__cplusplus)
#include <cstdint>
#else
#include <stdint.h>
#endif
#endif
	#define PRId8         #error               //!< Printing signed decimal format macro for the standard signed integer type of 8 bits.
	#define PRId16        "hd"                 //!< Printing signed decimal format macro for the standard signed integer type of 16 bits.
	#define PRId32        "I32d"               //!< Printing signed decimal format macro for the standard signed integer type of 32 bits.
	#define PRId64        "I64d"               //!< Printing signed decimal format macro for the standard signed integer type of 64 bits.
	#define PRIu8         #error               //!< Printing unsigned decimal format macro for the standard unsigned integer type of 8 bits.
	#define PRIu16        "hu"                 //!< Printing unsigned decimal format macro for the standard unsigned integer type of 16 bits.
	#define PRIu32        "I32u"               //!< Printing unsigned decimal format macro for the standard unsigned integer type of 32 bits.
	#define PRIu64        "I64u"               //!< Printing unsigned decimal format macro for the standard unsigned integer type of 64 bits.
#else
#if(__cplusplus && (__cplusplus > 199711L))
#include <cstdint>
#include <cinttypes>
	using std::int8_t;
	using std::int16_t;
	using std::int32_t;
	using std::int64_t;
	using std::uint8_t;
	using std::uint16_t;
	using std::uint32_t;
	using std::uint64_t;
#else
#include <stdint.h>
#include <inttypes.h>
#endif
#endif


//============================================================================================================
//! \brief Matrix element definition allows an easy change from float to double numerical types.
//!
//! All matrix manipulation should be done with this type.

typedef double MatElement;  // Use double numbers for the matrix type.


//============================================================================================================
//! \brief This type is used for all references to matrices.
//!
//! Using this type will prevent the user from needing to worry about matrix dimensions.

typedef struct
{
    MatElement *m; //!< Pointer to the matrix data.
    long r;        //!< Rows of the matrix.
    long c;        //!< Columns of the matrix.
    long tr;       //!< Number of allocated rows.
    long tc;       //!< Number of allocated columns.
} Mat;


//============================================================================================================
//! \brief Macro providing easy access to matrix elements.

#define e(A,r,c) ((A)->m[c+(r)*(int)(A)->tc])


//============================================================================================================
//! \brief Definition of empty (unallocated) matrix.

#define EMPTY_MAT {NULL, 0, 0, 0, 0}


//============================================================================================================
// Useful constants

#define MAX_INN_AGE              (10000)
#define DEV_ID_STRLEN                (9)
#define OS_SCRIPT_ID_STRLEN         (11)
#define BASE_STATION_ID_STRLEN       (5)
#define OMNISTAR_SERIAL_STRLEN      (15)
#define NCOMRX_BUFFER_SIZE         (512)
#define NCOM_PACKET_LENGTH          (72)
#define NCOM_STATUS_PACKET_LENGTH    (8)


//============================================================================================================
//! \brief Response types used by the decoder.
//!
//! In the future, we may wish to distinguish between invalid or incomplete incoming data.

typedef enum
{
	COM_NO_UPDATE,              //!< No update (invalid or incomplete data).
	COM_NEW_UPDATE              //!< Successful update.
} ComResponse;


//============================================================================================================
//! \brief Various output packet states.

typedef enum
{
	OUTPUT_PACKET_INVALID,      //!< An invalid output packet (only on invalidation).
	OUTPUT_PACKET_EMPTY,        //!< An empty output packet.
	OUTPUT_PACKET_REGULAR,      //!< A regular output packet.
	OUTPUT_PACKET_STATUS,       //!< Status only output packet.
	OUTPUT_PACKET_IN1DOWN,      //!< Trigger output packet (falling edge of input).
	OUTPUT_PACKET_IN1UP,        //!< Trigger2 output packet (rising edge of input).
	OUTPUT_PACKET_OUT1,         //!< Digital output packet.
	OUTPUT_PACKET_INTERPOLATED, //!< Interpolated output packet.
	OUTPUT_PACKET_UNKNOWN       //!< Unknown.
} OutputPacketType;


//============================================================================================================
//! \brief Navigation status.

typedef enum
{
	NAVIGATION_STATUS_NOTHING,
	NAVIGATION_STATUS_RAWIMU,
	NAVIGATION_STATUS_INIT,
	NAVIGATION_STATUS_LOCKING,
	NAVIGATION_STATUS_LOCKED,
	NAVIGATION_STATUS_UNLOCKED,
	NAVIGATION_STATUS_EXPIRED,
	NAVIGATION_STATUS_RESERVED_07,
	NAVIGATION_STATUS_RESERVED_08,
	NAVIGATION_STATUS_RESERVED_09,
	NAVIGATION_STATUS_STATUSONLY,
	NAVIGATION_STATUS_RESERVED_11,
	NAVIGATION_STATUS_RESERVED_12,
	NAVIGATION_STATUS_RESERVED_13,
	NAVIGATION_STATUS_RESERVED_14,
	NAVIGATION_STATUS_RESERVED_15,
	NAVIGATION_STATUS_RESERVED_16,
	NAVIGATION_STATUS_RESERVED_17,
	NAVIGATION_STATUS_RESERVED_18,
	NAVIGATION_STATUS_RESERVED_19,
	NAVIGATION_STATUS_TRIGINIT,
	NAVIGATION_STATUS_TRIGLOCKING,
	NAVIGATION_STATUS_TRIGLOCKED,
	NAVIGATION_STATUS_UNKNOWN
} NavigationStatus;


//============================================================================================================
//! \brief Definitions of IMU types.

typedef enum
{
	IMU_TYPE_SIIMUA      = 0,
	IMU_TYPE_IMU200      = 1,
	IMU_TYPE_IMU2        = 2,
	IMU_TYPE_IMU2X       = 3,
	IMU_TYPE_IMU3        = 4,
	IMU_TYPE_IMU3X       = 5
} IMUType;


//============================================================================================================
//! \brief Definitions of GPS types.

typedef enum
{
	GPS_TYPE_BEELINE     = 0,
	GPS_TYPE_OEM4        = 1,
	GPS_TYPE_NONE        = 2,
	GPS_TYPE_OEMV        = 3,
	GPS_TYPE_LEA4        = 4,
	GPS_TYPE_GENERIC     = 5,
	GPS_TYPE_TRIMBLE5700 = 6,
	GPS_TYPE_AGGPS132    = 7,
	GPS_TYPE_GB500       = 8,
	GPS_TYPE_SAPPHIRE    = 9,
	GPS_TYPE_LEA6        = 10,
	GPS_TYPE_BD920       = 11,
	GPS_TYPE_GX1200      = 12,
	GPS_TYPE_B110        = 13,
	GPS_TYPE_OEM6        = 14,
	GPS_TYPE_UNKNOWN     = 15
} GPSType;


//============================================================================================================
//! \brief Indexes into the packet.

typedef enum
{
	PI_SYNC           =  0,
	PI_TIME           =  1,
	PI_ACCEL_X        =  3,
	PI_ACCEL_Y        =  6,
	PI_ACCEL_Z        =  9,
	PI_ANG_RATE_X     = 12,
	PI_ANG_RATE_Y     = 15,
	PI_ANG_RATE_Z     = 18,
	PI_INS_NAV_MODE   = 21,
	PI_CHECKSUM_1     = 22,
	PI_POS_LAT        = 23,
	PI_POS_LON        = 31,
	PI_POS_ALT        = 39,
	PI_VEL_N          = 43,
	PI_VEL_E          = 46,
	PI_VEL_D          = 49,
	PI_ORIEN_H        = 52,
	PI_ORIEN_P        = 55,
	PI_ORIEN_R        = 58,
	PI_CHECKSUM_2     = 61,
	PI_CHANNEL_INDEX  = 62,
	PI_CHANNEL_STATUS = 63,
	PI_CHECKSUM_3     = 71
} PacketIndexes;


//============================================================================================================
//! \brief Indexes into the packet.
//!
//! Range is [start, stop).

typedef enum
{
	PCSR_CHECKSUM_1_START =  1,
	PCSR_CHECKSUM_1_STOP  = 22,
	PCSR_CHECKSUM_2_START = 22,
	PCSR_CHECKSUM_2_STOP  = 61,
	PCSR_CHECKSUM_3_START = 61,
	PCSR_CHECKSUM_3_STOP  = 71
} PacketChecksumRanges;



//############################################################################################################
//##                                                                                                        ##
//##  Filt2ndOrder                                                                                          ##
//##                                                                                                        ##
//############################################################################################################


//============================================================================================================
//! \brief Internally used 2nd order filter.
//!
//! \note Used as a 'private' member of NComRxC.
//!
//! The intention is that the user need not be concerned with this structure. The role of this structure is
//! to store the state variables required for the implementation of discrete 2nd order filter functionality.

typedef struct
{
	// Principal design parameters
	double mFreqSample;        //!< Input sampling frequency. [Hz]
	double mFreqCutoff;        //!< Filter cut-off frequency. [Hz]
	double mZeta;              //!< Filter damping ratio. [-]

	// Derived filter coefficients
	double mA0, mA1, mA2;      //!< Transfer function numerator coefficients.
	double mB0, mB1, mB2;      //!< Transfer function denominator coefficients.

	// Internal state variables
	double mT0, mT1, mT2;      //!< Time stamps of past inputs.
	double mX0, mX1, mX2;      //!< Past inputs.
	double mU0, mU1, mU2;      //!< Past outputs.
	int    mOutputValid;       //!< Is current output valid?

} Filt2ndOrder;




//############################################################################################################
//##                                                                                                        ##
//##  NComRxCInternal                                                                                       ##
//##                                                                                                        ##
//############################################################################################################


//============================================================================================================
//! \brief Internally used variables used in the decoding of a series NCom data packets.
//!
//! \note Used as a 'private' member of NComRxC.
//!
//! The intention is that the user need not be concerned with this structure. The role of this structure is
//! to provide work space and retain state information over a sequence of incoming packets. Basic decoding
//! statistics are also included in this structure since they are property of the decoder rather than the
//! decoded data. These statistics may be accessed (indirectly) using the functions NComNumChars,
//! NComSkippedChars and NComNumPackets defined for NComRxC.

typedef struct
{
	// Some hard coded options.
	int mHoldDistWhenSlow;  //!< Distance hold when Slow.

	// Buffer items.
	unsigned char  mCurPkt[NCOMRX_BUFFER_SIZE]; //!< Holds the incoming data.
	unsigned char *mCurStatus;                  //!< Direct acccess to the raw status byte data.
	int            mCurChannel;                 //!< Direct acccess to the raw status channel number.
	int            mCurLen;                     //!< Length of data in buffer.
	int            mPktProcessed;               //!< Flag indicates processed packet.

	// Byte counters.
	uint64_t mNumChars;     //!< Number of bytes.
	uint64_t mSkippedChars; //!< Number of skipped bytes.
	uint64_t mNumPackets;   //!< Number of packets.

	// For time (comes from Status and each cycle and needs combining).
	int32_t mMilliSecs;
	int32_t mMinutes;

	// To identify new triggers.
	unsigned char mTrigCount;
	unsigned char mTrig2Count;
	unsigned char mDigitalOutCount;

	// To compute distance travelled.
	int    mPrevDist2dValid;
	double mPrevDist2dTime;
	double mPrevDist2dSpeed;
	double mPrevDist2d;
	int    mPrevDist3dValid;
	double mPrevDist3dTime;
	double mPrevDist3dSpeed;
	double mPrevDist3d;

	// Storage for wheel speed information required to persist over several packets.
	int mIsOldWSpeedTimeValid;       double mOldWSpeedTime;
	int mIsOldWSpeedCountValid;      double mOldWSpeedCount;

	// Storage for reference frame information required to persist over several packets.
	int mIsAccurateRefLatValid;      double mAccurateRefLat;
	int mIsAccurateRefLonValid;      double mAccurateRefLon;
	int mIsAccurateRefAltValid;      double mAccurateRefAlt;
	int mIsAccurateRefHeadingValid;  double mAccurateRefHeading;

	// Work space for matrix calculations.
	Mat E;           //!< Euler angles.
	Mat Ab;          //!< Accels Body.
	Mat Al;          //!< Accels level.
	Mat Wb;          //!< Ang Rate Body.
	Mat Wl;          //!< Ang Rate level.
	Mat Vn;          //!< Velocities navigation-frame.
	Mat Vl;          //!< Velocities level.
	Mat Yb;          //!< Ang Accel Body.
	Mat Yl;          //!< Ang Accel level.
	Mat C_on;        //!< Rotation from output-frame to navigation-frame (all angles).
	int C_on_valid;  //!< Rotation from output-frame to navigation-frame (all angles) is valid.
	Mat C_oh;        //!< Rotation from output-frame to horizontal-frame (roll and pitch angles).
	int C_oh_valid;  //!< Rotation from output-frame to horizontal-frame (roll and pitch angles) is valid.
	Mat C_hn;        //!< Rotation from horizontal-frame to navigation-frame (heading angle).
	int C_hn_valid;  //!< Rotation from horizontal-frame to navigation-frame (heading angle) is valid.
	Mat C_sn;        //!< Rotation from surface-frame to navigation-frame.
	Mat C_os;        //!< Rotation from output-frame to surface-frame.
	int mMatrixHold; //!< Prevents matrix calculations when 1 (say for slow computers), normally 0.

	// Filters for linear acceleration.
	int          mIsLinAccFiltFixed;    //!< Filter flags.
	int          mHasLinAccFiltChanged; //!< Filter flags.
	int          mIsLinAccFiltOff;      //!< Filter flags.
	Filt2ndOrder FiltForAx;             //!< Filter for X-axis linear acceleration.
	Filt2ndOrder FiltForAy;             //!< Filter for Y-axis linear acceleration.
	Filt2ndOrder FiltForAz;             //!< Filter for Z-axis linear acceleration.

	// State variables for differentiation of angular rate into angular acceleration.
	double mPrevWx;      //!< Previous value of Wx. [deg/s]
	double mPrevWy;      //!< Previous value of Wy. [deg/s]
	double mPrevWz;      //!< Previous value of Wz. [deg/s]
	double mPrevWbTime;  //!< Time corresponding to previous values of Wx, Wy and Wz. [s]

	// Filters for angular acceleration.
	int          mIsAngAccFiltFixed;    //!< Filter flags.
	int          mHasAngAccFiltChanged; //!< Filter flags.
	int          mIsAngAccFiltOff;      //!< Filter flags.
	Filt2ndOrder FiltForYx;             //!< Filter for X-axis angular acceleration.
	Filt2ndOrder FiltForYy;             //!< Filter for Y-axis angular acceleration.
	Filt2ndOrder FiltForYz;             //!< Filter for Z-axis angular acceleration.

} NComRxCInternal;




//############################################################################################################
//##                                                                                                        ##
//##  NComRxCGps                                                                                            ##
//##                                                                                                        ##
//############################################################################################################


//============================================================================================================
//! \brief Structure to hold GPS information
//!
//! \note Used as a 'public' member of NComRxC.
//!
//! Note there are validity flags for everything. Data members are set to 'zero' and invalid when created.
//!
//! This structure collects most of the information for the primary, secondary and external GPS receivers.

typedef struct
{
	// *** Code Generation Begin - NComRxCGps Structure ***

	//--------------------------------------------------------------------------------------------------------
	// GPS Information

	// System information

	int mIsTypeValid; uint8_t mType; //!< Type of card fitted/connected.
	int mIsFormatValid; uint8_t mFormat; //!< Data format of card fitted/connected.

	int mIsRawRateValid; uint8_t mRawRate; //!< Raw update rate.
	int mIsPosRateValid; uint8_t mPosRate; //!< Position update rate.
	int mIsVelRateValid; uint8_t mVelRate; //!< Velocity update rate.

	int mIsAntStatusValid; uint8_t mAntStatus; //!< Antenna status.
	int mIsAntPowerValid; uint8_t mAntPower; //!< Antenna power.
	int mIsPosModeValid; uint8_t mPosMode; //!< Position mode.

	int mIsSerBaudValid; uint8_t mSerBaud; //!< Receiver baud.

	// Status

	int mIsNumSatsValid; int mNumSats; //!< Number of satellites tracked.

	int mIsCpuUsedValid; double mCpuUsed; //!< CPU usage. [%]
	int mIsCoreNoiseValid; double mCoreNoise; //!< Core noise. [%]
	int mIsCoreTempValid; double mCoreTemp; //!< Core temperature. [deg C]
	int mIsSupplyVoltValid; double mSupplyVolt; //!< Supply voltage. [V]

	// Received data statistics

	int mIsCharsValid; uint32_t mChars; //!< Number of bytes.
	int mIsCharsSkippedValid; uint32_t mCharsSkipped; //!< Number of invalid bytes.
	int mIsPktsValid; uint32_t mPkts; //!< Number of valid packets.
	int mIsOldPktsValid; uint32_t mOldPkts; //!< Number of out of date packets.

	// *** Code Generation End - NComRxCGps Structure ***

} NComRxCGps;



#ifdef __cplusplus
extern "C"
{
#endif


//============================================================================================================
// NComRxCGps access functions to return a textual description of values (such enumerated types) used in
// NComRxCGps. For information about function NComGpsGet<Thing>String see comments regarding <Thing> defined
// in the NComRxCGps structure.

// *** Code Generation Begin - NComRxCGps External String Functions ***

//------------------------------------------------------------------------------------------------------------
// GPS Information

// System information

extern const char *NComGpsGetTypeString(const NComRxCGps *Com);
extern const char *NComGpsGetFormatString(const NComRxCGps *Com);

extern const char *NComGpsGetRawRateString(const NComRxCGps *Com);
extern const char *NComGpsGetPosRateString(const NComRxCGps *Com);
extern const char *NComGpsGetVelRateString(const NComRxCGps *Com);

extern const char *NComGpsGetAntStatusString(const NComRxCGps *Com);
extern const char *NComGpsGetAntPowerString(const NComRxCGps *Com);
extern const char *NComGpsGetPosModeString(const NComRxCGps *Com);

extern const char *NComGpsGetSerBaudString(const NComRxCGps *Com);

// *** Code Generation End - NComRxCGps External String Functions ***


//============================================================================================================
// General function declarations.

extern NComRxCGps *NComGpsCreate();
extern void        NComGpsDestroy(NComRxCGps *Com);
extern void        NComGpsCopy(NComRxCGps *ComDestination, const NComRxCGps *ComSource);


#ifdef __cplusplus
}
#endif




//############################################################################################################
//##                                                                                                        ##
//##  NComRxC                                                                                               ##
//##                                                                                                        ##
//############################################################################################################


//============================================================================================================
//! \brief Structure to hold measurements and information that have been decoded from NCom data packet(s).
//!
//! Note there are validity flags for everything. Most data members are set to 'zero' and invalid when created.
//!
//! The data member "mInternal" should be assumed to be private as this contains decoder state and work space
//! information.

typedef struct
{
	//--------------------------------------------------------------------------------------------------------
	// Other structures.

	NComRxCGps *mGpsPrimary;      //!< Primary internal GPS receiver.
	NComRxCGps *mGpsSecondary;    //!< Secondary internal GPS receiver.
	NComRxCGps *mGpsExternal;     //!< External GPS receiver.

	// *** Code Generation Begin - NComRxC Structure ***

	//--------------------------------------------------------------------------------------------------------
	// General information

	// Status

	int mIsOutputPacketTypeValid; uint8_t mOutputPacketType; //!< Type of output packet from the decoder.
	int mIsInsNavModeValid; uint8_t mInsNavMode; //!< Navigation system mode.

	// System information

	int mIsSerialNumberValid; int mSerialNumber; //!< Unit serial number.
	int mIsDevIdValid; char mDevId[DEV_ID_STRLEN+1]; //!< Development ID.

	int mIsOsVersion1Valid; int mOsVersion1; //!< Operating system major version.
	int mIsOsVersion2Valid; int mOsVersion2; //!< Operating system minor version.
	int mIsOsVersion3Valid; int mOsVersion3; //!< Operating system revision version.
	int mIsOsScriptIdValid; char mOsScriptId[OS_SCRIPT_ID_STRLEN+1]; //!< Operating system boot up script ID.

	int mIsImuTypeValid; uint8_t mImuType; //!< Type of IMU fitted.
	int mIsCpuPcbTypeValid; uint8_t mCpuPcbType; //!< Type of CPU PCB fitted.
	int mIsInterPcbTypeValid; uint8_t mInterPcbType; //!< Type of interconnection PCB fitted.
	int mIsFrontPcbTypeValid; uint8_t mFrontPcbType; //!< Type of front panel PCB fitted.
	int mIsInterSwIdValid; uint8_t mInterSwId; //!< Software ID of interconnection PCB.
	int mIsHwConfigValid; uint8_t mHwConfig; //!< Reserved for testing.

	int mIsDiskSpaceValid; uint64_t mDiskSpace; //!< Remaining disk space. [B]
	int mIsFileSizeValid; uint64_t mFileSize; //!< Size of current raw data file. [B]
	int mIsUpTimeValid; uint32_t mUpTime; //!< Up time. [s]
	int mIsDualPortRamStatusValid; uint8_t mDualPortRamStatus; //!< Dual port RAM interface status.

	// IMU information

	int mIsUmacStatusValid; uint8_t mUmacStatus; //!< UMAC status.

	// Global Navigation Satellite System (GNSS) information

	int mIsGnssGpsEnabledValid; int mGnssGpsEnabled; //!< Global Positioning System (GPS) enabled.
	int mIsGnssGlonassEnabledValid; int mGnssGlonassEnabled; //!< GLObal NAvigation Satellite System (GLONASS) enabled.
	int mIsGnssGalileoEnabledValid; int mGnssGalileoEnabled; //!< Galileo enabled.
	int mIsGnssBeiDouEnabledValid; int mGnssBeiDouEnabled; //!< BeiDou enabled.

	int mIsPsrDiffEnabledValid; int mPsrDiffEnabled; //!< Pseudo-range differential.
	int mIsSBASEnabledValid; int mSBASEnabled; //!< Satellite Based Augmentation System (SBAS).
	int mIsOmniVBSEnabledValid; int mOmniVBSEnabled; //!< OmniSTAR Virtual Base Station (VBS).
	int mIsOmniHPEnabledValid; int mOmniHPEnabled; //!< OmniSTAR High Performance (HP).
	int mIsL1DiffEnabledValid; int mL1DiffEnabled; //!< L1 carrier-phase differential (RT20).
	int mIsL1L2DiffEnabledValid; int mL1L2DiffEnabled; //!< L1/L2 carrier-phase differential (RT2).

	int mIsRawRngEnabledValid; int mRawRngEnabled; //!< Raw pseudo-range output.
	int mIsRawDopEnabledValid; int mRawDopEnabled; //!< Raw Doppler output.
	int mIsRawL1EnabledValid; int mRawL1Enabled; //!< Raw L1 output.
	int mIsRawL2EnabledValid; int mRawL2Enabled; //!< Raw L2 output.
	int mIsRawL5EnabledValid; int mRawL5Enabled; //!< Raw L5 output.

	int mIsGpsPosModeValid; uint8_t mGpsPosMode; //!< Position mode.
	int mIsGpsVelModeValid; uint8_t mGpsVelMode; //!< Velocity mode.
	int mIsGpsAttModeValid; uint8_t mGpsAttMode; //!< Attitude mode.

	int mIsPDOPValid; double mPDOP; //!< Positional dilution of precision. [-]
	int mIsHDOPValid; double mHDOP; //!< Horizontal dilution of precision. [-]
	int mIsVDOPValid; double mVDOP; //!< Vertical dilution of precision. [-]

	int mIsGpsNumObsValid; int mGpsNumObs; //!< Number of satellites.
	int mIsUndulationValid; double mUndulation; //!< Difference between ellipsoidal altitude and geoidal altitude. [m]

	int mIsBaseStationIdValid; char mBaseStationId[BASE_STATION_ID_STRLEN+1]; //!< Differential base station ID.
	int mIsGpsDiffAgeValid; double mGpsDiffAge; //!< Differential corrections age to GPS. [s]

	// Heading computation status

	int mIsHeadQualityValid; uint8_t mHeadQuality; //!< Dual antenna Heading quality.
	int mIsHeadSearchTypeValid; uint8_t mHeadSearchType; //!< Dual antenna Heading search type.
	int mIsHeadSearchStatusValid; uint8_t mHeadSearchStatus; //!< Dual antenna Heading search status.
	int mIsHeadSearchReadyValid; uint8_t mHeadSearchReady; //!< Dual antenna Heading search ready.

	int mIsHeadSearchInitValid; int mHeadSearchInit; //!< Initial number of ambiguities in the heading search.
	int mIsHeadSearchNumValid; int mHeadSearchNum; //!< Remaining number of ambiguities in the heading search.
	int mIsHeadSearchTimeValid; int mHeadSearchTime; //!< Heading Search Duration. [s]
	int mIsHeadSearchConstrValid; int mHeadSearchConstr; //!< Number of constraints applied in the Heading Search.

	int mIsHeadSearchMasterValid; int mHeadSearchMaster; //!< Master Satellite PRN in the Heading Search.
	int mIsHeadSearchSlave1Valid; int mHeadSearchSlave1; //!< Slave 1 Satellite PRN in the Heading Search.
	int mIsHeadSearchSlave2Valid; int mHeadSearchSlave2; //!< Slave 2 Satellite PRN in the Heading Search.
	int mIsHeadSearchSlave3Valid; int mHeadSearchSlave3; //!< Slave 3 Satellite PRN in the Heading Search.

	// OmniSTAR information

	int mIsOmniStarSerialValid; char mOmniStarSerial[OMNISTAR_SERIAL_STRLEN+1]; //!< OmniSTAR serial string.
	int mIsOmniStarFreqValid; double mOmniStarFreq; //!< OmniSTAR frequency. [Hz]
	int mIsOmniStarSNRValid; double mOmniStarSNR; //!< OmniSTAR signal to noise ratio. [dB]
	int mIsOmniStarLockTimeValid; double mOmniStarLockTime; //!< OmniSTAR lock time. [s]

	int mIsOmniStatusVbsExpiredValid; int mOmniStatusVbsExpired; //!< Virtual Base Station status: Expired.
	int mIsOmniStatusVbsOutOfRegionValid; int mOmniStatusVbsOutOfRegion; //!< Virtual Base Station status: Out of region.
	int mIsOmniStatusVbsNoRemoteSitesValid; int mOmniStatusVbsNoRemoteSites; //!< Virtual Base Station status: No remote sites.

	int mIsOmniStatusHpExpiredValid; int mOmniStatusHpExpired; //!< High Performance status: Expired.
	int mIsOmniStatusHpOutOfRegionValid; int mOmniStatusHpOutOfRegion; //!< High Performance status: Out of region.
	int mIsOmniStatusHpNoRemoteSitesValid; int mOmniStatusHpNoRemoteSites; //!< High Performance status: No remote sites.
	int mIsOmniStatusHpNotConvergedValid; int mOmniStatusHpNotConverged; //!< High Performance status: Not converged.
	int mIsOmniStatusHpKeyInvalidValid; int mOmniStatusHpKeyInvalid; //!< High Performance status: Key is invalid.

	//--------------------------------------------------------------------------------------------------------
	// General user options

	// General options

	int mIsOptionLevelValid; uint8_t mOptionLevel; //!< Vehicle level during initialisation
	int mIsOptionVibrationValid; uint8_t mOptionVibration; //!< Vibration level.
	int mIsOptionGpsAccValid; uint8_t mOptionGpsAcc; //!< GPS environment.
	int mIsOptionUdpValid; uint8_t mOptionUdp; //!< Packet format transmitted over Ethernet.
	int mIsOptionSer1Valid; uint8_t mOptionSer1; //!< Packet format transmitted over serial port 1.
	int mIsOptionSer2Valid; uint8_t mOptionSer2; //!< Packet format transmitted over serial port 2.
	int mIsOptionSer3Valid; uint8_t mOptionSer3; //!< Packet format transmitted over serial port 3.
	int mIsOptionHeadingValid; uint8_t mOptionHeading; //!< Dual antenna heading initialisation mode.

	int mIsOptionInitSpeedValid; int mIsOptionInitSpeedConfig; double mOptionInitSpeed; //!< Initialisation speed. [m s^(-1)]
	int mIsOptionTopSpeedValid; int mIsOptionTopSpeedConfig; double mOptionTopSpeed; //!< Maximum vehicle speed. [m s^(-1)]

	// Output baud rate settings

	int mIsOptionSer1BaudValid; uint8_t mOptionSer1Baud; //!< Serial port 1 baud.

	int mIsOptionSer2BaudValid; uint8_t mOptionSer2Baud; //!< Serial port 2 baud.

	int mIsOptionSer3BaudValid; uint8_t mOptionSer3Baud; //!< Serial port 3 baud.

	int mIsOptionCanBaudValid; uint8_t mOptionCanBaud; //!< Controller area network (CAN) bus baud rate.

	//--------------------------------------------------------------------------------------------------------
	// General measurements

	// Timing

	int mIsTimeValid; double mTime; //!< Seconds from GPS time zero (0h 1980-01-06). [s]

	int mIsTimeWeekCountValid; uint32_t mTimeWeekCount; //!< GPS time format, week counter. [week]
	int mIsTimeWeekSecondValid; double mTimeWeekSecond; //!< GPS time format, seconds into week. [s]
	int mIsTimeUtcOffsetValid; int mTimeUtcOffset; //!< Offset between Coordinated Universal Time (UTC) and GPS time. [s]

	// Position

	int mIsLatValid; int mIsLatApprox; double mLat; //!< Latitude. [deg]
	int mIsLonValid; int mIsLonApprox; double mLon; //!< Longitude. [deg]
	int mIsAltValid; int mIsAltApprox; double mAlt; //!< Altitude. [m]

	int mIsNorthAccValid; double mNorthAcc; //!< North accuracy. [m]
	int mIsEastAccValid; double mEastAcc; //!< East accuracy. [m]
	int mIsAltAccValid; double mAltAcc; //!< Altitude accuracy. [m]

	// Distance

	int mIsDist2dValid; double mDist2d; //!< Distance travelled in horizontal directions. [m]
	int mIsDist3dValid; double mDist3d; //!< Distance travelled in all directions. [m]

	// Velocity

	int mIsVnValid; int mIsVnApprox; double mVn; //!< North velocity. [m s^(-1)]
	int mIsVeValid; int mIsVeApprox; double mVe; //!< East velocity. [m s^(-1)]
	int mIsVdValid; int mIsVdApprox; double mVd; //!< Down velocity. [m s^(-1)]

	int mIsVfValid; double mVf; //!< Forward velocity. [m s^(-1)]
	int mIsVlValid; double mVl; //!< Right velocity. [m s^(-1)]

	int mIsVnAccValid; double mVnAcc; //!< North velocity accuracy. [m s^(-1)]
	int mIsVeAccValid; double mVeAcc; //!< East velocity accuracy. [m s^(-1)]
	int mIsVdAccValid; double mVdAcc; //!< Down velocity accuracy. [m s^(-1)]

	int mIsIsoVnXValid; double mIsoVnX; //!< ISO earth-fixed system east velocity. [m s^(-1)]
	int mIsIsoVnYValid; double mIsoVnY; //!< ISO earth-fixed system north velocity. [m s^(-1)]
	int mIsIsoVnZValid; double mIsoVnZ; //!< ISO earth-fixed system vertical velocity. [m s^(-1)]

	int mIsIsoVhXValid; double mIsoVhX; //!< ISO intermediate system longitudinal velocity. [m s^(-1)]
	int mIsIsoVhYValid; double mIsoVhY; //!< ISO intermediate system lateral velocity. [m s^(-1)]
	int mIsIsoVhZValid; double mIsoVhZ; //!< ISO intermediate system vertical velocity. [m s^(-1)]

	int mIsIsoVoXValid; double mIsoVoX; //!< ISO vehicle system longitudinal velocity. [m s^(-1)]
	int mIsIsoVoYValid; double mIsoVoY; //!< ISO vehicle system lateral velocity. [m s^(-1)]
	int mIsIsoVoZValid; double mIsoVoZ; //!< ISO vehicle system vertical velocity. [m s^(-1)]

	// Speed

	int mIsSpeed2dValid; double mSpeed2d; //!< Speed in horizonal directions. [m s^(-1)]
	int mIsSpeed3dValid; double mSpeed3d; //!< Speed in all directions. [m s^(-1)]

	// Acceleration

	int mIsAxValid; double mAx; //!< IMU acceleration along the X axis. [m s^(-2)]
	int mIsAyValid; double mAy; //!< IMU acceleration along the Y axis. [m s^(-2)]
	int mIsAzValid; double mAz; //!< IMU acceleration along the Z axis. [m s^(-2)]

	int mIsAfValid; double mAf; //!< IMU forward acceleration. [m s^(-2)]
	int mIsAlValid; double mAl; //!< IMU right acceleration. [m s^(-2)]
	int mIsAdValid; double mAd; //!< IMU down acceleration. [m s^(-2)]

	int mIsIsoAnXValid; double mIsoAnX; //!< ISO earth-fixed system east acceleration. [m s^(-2)]
	int mIsIsoAnYValid; double mIsoAnY; //!< ISO earth-fixed system north acceleration. [m s^(-2)]
	int mIsIsoAnZValid; double mIsoAnZ; //!< ISO earth-fixed system vertical acceleration. [m s^(-2)]

	int mIsIsoAhXValid; double mIsoAhX; //!< ISO intermediate system longitudinal acceleration. [m s^(-2)]
	int mIsIsoAhYValid; double mIsoAhY; //!< ISO intermediate system lateral acceleration. [m s^(-2)]
	int mIsIsoAhZValid; double mIsoAhZ; //!< ISO intermediate system vertical acceleration. [m s^(-2)]

	int mIsIsoAoXValid; double mIsoAoX; //!< ISO vehicle system longitudinal acceleration. [m s^(-2)]
	int mIsIsoAoYValid; double mIsoAoY; //!< ISO vehicle system lateral acceleration. [m s^(-2)]
	int mIsIsoAoZValid; double mIsoAoZ; //!< ISO vehicle system vertical acceleration. [m s^(-2)]

	// Filtered acceleration

	int mIsFiltAxValid; double mFiltAx; //!< Filtered IMU acceleration along the X axis. [m s^(-2)]
	int mIsFiltAyValid; double mFiltAy; //!< Filtered IMU acceleration along the Y axis. [m s^(-2)]
	int mIsFiltAzValid; double mFiltAz; //!< Filtered IMU acceleration along the Z axis. [m s^(-2)]

	int mIsFiltAfValid; double mFiltAf; //!< Filtered IMU forward acceleration. [m s^(-2)]
	int mIsFiltAlValid; double mFiltAl; //!< Filtered IMU right acceleration. [m s^(-2)]
	int mIsFiltAdValid; double mFiltAd; //!< Filtered IMU down acceleration. [m s^(-2)]

	int mIsFiltIsoAnXValid; double mFiltIsoAnX; //!< ISO earth-fixed system east filtered acceleration. [m s^(-2)]
	int mIsFiltIsoAnYValid; double mFiltIsoAnY; //!< ISO earth-fixed system north filtered acceleration. [m s^(-2)]
	int mIsFiltIsoAnZValid; double mFiltIsoAnZ; //!< ISO earth-fixed system vertical filtered acceleration. [m s^(-2)]

	int mIsFiltIsoAhXValid; double mFiltIsoAhX; //!< ISO intermediate system longitudinal filtered acceleration. [m s^(-2)]
	int mIsFiltIsoAhYValid; double mFiltIsoAhY; //!< ISO intermediate system lateral filtered acceleration. [m s^(-2)]
	int mIsFiltIsoAhZValid; double mFiltIsoAhZ; //!< ISO intermediate system vertical filtered acceleration. [m s^(-2)]

	int mIsFiltIsoAoXValid; double mFiltIsoAoX; //!< ISO vehicle system longitudinal filtered acceleration. [m s^(-2)]
	int mIsFiltIsoAoYValid; double mFiltIsoAoY; //!< ISO vehicle system lateral filtered acceleration. [m s^(-2)]
	int mIsFiltIsoAoZValid; double mFiltIsoAoZ; //!< ISO vehicle system vertical filtered acceleration. [m s^(-2)]

	// Orientation

	int mIsHeadingValid; int mIsHeadingApprox; double mHeading; //!< Heading. [deg]
	int mIsPitchValid; int mIsPitchApprox; double mPitch; //!< Pitch. [deg]
	int mIsRollValid; int mIsRollApprox; double mRoll; //!< Roll. [deg]

	int mIsHeadingAccValid; double mHeadingAcc; //!< Heading accuracy. [deg]
	int mIsPitchAccValid; double mPitchAcc; //!< Pitch accuracy. [deg]
	int mIsRollAccValid; double mRollAcc; //!< Roll accuracy. [deg]

	int mIsIsoYawValid; double mIsoYaw; //!< ISO yaw angle. [deg]
	int mIsIsoPitchValid; double mIsoPitch; //!< ISO pitch angle. [deg]
	int mIsIsoRollValid; double mIsoRoll; //!< ISO roll angle. [deg]

	// Special

	int mIsTrackValid; double mTrack; //!< Track angle. [deg]

	int mIsSlipValid; double mSlip; //!< Slip angle. [deg]

	int mIsCurvatureValid; double mCurvature; //!< Curvature. [m^(-1)]

	// Angular rate

	int mIsWxValid; double mWx; //!< Angular rate about the X axis. [deg s^(-1)]
	int mIsWyValid; double mWy; //!< Angular rate about the Y axis. [deg s^(-1)]
	int mIsWzValid; double mWz; //!< Angular rate about the Z axis. [deg s^(-1)]

	int mIsWfValid; double mWf; //!< Angular rate about the forward axis. [deg s^(-1)]
	int mIsWlValid; double mWl; //!< Angular rate about the right axis. [deg s^(-1)]
	int mIsWdValid; double mWd; //!< Angular rate about the down axis. [deg s^(-1)]

	int mIsIsoWnXValid; double mIsoWnX; //!< ISO earth-fixed system roll velocity. [deg s^(-1)]
	int mIsIsoWnYValid; double mIsoWnY; //!< ISO earth-fixed system pitch velocity. [deg s^(-1)]
	int mIsIsoWnZValid; double mIsoWnZ; //!< ISO earth-fixed system yaw velocity. [deg s^(-1)]

	int mIsIsoWhXValid; double mIsoWhX; //!< ISO intermediate system roll velocity. [deg s^(-1)]
	int mIsIsoWhYValid; double mIsoWhY; //!< ISO intermediate system pitch velocity. [deg s^(-1)]
	int mIsIsoWhZValid; double mIsoWhZ; //!< ISO intermediate system yaw velocity. [deg s^(-1)]

	int mIsIsoWoXValid; double mIsoWoX; //!< ISO vehicle system roll velocity. [deg s^(-1)]
	int mIsIsoWoYValid; double mIsoWoY; //!< ISO vehicle system pitch velocity. [deg s^(-1)]
	int mIsIsoWoZValid; double mIsoWoZ; //!< ISO vehicle system yaw velocity. [deg s^(-1)]

	// Angular acceleration

	int mIsYxValid; double mYx; //!< Angular acceleration about the X axis. [deg s^(-2)]
	int mIsYyValid; double mYy; //!< Angular acceleration about the Y axis. [deg s^(-2)]
	int mIsYzValid; double mYz; //!< Angular acceleration about the Z axis. [deg s^(-2)]

	int mIsYfValid; double mYf; //!< Angular acceleration about the forward axis. [deg s^(-2)]
	int mIsYlValid; double mYl; //!< Angular acceleration about the right axis. [deg s^(-2)]
	int mIsYdValid; double mYd; //!< Angular acceleration about the down axis. [deg s^(-2)]

	int mIsIsoYnXValid; double mIsoYnX; //!< ISO earth-fixed system roll acceleration. [deg s^(-2)]
	int mIsIsoYnYValid; double mIsoYnY; //!< ISO earth-fixed system pitch acceleration. [deg s^(-2)]
	int mIsIsoYnZValid; double mIsoYnZ; //!< ISO earth-fixed system yaw acceleration. [deg s^(-2)]

	int mIsIsoYhXValid; double mIsoYhX; //!< ISO intermediate system roll acceleration. [deg s^(-2)]
	int mIsIsoYhYValid; double mIsoYhY; //!< ISO intermediate system pitch acceleration. [deg s^(-2)]
	int mIsIsoYhZValid; double mIsoYhZ; //!< ISO intermediate system yaw acceleration. [deg s^(-2)]

	int mIsIsoYoXValid; double mIsoYoX; //!< ISO vehicle system roll acceleration. [deg s^(-2)]
	int mIsIsoYoYValid; double mIsoYoY; //!< ISO vehicle system pitch acceleration. [deg s^(-2)]
	int mIsIsoYoZValid; double mIsoYoZ; //!< ISO vehicle system yaw acceleration. [deg s^(-2)]

	// Filtered angular acceleration

	int mIsFiltYxValid; double mFiltYx; //!< Filtered angular acceleration about the X axis. [deg s^(-2)]
	int mIsFiltYyValid; double mFiltYy; //!< Filtered angular acceleration about the Y axis. [deg s^(-2)]
	int mIsFiltYzValid; double mFiltYz; //!< Filtered angular acceleration about the Z axis. [deg s^(-2)]

	int mIsFiltYfValid; double mFiltYf; //!< Filtered angular acceleration about the forward axis. [deg s^(-2)]
	int mIsFiltYlValid; double mFiltYl; //!< Filtered angular acceleration about the right axis. [deg s^(-2)]
	int mIsFiltYdValid; double mFiltYd; //!< Filtered angular acceleration about the down axis. [deg s^(-2)]

	int mIsFiltIsoYnXValid; double mFiltIsoYnX; //!< ISO earth-fixed system filtered roll acceleration. [deg s^(-2)]
	int mIsFiltIsoYnYValid; double mFiltIsoYnY; //!< ISO earth-fixed system filtered pitch acceleration. [deg s^(-2)]
	int mIsFiltIsoYnZValid; double mFiltIsoYnZ; //!< ISO earth-fixed system filtered yaw acceleration. [deg s^(-2)]

	int mIsFiltIsoYhXValid; double mFiltIsoYhX; //!< ISO intermediate system filtered roll acceleration. [deg s^(-2)]
	int mIsFiltIsoYhYValid; double mFiltIsoYhY; //!< ISO intermediate system filtered pitch acceleration. [deg s^(-2)]
	int mIsFiltIsoYhZValid; double mFiltIsoYhZ; //!< ISO intermediate system filtered yaw acceleration. [deg s^(-2)]

	int mIsFiltIsoYoXValid; double mFiltIsoYoX; //!< ISO vehicle system filtered roll acceleration. [deg s^(-2)]
	int mIsFiltIsoYoYValid; double mFiltIsoYoY; //!< ISO vehicle system filtered pitch acceleration. [deg s^(-2)]
	int mIsFiltIsoYoZValid; double mFiltIsoYoZ; //!< ISO vehicle system filtered yaw acceleration. [deg s^(-2)]

	// Filter characteristics

	int mIsLinAccFiltFreqValid; double mLinAccFiltFreq; //!< Cut-off frequency of linear acceleration low-pass filter. [Hz]
	int mIsLinAccFiltZetaValid; double mLinAccFiltZeta; //!< Damping ratio of linear acceleration low-pass filter. [-]

	int mIsAngAccFiltFreqValid; double mAngAccFiltFreq; //!< Cut-off frequency of angular acceleration low-pass filter. [Hz]
	int mIsAngAccFiltZetaValid; double mAngAccFiltZeta; //!< Damping ratio of angular acceleration low-pass filter. [-]

	//--------------------------------------------------------------------------------------------------------
	// Model particulars

	// Innovations (discrepancy between GPS and IMU)

	int mInnPosXAge; double mInnPosX; //!< Innovation in latitude. [-]
	int mInnPosYAge; double mInnPosY; //!< Innovation in longitude. [-]
	int mInnPosZAge; double mInnPosZ; //!< Innovation in altitude. [-]

	int mInnVelXAge; double mInnVelX; //!< Innovation in north velocity. [-]
	int mInnVelYAge; double mInnVelY; //!< Innovation in east velocity. [-]
	int mInnVelZAge; double mInnVelZ; //!< Innovation in down velocity. [-]

	int mInnHeadingAge; double mInnHeading; //!< Innovation in heading. [-]
	int mInnPitchAge; double mInnPitch; //!< Innovation in pitch. [-]

	// Gyroscope bias and scale factor

	int mIsWxBiasValid; double mWxBias; //!< X gyroscope bias. [deg s^(-1)]
	int mIsWyBiasValid; double mWyBias; //!< Y gyroscope bias. [deg s^(-1)]
	int mIsWzBiasValid; double mWzBias; //!< Z gyroscope bias. [deg s^(-1)]

	int mIsWxBiasAccValid; double mWxBiasAcc; //!< X gyroscope bias accuracy. [deg s^(-1)]
	int mIsWyBiasAccValid; double mWyBiasAcc; //!< Y gyroscope bias accuracy. [deg s^(-1)]
	int mIsWzBiasAccValid; double mWzBiasAcc; //!< Z gyroscope bias accuracy. [deg s^(-1)]

	int mIsWxSfValid; double mWxSf; //!< X gyroscope scale factor deviation. [-]
	int mIsWySfValid; double mWySf; //!< Y gyroscope scale factor deviation. [-]
	int mIsWzSfValid; double mWzSf; //!< Z gyroscope scale factor deviation. [-]

	int mIsWxSfAccValid; double mWxSfAcc; //!< X gyroscope scale factor deviation accuracy. [-]
	int mIsWySfAccValid; double mWySfAcc; //!< Y gyroscope scale factor deviation accuracy. [-]
	int mIsWzSfAccValid; double mWzSfAcc; //!< Z gyroscope scale factor deviation accuracy. [-]

	// Accelerometer bias and scale factor

	int mIsAxBiasValid; double mAxBias; //!< X accelerometer bias. [m s^(-2)]
	int mIsAyBiasValid; double mAyBias; //!< Y accelerometer bias. [m s^(-2)]
	int mIsAzBiasValid; double mAzBias; //!< Z accelerometer bias. [m s^(-2)]

	int mIsAxBiasAccValid; double mAxBiasAcc; //!< X accelerometer bias accuracy. [m s^(-2)]
	int mIsAyBiasAccValid; double mAyBiasAcc; //!< Y accelerometer bias accuracy. [m s^(-2)]
	int mIsAzBiasAccValid; double mAzBiasAcc; //!< Z accelerometer bias accuracy. [m s^(-2)]

	int mIsAxSfValid; double mAxSf; //!< X accelerometer scale factor deviation. [-]
	int mIsAySfValid; double mAySf; //!< Y accelerometer scale factor deviation. [-]
	int mIsAzSfValid; double mAzSf; //!< Z accelerometer scale factor deviation. [-]

	int mIsAxSfAccValid; double mAxSfAcc; //!< X accelerometer scale factor deviation accuracy. [-]
	int mIsAySfAccValid; double mAySfAcc; //!< Y accelerometer scale factor deviation accuracy. [-]
	int mIsAzSfAccValid; double mAzSfAcc; //!< Z accelerometer scale factor deviation accuracy. [-]

	// GNSS antenna position

	int mIsGAPxValid; double mGAPx; //!< Primary GNSS antenna position X offset. [m]
	int mIsGAPyValid; double mGAPy; //!< Primary GNSS antenna position Y offset. [m]
	int mIsGAPzValid; double mGAPz; //!< Primary GNSS antenna position Z offset. [m]

	int mIsGAPxAccValid; double mGAPxAcc; //!< Primary GNSS antenna position X offset accuracy. [m]
	int mIsGAPyAccValid; double mGAPyAcc; //!< Primary GNSS antenna position Y offset accuracy. [m]
	int mIsGAPzAccValid; double mGAPzAcc; //!< Primary GNSS antenna position Z offset accuracy. [m]

	int mIsAtHValid; double mAtH; //!< Secondary GNSS antenna relative heading. [deg]
	int mIsAtPValid; double mAtP; //!< Secondary GNSS antenna relative pitch. [deg]

	int mIsAtHAccValid; double mAtHAcc; //!< Secondary GNSS antenna relative heading accuracy. [deg]
	int mIsAtPAccValid; double mAtPAcc; //!< Secondary GNSS antenna relative pitch accuracy. [deg]

	int mIsBaseLineLengthValid; int mIsBaseLineLengthConfig; double mBaseLineLength; //!< Distance between GNSS antennas. [m]

	int mIsBaseLineLengthAccValid; int mIsBaseLineLengthAccConfig; double mBaseLineLengthAcc; //!< Distance between GNSS antennas accuracy. [m]

	//--------------------------------------------------------------------------------------------------------
	// Statistics

	// IMU hardware status

	int mIsImuMissedPktsValid; uint32_t mImuMissedPkts; //!< Number of IMU hardware missed packets.
	int mIsImuResetCountValid; uint32_t mImuResetCount; //!< Number of IMU hardware resets.
	int mIsImuErrorCountValid; uint32_t mImuErrorCount; //!< Number of IMU hardware errors.

	// GPS successive rejected aiding updates

	int mIsGPSPosRejectValid; uint32_t mGPSPosReject; //!< Number of successive GNSS position updates rejected.
	int mIsGPSVelRejectValid; uint32_t mGPSVelReject; //!< Number of successive GNSS velocity updates rejected.
	int mIsGPSAttRejectValid; uint32_t mGPSAttReject; //!< Number of successive GNSS attitude updates rejected.

	// Received data statistics

	int mIsImuCharsValid; uint32_t mImuChars; //!< IMU number of bytes.
	int mIsImuCharsSkippedValid; uint32_t mImuCharsSkipped; //!< IMU number of invalid bytes.
	int mIsImuPktsValid; uint32_t mImuPkts; //!< IMU number of valid packets.

	int mIsCmdCharsValid; uint32_t mCmdChars; //!< Command number of bytes.
	int mIsCmdCharsSkippedValid; uint32_t mCmdCharsSkipped; //!< Command number of invalid bytes.
	int mIsCmdPktsValid; uint32_t mCmdPkts; //!< Command number of valid packets.
	int mIsCmdErrorsValid; uint32_t mCmdErrors; //!< Command number of errors.

	//--------------------------------------------------------------------------------------------------------
	// Transformation Euler angles

	// Orientation of vehicle-frame relative to IMU-frame

	int mIsImu2VehHeadingValid; double mImu2VehHeading; //!< Heading. [deg]
	int mIsImu2VehPitchValid; double mImu2VehPitch; //!< Pitch. [deg]
	int mIsImu2VehRollValid; double mImu2VehRoll; //!< Roll. [deg]

	// Orientation of output-frame relative to surface-frame

	int mIsSurf2OutHeadingValid; double mSurf2OutHeading; //!< Heading. [deg]
	int mIsSurf2OutPitchValid; double mSurf2OutPitch; //!< Pitch. [deg]
	int mIsSurf2OutRollValid; double mSurf2OutRoll; //!< Roll. [deg]

	// Orientation of surface-frame relative to NED-frame

	int mIsNed2SurfHeadingValid; double mNed2SurfHeading; //!< Heading. [deg]
	int mIsNed2SurfPitchValid; double mNed2SurfPitch; //!< Pitch. [deg]
	int mIsNed2SurfRollValid; double mNed2SurfRoll; //!< Roll. [deg]

	//--------------------------------------------------------------------------------------------------------
	// Miscellaneous items

	// Triggers

	int mIsTrigTimeValid; int mIsTrigTimeNew; double mTrigTime; //!< Time of last trigger falling edge. [s]
	int mIsTrig2TimeValid; int mIsTrig2TimeNew; double mTrig2Time; //!< Time of last trigger rising edge. [s]
	int mIsDigitalOutTimeValid; int mIsDigitalOutTimeNew; double mDigitalOutTime; //!< Time of last digital output. [s]

	// Remote lever arm option

	int mIsRemoteLeverArmXValid; double mRemoteLeverArmX; //!< Remote lever arm X position. [m]
	int mIsRemoteLeverArmYValid; double mRemoteLeverArmY; //!< Remote lever arm Y position. [m]
	int mIsRemoteLeverArmZValid; double mRemoteLeverArmZ; //!< Remote lever arm Z position. [m]

	// Local reference frame (definition)

	int mIsRefLatValid; double mRefLat; //!< Reference frame latitude. [deg]
	int mIsRefLonValid; double mRefLon; //!< Reference frame longitude. [deg]
	int mIsRefAltValid; double mRefAlt; //!< Reference frame altitude. [m]
	int mIsRefHeadingValid; double mRefHeading; //!< Reference frame heading. [deg]

	//--------------------------------------------------------------------------------------------------------
	// Zero velocity

	// Innovations

	int mInnZeroVelXAge; double mInnZeroVelX; //!< Discrepancy from zero of north velocity. [-]
	int mInnZeroVelYAge; double mInnZeroVelY; //!< Discrepancy from zero of east velocity. [-]
	int mInnZeroVelZAge; double mInnZeroVelZ; //!< Discrepancy from zero of downward velocity. [-]

	// Lever arm options

	int mIsZeroVelLeverArmXValid; double mZeroVelLeverArmX; //!< Zero velocity position X offset. [m]
	int mIsZeroVelLeverArmYValid; double mZeroVelLeverArmY; //!< Zero velocity position Y offset. [m]
	int mIsZeroVelLeverArmZValid; double mZeroVelLeverArmZ; //!< Zero velocity position Z offset. [m]

	int mIsZeroVelLeverArmXAccValid; double mZeroVelLeverArmXAcc; //!< Zero velocity position X offset accuracy. [m]
	int mIsZeroVelLeverArmYAccValid; double mZeroVelLeverArmYAcc; //!< Zero velocity position Y offset accuracy. [m]
	int mIsZeroVelLeverArmZAccValid; double mZeroVelLeverArmZAcc; //!< Zero velocity position Z offset accuracy. [m]

	// User Options

	int mIsOptionSZVDelayValid; int mIsOptionSZVDelayConfig; double mOptionSZVDelay; //!< Garage mode setting. [s]
	int mIsOptionSZVPeriodValid; int mIsOptionSZVPeriodConfig; double mOptionSZVPeriod; //!< Garage mode setting. [s]

	//--------------------------------------------------------------------------------------------------------
	// Advanced lateral slip

	// Innovations

	int mInnNoSlipHAge; double mInnNoSlipH; //!< Discrepancy of lateral slip angle. [-]

	// Lever arm options

	int mIsNoSlipLeverArmXValid; double mNoSlipLeverArmX; //!< Lateral no slip position X offset. [m]
	int mIsNoSlipLeverArmYValid; double mNoSlipLeverArmY; //!< Lateral no slip position Y offset. [m]
	int mIsNoSlipLeverArmZValid; double mNoSlipLeverArmZ; //!< Lateral no slip position Z offset. [m]

	int mIsNoSlipLeverArmXAccValid; double mNoSlipLeverArmXAcc; //!< Lateral no slip position X offset accuracy. [m]
	int mIsNoSlipLeverArmYAccValid; double mNoSlipLeverArmYAcc; //!< Lateral no slip position Y offset accuracy. [m]
	int mIsNoSlipLeverArmZAccValid; double mNoSlipLeverArmZAcc; //!< Lateral no slip position Z offset accuracy. [m]

	// User Options

	int mIsOptionNSDelayValid; int mIsOptionNSDelayConfig; double mOptionNSDelay; //!< Advanced lateral slip setting. [s]
	int mIsOptionNSPeriodValid; int mIsOptionNSPeriodConfig; double mOptionNSPeriod; //!< Advanced lateral slip setting. [s]
	int mIsOptionNSAngleStdValid; int mIsOptionNSAngleStdConfig; double mOptionNSAngleStd; //!< Advanced lateral slip setting. [deg]
	int mIsOptionNSHAccelValid; int mIsOptionNSHAccelConfig; double mOptionNSHAccel; //!< Advanced lateral slip setting. [m s^(-2)]
	int mIsOptionNSVAccelValid; int mIsOptionNSVAccelConfig; double mOptionNSVAccel; //!< Advanced lateral slip setting. [m s^(-2)]
	int mIsOptionNSSpeedValid; int mIsOptionNSSpeedConfig; double mOptionNSSpeed; //!< Advanced lateral slip setting. [m s^(-1)]
	int mIsOptionNSRadiusValid; int mIsOptionNSRadiusConfig; double mOptionNSRadius; //!< Advanced lateral slip setting. [m]

	// Measurements

	int mIsHeadingMisAlignValid; double mHeadingMisAlign; //!< Estimated heading offset between unit and vehicle. [deg]
	int mIsHeadingMisAlignAccValid; double mHeadingMisAlignAcc; //!< Estimated accuracy of heading offset between unit and vehicle. [deg]

	//--------------------------------------------------------------------------------------------------------
	// Wheel speed input

	// Innovations

	int mInnWSpeedAge; double mInnWSpeed; //!< Wheel speed innovation. [-]

	// Wheel speed lever arm option

	int mIsWSpeedLeverArmXValid; double mWSpeedLeverArmX; //!< Wheel speed position X offset. [m]
	int mIsWSpeedLeverArmYValid; double mWSpeedLeverArmY; //!< Wheel speed position Y offset. [m]
	int mIsWSpeedLeverArmZValid; double mWSpeedLeverArmZ; //!< Wheel speed position Z offset. [m]

	int mIsWSpeedLeverArmXAccValid; double mWSpeedLeverArmXAcc; //!< Wheel speed position X offset accuracy. [m]
	int mIsWSpeedLeverArmYAccValid; double mWSpeedLeverArmYAcc; //!< Wheel speed position Y offset accuracy. [m]
	int mIsWSpeedLeverArmZAccValid; double mWSpeedLeverArmZAcc; //!< Wheel speed position Z offset accuracy. [m]

	// User Options

	int mIsOptionWSpeedDelayValid; int mIsOptionWSpeedDelayConfig; double mOptionWSpeedDelay; //!< Wheel speed setting. [s]
	int mIsOptionWSpeedZVDelayValid; int mIsOptionWSpeedZVDelayConfig; double mOptionWSpeedZVDelay; //!< Wheel speed setting. [s]
	int mIsOptionWSpeedNoiseStdValid; int mIsOptionWSpeedNoiseStdConfig; double mOptionWSpeedNoiseStd; //!< Wheel speed setting.

	// Measurements

	int mIsWSpeedScaleValid; int mIsWSpeedScaleConfig; double mWSpeedScale; //!< Wheel speed scale factor. [pulse m^(-1)]
	int mIsWSpeedScaleStdValid; int mIsWSpeedScaleStdConfig; double mWSpeedScaleStd; //!< Wheel speed scale factor accuracy. [%]

	int mIsWSpeedTimeValid; double mWSpeedTime; //!< Time of last wheel speed measurement. [s]
	int mIsWSpeedCountValid; double mWSpeedCount; //!< Count at last wheel speed measurement.
	int mIsWSpeedTimeUnchangedValid; double mWSpeedTimeUnchanged; //!< Time since last count. [s]
	int mIsWSpeedFreqValid; double mWSpeedFreq; //!< Wheel speed frequency. [Hz]

	//--------------------------------------------------------------------------------------------------------
	// Heading lock

	// Innovations

	int mInnHeadingHAge; double mInnHeadingH; //!< Heading lock innovation. [-]

	// User options

	int mIsOptionHLDelayValid; int mIsOptionHLDelayConfig; double mOptionHLDelay; //!< Heading lock delay setting. [s]
	int mIsOptionHLPeriodValid; int mIsOptionHLPeriodConfig; double mOptionHLPeriod; //!< Heading lock period setting. [s]
	int mIsOptionHLAngleStdValid; int mIsOptionHLAngleStdConfig; double mOptionHLAngleStd; //!< Heading lock angle accuracy setting. [deg]

	int mIsOptionStatDelayValid; int mIsOptionStatDelayConfig; double mOptionStatDelay; //!< Stationary delay setting. [s]
	int mIsOptionStatSpeedValid; int mIsOptionStatSpeedConfig; double mOptionStatSpeed; //!< Stationary speed setting. [m s^(-1)]

	//--------------------------------------------------------------------------------------------------------
	// For use in testing

	// Reserved for testing

	int mIsTimeMismatchValid; int mTimeMismatch; //!< Time mismatch.
	int mIsImuTimeDiffValid; int mImuTimeDiff; //!< IMU time difference.
	int mIsImuTimeMarginValid; int mImuTimeMargin; //!< IMU time margin.
	int mIsImuLoopTimeValid; int mImuLoopTime; //!< IMU loop time.
	int mIsOpLoopTimeValid; int mOpLoopTime; //!< Output loop time.

	int mIsBnsLagValid; int mBnsLag; //!< Blended navigation system lag.
	int mIsBnsLagFiltValid; double mBnsLagFilt; //!< Filtered blended navigation system lag.

	// *** Code Generation End - NComRxC Structure ***

	//--------------------------------------------------------------------------------------------------------
	// Decoder private area.

	NComRxCInternal *mInternal;  //!< Private decoder state and work space.

} NComRxC;



#ifdef __cplusplus
extern "C"
{
#endif


//============================================================================================================
// NComRxC access functions to return a textual description of values (such enumerated types) used in NComRxC.
// For information about function NComGet<Thing>String see comments regarding <Thing> defined in the NComRxC
// structure.

// *** Code Generation Begin - NComRxC External String Functions ***

//------------------------------------------------------------------------------------------------------------
// General information

// Status

extern const char *NComGetOutputPacketTypeString(const NComRxC *Com);
extern const char *NComGetInsNavModeString(const NComRxC *Com);

// System information

extern const char *NComGetImuTypeString(const NComRxC *Com);
extern const char *NComGetCpuPcbTypeString(const NComRxC *Com);
extern const char *NComGetInterPcbTypeString(const NComRxC *Com);
extern const char *NComGetFrontPcbTypeString(const NComRxC *Com);
extern const char *NComGetInterSwIdString(const NComRxC *Com);
extern const char *NComGetHwConfigString(const NComRxC *Com);

extern const char *NComGetDualPortRamStatusString(const NComRxC *Com);

// IMU information

extern const char *NComGetUmacStatusString(const NComRxC *Com);

// Global Navigation Satellite System (GNSS) information

extern const char *NComGetGpsPosModeString(const NComRxC *Com);
extern const char *NComGetGpsVelModeString(const NComRxC *Com);
extern const char *NComGetGpsAttModeString(const NComRxC *Com);

// Heading computation status

extern const char *NComGetHeadQualityString(const NComRxC *Com);
extern const char *NComGetHeadSearchTypeString(const NComRxC *Com);
extern const char *NComGetHeadSearchStatusString(const NComRxC *Com);
extern const char *NComGetHeadSearchReadyString(const NComRxC *Com);

//------------------------------------------------------------------------------------------------------------
// General user options

// General options

extern const char *NComGetOptionLevelString(const NComRxC *Com);
extern const char *NComGetOptionVibrationString(const NComRxC *Com);
extern const char *NComGetOptionGpsAccString(const NComRxC *Com);
extern const char *NComGetOptionUdpString(const NComRxC *Com);
extern const char *NComGetOptionSer1String(const NComRxC *Com);
extern const char *NComGetOptionSer2String(const NComRxC *Com);
extern const char *NComGetOptionSer3String(const NComRxC *Com);
extern const char *NComGetOptionHeadingString(const NComRxC *Com);

// Output baud rate settings

extern const char *NComGetOptionSer1BaudString(const NComRxC *Com);

extern const char *NComGetOptionSer2BaudString(const NComRxC *Com);

extern const char *NComGetOptionSer3BaudString(const NComRxC *Com);

extern const char *NComGetOptionCanBaudString(const NComRxC *Com);

// *** Code Generation End - NComRxC External String Functions ***


//============================================================================================================
// General function declarations.

extern void                 NComInvalidate(NComRxC *Com);
extern NComRxC             *NComCreateNComRxC();
extern void                 NComDestroyNComRxC(NComRxC *Com);
extern void                 NComCopy(NComRxC *ComDestination, const NComRxC *ComSource);
extern ComResponse          NComNewChar(NComRxC *Com, unsigned char c);
extern ComResponse          NComNewChars(NComRxC *Com, const unsigned char *data, int num);
extern uint64_t             NComNumChars(const NComRxC *Com);
extern uint64_t             NComSkippedChars(const NComRxC *Com);
extern uint64_t             NComNumPackets(const NComRxC *Com);
extern unsigned int         NComGetCurrentPacketSize(const NComRxC *Com);
extern const unsigned char *NComGetCurrentPacketData(const NComRxC *Com);
extern int                  NComGetCurrentStatusChannel(const NComRxC *Com);
extern unsigned int         NComGetCurrentStatusPacketSize(const NComRxC *Com);
extern const unsigned char *NComGetCurrentStatusPacketData(const NComRxC *Com);
extern void                 NComUpdateInnAge(NComRxC *Com);
extern void                 NComInterpolate(NComRxC *Com, double a, const NComRxC *A, double b, const NComRxC *B);


#ifdef __cplusplus
}
#endif




//============================================================================================================
// Backwards compatibility.
//
// This decoder is not quite compatible with some older versions, hence the change to the type NComRxC from
// NComRx. The changes are slight, however, so one may wish to continue using the older type NComRx. To do so
// enable the following type definition.

// typedef NComRxC NComRx;


#endif
