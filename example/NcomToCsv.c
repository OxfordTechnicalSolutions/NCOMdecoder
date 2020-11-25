//============================================================================================================
//!
//! \file main.c
//!
//! \brief Program for converting NCom files to text. Also demonstrates how to use the C NCom libraries.
//!
//!
//! Last edited by: HH 161005
//!
//============================================================================================================

#define MAIN_DEV_ID "161005"  //!< Development identification.
#define MPS2KMPH           (3.6)        //!< m/s to km/h conversion constant


#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <wchar.h>
#include <locale.h>

#include "NComRxC.h"


//============================================================================================================
// Prototypes for some helper functions.

static void report(const NComRxC *nrx);
static void print(FILE *fp, const NComRxC *nrx);


//============================================================================================================
//! \brief Function where the program starts execution.
//!
//! Program for converting NCom files to text.
//!
//! \return Exit status.

int main(
		int argc,           //!< Number of arguments supplied to the program.
		const char *argv[]  //!< Array of arguments supplied to the program.
	)
{
	setlocale(LC_CTYPE, ""); //Allows printing non-ASCII characters

	FILE *fpin   = NULL;  // input file
	FILE *fpout  = NULL;  // output file
	FILE *fptrig = NULL;  // optional trigger file

	int c;                // char from input file
	NComRxC *nrx;         // NComRxC object

	// Output the header and the Development ID
	printf("NcomToCsv: Converts NCom file data to text. (ID: " MAIN_DEV_ID ")\n");

	// Check the command line for 2 or 3 user parameters
	if(argc !=3 && argc != 4)
	{
		fprintf(stderr, "Usage: NcomToCsv <input file> <output file> [<trig_file>]\n");
		exit(EXIT_FAILURE);
	}

	// Open the input file
	fpin = fopen(argv[1], "rb");
	if(fpin == NULL)
	{
		fprintf(stderr, "Error: Could not open input file '%s'.\n", argv[1]);
		exit(EXIT_FAILURE);
	}

	// Open the output file
	fpout = fopen(argv[2], "wt");
	if(fpout == NULL)
	{
		fprintf(stderr, "Error: Could not open output file '%s'.\n", argv[2]);
		if(fpin != NULL) fclose(fpin);
		exit(EXIT_FAILURE);
	}

	// Open the (optional) output trigger text file
	if(argc >= 4)
	{
		fptrig = fopen(argv[3], "wt");
		if(fptrig == NULL)
		{
			fprintf(stderr, "Error: Could not open output trigger file '%s'.\n", argv[3]);
			if(fpin  != NULL) fclose(fpin);
			if(fpout != NULL) fclose(fpout);
			exit(EXIT_FAILURE);
		}
	}

	// Create NCom decoder and check
	nrx = NComCreateNComRxC();
	if(nrx == NULL)
	{
		fprintf(stderr, "Error: Unable to create NCom decoder.\n");
		if(fpin   != NULL) fclose(fpin);
		if(fpout  != NULL) fclose(fpout);
		if(fptrig != NULL) fclose(fptrig);
		exit(EXIT_FAILURE);
	}

	// Read all of the input file and convert to text
	while((c = fgetc(fpin)) != EOF)
	{
		// Decode the data
		if(NComNewChar(nrx, (unsigned char) c) == COM_NEW_UPDATE)
		{
			// For regular updates then output to main output file, otherwise,
			// for falling edge input triggers then output to trigger file.
			switch(nrx->mOutputPacketType)
			{
				case OUTPUT_PACKET_REGULAR : print(fpout,  nrx); break;
				case OUTPUT_PACKET_IN1DOWN : print(fptrig, nrx); break;
				default : break;
			}
		}

		// Report some statistics every 4096 chars processed
		if( (NComNumChars(nrx) & UINT32_C(0xFFF)) == 0) report(nrx);
	}

	// Report final statistics
	report(nrx);
	printf("\n");

	// Clean up
	if(fpin   != NULL) fclose(fpin);
	if(fpout  != NULL) fclose(fpout);
	if(fptrig != NULL) fclose(fptrig);
	NComDestroyNComRxC(nrx);

	return EXIT_SUCCESS;
}


//============================================================================================================
//! \brief Simple decoding progress report.

static void report(const NComRxC *nrx)
{
	assert(nrx);

	printf("\rChars Read %" PRIu64 ", Packets Read %" PRIu64 ", Chars Skipped %" PRIu64,
		NComNumChars(nrx), NComNumPackets(nrx), NComSkippedChars(nrx));

	fflush(stdout);
}


//============================================================================================================
//! \brief Used to write some of the NCom data to a file pointer.
//!
//! There are only a few examples here of how to use the data values.

static void print(FILE *fp, const NComRxC *nrx)
{
	static int HeaderWritten = 0;
	assert(fp);
	assert(nrx);

	// Add in the headers to the file
	if (HeaderWritten == 0)
	{
#if __linux__
		fprintf(fp," Time(),"
#else
		fwprintf(fp, L" Time(),"
#endif
					" Date,"
					" Local Time,"
					" Latitude(deg),"
					" Longitude(deg),"
					" Altitude(m),"
					" Distance horizontal(m),"
					" Velocity north(m/s),"
					" Velocity east(m/s),"
					" Velocity down(m/s),"
					" Velocity up(m/s),"
					" Velocity forward(m/s),"
					" Velocity lateral(m/s),"
					" ISO e.f.s. east velocity(m/s),"
					" ISO e.f.s. north velocity(m/s),"
					" ISO e.f.s. vertical velocity(m/s),"
					" ISO i.s. longitudinal velocity(m/s),"
					" ISO i.s. lateral velocity(m/s),"
					" ISO i.s. vertical velocity(m/s),"
					" ISO v.s. longitudinal velocity(m/s),"
					" ISO v.s. lateral velocity(m/s),"
					" ISO v.s. vertical velocity(m/s),"
					" Speed horizontal(m/s),"
					" Acceleration Xv(m/s²),"
					" Acceleration Yv(m/s²),"
					" Acceleration Zv(m/s²),"
					" Acceleration forward(m/s²),"
					" Acceleration lateral(m/s²),"
					" Acceleration down(m/s²),"
					" ISO e.f.s. east acceleration(m/s²),"
					" ISO e.f.s. north acceleration(m/s²),"
					" ISO e.f.s. vertical acceleration(m/s²),"
					" ISO i.s. longitudinal acceleration(m/s²),"
					" ISO i.s. lateral acceleration(m/s²),"
					" ISO i.s. vertical acceleration(m/s²),"
					" ISO v.s. longitudinal acceleration(m/s²),"
					" ISO v.s. lateral acceleration(m/s²),"
					" ISO v.s. vertical acceleration(m/s²),"
					" Heading(deg),"
					" Pitch(deg),"
					" Roll(deg),"
					" ISO yaw angle(deg),"
					" ISO pitch angle(deg),"
					" ISO roll angle(deg),"
					" Angular rate Xv(deg/s),"
					" Angular rate Yv(deg/s),"
					" Angular rate Zv(deg/s),"
					" Angular rate forward(deg/s),"
					" Angular rate lateral(deg/s),"
					" Angular rate down(deg/s),"
					" ISO e.f.s. roll velocity(deg/s),"
					" ISO e.f.s. pitch velocity(deg/s),"
					" ISO e.f.s. yaw velocity(deg/s),"
					" ISO i.s. roll velocity(deg/s),"
					" ISO i.s. pitch velocity(deg/s),"
					" ISO i.s. yaw velocity(deg/s),"
					" ISO v.s. roll velocity(deg/s),"
					" ISO v.s. pitch velocity(deg/s),"
					" ISO v.s. yaw velocity(deg/s),"
					" Angular acceleration Xv(deg/s²),"
					" Angular acceleration Yv(deg/s²),"
					" Angular acceleration Zv(deg/s²),"
					" Angular acceleration forward(deg/s²),"
					" Angular acceleration lateral(deg/s²),"
					" Angular acceleration down(deg/s²),"
					" ISO e.f.s. roll acceleration(deg/s²),"
					" ISO e.f.s. pitch acceleration(deg/s²),"
					" ISO e.f.s. yaw acceleration(deg/s²),"
					" ISO i.s. roll acceleration(deg/s²),"
					" ISO i.s. pitch acceleration(deg/s²),"
					" ISO i.s. yaw acceleration(deg/s²),"
					" ISO v.s. roll acceleration(deg/s²),"
					" ISO v.s. pitch acceleration(deg/s²),"
					" ISO v.s. yaw acceleration(deg/s²),"
					// " Acceleration filtered Xv(m/s²),"
					// " Acceleration filtered Yv(m/s²),"
					// " Acceleration filtered Zv(m/s²),"
					// " Acceleration filtered forward(m/s²),"
					// " Acceleration filtered lateral(m/s²),"
					// " Acceleration filtered down(m/s²),"
					// " ISO e.f.s. east filtered acceleration(m/s²),"
					// " ISO e.f.s. north filtered acceleration(m/s²),"
					// " ISO e.f.s. vertical filtered acceleration(m/s²),"
					// " ISO i.s. longitudinal filtered acceleration(m/s²),"
					// " ISO i.s. lateral filtered acceleration(m/s²),"
					// " ISO i.s. vertical filtered acceleration(m/s²),"
					// " ISO v.s. longitudinal filtered acceleration(m/s²),"
					// " ISO v.s. lateral filtered acceleration(m/s²),"
					// " ISO v.s. vertical filtered acceleration(m/s²),"
					// " Angular acceleration filtered Xv(deg/s²),"
					// " Angular acceleration filtered Yv(deg/s²),"
					// " Angular acceleration filtered Zv(deg/s²),"
					// " Angular acceleration filtered forward(deg/s²),"
					// " Angular acceleration filtered lateral(deg/s²),"
					// " Angular acceleration filtered down(deg/s²),"
					// " ISO e.f.s. roll filtered acceleration(deg/s²),"
					// " ISO e.f.s. pitch filtered acceleration(deg/s²),"
					// " ISO e.f.s. yaw filtered acceleration(deg/s²),"
					// " ISO i.s. roll filtered acceleration(deg/s²),"
					// " ISO i.s. pitch filtered acceleration(deg/s²),"
					// " ISO i.s. yaw filtered acceleration(deg/s²),"
					// " ISO v.s. roll filtered acceleration(deg/s²),"
					// " ISO v.s. pitch filtered acceleration(deg/s²),"
					// " ISO v.s. yaw filtered acceleration(deg/s²),"

		);
	  fprintf(fp, "\n");
		HeaderWritten = 1;
	}

	// Print the time - GPS time, local date and time zone.
	if (nrx->mIsTimeValid)
	{
		double     gps2machine, mMachineTime;
		time_t     t1;
		struct tm *td;
		int        ms;

		// Convert GPS seconds (from 1980-01-06 00:00:00) to machine seconds (from 1970-01-01 00:00:00). It is
		// very likely the machine will adjust for leap seconds, hence the correct GPS UTC difference is
		// applied. If the local machine time does not start from 1970-01-01 00:00:00 then the value of
		// gps2machine below needs to change.
		gps2machine  = 315964800.0;

		if(nrx->mIsTimeUtcOffsetValid)
		{ mMachineTime = nrx->mTime + gps2machine + nrx->mTimeUtcOffset;
		} else {mMachineTime = nrx->mTime + gps2machine - 17;}

		// Compute local time
		t1 = (time_t) floor(mMachineTime);
		td = localtime(&t1);
		ms = floor(0.5 + (mMachineTime - t1) * 1000.0);
		if(ms < 0) ms = 0; else if(ms > 999) ms = 999;

		// Print: GPS time, local date, time zone.
		fprintf(fp, "%10.3f,%04d-%02d-%02d,%02d:%02d:%02d.%03d,",
			nrx->mTime,
			1900+td->tm_year, 1+td->tm_mon, td->tm_mday, td->tm_hour, td->tm_min, td->tm_sec, ms);
	}

	// Print the measurments listed in the header
	if (nrx->mIsTimeValid)
	{
		// Print the 	PosLat (deg)
		if(nrx->mIsLatValid) fprintf(fp, "%.8f", nrx->mLat);
		fprintf(fp, ",");

		// Print the 	PosLon (deg)
		if(nrx->mIsLonValid) fprintf(fp, "%.8f", nrx->mLon);
		fprintf(fp, ",");

		// Print the 	PosAlt (m)
		if(nrx->mIsAltValid) fprintf(fp, "%.3f",nrx->mAlt);
		fprintf(fp, ",");

		// Print the 	Distance (m)
		if(nrx->mIsDist2dValid) fprintf(fp, "%.3f",nrx->mDist2d);
		fprintf(fp, ",");

		// Print the 	VelNorth (km/h)
		if(nrx->mIsVnValid) fprintf(fp, "%.3f",nrx->mVn);
		fprintf(fp, ",");

		// Print the 	VelEast (km/h)
		if(nrx->mIsVeValid) fprintf(fp, "%.3f",nrx->mVe);
		fprintf(fp, ",");

		// Print the 	VelDown (km/h)
		if(nrx->mIsVdValid) fprintf(fp, "%.3f",nrx->mVd);
		fprintf(fp, ",");

		// Print the 	Velup (km/h)
		fprintf(fp, "-,");

		// Print the 	VelForward (km/h)
		if(nrx->mIsVfValid) fprintf(fp, "%.3f",nrx->mVf);
		fprintf(fp, ",");

		// Print the 	VelLateral (km/h)
		if(nrx->mIsVlValid) fprintf(fp, "%.3f",nrx->mVl);
		fprintf(fp, ",");

		// Print the "ISO e.f.s. east velocity(m/s),"
		if(nrx->mIsIsoVnXValid) fprintf(fp, "%.2f",nrx->mIsoVnX);
		fprintf(fp, ",");

		// Print the "ISO e.f.s. north velocity(m/s),"
		if(nrx->mIsIsoVnYValid) fprintf(fp, "%.2f",nrx->mIsoVnY);
		fprintf(fp, ",");

		// Print the "ISO e.f.s. vertical velocity (m/s)"
		if(nrx->mIsIsoVnZValid) fprintf(fp, "%.2f",nrx->mIsoVnZ);
		fprintf(fp, ",");

		// Print the "ISO i.s. longitudinal velocity(m/s²),"
		if(nrx->mIsIsoVhXValid) fprintf(fp, "%.2f",nrx->mIsoVhX);
		fprintf(fp, ",");

		// Print the "ISO i.s. lateral velocity(m/s²),"
		if(nrx->mIsIsoVhYValid) fprintf(fp, "%.2f",nrx->mIsoVhY);
		fprintf(fp, ",");

		// Print the "ISO i.s. vertical velocity(m/s²)"
		if(nrx->mIsIsoVhZValid) fprintf(fp, "%.2f",nrx->mIsoVhZ);
		fprintf(fp, ",");

		// Print the "ISO v.s. longitudinal velocity(m/s²),"
		if(nrx->mIsIsoVoXValid) fprintf(fp, "%.2f",nrx->mIsoVoX);
		fprintf(fp, ",");

		// Print the "ISO v.s. lateral velocity(m/s²),"
		if(nrx->mIsIsoVoYValid) fprintf(fp, "%.2f",nrx->mIsoVoY);
		fprintf(fp, ",");

		// Print the "ISO v.s. vertical velocity(m/s²)"
		if(nrx->mIsIsoVoZValid) fprintf(fp, "%.2f",nrx->mIsoVoZ);
		fprintf(fp, ",");

		//Print the Speed2D (m/s)
		if(nrx->mIsSpeed2dValid) fprintf(fp, "%.2f", nrx->mSpeed2d);
		fprintf(fp, ",");

		// Print the 	AccelX (m/s²)
		if(nrx->mIsAxValid) fprintf(fp, "%.3f",nrx->mAx);
		fprintf(fp, ",");

		// Print the 	AccelY (m/s²)
		if(nrx->mIsAyValid) fprintf(fp, "%.3f",nrx->mAy);
		fprintf(fp, ",");

		// Print the 	AccelZ (m/s²)
		if(nrx->mIsAzValid) fprintf(fp, "%.3f",nrx->mAz);
		fprintf(fp, ",");

		// Print the 	AccelForward (m/s²)
		if(nrx->mIsAfValid) fprintf(fp, "%.3f",nrx->mAf);
		fprintf(fp, ",");

		// Print the 	AccelLateral (m/s²)
		if(nrx->mIsAlValid) fprintf(fp, "%.3f",nrx->mAl);
		fprintf(fp, ",");

		// Print the 	AccelDown (m/s²)
		if(nrx->mIsAdValid) fprintf(fp, "%.3f",nrx->mAd);
		fprintf(fp, ",");

		// Print the "ISO e.f.s. east acceleration(m/s²),"
		if(nrx->mIsIsoAnXValid) fprintf(fp, "%.2f",nrx->mIsoAnX);
		fprintf(fp, ",");

		// Print the "ISO e.f.s. north acceleration(m/s²),"
		if(nrx->mIsIsoAnYValid) fprintf(fp, "%.2f",nrx->mIsoAnY);
		fprintf(fp, ",");

		// Print the "ISO e.f.s. vertical acceleration(m/s²)"
		if(nrx->mIsIsoAnZValid) fprintf(fp, "%.2f",nrx->mIsoAnZ);
		fprintf(fp, ",");

		// Print the "ISO i.s. longitudinal acceleration(m/s²),"
		if(nrx->mIsIsoAhXValid) fprintf(fp, "%.2f",nrx->mIsoAhX);
		fprintf(fp, ",");

		// Print the "ISO i.s. lateral acceleration(m/s²),"
		if(nrx->mIsIsoAhYValid) fprintf(fp, "%.2f",nrx->mIsoAhY);
		fprintf(fp, ",");

		// Print the "ISO i.s. vertical acceleration(m/s²)"
		if(nrx->mIsIsoAhZValid) fprintf(fp, "%.2f",nrx->mIsoAhZ);
		fprintf(fp, ",");

		// Print the "ISO v.s. longitudinal acceleration(m/s²),"
		if(nrx->mIsIsoAoXValid) fprintf(fp, "%.2f",nrx->mIsoAoX);
		fprintf(fp, ",");

		// Print the "ISO v.s. lateral acceleration(m/s²),"
		if(nrx->mIsIsoAoYValid) fprintf(fp, "%.2f",nrx->mIsoAoY);
		fprintf(fp, ",");

		// Print the "ISO v.s. vertical acceleration(m/s²)"
		if(nrx->mIsIsoAoZValid) fprintf(fp, "%.2f",nrx->mIsoAoZ);
		fprintf(fp, ",");

		// Print the 	AngleHeading (deg)
		if(nrx->mIsHeadingValid) fprintf(fp, "%.3f",nrx->mHeading);
		fprintf(fp, ",");

		// Print the 	AnglePitch (deg)
		if(nrx->mIsPitchValid) fprintf(fp, "%.3f",nrx->mPitch);
		fprintf(fp, ",");

		// Print the 	AngleRoll (deg)
		if(nrx->mIsRollValid) fprintf(fp, "%.3f",nrx->mRoll);
		fprintf(fp, ",");

		// Print the  ISO Yaw
		if(nrx->mIsIsoYawValid) fprintf(fp, "%.2f",nrx->mIsoYaw);
		fprintf(fp, ",");

		// Print the ISO Pitch
		if(nrx->mIsIsoPitchValid) fprintf(fp, "%.2f",nrx->mIsoPitch);
		fprintf(fp, ",");

		// Print the  ISO Roll
		if(nrx->mIsIsoRollValid) fprintf(fp, "%.2f",nrx->mIsoRoll);
		fprintf(fp, ",");

		// // Print the 	AngleSlip (deg)
		// if(nrx->mIsSlipValid) fprintf(fp, "%.3f",nrx->mSlip);
		// fprintf(fp, ",");

		// // Print the 	AngleTrack (deg) edited for +ve measurments only
		// if(nrx->mIsTrackValid)
		// {
		// 	if(nrx->mTrack<0) fprintf(fp, "%.3f",360 + nrx->mTrack); else fprintf(fp, "%.3f",nrx->mTrack);
		// }
		// fprintf(fp, ",");

		// // Print the 	Curvature (1/m)
		// if(nrx->mIsCurvatureValid) fprintf(fp, "%.4f",nrx->mCurvature);
		// fprintf(fp, ",");

		// Print the 	AngleLocalYaw (deg)

		// Print the 	AngleRateX (deg/s)
		if(nrx->mIsWxValid) fprintf(fp, "%.3f",nrx->mWx);
		fprintf(fp, ",");

		// Print the 	AngleRateY (deg/s)
		if(nrx->mIsWyValid) fprintf(fp, "%.3f",nrx->mWy);
		fprintf(fp, ",");

		// Print the 	AngleRateZ (deg/s)
		if(nrx->mIsWzValid) fprintf(fp, "%.3f",nrx->mWz);
		fprintf(fp, ",");

		// Print the 	AngleRateForward (deg/s)
		if(nrx->mIsWfValid) fprintf(fp, "%.3f",nrx->mWf);
		fprintf(fp, ",");

		// Print the 	AngleRateLateral (deg/s)
		if(nrx->mIsWlValid) fprintf(fp, "%.3f",nrx->mWl);
		fprintf(fp, ",");

		// Print the 	AngleRateDown (deg/s)
		if(nrx->mIsWdValid) fprintf(fp, "%.3f",nrx->mWd);
		fprintf(fp, ",");

		//Print the "ISO e.f.s. roll velocity(deg/s),"
		if(nrx->mIsIsoWnXValid) fprintf(fp, "%.3f",nrx->mIsoWnX);
		fprintf(fp, ",");

		//Print the "ISO e.f.s. pitch velocity(deg/s),"
		if(nrx->mIsIsoWnYValid) fprintf(fp, "%.3f",nrx->mIsoWnY);
		fprintf(fp, ",");

		// Print the "ISO e.f.s. yaw velocity(deg/s),"
		if(nrx->mIsIsoWnZValid) fprintf(fp, "%.3f",nrx->mIsoWnZ);
		fprintf(fp, ",");

		//Print the "ISO i.s. roll velocity(deg/s),"
		if(nrx->mIsIsoWhXValid) fprintf(fp, "%.3f",nrx->mIsoWhX);
		fprintf(fp, ",");

		//Print "ISO i.s. pitch velocity(deg/s),"
		if(nrx->mIsIsoWhYValid) fprintf(fp, "%.3f",nrx->mIsoWhY);
		fprintf(fp, ",");

		//Print "ISO i.s. yaw velocity(deg/s),"
		if(nrx->mIsIsoWhZValid) fprintf(fp, "%.3f",nrx->mIsoWhZ);
		fprintf(fp, ",");

		//Print "ISO v.s. roll velocity(deg/s),"
		if(nrx->mIsIsoWoXValid) fprintf(fp, "%.3f",nrx->mIsoWoX);
		fprintf(fp, ",");

		//Print "ISO v.s. pitch velocity(deg/s),"
		if(nrx->mIsIsoWoYValid) fprintf(fp, "%.3f",nrx->mIsoWoY);
		fprintf(fp, ",");

		//Print "ISO v.s. yaw velocity(deg/s),"
		if(nrx->mIsIsoWoZValid) fprintf(fp, "%.3f",nrx->mIsoWoZ);
		fprintf(fp, ",");

		//Print "Angular acceleration Xv(deg/s²),"
		if(nrx->mIsYxValid) fprintf(fp, "%.3f",nrx->mYx);
		fprintf(fp, ",");

		//Print "Angular acceleration Yv(deg/s²),"
		if(nrx->mIsYyValid) fprintf(fp, "%.3f",nrx->mYy);
		fprintf(fp, ",");

		//Print "Angular acceleration Zv(deg/s²),"
		if(nrx->mIsYzValid) fprintf(fp, "%.3f",nrx->mYz);
		fprintf(fp, ",");

		//Print "Angular acceleration forward(deg/s²),"
		if(nrx->mIsYfValid) fprintf(fp, "%.3f",nrx->mYf);
		fprintf(fp, ",");

		//Print "Angular acceleration lateral(deg/s²),"
		if(nrx->mIsYlValid) fprintf(fp, "%.3f",nrx->mYl);
		fprintf(fp, ",");

		//Print "Angular acceleration down(deg/s²),"
		if(nrx->mIsYdValid) fprintf(fp, "%.3f",nrx->mYd);
		fprintf(fp, ",");

		//Print "ISO e.f.s. roll acceleration(deg/s²),"
		if(nrx->mIsIsoYnXValid) fprintf(fp, "%.3f",nrx->mIsoYnX);
		fprintf(fp, ",");

		//Print "ISO e.f.s. pitch acceleration(deg/s²),"
		if(nrx->mIsIsoYnYValid) fprintf(fp, "%.3f",nrx->mIsoYnY);
		fprintf(fp, ",");

		//Print "ISO e.f.s. yaw acceleration(deg/s²),"
		if(nrx->mIsIsoYnZValid) fprintf(fp, "%.3f",nrx->mIsoYnZ);
		fprintf(fp, ",");

		//Print "ISO i.s. roll acceleration(deg/s²),"
		if(nrx->mIsIsoYhXValid) fprintf(fp, "%.3f",nrx->mIsoYhX);
		fprintf(fp, ",");

		//Print "ISO i.s. pitch acceleration(deg/s²),"
		if(nrx->mIsIsoYhYValid) fprintf(fp, "%.3f",nrx->mIsoYhY);
		fprintf(fp, ",");

		//Print "ISO i.s. yaw acceleration(deg/s²),"
		if(nrx->mIsIsoYhZValid) fprintf(fp, "%.3f",nrx->mIsoYhZ);
		fprintf(fp, ",");

		//Print "ISO v.s. roll acceleration(deg/s²),"
		if(nrx->mIsIsoYoXValid) fprintf(fp, "%.3f",nrx->mIsoYoX);
		fprintf(fp, ",");

		//Print "ISO v.s. pitch acceleration(deg/s²),"
		if(nrx->mIsIsoYoYValid) fprintf(fp, "%.3f",nrx->mIsoYoY);
		fprintf(fp, ",");

		//Print "ISO v.s. yaw acceleration(deg/s²),"
		if(nrx->mIsIsoYoZValid) fprintf(fp, "%.3f",nrx->mIsoYoZ);
		fprintf(fp, ",");



		// // Print the 	filtered	AccelX (m/s²)
		// if(nrx->mIsFiltAxValid) fprintf(fp, "%.3f",nrx->mFiltAx);
		// fprintf(fp, ",");
		//
		// // Print the 	filtered	AccelY (m/s²)
		// if(nrx->mIsFiltAyValid) fprintf(fp, "%.3f",nrx->mFiltAy);
		// fprintf(fp, ",");
		//
		// // Print the 	filtered	AccelZ (m/s²)
		// if(nrx->mIsFiltAzValid) fprintf(fp, "%.3f",nrx->mFiltAz);
		// fprintf(fp, ",");
		//
		// // Print the 	filtered	AccelForward (m/s²)
		// if(nrx->mIsFiltAfValid) fprintf(fp, "%.3f",nrx->mFiltAf);
		// fprintf(fp, ",");
		//
		// // Print the 	filtered	AccelLateral (m/s²)
		// if(nrx->mIsFiltAlValid) fprintf(fp, "%.3f",nrx->mFiltAl);
		// fprintf(fp, ",");
		//
		// // Print the 	filtered	AccelDown (m/s²)
		// if(nrx->mIsFiltAdValid) fprintf(fp, "%.3f",nrx->mFiltAd);
		// fprintf(fp, ",");
		//
		//
		//
		// // Print the 	filtered AccelX (m/s²)
		// if(nrx->mIsFiltAxValid) fprintf(fp, "%.3f",nrx->mFiltAx);
		// fprintf(fp, ",");
		//
		// // Print the filtered	AccelY (m/s²)
		// if(nrx->mIsFiltAyValid) fprintf(fp, "%.3f",nrx->mFiltAy);
		// fprintf(fp, ",");
		//
		// // Print the 	filtered	AccelZ (m/s²)
		// if(nrx->mIsFiltAzValid) fprintf(fp, "%.3f",nrx->mFiltAz);
		// fprintf(fp, ",");
		//
		// // Print the 	filtered	AccelForward (m/s²)
		// if(nrx->mIsFiltAfValid) fprintf(fp, "%.3f",nrx->mFiltAf);
		// fprintf(fp, ",");
		//
		// // Print the 	filtered	AccelLateral (m/s²)
		// if(nrx->mIsFiltAlValid) fprintf(fp, "%.3f",nrx->mFiltAl);
		// fprintf(fp, ",");
		//
		// // Print the 	filtered	AccelDown (m/s²)
		// if(nrx->mIsFiltAdValid) fprintf(fp, "%.3f",nrx->mFiltAd);
		// fprintf(fp, ",");
		//
		//
		// // " ISO e.f.s. east filtered acceleration(m/s²),"
		// if(nrx->mIsFiltIsoAnXValid) fprintf(fp, "%.3f",nrx->mFiltIsoAnX);
		// fprintf(fp, ",");
		//
		// // " ISO e.f.s. north filtered acceleration(m/s²),"
		// if(nrx->mIsFiltIsoAnYValid) fprintf(fp, "%.3f",nrx->mFiltIsoAnY);
		// fprintf(fp, ",");
		//
		// // " ISO e.f.s. vertical filtered acceleration(m/s²),"
		// if(nrx->mIsFiltIsoAnZValid) fprintf(fp, "%.3f",nrx->mFiltIsoAnZ);
		// fprintf(fp, ",");
		//
		// // " ISO i.s. longitudinal filtered acceleration(m/s²),"
		// if(nrx->mIsFiltIsoAhXValid) fprintf(fp, "%.3f",nrx->mFiltIsoAhX);
		// fprintf(fp, ",");
		//
		// // " ISO i.s. lateral filtered acceleration(m/s²),"
		// if(nrx->mIsFiltIsoAhYValid) fprintf(fp, "%.3f",nrx->mFiltIsoAhY);
		// fprintf(fp, ",");
		//
		// // " ISO i.s. vertical filtered acceleration(m/s²),"
		// if(nrx->mIsFiltIsoAhZValid) fprintf(fp, "%.3f",nrx->mFiltIsoAhZ);
		// fprintf(fp, ",");
		//
		// // " ISO v.s. longitudinal filtered acceleration(m/s²),"
		// if(nrx->mIsFiltIsoAoXValid) fprintf(fp, "%.3f",nrx->mFiltIsoAoX);
		// fprintf(fp, ",");
		//
		// // " ISO v.s. lateral filtered acceleration(m/s²),"
		// if(nrx->mIsFiltIsoAoYValid) fprintf(fp, "%.3f",nrx->mFiltIsoAoY);
		// fprintf(fp, ",");
		//
		// // " ISO v.s. vertical filtered acceleration(m/s²),"
		// if(nrx->mIsFiltIsoAoZValid) fprintf(fp, "%.3f",nrx->mFiltIsoAoZ);
		// fprintf(fp, ",");
		//
		//
		//
		// // Print the 	Filtered AngAccelX (deg/s²)
		// if(nrx->mIsFiltYxValid) fprintf(fp, "%.3f",nrx->mFiltYx);
		// fprintf(fp, ",");
		//
		// // Print the 	Filtered AngAccelY (deg/s²)
		// if(nrx->mIsFiltYyValid) fprintf(fp, "%.3f",nrx->mFiltYy);
		// fprintf(fp, ",");
		//
		// // Print the 	Filtered AngAccelZ (deg/s²)
		// if(nrx->mIsFiltYzValid) fprintf(fp, "%.3f",nrx->mFiltYz);
		// fprintf(fp, ",");
		//
		// // Print the 	Filtered AngAccelForward (deg/s²)
		// if(nrx->mIsFiltYfValid) fprintf(fp, "%.3f",nrx->mFiltYf);
		// fprintf(fp, ",");
		//
		// // Print the 	Filtered AngAccelLateral (deg/s²)
		// if(nrx->mIsFiltYlValid) fprintf(fp, "%.3f",nrx->mFiltYl);
		// fprintf(fp, ",");
		//
		// // Print the 	Filtered AngAccelDown (deg/s²)
		// if(nrx->mIsFiltYdValid) fprintf(fp, "%.3f",nrx->mFiltYd);
		// fprintf(fp, ",");
		//
		//
		//
		//
		// // " ISO e.f.s. roll filtered acceleration(deg/s²),"
		// if(nrx->mIsFiltIsoYnXValid) fprintf(fp, "%.3f",nrx->mFiltIsoYnX);
		// fprintf(fp, ",");
		//
		// // " ISO e.f.s. pitch filtered acceleration(deg/s²),"
		// if(nrx->mIsFiltIsoYnYValid) fprintf(fp, "%.3f",nrx->mFiltIsoYnY);
		// fprintf(fp, ",");
		//
		// // " ISO e.f.s. yaw filtered acceleration(deg/s²),"
		// if(nrx->mIsFiltIsoYnZValid) fprintf(fp, "%.3f",nrx->mFiltIsoYnZ);
		// fprintf(fp, ",");
		//
		// // " ISO i.s. roll filtered acceleration(deg/s²),"
		// if(nrx->mIsFiltIsoYhXValid) fprintf(fp, "%.3f",nrx->mFiltIsoYhX);
		// fprintf(fp, ",");
		//
		// // " ISO i.s. pitch filtered acceleration(deg/s²),"
		// if(nrx->mIsFiltIsoYhYValid) fprintf(fp, "%.3f",nrx->mFiltIsoYhY);
		// fprintf(fp, ",");
		//
		// // " ISO i.s. yaw filtered acceleration(deg/s²),"
		// if(nrx->mIsFiltIsoYhZValid) fprintf(fp, "%.3f",nrx->mFiltIsoYhZ);
		// fprintf(fp, ",");
		//
		// // " ISO v.s. roll filtered acceleration(deg/s²),"
		// if(nrx->mIsFiltIsoYoXValid) fprintf(fp, "%.3f",nrx->mFiltIsoYoX);
		// fprintf(fp, ",");
		//
		// // " ISO v.s. pitch filtered acceleration(deg/s²),"
		// if(nrx->mIsFiltIsoYoYValid) fprintf(fp, "%.3f",nrx->mFiltIsoYoY);
		// fprintf(fp, ",");
		//
		// // " ISO v.s. yaw filtered acceleration(deg/s²),"
		// if(nrx->mIsFiltIsoYoZValid) fprintf(fp, "%.3f",nrx->mFiltIsoYoZ);
		// fprintf(fp, ",");





		// // Print the 	PosLatStdev (m)
		// if(nrx->mIsNorthAccValid) fprintf(fp, "%.3f",nrx->mNorthAcc);
		// fprintf(fp, ",");
		//
		// // Print the 	PosLonStdev (m)
		// if(nrx->mIsEastAccValid) fprintf(fp, "%.3f",nrx->mEastAcc);
		// fprintf(fp, ",");

		// // Print the 	PosAltStdev (m)
		// if(nrx->mIsAltAccValid) fprintf(fp, "%.3f",nrx->mAltAcc);
		// fprintf(fp, ",");
		//
		// // Print the 	VelNorthStdev (km/h)
		// if(nrx->mIsVnAccValid) fprintf(fp, "%.3f",(nrx->mVnAcc)*MPS2KMPH);
		// fprintf(fp, ",");
		//
		// // Print the 	VelEastStdev (km/h)
		// if(nrx->mIsVeAccValid) fprintf(fp, "%.3f",(nrx->mVeAcc)*MPS2KMPH);
		// fprintf(fp, ",");
		//
		// // Print the 	VelDownStdev (km/h)
		// if(nrx->mIsVdAccValid) fprintf(fp, "%.3f",(nrx->mVdAcc)*MPS2KMPH);
		// fprintf(fp, ",");
		//
		// // Print the 	AngleHeadingStdev (deg)
		// if(nrx->mIsHeadingAccValid) fprintf(fp, "%.3f",nrx->mHeadingAcc);
		// fprintf(fp, ",");
		//
		// // Print the 	AnglePitchStdev (deg)
		// if(nrx->mIsPitchAccValid) fprintf(fp, "%.3f",nrx->mPitchAcc);
		// fprintf(fp, ",");
		//
		// // Print the 	AngleRollStdev (deg)
		// if(nrx->mIsRollAccValid) fprintf(fp, "%.3f",nrx->mRollAcc);
		// fprintf(fp, ",");
		//
		// // Print the 	GpsPosMode
		// if(nrx->mIsGpsPosModeValid) fprintf(fp, "%s",NComGetGpsPosModeString(nrx));
		// fprintf(fp, ",");
		//
		// // Print the 	GpsVelMode
		// if(nrx->mIsGpsVelModeValid) fprintf(fp, "%s",NComGetGpsVelModeString(nrx));
		// fprintf(fp, ",");
		//
		// // Print the 	GpsAttMode
		// if(nrx->mIsGpsAttModeValid) fprintf(fp, "%s",NComGetGpsAttModeString(nrx));
		// fprintf(fp, ",");
		//
		// // Print the 	GpsNumSats
		// if(nrx->mIsGpsNumObsValid) fprintf(fp, "%i",nrx->mGpsNumObs);
		// fprintf(fp, ",");
		//
		// // Print the 	GpsDiffAge (s)
		// if(nrx->mIsGpsDiffAgeValid) fprintf(fp, "%.1f",nrx->mGpsDiffAge);
		// fprintf(fp, ",");

		fprintf(fp, "\n");
	}
}
