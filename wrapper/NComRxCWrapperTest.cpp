#include "NComRxCWrapper.hpp"
#include <iostream>
#include <fstream>

int main()
{
    NComRxC* nrxPtr = CreateNComRxC();

	std::string fileName("/Users/hhaider/Projects/NCOMdecoder/example/171019_031603.ncom");

	if (nrxPtr != nullptr)
	{
		std::cout << "Create NComRxC succesfully" << std::endl;
	}

	std::ifstream reader(fileName, std::ios::binary | std::ios::in);

	if (!reader)
	{
		std::cout << "Error: unable to readfile" << std::endl;
		reader.close();
		exit(EXIT_FAILURE);
	}
    
	char inputChar;

	//Main reading loop
	for (int i = 0; !reader.eof(); i++)
	{
		reader.get(inputChar);

		if(UpdatePacket(nrxPtr, (unsigned char)inputChar))
		{
        	std::cout 	<< GetMeasurement(nrxPtr, "Lat") << ", " 
					  	<< GetMeasurement(nrxPtr, "Lon") << ", "
						<< GetMeasurement(nrxPtr, "InsNavMode"  ) << ", "
						<< GetMeasurement(nrxPtr, "SerialNumber") << ", "
						<< GetMeasurement(nrxPtr, "GpsPosMode"  ) << ", "
						<< GetMeasurement(nrxPtr, "GpsVelMode"  ) << ", "
						<< GetMeasurement(nrxPtr, "GpsAttMode"  ) << ", "
						<< std::endl;
		}
    }

}
