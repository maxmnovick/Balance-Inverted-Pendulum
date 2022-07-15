#ifndef HMC5883_h
#define HMC5883_h

#include "Arduino.h" // for byte data type


class HMC5883
{
	public:
		typedef struct vector
		{
			float x, y, z, heading;
		} vec;
		
		vec m; // magnetometer readings

		void initialize();
		
		float read();
};

#endif



