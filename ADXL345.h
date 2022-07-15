#ifndef ADXL345_H
#define ADXL345_H

class ADXL345
{
	int ADXAddress = 0xA7 >> 1;  // the default 7-bit slave address
	int reading = 0;
	int val = 0;
	int X0,X1,X_out;
	int Y0,Y1,Y_out;
	int Z1,Z0,Z_out;
	
	public:
		double x;
		double y;
		double z;
		void initialize();
		void read_axes(); //returns g forces

};

#endif
