#include "HMC5883.h"
#include <Wire.h>
#include <math.h>

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

void HMC5883::initialize()
{
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}

float HMC5883::read()
{
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()) {
    /*m.x = Wire.read()<<8; //X msb
    m.x |= Wire.read(); //X lsb
    m.z = Wire.read()<<8; //Z msb
    m.z |= Wire.read(); //Z lsb
    m.y = Wire.read()<<8; //Y msb
    m.y |= Wire.read(); //Y lsb*/

    uint8_t xla = Wire.read();
    uint8_t xha = Wire.read();
    uint8_t zla = Wire.read();
    uint8_t zha = Wire.read();
    uint8_t yla = Wire.read();
    uint8_t yha = Wire.read();

    m.x = xha << 8 | xla;
    m.z = yha << 8 | yla;
    m.y = zha << 8 | zla;
  }
    m.heading = atan2(-m.y , m.x) / M_PI * 180; // angle is atan(-y/x)
    
    if(m.heading < 0){
     m.heading += 360; // angle from 0 to 359 instead of plus/minus 180
    }
    return m.heading;

     

    
  }

