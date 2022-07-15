#include "sensor_obj.h"
#include "robojay_config.h"
#include "L3G4200D.h"
#include "ADXL345.h"
#include <SPI.h>  //dont need to include arduino because it is included here

sensor::sensor(int le_type) {
  type = le_type;
  
  switch(type){
    case GYRO: {
      gyro.enableDefault();
      break;
    }
    
    case MAG: {
      //mag.initialize();
      break;
    }
    
    case ACC: {
      //acc.initialize();
      break;
    }
    
    case INCL: {
      sensor_pin = INCL_PIN;
      //inclinometer.set_pin(INCL_PIN);
      //inclinometer.set_type(INCL);
      break;
    }
    
    case GPS: {
      
      break;
    }
    
    case DIST: {
      
      break;
    }    
  } 
}

double sensor::get_val() {    
    //process as needed and store in val
    switch(type){
      
      case GYRO: {
        gyro.read();
        val = gyro.g.GYRO_AXIS;
        
        if (SENSOR_DEBUGGING) {
          PRINT("Detected anglular velocity:");
          PRINT(val);
        }
        
        return val;
      }
    
      case MAG: {
        mag.read();
        val = mag.m.MAG_AXIS;
        
        if (SENSOR_DEBUGGING) {
          PRINT("Detected heading:");
          PRINT(val);
        }
        
        return val;
      }
    
      case ACC: {
        return 0;
        //acc.read_axes();
        //val = acc.ACC_AXIS;
        
        if (SENSOR_DEBUGGING) {
          PRINT("Detected Acceleration:");
          PRINT(val);
        }
        
        return val;
      }

      case INCL: {
        SPI.beginTransaction(SPISettings(1500000, MSBFIRST, SPI_MODE3));//settings from datasheet
        
        //transmit something to recieve the output, output is 2 bytes
        byte result = 0;
        int temp1, temp2;
        temp1 = temp2 = 0;
        int8_t mask = 0b00111111;
        
        digitalWrite(sensor_pin, LOW);  //set inclinometer_read pin low to access inclinometer
        
        //now we need to transmit a byte to say what we want to read
        //datasheet says to do so in data frames

        //dataframe 1
        SPI.transfer(INCL_REG);         //set mosi to request tilt data
        SPI.transfer(0);
        digitalWrite(sensor_pin, HIGH);
                
        //dataframe 2
        digitalWrite(sensor_pin, LOW);        
        temp1 = SPI.transfer(0);
        temp2 = SPI.transfer(0);      
        digitalWrite(sensor_pin, HIGH);//done writing data, tell the inclinometer as much
        SPI.endTransaction();          //end the transaction      
        
        temp1 = temp1 & mask;
        temp1 = temp1 << 8;
        temp1 = temp1 | temp2;        
        val = temp1 * INCL_SF;

        if(val > 180){
          val = (val - 410); //adjust val
        }

         val = val - A_OFFST; //adjust some more
        
        if (SENSOR_DEBUGGING) {
          PRINT("Detected deg/s:");
          PRINT(val);
        }
        
        return val; //return  
      }

      case GPS: {
        val = 5;
        
        return val;
      }

      case DIST: {
        val = 6;
        if (SENSOR_DEBUGGING) {
          PRINT("Detected Distance:");
          PRINT(val);
        }
        return val;
      }
    
    return val;
  }
}
