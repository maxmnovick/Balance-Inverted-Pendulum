//================sensor class==========================
#ifndef SENSOR
#define SENSOR

#include "L3G4200D.h"
#include "HMC5883.h"
#include "ADXL345.h"

class sensor {
    int sensor_pin;
    int type;
    double val;

    public:
        //sensor() {};    //constructor
        sensor(int);    //constructor
        int get_type() {return type;};  //the type of the sensor, determines how the input is parsed
        ~sensor() {};   //deconstructor
        //sensor(int pin) {sensor_pin = pin;};  //pin 
        void set_pin(int pin){ sensor_pin = pin;};
        double get_val();
        void set_type(int le_type) {type = le_type;};
        L3G4200D gyro;
        HMC5883 mag;
        ADXL345 acc;
        

    friend class robojay_obj;
    friend class pid_obj;
};

#endif
