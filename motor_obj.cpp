#include "Arduino.h"

#include "robojay_config.h"
#include "motor_obj.h"

void motor::set_pins(int drive, int dir, int s) {
    motor_pin = drive;
    direction_pin = dir;
    speed_pin = s;
}

int motor::get_speed(){ //not done
    //read speed from sensor
    //update speed variable
    return _speed;//return speed
}

int motor::set_drive(int le_drive, int dir){
    drive = scale_factor * le_drive;         //multiply speed by scale_factor
    digitalWrite(direction_pin, dir);        //with the given direction
    analogWrite(motor_pin, drive);           //write speed to motor's pin
    
    if (MOTOR_DEBUGGING) {
      PRINT("Drive applied:");
      PRINT(drive); 
    }
    
    return drive;                            //return applied drive   
}

void motor::kill(){
  scale_factor = 0;
  set_drive(0, 0);
}

