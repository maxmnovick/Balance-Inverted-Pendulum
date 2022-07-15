#ifndef _CONFIG
#define _CONFIG

//============= All this stuff needs to go in a seperate config file ===========
#define NAME "robojay"

//value of pi, from wolfram alpha
#define PI 3.1415926535897932384626433832795028841971693993751058

//dt for pid, sample every 'DT' ms
#define DT 20

//circumfrence of the earth, in meters
#define EARTH_CIRC 6317000

//the diameter of the wheels
#define WHEEL_DIA 0

//the circumfrence of the wheels
#define WHEEL_CIRC (2 * PI * WHEEL_DIA)

//boolean indicating a motor should spin forward
#define FORWARD 0

//boolean indicating a motor should spin in reverse
#define REVERSE 1

//the weight given to the gyroscope in the complementary filter used
//for balancing. this and the accelerometer bias (ACC_BIAS) must add up
//to zero
#define GRYRO_BIAS 0.98

//weight given to the accelerometer reading in the complementary filter
//used fo balancing. This and the gyroscope bias (GYRO_BIAS) must add up
//to zero
#define ACC_BIAS 0.02

//the axis about which the gyroscope is rotating
#define GYRO_AXIS z

#define MAG_AXIS z

#define ACC_AXIS z //this needs to be checked

//the maximum drive that can be applied to a wheel
#define MAX_DRIVE 255

//maximus allowed angle. past observation of an angle past this point
//will disable the motors and the motor control board will need to be
//reset
#define MAX_ANGLE 20

//the max drive angle. the current angle specified by pwm will be
//calculated using this angle
#define MAX_DRIVE_ANGLE 5

//degrees per pulse width duty (a made up unit)
//multiplied by the observed pwm signal to obtain the degrees specified
//by the controller
#define DEG_PER_PWD MAX_DRIVE_ANGLE/125

//the width of the 'deadband' around the center point (0, origin, etc)
//if a value is within this deadband, if should be considered to be 0.
// (mitigates misreadings when trying ot stay still, not as important
//during motion
#define DEADBAND 2

#define BAL_TEST 1

#define T 
#define KU 30

#define KP (0.6 * KU)
#define KI (2 * KP) / DELAY
#define KD (KP * DELAY) / 8

//the number of distance sensors
#define NDS 0

#define BACKLASH 1.2
#define BL_DRIVE 0
#define A_OFFST -2.5

//======================== sensor pins =================================
#define GYRO_REG 
#define ACC_PIN -1
#define MAG_PIN -1

#define INCL_PIN 10  //inclinometer uses SPI, this is its slave select pin
#define INCL_REG 0x0c  //the register to read for angle data
#define INCL_BYTES 2  //the number of bytes in the output from the inclinometer
#define INCL_SF  0.025  //the scaling factor for the data recieved from the inclinometer

#define GPS_PIN -1

//specify each pin for the Distance sensors
#define D1 -1
#define D2 -1
#define D3 -1
#define D4 -1
#define D5 -1
#define D6 -1
#define D7 -1
#define D8 -1
#define D9 -1
#define D10 -1

//put the distance sensprs in an array so we can iterate through
const int DIST_PINS[] = {D1, D2, D3, D4, D5, D6, D7, D8, D9, D10};


//========================== motor pins ================================

//right motor drive pin
#define RIGHTM_PIN 3

//right motor direction pin
#define RIGHTM_DIR 2

//right motor encoder pin
#define RIGHTM_ENC -1

//left motors drive pin
#define LEFTM_PIN 5

//left motors direction pin
#define LEFTM_DIR 4

//left motor encoder pin
#define LEFTM_ENC -1

//========================= External control ==============================

// pwm pin to control tilt angle. a pwm signal applied to this pin
// will cause robojay to tilt by the amount calculated from the values
// MAX_DRIVE_ANGLE and DEG_PER_PWD specified above
#define TILT_PIN A0
#define TILT_TEST A2

//pwm pin to control rotation. exact mechanism to be fleshed out
#define TURN_PIN A1
#define TRURN_TEST A3

//an enumeration for the sensor types, saves space
enum sensor_types {
  GYRO,
  INCL,
  ACC,
  MAG,
  GPS,
  DIST
};

#define PRINT Serial.println

//============= Debugging Flags =========================
#define DEBUGGING 1
#define SENSOR_DEBUGGING 1
#define MOTOR_DEBUGGING 1
#define PID_DEBUGGING 1

#endif
