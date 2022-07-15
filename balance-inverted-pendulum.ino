#include <SPI.h>
#include <Wire.h> 
#include "robojay_config.h"

#include "sensor_obj.h"
#include "motor_obj.h"


//============================= PID Object =============================
//from https://en.wikipedia.org/wiki/PID_controller
#ifndef PID_OBJ
#define PID_OBJ


class pid_obj {
    int P, I, D;
    int kP, kI, kD;
    int output, max_output;
    int error, prev_error;  

    public:
        pid_obj();
        ~pid_obj(){};
        pid_obj(double, double, double);
        void set_kP(double le_kP) {kP = le_kP;};
        void set_kI(double le_kI) {kI = le_kI;};
        void set_kD(double le_kD) {kD = le_kD;};
        void set_kPID(double, double, double);
        int calculate_output(double, double, double);
    friend class balancer;
};

#endif


pid_obj::pid_obj() {
    P = I = D = kP = kI = kD = error = output = prev_error = 0;
}

pid_obj::pid_obj(double le_P, double le_I, double le_D) {
    kP = le_P;
    kI = le_I;
    kD = le_D;
    P = I = D = error = prev_error = 0;
}

void pid_obj::set_kPID(double p, double i, double d) {
    kP = p;
    kI = i;
    kD = d;
}

int pid_obj::calculate_output(double setpoint, double curr_val, double dt) {
    error = setpoint - curr_val;    //get our deviation from the desired goal
    I = I + (error * dt);           //discrete intergration
    D = (error - prev_error) / dt;  //discrete derivation
    prev_error = error;             //current error is next time's previous error
    output = (kP * error) + (kI * I) + (kD * D);   //output is the summation of P, I and D with the weights kP kI and kD
    delay(DT);  //wait some time to have an effect
    return output;                  //return the result to the caller
    return (kP * error) + (kI * I) + (kD * D);
}



//==================== Balancer Class ==================================
class balancer {
    pid_obj angle_ctrl;
    pid_obj speed_ctrl;

    double angle = 0;       //the current angle
    double set_point;   //the point we want to reach, an angle in degrees
    double drive;       //the drive applied
    double last_angle;
    int prev_dir;

    sensor inclinometer = sensor(INCL);
    sensor gyroscope = sensor(GYRO);
    sensor accelerometer = sensor(ACC);
    
    motor left_motor, right_motor;

    public:
        balancer();
        ~balancer() {};
        void set_set_point(double s){set_point = s;};
        double get_angle();    //use kalman filter on gyroscope and accelerometer readings
        void balance();        //adjust to try to make measured angle == set point
        float read_angle();     //read from TILT_PIN an analog value, return it in degrees
};

balancer::balancer() {  
  drive = 0;
  angle_ctrl.set_kPID(10, 0, 0); //30 worked well
  //speed_ctrl.set_kPID(5, 3, 2);
  left_motor.set_pins(LEFTM_PIN, LEFTM_DIR, LEFTM_ENC);
  right_motor.set_pins(RIGHTM_PIN, RIGHTM_DIR, RIGHTM_ENC);


}

double balancer::get_angle() {
  //read values from gyro and accel
  angle = (GRYRO_BIAS * (angle + gyroscope.get_val() * DT)) + (ACC_BIAS * (accelerometer.get_val()));
  //use kalman filter to combine measurements, store in angle
  //use less computationally expensive complementary filter instead

  //ooorrrrrr get angle from inclinometer, thanks Max!
  //angle = inclinometer.get_val();

  return angle;
}

void balancer::balance() {  //use cascading pid controller

  int le_drive = 0;
  //calculate the drive based on current angle and setpoint
  double le_angle = get_angle();  //get the angle

  //if the angle is too steep, kill the motors
  if (le_angle > MAX_ANGLE || le_angle < -MAX_ANGLE){
    
     //by multiplying the drive applied by zero
    left_motor.scale_factor = right_motor.scale_factor = 0; 
  }
  
  //if we change direction, account for backlash,
  if (((last_angle > 0) && (le_angle < 0))) {
    le_angle -= BACKLASH;
  }
  if (((last_angle < 0) && (le_angle > 0))) {
    le_angle += BACKLASH;
  }

  //update the last angle seen
  last_angle = le_angle;

  //find out what the setpoint is by reading the tilt pin
  //set_point = read_angle();
  if (BAL_TEST) {
    set_point = 0;
  }
  
  //calculate the output
  drive = angle_ctrl.calculate_output(set_point, le_angle, (DT / 1000) ); //convert ms to s

  //debugging block
  if (PID_DEBUGGING) {  //debugging print statements
    //PRINT("Angle PID output:");
    //PRINT(cal_drive);   //cascading PID wasnt working
    PRINT("Drive PID output:");
    PRINT(drive);
  }
  
  le_drive = drive;       //store variable before manipulating so we can do math now and later
  int dir = REVERSE;      //turn the wheels backwards
  if (drive < 0) {        //if falling forward, turn the wheels forward
    le_drive = -drive;    //remove the negative and use it for direction
    dir = FORWARD;
  }
  
  //if we change direction, account for backlash
  //if (prev_dir != dir) {
  //  drive += BL_DRIVE;
  //}

  //limit the applied drive to MAX_DRIVE (defined in config file)  
  if (drive > MAX_DRIVE) {  
    drive = le_drive = MAX_DRIVE;
  }

 // if (drive < -MAX_DRIVE){
 //   drive = le_drive = MAX_DRIVE;
 // }

   //debugging print statements
  if (MOTOR_DEBUGGING) {  
    PRINT("Balancing direction:");
    PRINT(dir);
    PRINT("Left Motor Drive:");
  }

  //set the speeds for the left and right wheels
  left_motor.set_drive(le_drive, dir);   

  if (MOTOR_DEBUGGING) {
    PRINT("Right Motor Drive:");
  }
  right_motor.set_drive(le_drive, dir);
}

float balancer::read_angle() {
  //read the analog value, multiply by DEG_PER_PWD
  //  =>deg/pwd * pwd = deg
  int read_val = analogRead(TILT_PIN);
  int cal_ang = DEG_PER_PWD * analogRead(TILT_PIN);

  //if the observed_value is within the "DEADBAND range, return 0)
  if ((read_val < 125 + DEADBAND) && (read_val > 125 - DEADBAND)) {
    return 0;
  }
  return cal_ang;
}

void setup() {
  //SPI.begin();
  Wire.begin();
  Serial.begin(9600);
  
  pinMode(INCL_PIN, OUTPUT);
  
  pinMode(LEFTM_PIN, OUTPUT);
  pinMode(LEFTM_DIR, OUTPUT);
  
  pinMode(RIGHTM_PIN, OUTPUT);
  pinMode(RIGHTM_DIR, OUTPUT);

  //set up motor controller reset pins
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  //reset motor controllers
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  delay(50);
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  
  PRINT("done in setup");
}

balancer legs = balancer();

//set the setpoint by proving pwm signal
//wire output to TILT_PIN for testing



void loop() {
  //analogWrite(TILT_TEST, 125);
  if (DEBUGGING) {
    PRINT("\n");
    PRINT("Setpoint:");
    //PRINT(legs.read_angle());
  }

  legs.balance();
  
  //delay(1000);
}

