#ifndef MOTOR
#define MOTOR

//====================motor class===================
class motor {
    int motor_pin, direction_pin, drive, speed_pin, _speed;
  public:
    motor() {scale_factor = 1;};
    ~motor() {};
    void set_pins(int, int, int);
    int get_speed();
    int get_drive() {return drive;};
    int set_drive(int, int);
    void kill();
    float scale_factor = 1;//the percentage of given drive to be applied to the motor

    friend class robojay_obj;
    friend class pid_obj;
};



#endif
