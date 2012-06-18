#ifndef _COMM_H_
#define _COMM_H_

class Comm
{
  public:
    virtual ~Comm() { }
    virtual void send_odometry(double z, double x, double theta, double v_encoder,
        double v_encoder_angular, int wheel_a, int wheel_b, double v_encoder_left, double v_encoder_right) = 0;
    virtual void send_sonar_leftBack(int ir_left_back) = 0;
    virtual void send_sonar_front_usound_leftFront_left(int ir_right_front, int
        usound, int ir_left_front, int ir_left) = 0;
    virtual void send_sonar_back_rightBack_rightFront(int ir_back, int
        ir_right_back, int ir_right) = 0;
    virtual void send_pitch_roll(double pitch, double roll) = 0;
    virtual void send_gyro(double theta, double sigma) = 0;
    virtual void send_rotunit(double rot) = 0;
};

#endif
