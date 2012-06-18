#ifndef _STDOUTCOMM_H_
#define _STDOUTCOMM_H_

#include <iostream>

#include "comm.h"

class STDoutComm : public Comm
{
  public:
    STDoutComm() : sum_ticks_a_(0), sum_ticks_b_(0) { }
    void send_odometry(double z, double x, double theta, double v_encoder, double v_encoder_angular, int wheel_a, int wheel_b, double v_encoder_left, double v_encoder_right)
    {
      std::cout << "Odometry: z: " << z << " x: " << x << " theta: " << theta << std::endl;
      std::cout << "Encoder: wheel_a: " << wheel_a  << " wheel_b: " << wheel_b << std::endl;
      std::cout << "Encoder Speed: v_encoder: " << v_encoder << " v_encoder_angular: " << v_encoder_angular << std::endl;

      sum_ticks_a_ += wheel_a;
      sum_ticks_b_ += wheel_b;
      v_encoder_ = v_encoder;
      v_encoder_left_ = v_encoder_left;
      v_encoder_right_ = v_encoder_right;
    }

    void send_sonar_leftBack(int ir_left_back)
    {
      std::cout << "IR left back: " << ir_left_back << std::endl;
    }

    void send_sonar_front_usound_leftFront_left(int ir_right_front, int usound, int ir_left_front, int ir_left)
    {
      std::cout << "IR right front: " << ir_right_front << std::endl;
      std::cout << "ultrasound front: " << usound << std::endl;
      std::cout << "IR left front: " << ir_left_front << std::endl;
      std::cout << "IR left: " << ir_left << std::endl;
    }

    void send_sonar_back_rightBack_rightFront(int ir_back, int ir_right_back, int ir_right)
    {
      std::cout << "IR back: " << ir_back << std::endl;
      std::cout << "IR right back: " << ir_right_back << std::endl;
      std::cout << "IR right: " << ir_right << std::endl;
    }

    void send_pitch_roll(double pitch, double roll)
    {
      std::cout << "pitch: " << pitch << " roll: " << roll << std::endl;
    }

    void send_gyro(double theta, double sigma)
    {
      std::cout << "Gyro: theta: " << theta << " sigma: " << sigma << std::endl;
    }

    void send_rotunit(double rot)
    {
      std::cout << "Rotunit" << rot <<  std::endl;
    }

    unsigned long long K_get_sum_ticks_a()
    {
      return sum_ticks_a_;
    }

    unsigned long long K_get_sum_ticks_b()
    {
      return sum_ticks_b_;
    }

    double v1()
    {
      return v_encoder_;
    }

    double v1_left()
    {
      return v_encoder_left_;
    }

    double v1_right()
    {
      return v_encoder_right_;
    }

  private:
    unsigned long long sum_ticks_a_;
    unsigned long long sum_ticks_b_;
    double v_encoder_;
    double v_encoder_left_, v_encoder_right_;
};

#endif
