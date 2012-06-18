#ifndef _KURT_H_
#define _KURT_H_

#include <string>

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>

#include "can.h"
#include "comm.h"

//CAN IDs
#define CAN_CONTROL    0x00000001 // control message
#define CAN_ADC00_03   0x00000005 // analog input channels: 0 - 3
#define CAN_ADC04_07   0x00000006 // analog input channels: 4 - 7
#define CAN_ADC08_11   0x00000007 // analog input channels: 8 - 11
#define CAN_ENCODER    0x00000009 // 2 motor encoders
#define CAN_TILT_COMP  0x0000000D // data from tilt sensor
#define CAN_GYRO_MC1   0x0000000E // data from gyro connected to 1st C167
#define CAN_GETROTUNIT 0x00000010 // current rotunit angle
#define CAN_SETROTUNT  0x00000080 // send rotunit speed

//unused CAN IDs
#define CAN_INFO_1     0x00000004 // info message (hardware identification, firmware version, loop count): hw_id[2], fw_version[2], loop[4]
#define CAN_ADC12_15   0x00000008 // analog input channels: 12 - 15, motor current right and left in milli Amper, adc channel 14, board temperature
#define CAN_BUMPERC    0x0000000A // bumpers and remote control
#define CAN_DEADRECK   0x0000000B // position as ascertained by odometry: position_x[3], position_y[3], orientation[2]
#define CAN_GETSPEED   0x0000000C // current transl. and rot. speed (MACS spec say accumulated values of left and right motor's encoders: enc_odo_left[4], enc_odo_right[4]
#define CAN_BDC00_03   0x00000015 // analog input channels: 0 - 3
#define CAN_BDC04_07   0x00000016 // analog input channels: 4 - 7
#define CAN_BDC08_11   0x00000017 // analog input channels: 8 - 11
#define CAN_BDC12_15   0x00000018 // analog input channels: 12 - 15
#define CAN_GYRO_MC2   0x0000001E // data from gyro connected to 2nd C167

#define RAW            0          // raw control mode
#define SPEED_CM       2          // speed (cm/s) control mode
#define MAX_V_LIST     200

// values from Sharp GP2D12 IR ranger data sheet
#define IR_MIN         0.10 // [m]
#define IR_MAX         0.80 // [m]
#define IR_FOV         0.074859848 // [rad]

// values from Baumer UNDK30I6103 ultrasonic data sheet
#define SONAR_MIN      0.10 // [m]
#define SONAR_MAX      1.00 // [m]
#define SONAR_FOV      0.17809294 // [rad]

class Kurt
{
  public:
    Kurt(
        Comm &comm,
        double wheel_perimeter,
        double axis_length,
        double turning_adaptation,
        int ticks_per_turn_of_wheel) :
      comm_(comm),
      wheel_perimeter_(wheel_perimeter),
      axis_length_(axis_length),
      turning_adaptation_(turning_adaptation),
      ticks_per_turn_of_wheel_(ticks_per_turn_of_wheel),
      use_microcontroller_(true),
      use_rotunit_(false),
      nr_v_(1000),
      leerlauf_adapt_(0),
      v_encoder_left_(0.0),
      v_encoder_right_(0.0) { }
    ~Kurt();

    void setPWMData(const std::string &speedPwmLeerlaufTable, double feedforward_turn, double ki, double kp);

    int can_motor(int left_pwm,  char left_dir,  char left_brake,
        int right_pwm, char right_dir, char right_brake);
    void set_wheel_speed(double _v_l_soll, double _v_r_soll, double _AntiWindup);
    int can_read_fifo();

    void can_rotunit_send(double speed);

  private:
    CAN can_;
    Comm &comm_;

    //odometry
    double wheel_perimeter_;
    double axis_length_;
    double turning_adaptation_;
    int ticks_per_turn_of_wheel_;

    bool use_microcontroller_;
    bool use_rotunit_;

    //PWM data
    const int nr_v_;
    double vmax_;
    int *pwm_v_l_, *pwm_v_r_;
    double kp_l, kp_r; // schnell aenderung folgen
    double ki_l, ki_r; // integrierer relative langsam
    int leerlauf_adapt_;
    double feedforward_turn_; // in v = m/s
    // speed from encoder in m/s
    double v_encoder_left_, v_encoder_right_;

    //motor
    void k_hard_stop(void);
    void set_wheel_speed1(double v_l, double v_r, int integration_l, int integration_r);
    void set_wheel_speed2(double _v_l_soll, double _v_r_soll, double _v_l_ist,
        double _v_r_ist, double _omega, double _AntiWindup);
    void set_wheel_speed2_mc(double _v_l_soll, double _v_r_soll, double _omega,
        double _AntiWindup);
    void odometry(int wheel_a, int wheel_b);
    bool read_speed_to_pwm_leerlauf_tabelle(const std::string &filename, int *nr,
        double **v_pwm_l, double **v_pwm_r);
    void make_pwm_v_tab(int nr, double *v_pwm_l, double *v_pwm_r, int nr_v, int
        **pwm_v_l, int **pwm_v_r, double *v_max);

    //sensors
    void can_encoder(const can_frame &frame);
    int normalize_ir(int ir);
    int normalize_sonar(int s);
    void can_sonar8_9(const can_frame &frame);
    void can_sonar4_7(const can_frame &frame);
    void can_sonar0_3(const can_frame &frame);
    void can_tilt_comp(const can_frame &frame);
    void can_gyro_mc1(const can_frame &frame);

    void can_rotunit(const can_frame &frame);
};

#endif
