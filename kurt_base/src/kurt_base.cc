#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

#include <string>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cerrno>
#include <cstdio>

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

//////////////////////////// CAN /////////////////////////////

class CAN
{
  public:
    CAN();
    ~CAN();

    bool send_frame(const can_frame *frame);
    bool receive_frame(can_frame *frame);

  private:
    int cansocket; // can raw socket
};

CAN::CAN()
{
  sockaddr_can addr;
  ifreq ifr;
  char caninterface[] = "can0"; //TODO automatic searching for caninterface

  cansocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (cansocket < 0) {
    ROS_ERROR("can_init: Error opening socket (%s)", strerror(errno));
    exit(1);
  }

  addr.can_family = AF_CAN;

  strcpy(ifr.ifr_name, caninterface);
  if (ioctl(cansocket, SIOCGIFINDEX, &ifr) < 0) {
    ROS_ERROR("can_init: Error setting SIOCGIFINDEX for interace %s (%s)", caninterface, strerror(errno));
    exit(1);
  }

  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(cansocket, (sockaddr *)&addr, sizeof(addr)) < 0) {
    ROS_ERROR("can_init: Error binding socket (%s)", strerror(errno));
    exit(1);
  }

  ROS_INFO("CAN interface init done");
}

CAN::~CAN()
{
  if (close(cansocket) != 0)
    ROS_ERROR("can_close: Error closing can socket (%s)", strerror(errno));
}

bool CAN::send_frame(const can_frame *frame)
{
  if (write(cansocket, frame, sizeof(*frame)) != sizeof(*frame))
  {
    ROS_ERROR("send_frame: Error writing socket (%s)", strerror(errno));
    return false;
  }
  return true;
}

bool CAN::receive_frame(can_frame *frame)
{
  fd_set rfds;

  FD_ZERO(&rfds);
  FD_SET(cansocket, &rfds);

  int rc = 1;
  timeval timeout;

  timeout.tv_sec = 5;
  timeout.tv_usec = 0;

  rc = select(cansocket+1, &rfds, NULL, NULL, &timeout);

  if (rc == 0)
  {
    ROS_ERROR("recive_frame: Receiving frame timed out (Kurt switched off?)");
    return false;
  }
  else if (rc == -1)
  {
    ROS_WARN("recive_frame: Error receiving frame (%s)", strerror(errno));
    return false;
  }

  //TODO read time stamp
  if (read(cansocket, frame, sizeof(*frame)) != sizeof(*frame))
  {
    ROS_WARN("receive_frame: Error reading socket (%s)", strerror(errno));
    return false;
  }
  return true;
}

////////////////////////////////// Kurt ////////////////////////////////

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
#define CAN_INFO_1     0x00000004 // info message
#define CAN_ADC12_15   0x00000008 // analog input channels: 12 - 15
#define CAN_BUMPERC    0x0000000A // bumpers and remote control
#define CAN_DEADRECK   0x0000000B // position (dead reckoning)
#define CAN_GETSPEED   0x0000000C // current transl. and rot. speed
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
        double wheel_perimeter,
        double axis_length,
        double turning_adaptation,
        int ticks_per_turn_of_wheel, 
        double sigma_x, 
        double sigma_theta, 
        double cov_x_y,
        double cov_x_theta, 
        double cov_y_theta,
        const ros::NodeHandle &n) :
      use_microcontroller_(true),
      use_rotunit_(false),
      wheel_perimeter_(wheel_perimeter),
      axis_length_(axis_length),
      nr_v_(1000),
      leerlauf_adapt_(0),
      v_encoder_left_(0.0),
      v_encoder_right_(0.0),
      turning_adaptation_(turning_adaptation),
      ticks_per_turn_of_wheel_(ticks_per_turn_of_wheel),
      sigma_x_(sigma_x),
      sigma_theta_(sigma_theta),
      cov_x_y_(cov_x_y),
      cov_x_theta_(cov_x_theta),
      cov_y_theta_(cov_y_theta),
      publish_tf_(false),
      n_(n) { }
    ~Kurt();

    void run();
    void setPWMData(const std::string &speedPwmLeerlaufTable, double feedforward_turn, double ki, double kp);

    void can_rotunit_send(int speed);

    //ROS
    void setTFPrefix(const std::string &tf_prefix);
    void velCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void rotunitCallback(const geometry_msgs::Twist::ConstPtr &msg);

  private:
    CAN can_;
    bool use_microcontroller_;
    bool use_rotunit_;

    double wheel_perimeter_;
    double axis_length_;

    //PWM data
    const int nr_v_;
    double vmax_;
    int *pwm_v_l, *pwm_v_r;
    double kp_l, kp_r; // schnell aenderung folgen
    double ki_l, ki_r; // integrierer relative langsam
    int leerlauf_adapt_;
    double feedforward_turn_; // in v = m/s
    // speed from encoder in m/s
    double v_encoder_left_, v_encoder_right_;

    //odometry
    double turning_adaptation_;
    int ticks_per_turn_of_wheel_;
    double sigma_x_, sigma_theta_, cov_x_y_, cov_x_theta_, cov_y_theta_;

    //ROS
    bool publish_tf_;
    ros::NodeHandle n_;
    std::string tf_prefix_;

    tf::TransformBroadcaster odom_broadcaster;
    ros::Publisher odom_pub;
    ros::Publisher joint_pub;
    ros::Publisher range_pub;
    ros::Publisher imu_pub;

    //motor
    int can_motor(int left_pwm,  char left_dir,  char left_brake,
        int right_pwm, char right_dir, char right_brake);
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
    void can_read_fifo();

    void can_encoder(const can_frame &frame);
    int normalize_ir(int ir);
    int normalize_sonar(int s);
    void can_sonar8_9(const can_frame &frame);
    void can_sonar4_7(const can_frame &frame);
    void can_sonar0_3(const can_frame &frame);
    void can_tilt_comp(const can_frame &frame);
    void can_gyro_mc1(const can_frame &frame);

    void can_rotunit(const can_frame &frame);

    //ROS
    void populateCovariance(nav_msgs::Odometry &msg, double v_encoder, double
        v_encoder_angular);
    void ros_send_odometry(double z, double x, double theta, double v_encoder,
        double v_encoder_angular, double wheelpos_l, double wheelpos_r);
    void ros_send_sonar_leftBack(int ir_left_back);
    void ros_send_sonar_front_usound_leftFront_left(int ir_right_front, int
        usound, int ir_left_front, int ir_left);
    void ros_send_sonar_back_rightBack_rightFront(int ir_back, int
        ir_right_back, int ir_right);
    void ros_send_pitch_roll(double pitch, double roll);
    void ros_send_gyro(double theta);
    void ros_send_rotunit(double rot);
};

Kurt::~Kurt()
{
  if(use_rotunit_)
    can_rotunit_send(0);
  k_hard_stop();
  if(!use_microcontroller_)
  {
    free(pwm_v_l);
    free(pwm_v_r);
  }
}

void Kurt::run()
{
  odom_pub = n_.advertise<nav_msgs::Odometry> ("odom", 10);
  range_pub = n_.advertise<sensor_msgs::Range> ("range", 10);
  imu_pub = n_.advertise<sensor_msgs::Imu> ("imu", 10);
  joint_pub = n_.advertise<sensor_msgs::JointState> ("joint_states", 1);

  while (ros::ok())
  {
    can_read_fifo();
    ros::spinOnce();
  }
}

void Kurt::setPWMData(const std::string &speedPwmLeerlaufTable, double feedforward_turn, double ki, double kp)
{
  ki_l = ki_r = ki;
  kp_l = kp_r = kp;

  int nr;
  double *v_pwm_l, *v_pwm_r;
  if (!read_speed_to_pwm_leerlauf_tabelle(speedPwmLeerlaufTable, &nr, &v_pwm_l, &v_pwm_r))
  {
    return;
  }
  make_pwm_v_tab(nr, v_pwm_l, v_pwm_r, nr_v_, &pwm_v_l, &pwm_v_r, &vmax_);
  free(v_pwm_l);
  free(v_pwm_r);
  use_microcontroller_ = false;
}

int Kurt::can_motor(int left_pwm,  char left_dir,  char left_brake,
    int right_pwm, char right_dir, char right_brake)
{
  char left_dir_brake =  (left_dir << 1) + left_brake;
  char right_dir_brake = (right_dir << 1) + right_brake;

  can_frame frame;
  frame.can_id = CAN_CONTROL;
  frame.can_dlc = 8;
  frame.data[0] = RAW >> 8;
  frame.data[1] = RAW;
  frame.data[2] = (left_dir_brake);
  frame.data[3] = (left_pwm >> 8);
  frame.data[4] = (left_pwm);
  frame.data[5] = (right_dir_brake);
  frame.data[6] = (right_pwm >> 8);
  frame.data[7] = (right_pwm);

  if (!can_.send_frame(&frame))
  {
    ROS_ERROR("can_motor: Error sending PWM data");
    return 1;
  }
  return 0;
}

void Kurt::k_hard_stop(void)
{
  unsigned short pwm_left, pwm_right; 
  unsigned char dir_left, dir_right, brake_left, brake_right;
  pwm_left = 1023;
  dir_left = 0;
  brake_left = 1;
  pwm_right = 1023;
  dir_right = 0;
  brake_right = 1;

  do
  { }
  while (!can_motor(pwm_left, dir_left, brake_left, pwm_right, dir_right, brake_right));
}

// PWM Lookup
// basis zuordnung
// darueber liegt ein PID geschwindigkeits regler
// Leerlauf adaption fuer den Teppich boden und geradeaus fahrt
void Kurt::set_wheel_speed1(double v_l, double v_r, int integration_l, int integration_r)
{
  unsigned short pwm_left, pwm_right; 
  unsigned char dir_left, dir_right, brake_left, brake_right;

  int index_l, index_r; // point in speed_pwm tabelle

  // calc pwm values form speed array
  index_l = (int)(fabs(v_l) / vmax_ * nr_v_);
  index_r = (int)(fabs(v_r) / vmax_ * nr_v_);

  index_l = std::min(nr_v_ - 1, index_l);
  index_r = std::min(nr_v_ - 1, index_r);

  // 1023 = zero, 0 = maxspeed
  if (fabs(v_l) > 0.01)
    pwm_left = std::max(0, std::min(1023, 1024 - pwm_v_l[index_l] - leerlauf_adapt_ - integration_l));
  else
    pwm_left = 1023;
  if (fabs(v_r) > 0.01)
    pwm_right = std::max(0, std::min(1023, 1024 - pwm_v_r[index_r] - leerlauf_adapt_ - integration_r));
  else
    pwm_right = 1023;

  if (v_l >= 0.0)
    dir_left = 0; // forward
  else
    dir_left = 1; // backward
  if (v_r >= 0.0)
    dir_right = 0; // s.o right wheel
  else
    dir_right = 1;

  // relase breaks
  brake_left = 0;
  brake_right = 0;

  can_motor(pwm_left, dir_left, brake_left, pwm_right, dir_right, brake_right);
}

// pid geschwindigkeits regler fuers linke und rechte rad
// omega wird benoetig um die integration fuer den darunterstehenden regler
// zu berechnen
void Kurt::set_wheel_speed2(double _v_l_soll, double _v_r_soll, double _v_l_ist,
    double _v_r_ist, double _omega, double _AntiWindup)
{
  // stellgroessen v=speed, l= links, r= rechts
  static double zl = 0.0, zr = 0.0;
  static double last_zl = 0.0, last_zr = 0.0;
  // regelabweichung e = soll - ist;
  static double el = 0.0, er = 0.0;
  static double last_el = 0.0, last_er = 0.0;
  // integral
  static double int_el = 0.0, int_er = 0.0;
  // differenzieren
  static double del = 0.0, der = 0.0;
  // zeitinterval
  double dt = 0.1;
  // filter fuer gueltige Werte
  static double last_v_l_ist = 0.0, last_v_r_ist = 0.0;
  // static int reached = 0;
  static double v_l_list[MAX_V_LIST], v_r_list[MAX_V_LIST];
  static int vl_index = 0, vr_index = 0;
  int i;
  static double f_v_l_ist, f_v_r_ist;
  // kd_l and kd_r allways 0 (using only pi controller here)
  double kd_l = 0.0, kd_r = 0.0; // nur pi regler d-anteil ausblenden

  //TODO use timer
  dt = 0.01;

  int_el *= _AntiWindup;
  int_er *= _AntiWindup;

  last_v_l_ist *= _AntiWindup;
  last_v_r_ist *= _AntiWindup;

  double turn_feedforward_l = -_omega / M_PI * feedforward_turn_;
  double turn_feedforward_r = _omega / M_PI * feedforward_turn_;

  // filtern: grosser aenderungen deuten auf fehlerhafte messungen hin
  if (fabs(_v_l_ist - last_v_l_ist) < 0.19)
  {
    // filter glaettung werte speichern
    v_l_list[vl_index] = _v_l_ist;
    f_v_l_ist = (v_l_list[vl_index] + v_l_list[vl_index - 1] + v_l_list[vl_index - 2] + v_l_list[vl_index - 3]) / 4.0;
    vl_index++; // achtung auf ueberlauf
    if (vl_index >= MAX_V_LIST)
    {
      vl_index = 3; // zum schutz vor ueberlauf kopieren bei hold einen mehr kopieren
      for (i = 0; i < 3; i++)
        v_l_list[2 - i] = v_l_list[MAX_V_LIST - i - 1];
    }

    el = _v_l_soll - f_v_l_ist;
    del = (el - last_el) / dt;
    int_el += el * dt;

    zl = kp_l * el + kd_l * del + ki_l * int_el + _v_l_soll + turn_feedforward_l;

    last_el = el; // last e
  }

  // filtern: grosser aenderungen deuten auf fehlerhafte messungen hin
  if (fabs(_v_r_ist - last_v_r_ist) < 0.19)
  {
    // filter glaettung werte speichern
    v_r_list[vr_index] = _v_r_ist;
    f_v_r_ist = (v_r_list[vr_index] + v_r_list[vr_index - 1] + v_r_list[vr_index - 2]) / 3.0;
    vr_index++; // achtung auf ueberlauf
    if (vr_index >= MAX_V_LIST)
    {
      vr_index = 3; // zum schutz vor ueberlauf kopieren bei hold einen mehr kopieren
      for (i = 0; i < 3; i++)
      {
        v_r_list[2 - i] = v_r_list[MAX_V_LIST - i - 1];
        ;
      }
    }

    er = _v_r_soll - f_v_r_ist;
    der = (er - last_er) / dt;
    int_er += er * dt;

    zr = kp_r * er + kd_r * der + ki_r * int_er + _v_r_soll + turn_feedforward_r;

    last_er = er; // last e
  }

  // range check und antiwindup stellgroessenbeschraenkung
  // verhindern das der integrier weiter hochlaeuft
  // deshalb die vorher addierten werte wieder abziehen
  if (zl > vmax_)
  {
    zl = vmax_;
    int_el -= el * dt;
  }
  if (zr > vmax_)
  {
    zr = vmax_;
    int_er -= er * dt;
  }
  if (zl < -vmax_)
  {
    zl = -vmax_;
    int_el -= el * dt;
  }
  if (zr < -vmax_)
  {
    zr = -vmax_;
    int_er -= er * dt;
  }

  // reduzieren

  double step_max = vmax_ * 0.5;
  /* kraft begrenzung damit die Kette nicht springt bzw
     der Motor ein wenig entlastet wird. bei vorgabe von max
     geschwindigkeit braucht es so 5 * 10 ms bevor die Maximale
     Kraft anliegt */
  if ((zl - last_zl) > step_max)
  {
    zl = last_zl + step_max;
  }
  if ((zl - last_zl) < -step_max)
  {
    zl = last_zl - step_max;
  }
  if ((zr - last_zr) > step_max)
  {
    zr = last_zr + step_max;
  }
  if ((zr - last_zr) < -step_max)
  {
    zr = last_zr - step_max;
  }

  // store old val for deviation plotting
  last_v_l_ist = _v_l_ist;
  last_v_r_ist = _v_r_ist;
  last_zl = zl;
  last_zr = zr;

  set_wheel_speed1(zl, zr, 0, 0);
}

void Kurt::set_wheel_speed2_mc(double _v_l_soll, double _v_r_soll, double _omega,
    double _AntiWindup)
{
  // divisor for floating points; will only send integer values ([+-]32e3) to Kurt
  const long factor = 100;
  int left_speed = (int)(_v_l_soll * factor);
  int right_speed = (int)(_v_r_soll * factor);
  int omega = (int)(factor * _omega / M_PI);
  int AntiWindup = (int)_AntiWindup;
  int sign   = omega < 0      ? 1 : 0;
  int windup = AntiWindup > 0 ? 1 : 0;

  omega = abs(omega);
  omega = (omega << 1) + sign;
  omega = (omega << 1) + windup;

  can_frame frame;
  frame.can_id = CAN_CONTROL;
  frame.can_dlc = 8;
  frame.data[0] = SPEED_CM >> 8;
  frame.data[1] = SPEED_CM;
  frame.data[2] = left_speed >> 8;
  frame.data[3] = left_speed;
  frame.data[4] = right_speed >> 8;
  frame.data[5] = right_speed;
  frame.data[6] = omega >> 8;
  frame.data[7] = omega;

  if(!can_.send_frame(&frame))
  {
    ROS_ERROR("set_wheel_speed2_mc: Error sending speed");
  }
}

// reads init data from pmw to speed experiment
bool Kurt::read_speed_to_pwm_leerlauf_tabelle(const std::string &filename, int *nr, double **v_pwm_l, double **v_pwm_r)
{
  int i;
  FILE *fpr_fftable = NULL;
  double t, v, vl, vr;
  int pwm;

  ROS_INFO("Open FeedForwardTabelle: %s", filename.c_str());
  fpr_fftable = fopen(filename.c_str(), "r");

  if (fpr_fftable == NULL)
  {
    ROS_ERROR("ERROR opening speedtable.");
    return false;
  }

  *nr = 1025; //TODO use global nr_v
  *v_pwm_l = (double *)calloc(*nr, sizeof(double));
  *v_pwm_r = (double *)calloc(*nr, sizeof(double));
  for (i = 0; i < *nr; i++)
  {
    //TODO ignoring return value
    fscanf(fpr_fftable, "%lf %lf %lf %lf %d", &t, &v, &vl, &vr, &pwm);
    (*v_pwm_l)[pwm] = vl;
    (*v_pwm_r)[pwm] = vr;
  }
  fclose(fpr_fftable);
  return true;
}

// generate reverse speedtable
void Kurt::make_pwm_v_tab(int nr, double *v_pwm_l, double *v_pwm_r, int nr_v, int **pwm_v_l, int **pwm_v_r, double *v_max)
{
  int i, index_l, index_r;
  //int last_l, last_r;
  double v_max_l = 0.0, v_max_r = 0.0;
  *v_max = 0.0;

  *pwm_v_l = (int *)calloc(nr_v + 1, sizeof(int));
  *pwm_v_r = (int *)calloc(nr_v + 1, sizeof(int));

  // check max wheel speed left right wheel
  for (i = 0; i < nr; i++)
  {
    if (v_pwm_l[i] > v_max_l)
      v_max_l = v_pwm_l[i];
    if (v_pwm_r[i] > v_max_r)
      v_max_r = v_pwm_r[i];
    *v_max = std::min(v_max_l, v_max_r);
  }

  // diskretisieren
  for (i = 0; i < nr; i++)
  {
    index_l = std::min(nr_v, std::max(0, (int)((v_pwm_l[i]) * (nr_v) / *v_max)));
    (*pwm_v_l)[index_l] = i;
    index_r = std::min(nr_v, std::max(0, (int)((v_pwm_r[i]) * (nr_v) / *v_max)));
    (*pwm_v_r)[index_r] = i;
  }
  (*pwm_v_l)[0] = 0; // for a stop
  (*pwm_v_r)[0] = 0;

  // interpolate blank entries
  //last_l = (*pwm_v_l)[0];
  //last_r = (*pwm_v_r)[0];
  for (i = 1; i < nr_v; i++)
  {
    // interpolation simple methode, pwm must be increasing
    if ((*pwm_v_l)[i] < (*pwm_v_l)[i - 1])
      (*pwm_v_l)[i] = (*pwm_v_l)[i - 1];
    if ((*pwm_v_r)[i] < (*pwm_v_r)[i - 1])
      (*pwm_v_r)[i] = (*pwm_v_r)[i - 1];
    /* interpolation advancde methode
       if ((*pwm_v_l)[i] == 0) {
       for (j = i+1; j <= nr_v; j++)
       if ((*pwm_v_l)[j] != 0) { next_l = (*pwm_v_l)[j]; j = nr_v; }
       (*pwm_v_l)[i] = (next_l + last_l ) / 2;
       } else last_l = (*pwm_v_l)[i];
       if ((*pwm_v_r)[i] == 0) {
       for (j = i+1; j <= nr_v; j++)
       if ((*pwm_v_r)[j] != 0) {next_r = (*pwm_v_r)[j];j = nr_v;}
       (*pwm_v_r)[i] = (next_r + last_r ) / 2;
       } else last_r = (*pwm_v_r)[i];
       */
  }
}

void Kurt::odometry(int wheel_a, int wheel_b)
{
  //TODO use timer
  // time_diff in sec; we hope kurt is precise ?? !! and sends every 10 ms
  double time_diff = 0.01;

  // store wheel position for published joint states
  double wheelpos_l = 2.0 * M_PI * wheel_a / ticks_per_turn_of_wheel_;
  if (wheelpos_l > M_PI)
    wheelpos_l -= 2.0 * M_PI;
  if (wheelpos_l < -M_PI)
    wheelpos_l += 2.0 * M_PI;

  double wheelpos_r = 2 * M_PI * wheel_b / ticks_per_turn_of_wheel_;
  if (wheelpos_r > M_PI)
    wheelpos_r -= 2.0 * M_PI;
  if (wheelpos_r < -M_PI)
    wheelpos_r += 2.0 * M_PI;

  // covered distance of wheels in meter
  double wheel_L = wheel_perimeter_ * wheel_a / ticks_per_turn_of_wheel_;
  double wheel_R = wheel_perimeter_ * wheel_b / ticks_per_turn_of_wheel_;

  // calc different speeds in meter / sec
  v_encoder_left_ = wheel_L / time_diff;
  v_encoder_right_ = wheel_R / time_diff;
  double v_encoder = (v_encoder_right_ + v_encoder_left_) * 0.5;
  // mehr Bahn als Winkelgeschwindigkeit
  double v_encoder_angular = (v_encoder_right_ - v_encoder_left_) * 0.5;

  // calc position deltas
  double local_dx, local_dz, dtheta_y = 0.0;
  const double EPSILON = 0.0001;

  if (fabs(wheel_L - wheel_R) < EPSILON)
  {
    if (fabs(wheel_L) < EPSILON)
    {
      local_dx = 0.0;
      local_dz = 0.0;
    }
    else // beide fast gleich (wheel_distance_a == wheel_distance_b)
    {
      local_dx = 0.0;
      local_dz = (wheel_L + wheel_R) * 0.5;
    }
  }
  else // (wheel_distance_a != wheel_distance_b) and > 0
  {
    double hypothenuse = 0.5 * (wheel_L + wheel_R);

    dtheta_y = (wheel_L - wheel_R) / axis_length_ * turning_adaptation_;

    local_dx = hypothenuse * sin(dtheta_y);
    local_dz = hypothenuse * cos(dtheta_y);
  }

  // Odometrie : Koordinatentransformation in Weltkoordinaten
  static double x_from_encoder = 0.0;
  static double z_from_encoder = 0.0;
  static double theta_from_encoder = 0.0;

  x_from_encoder += local_dx * cos(theta_from_encoder) + local_dz * sin(theta_from_encoder);
  z_from_encoder += -local_dx * sin(theta_from_encoder) + local_dz * cos(theta_from_encoder);

  theta_from_encoder += dtheta_y;
  if (theta_from_encoder > M_PI)
    theta_from_encoder -= 2.0 * M_PI;
  if (theta_from_encoder < -M_PI)
    theta_from_encoder += 2.0 * M_PI;

  ros_send_odometry(z_from_encoder, x_from_encoder, theta_from_encoder, v_encoder, v_encoder_angular, wheelpos_l, wheelpos_r);
}

////////////////// rotunit //////////////////////////////////////

void Kurt::can_rotunit_send(int speed)
{
  can_frame frame;
  frame.can_id = CAN_SETROTUNT;
  frame.can_dlc = 8;
  frame.data[0] = (speed >> 8); //high
  frame.data[1] = (speed & 0xFF); //low
  frame.data[2] = 0;
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  if(!can_.send_frame(&frame))
  {
    ROS_ERROR("can_rotunit_send: Error sending rotunit speed");
  }
  else
  {
    use_rotunit_ = true;
  }
}

void Kurt::can_rotunit(const can_frame &frame)
{
  int rot = (frame.data[1] << 8) + frame.data[2];
  double rot2 = rot * 2 * M_PI / 10240;
  ros_send_rotunit(rot2);
}

//////////////////// Kurt Sensor ////////////////////////////////

void Kurt::can_encoder(const can_frame &frame)
{
  int left_encoder = 0, right_encoder = 0;
  if (frame.data[0] & 0x80) // negative Zahl auf 15 Bit genau
    left_encoder = (frame.data[0] << 8) + frame.data[1]-65536;
  else
    left_encoder = (frame.data[0] << 8) + frame.data[1];

  if (frame.data[2] & 0x80) // negative Zahl auf 15 Bit genau
    right_encoder = (frame.data[2] << 8) + frame.data[3]-65536;
  else
    right_encoder = (frame.data[2] << 8) + frame.data[3];

  odometry(left_encoder, right_encoder);
}

int Kurt::normalize_ir(int ir)
{
  if (ir < IR_MIN * 1000 || ir > IR_MAX * 1000) // convert m --> mm
  {
    return -1;
  }
  return (int)(pow(30000.0 / ((double)ir - 10.0), 1.0 / 1.4) - 10.0);
}

int Kurt::normalize_sonar(int s)
{
  if (s < SONAR_MIN * 1000 || s > SONAR_MAX * 1000) // convert m --> mm
  {
    return -1;
  }
  return (int)((double)s * 0.110652 + 11.9231);
}

void Kurt::can_sonar8_9(const can_frame &frame)
{
  int sonar1 = normalize_ir((frame.data[2] << 8) + frame.data[3]);

  ros_send_sonar_leftBack(sonar1);
}

void Kurt::can_sonar4_7(const can_frame &frame)
{
  int sonar0 = normalize_ir((frame.data[0] << 8) + frame.data[1]);
  int sonar1 = normalize_sonar((frame.data[2] << 8) + frame.data[3]);
  int sonar2 = normalize_ir((frame.data[4] << 8) + frame.data[5]);
  int sonar3 = normalize_ir((frame.data[6] << 8) + frame.data[7]);

  ros_send_sonar_front_usound_leftFront_left(sonar0, sonar1, sonar2, sonar3);
}

void Kurt::can_sonar0_3(const can_frame &frame)
{
  int sonar0 = normalize_ir((frame.data[0] << 8) + frame.data[1]);
  int sonar1 = normalize_ir((frame.data[2] << 8) + frame.data[3]);
  int sonar2 = normalize_ir((frame.data[4] << 8) + frame.data[5]);

  ros_send_sonar_back_rightBack_rightFront(sonar0, sonar1, sonar2);
}

void Kurt::can_tilt_comp(const can_frame &frame)
{
  double a0, a1;
  unsigned int t0, t1;

  t0 = (frame.data[0] << 8) + (frame.data[1]);
  t1 = (frame.data[2] << 8) + (frame.data[3]);
  //int comp1 = (frame.data[4] << 8) + frame.data[5];
  //int comp2 = (frame.data[6] << 8) + frame.data[7];

  // calculate g values (offset and sensitivity correction)
  a0 = ((double)t0 - 32768.0) / 3932.0;
  a1 = ((double)t1 - 32768.0) / 3932.0;

  // calculate angles and convert do deg
  double tilt_lr = asin(a0);
  double tilt_fb = asin(a1);

  double roll = tilt_lr * 180.0 / M_PI;
  double pitch = tilt_fb * 180.0 / M_PI;
  ros_send_pitch_roll(pitch, roll);
}

void Kurt::can_gyro_mc1(const can_frame &frame)
{
  static int gyro_offset_read = 0;
  static double offset, delta; // initial offset
  signed long gyro_raw;

  gyro_raw = (frame.data[0] << 24) + (frame.data[1] << 16)
    + (frame.data[2] << 8)  + (frame.data[3]);
  double theta = (double)gyro_raw / 4992511.0 * M_PI / 180.0;

  // sigma is not used because its undocumented..
  // correction factor needed for gyro_sigma
  /*double sigma_deg = (double)((frame.data[4] << 24) + (frame.data[5] << 16)
      + (frame.data[6] << 8)  + (frame.data[7])) / 10000;

  double tmp = (sqrt(sigma_deg) * M_PI / 180.0);
  double sigma = tmp * tmp;*/

  // wait until gyro is stable
  if (gyro_offset_read++ < 100)
    return;

  if(gyro_offset_read == 100)
  {
    offset = theta;
    delta = offset / 100.0;
    if (delta >  M_PI) delta -= 2.0 * M_PI;
    if (delta < -M_PI) delta += 2.0 * M_PI;
  }

  offset += delta;
  if (offset >  M_PI) offset -= 2.0 * M_PI;
  if (offset < -M_PI) offset += 2.0 * M_PI;

  theta -= offset;

  if (theta >  M_PI) theta -= 2.0 * M_PI;
  if (theta < -M_PI) theta += 2.0 * M_PI;

  ros_send_gyro(theta);
}

void Kurt::can_read_fifo()
{
  can_frame frame;

  if(!can_.receive_frame(&frame))
    return;

  switch (frame.can_id) {
    case CAN_ADC00_03:
      can_sonar0_3(frame);
      break;
    case CAN_ADC04_07:
      can_sonar4_7(frame);
      break;
    case CAN_ADC08_11:
      can_sonar8_9(frame);
      break;
    case CAN_ENCODER:
      can_encoder(frame);
      break;
    case CAN_TILT_COMP:
      can_tilt_comp(frame);
      break;
    case CAN_GYRO_MC1:
      can_gyro_mc1(frame);
      break;
    case CAN_GETROTUNIT:
      can_rotunit(frame);
      break;
    /*case CAN_CONTROL:
      ROS_DEBUG("can_read_fifo: Unused CAN message ID: %X (control message)", frame.can_id);
      break;
    case CAN_INFO_1:
      ROS_DEBUG("can_read_fifo: Unused CAN message ID: %X (info message)", frame.can_id);
      break;
    case CAN_ADC12_15:
      ROS_DEBUG("can_read_fifo: Unused CAN message ID: %X (analog input channels: 12 - 15)", frame.can_id);
      break;
    case CAN_BUMPERC:
      ROS_DEBUG("can_read_fifo: Unused CAN message ID: %X (bumpers and remote control)", frame.can_id);
      break;
    case CAN_BDC00_03:
      ROS_DEBUG("can_read_fifo: Unused CAN message ID: %X (analog input channels: 0 - 3)", frame.can_id);
      break;
    case CAN_BDC04_07:
      ROS_DEBUG("can_read_fifo: Unused CAN message ID: %X (analog input channels: 4 - 7)", frame.can_id);
      break;
    case CAN_BDC08_11:
      ROS_DEBUG("can_read_fifo: Unused CAN message ID: %X (analog input channels: 8 - 11)", frame.can_id);
      break;
    case CAN_BDC12_15:
      ROS_DEBUG("can_read_fifo: Unused CAN message ID: %X (analog input channels: 12 - 15)", frame.can_id);
      break;
    case CAN_GYRO_MC2:
      ROS_DEBUG("can_read_fifo: Unused CAN message ID: %X (data from gyro connected to 2nd C167)", frame.can_id);
      break;
    case CAN_DEADRECK:
      ROS_DEBUG("can_read_fifo: Unused CAN message ID: %X (position (dead reckoning))", frame.can_id);
      break;
    case CAN_GETSPEED:
      ROS_DEBUG("can_read_fifo: Unused CAN message ID: %X current transl. and rot. speed)", frame.can_id);
      break;
    default:
      ROS_DEBUG("can_read_fifo: Unknown CAN ID: %X", frame.can_id);*/
  }
}

//////////////////// ROS //////////////////////////////////////

void Kurt::setTFPrefix(const std::string &tf_prefix)
{
  publish_tf_ = true;
  tf_prefix_ = tf_prefix;
}

void Kurt::velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double AntiWindup = 1.0;
  double v_l_soll = msg->linear.x - axis_length_ * msg->angular.z /*/ wheelRadius*/;
  double v_r_soll = msg->linear.x + axis_length_ * msg->angular.z/*/wheelRadius*/;

  if (msg->linear.x == 0 && msg->angular.z == 0)
  {
    AntiWindup = 0.0;
  }

  if (use_microcontroller_)
  {
    set_wheel_speed2_mc(v_l_soll, v_r_soll, 0, AntiWindup);
  }
  else
  {
    set_wheel_speed2(v_l_soll, v_r_soll, v_encoder_left_, v_encoder_right_, 0, AntiWindup);
  }
}

void Kurt::rotunitCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  can_rotunit_send(msg->angular.z);
}

void Kurt::populateCovariance(nav_msgs::Odometry &msg, double v_encoder, double v_encoder_angular)
{
  double odom_multiplier = 1.0;

  if (fabs(v_encoder) <= 1e-8 && fabs(v_encoder_angular) <= 1e-8)
  {
    //nav_msgs::Odometry has a 6x6 covariance matrix
    msg.twist.covariance[0] = 1e-12;
    msg.twist.covariance[35] = 1e-12;

    msg.twist.covariance[30] = 1e-12;
    msg.twist.covariance[5] = 1e-12;
  }
  else
  {
    //nav_msgs::Odometry has a 6x6 covariance matrix
    msg.twist.covariance[0] = odom_multiplier * pow(sigma_x_, 2);
    msg.twist.covariance[35] = odom_multiplier * pow(sigma_theta_, 2);

    msg.twist.covariance[30] = odom_multiplier * cov_x_theta_;
    msg.twist.covariance[5] = odom_multiplier * cov_x_theta_;
  }

  msg.twist.covariance[7] = DBL_MAX;
  msg.twist.covariance[14] = DBL_MAX;
  msg.twist.covariance[21] = DBL_MAX;
  msg.twist.covariance[28] = DBL_MAX;

  msg.pose.covariance = msg.twist.covariance;

  if (fabs(v_encoder) <= 1e-8 && fabs(v_encoder_angular) <= 1e-8)
  {
    msg.pose.covariance[7] = 1e-12;

    msg.pose.covariance[1] = 1e-12;
    msg.pose.covariance[6] = 1e-12;

    msg.pose.covariance[31] = 1e-12;
    msg.pose.covariance[11] = 1e-12;
  }
  else
  {
    msg.pose.covariance[7] = odom_multiplier * pow(sigma_x_, 2) * pow(sigma_theta_, 2);

    msg.pose.covariance[1] = odom_multiplier * cov_x_y_;
    msg.pose.covariance[6] = odom_multiplier * cov_x_y_;

    msg.pose.covariance[31] = odom_multiplier * cov_y_theta_;
    msg.pose.covariance[11] = odom_multiplier * cov_y_theta_;
  }
}

void Kurt::ros_send_odometry(double z, double x, double theta, double v_encoder, double v_encoder_angular, double wheelpos_l, double wheelpos_r)
{
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom_combined";
  odom.child_frame_id = "base_footprint";

  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = z;
  odom.pose.pose.position.y = -x;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(-theta);

  odom.twist.twist.linear.x = v_encoder;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = v_encoder_angular;
  populateCovariance(odom, v_encoder, v_encoder_angular);

  odom_pub.publish(odom);

  if (publish_tf_)
  {
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = tf::resolve(tf_prefix_, "odom_combined");
    odom_trans.child_frame_id = tf::resolve(tf_prefix_, "base_footprint");

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = z;
    odom_trans.transform.translation.y = -x;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(-theta);

    odom_broadcaster.sendTransform(odom_trans);
  }

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(6);
  joint_state.position.resize(6);
  joint_state.name[0] = "left_front_wheel_joint";
  joint_state.name[1] = "left_middle_wheel_joint";
  joint_state.name[2] = "left_rear_wheel_joint";
  joint_state.name[3] = "right_front_wheel_joint";
  joint_state.name[4] = "right_middle_wheel_joint";
  joint_state.name[5] = "right_rear_wheel_joint";

  joint_state.position[0] = joint_state.position[1] = joint_state.position[2] = wheelpos_l;
  joint_state.position[3] = joint_state.position[4] = joint_state.position[5] = wheelpos_r;

  // note: we reuse joint_state here, i.e., we modify joint_state after publishing.
  // this is only safe as long as nothing in the same process subscribes to the
  // joint_states topic. same for imu above.
  joint_pub.publish(joint_state);
}

void Kurt::ros_send_sonar_leftBack(int ir_left_back)
{
  sensor_msgs::Range range;
  range.header.stamp = ros::Time::now();

  range.header.frame_id = "ir_left_back";
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = IR_FOV;
  range.min_range = IR_MIN;
  range.max_range = IR_MAX;
  range.range = ir_left_back / 1000.0;
  range_pub.publish(range);
}

void Kurt::ros_send_sonar_front_usound_leftFront_left(int ir_right_front, int usound, int ir_left_front, int ir_left)
{
  sensor_msgs::Range range;
  range.header.stamp = ros::Time::now();

  range.header.frame_id = "ir_right_front";
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = IR_FOV;
  range.min_range = IR_MIN;
  range.max_range = IR_MAX;
  range.range = ir_right_front / 1000.0;
  range_pub.publish(range);

  range.header.frame_id = "ultrasound_front";
  range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range.field_of_view = SONAR_FOV;
  range.min_range = SONAR_MIN;
  range.max_range = SONAR_MAX;
  range.range = usound / 1000.0;
  range_pub.publish(range);

  range.header.frame_id = "ir_left_front";
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = IR_FOV;
  range.min_range = IR_MIN;
  range.max_range = IR_MAX;
  range.range = ir_left_front / 1000.0;
  range_pub.publish(range);

  range.header.frame_id = "ir_left";
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = IR_FOV;
  range.min_range = IR_MIN;
  range.max_range = IR_MAX;
  range.range = ir_left / 1000.0;
  range_pub.publish(range);
}

void Kurt::ros_send_sonar_back_rightBack_rightFront(int ir_back, int ir_right_back, int ir_right)
{
  sensor_msgs::Range range;
  range.header.stamp = ros::Time::now();

  range.header.frame_id = "ir_back";
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = IR_FOV;
  range.min_range = IR_MIN;
  range.max_range = IR_MAX;
  range.range = ir_back / 1000.0;
  range_pub.publish(range);

  range.header.frame_id = "ir_right_back";
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = IR_FOV;
  range.min_range = IR_MIN;
  range.max_range = IR_MAX;
  range.range = ir_right_back / 1000.0;
  range_pub.publish(range);

  range.header.frame_id = "ir_right";
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = IR_FOV;
  range.min_range = IR_MIN;
  range.max_range = IR_MAX;
  range.range = ir_right / 1000.0;
  range_pub.publish(range);
}

void Kurt::ros_send_pitch_roll(double pitch, double roll)
{
  //TODO
}

void Kurt::ros_send_gyro(double theta)
{
  sensor_msgs::Imu imu;

  // this is intentionally base_link (the location of the imu) and not base_footprint,
  // but because they are connected by a fixed link, it doesn't matter
  imu.header.frame_id = "base_link";
  imu.header.stamp = ros::Time::now();

  imu.angular_velocity_covariance[0] = -1; // no data avilable, see Imu.msg
  imu.linear_acceleration_covariance[0] = -1;

  imu.orientation = tf::createQuaternionMsgFromYaw(theta);
  imu_pub.publish(imu);
}

void Kurt::ros_send_rotunit(double rot)
{
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(1);
  joint_state.position.resize(1);
  joint_state.name[0] = "laser_rot_joint";
  joint_state.position[0] = rot;

  joint_pub.publish(joint_state);
}

//////////////// main //////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kurt_base");
  ros::NodeHandle n;
  ros::NodeHandle nh_ns("~");

  //Odometry parameter (defaults for kurt2 indoor)
  double wheel_perimeter;
  nh_ns.param("wheel_perimeter", wheel_perimeter, 37.9);
  double axis_length;
  nh_ns.param("axis_length", axis_length, 28.0);
  // parameter still in cm, converting to meter
  wheel_perimeter *= 0.01;
  axis_length *= 0.01;

  double turning_adaptation;
  nh_ns.param("turning_adaptation", turning_adaptation, 0.69);
  int ticks_per_turn_of_wheel;
  nh_ns.param("ticks_per_turn_of_wheel", ticks_per_turn_of_wheel, 21950);

  double sigma_x, sigma_theta, cov_x_y, cov_x_theta, cov_y_theta;
  nh_ns.param("x_stddev", sigma_x, 0.002);
  nh_ns.param("rotation_stddev", sigma_theta, 0.017);
  nh_ns.param("cov_xy", cov_x_y, 0.0);
  nh_ns.param("cov_xrotation", cov_x_theta, 0.0);
  nh_ns.param("cov_yrotation", cov_y_theta, 0.0);

  Kurt kurt(wheel_perimeter, axis_length, turning_adaptation, ticks_per_turn_of_wheel, sigma_x, sigma_theta, cov_x_y, cov_x_theta, cov_y_theta, n);

  //PID parameter (disables micro controller)
  std::string speedPwmLeerlaufTable;
  if (nh_ns.getParam("speedtable", speedPwmLeerlaufTable))
  {
    double feedforward_turn;
    nh_ns.param("feedforward_turn", feedforward_turn, 0.35);
    double ki, kp;
    nh_ns.param("ki", ki, 3.4);
    nh_ns.param("kp", kp, 0.4);
    kurt.setPWMData(speedPwmLeerlaufTable, feedforward_turn, ki, kp);
  }

  bool use_rotunit;
  nh_ns.param("use_rotunit", use_rotunit, false);
  if (use_rotunit) {
    int rotunit_speed;
    nh_ns.param("InitialSpeed", rotunit_speed, 42);
    kurt.can_rotunit_send(rotunit_speed);
  }

  bool publish_tf;
  nh_ns.param("publish_tf", publish_tf, true);
  if (publish_tf)
  {
    std::string prefix_param, tf_prefix;
    n.searchParam("tf_prefix", prefix_param);
    n.getParam(prefix_param, tf_prefix);
    kurt.setTFPrefix(tf_prefix);
  }

  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, &Kurt::velCallback, &kurt);
  ros::Subscriber rot_vel_sub;
  if (use_rotunit)
    rot_vel_sub = n.subscribe("rot_vel", 10, &Kurt::rotunitCallback, &kurt);

  kurt.run();

  return 0;
}
