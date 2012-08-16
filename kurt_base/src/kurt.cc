#include <ros/console.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>

#include "comm.h"
#include "kurt.h"

Kurt::~Kurt()
{
  if(use_rotunit_)
    can_rotunit_send(0.0);
  k_hard_stop();
  if(!use_microcontroller_)
  {
    free(pwm_v_l_);
    free(pwm_v_r_);
  }
}

bool Kurt::setPWMData(const std::string &speedPwmLeerlaufTable, double feedforward_turn, double ki, double kp)
{
  ki_l = ki_r = ki;
  kp_l = kp_r = kp;

  int nr;
  double *v_pwm_l, *v_pwm_r;
  if (!read_speed_to_pwm_leerlauf_tabelle(speedPwmLeerlaufTable, &nr, &v_pwm_l, &v_pwm_r))
  {
    return false;
  }
  make_pwm_v_tab(nr, v_pwm_l, v_pwm_r, nr_v_, &pwm_v_l_, &pwm_v_r_, &vmax_);
  free(v_pwm_l);
  free(v_pwm_r);
  use_microcontroller_ = false;

  return true;
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
    pwm_left = std::max(0, std::min(1023, 1024 - pwm_v_l_[index_l] - leerlauf_adapt_ - integration_l));
  else
    pwm_left = 1023;
  if (fabs(v_r) > 0.01)
    pwm_right = std::max(0, std::min(1023, 1024 - pwm_v_r_[index_r] - leerlauf_adapt_ - integration_r));
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
  double dt = 0.01;
  // filter fuer gueltige Werte
  static double last_v_l_ist = 0.0, last_v_r_ist = 0.0;
  // static int reached = 0;
  static double v_l_list[MAX_V_LIST], v_r_list[MAX_V_LIST];
  static int vl_index = 0, vr_index = 0;
  int i;
  static double f_v_l_ist, f_v_r_ist;
  // kd_l and kd_r allways 0 (using only pi controller here)
  double kd_l = 0.0, kd_r = 0.0; // nur pi regler d-anteil ausblenden

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

void Kurt::set_wheel_speed(double _v_l_soll, double _v_r_soll, double _AntiWindup)
{
  if (use_microcontroller_)
  {
    set_wheel_speed2_mc(_v_l_soll, _v_r_soll, 0, _AntiWindup);
  }
  else
  {
    set_wheel_speed2(_v_l_soll, _v_r_soll, v_encoder_left_, v_encoder_right_, 0, _AntiWindup);
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

  *nr = 1025;
  *v_pwm_l = (double *)calloc(*nr, sizeof(double));
  *v_pwm_r = (double *)calloc(*nr, sizeof(double));
  for (i = 0; i < *nr; i++)
  {
    if(fscanf(fpr_fftable, "%lf %lf %lf %lf %d", &t, &v, &vl, &vr, &pwm) == EOF)
    {
      ROS_ERROR("ERROR reading speedtable.");
      fclose(fpr_fftable);
      return false;
    }
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
  // time_diff in sec; we hope kurt is precise ?? !! and sends every 10 ms
  double time_diff = 0.01;

  // covered distance of wheels in meter
  double wheel_L = wheel_perimeter_ * wheel_a / ticks_per_turn_of_wheel_;
  double wheel_R = wheel_perimeter_ * wheel_b / ticks_per_turn_of_wheel_;

  // calc different speeds in meter / sec
  v_encoder_left_ = wheel_L / time_diff;
  v_encoder_right_ = wheel_R / time_diff;
  double v_encoder = (v_encoder_right_ + v_encoder_left_) * 0.5;
  // angular velocity in rad/s
  double v_encoder_angular = (v_encoder_right_ - v_encoder_left_) / axis_length_ * turning_adaptation_;

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

  comm_.send_odometry(z_from_encoder, x_from_encoder, theta_from_encoder, v_encoder, v_encoder_angular, wheel_a, wheel_b, v_encoder_left_, v_encoder_right_);
}

////////////////// rotunit //////////////////////////////////////

void Kurt::can_rotunit_send(double speed)
{
  int ticks =  (int)(speed / (2.0 * M_PI) * 10240 / 20);
  can_frame frame;
  frame.can_id = CAN_SETROTUNT;
  frame.can_dlc = 8;
  frame.data[0] = (ticks >> 8); //high
  frame.data[1] = (ticks & 0xFF); //low
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
  comm_.send_rotunit(rot2);
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

  comm_.send_sonar_leftBack(sonar1);
}

void Kurt::can_sonar4_7(const can_frame &frame)
{
  int sonar0 = normalize_ir((frame.data[0] << 8) + frame.data[1]);
  int sonar1 = normalize_sonar((frame.data[2] << 8) + frame.data[3]);
  int sonar2 = normalize_ir((frame.data[4] << 8) + frame.data[5]);
  int sonar3 = normalize_ir((frame.data[6] << 8) + frame.data[7]);

  comm_.send_sonar_front_usound_leftFront_left(sonar0, sonar1, sonar2, sonar3);
}

void Kurt::can_sonar0_3(const can_frame &frame)
{
  int sonar0 = normalize_ir((frame.data[0] << 8) + frame.data[1]);
  int sonar1 = normalize_ir((frame.data[2] << 8) + frame.data[3]);
  int sonar2 = normalize_ir((frame.data[4] << 8) + frame.data[5]);

  comm_.send_sonar_back_rightBack_rightFront(sonar0, sonar1, sonar2);
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
  comm_.send_pitch_roll(pitch, roll);
}

void Kurt::can_gyro_mc1(const can_frame &frame)
{
  static int gyro_offset_read = 0;
  static double offset, delta; // initial offset
  signed long gyro_raw;

  gyro_raw = (frame.data[0] << 24) + (frame.data[1] << 16)
    + (frame.data[2] << 8)  + (frame.data[3]);
  double theta = (double)gyro_raw / 4992511.0 * M_PI / 180.0;

  double sigma_deg = (double)((frame.data[4] << 24) + (frame.data[5] << 16)
      + (frame.data[6] << 8)  + (frame.data[7])) / 10000;

  double tmp = (sqrt(sigma_deg) * M_PI / 180.0);
  double sigma = tmp * tmp;

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

  comm_.send_gyro(theta, sigma);
}

int Kurt::can_read_fifo()
{
  can_frame frame;

  if(!can_.receive_frame(&frame))
    return -1;

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

  return frame.can_id;
}
