#include "kurt2can.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
//#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>

#include <sys/time.h>
#include <signal.h>
#include <stdlib.h>

#include <cmath>
#include <string>
using std::string;
#include <algorithm>
using std::min;
using std::max;

const double EPSILON = 0.000001;
const int MAX_V_LIST = 200;
const int nr_v = 1000;

// speed (controller)
double v_l_soll = 0.0, v_r_soll = 0.0;
double v_l_ist = 0.0, v_r_ist = 0.0;
// poses from sensors
double theta_from_encoder = 0.0, x_from_encoder = 0.0, z_from_encoder = 0.0;

// robot states
double AntiWindup = 1.0;

// data fro gyro and kalman filter
double theta_alt_from_encoder = 0.0, theta_alt_from_gyro = 0.0;
double theta_neu_from_gyro, sigma_from_gyro; //sigma is not used because its undocumented..
double theta_from_gyro = 0.0, x_from_gyro = 0.0, z_from_gyro = 0.0;
double gyro_fehler = 0.0;

// various
bool gyro_offset_read = false;

// tilt information
double pitch = 0.0, roll = 0.0;

long double Time_Diff = 0.0;

double local_dx = 0.0;
double local_dz = 0.0;

MOTOR_CTRL kurt2_ctrl;
double vmax;
int *pwm_v_l, *pwm_v_r;
double kp_l, kp_r; // schnell aenderung folgen
double ki_l, ki_r; // integrierer relative langsam
int leerlauf_adapt = 0;
double feedforward_turn; // in v = m/s
double step_max;
bool use_microcontroller;
int ticks_per_turn_of_wheel;
double wheel_perimeter;
double axis_length;
double turning_adaptation;

int IRdist[10], IRnr;

bool use_gyro;

bool read_speed_to_pwm_leerlauf_tabelle(string &filename, int *nr, double **v_pwm_l, double **v_pwm_r);
void make_pwm_v_tab(int nr, double *v_pwm_l, double *v_pwm_r, int nr_v, int **pwm_v_l, int **pwm_v_r, double *v_max);
int k_can_init(void);
bool k_read_wheel_encoder (long *channel_1, long *channel_2);
bool odometry();
void gyro();
bool infrared_sonar();
void get_tilt();
void velCallback(const geometry_msgs::Twist::ConstPtr& msg);
void set_values();
void k_can_close(void);
void set_wheel_speed2(double _v_l_soll, double _v_r_soll,
    double _v_l_ist, double _v_r_ist,
    double _omega, long double Time, double _AntiWindup);
void set_wheel_speed2_mc(double _v_l_soll, double _v_r_soll,
    double _v_l_ist, double _v_r_ist,
    double _omega, long double Time, double _AntiWindup);
bool local_odometry(double &v_encoder, double &v_encoder_left, double &v_encoder_right, double &dx, double &dz, double &dtheta_y, double &wheel_L, double &wheel_R);

unsigned long GetCurrentTimeInMilliSec(void)
{
  static struct timeval tv;
  static unsigned long milliseconds;
  gettimeofday(&tv, NULL);
  milliseconds = tv.tv_sec * 1000 + tv.tv_usec / 1000; //TODO wtf
  return milliseconds;
}

// Zeitmessung NR_TIMER -1 special since programm start
// timer 0 is for local use
long double Get_mtime_diff()
{
  static double   prev_clock;
  static int      first_time = 1;
  double          new_clock, clock_diff;

  if (first_time) {
    new_clock = GetCurrentTimeInMilliSec();
    prev_clock = new_clock;
    clock_diff =  new_clock;
    first_time = 0;
  }
  else {
    new_clock = GetCurrentTimeInMilliSec();
    clock_diff = new_clock - prev_clock;
    prev_clock = new_clock;
  }

  return(clock_diff);
}

void quit(int sig)
{
  ROS_INFO("close");
  k_can_close();
  free(pwm_v_l);
  free(pwm_v_r);
  exit(0);
}

int main(int argc, char** argv)
{
  double ki, kp;
  string speedPwmLeerlaufTable;
  ros::init(argc, argv, "kurt_base");
  ros::NodeHandle n;
  ros::NodeHandle nh_ns("~");
  nh_ns.param("wheel_perimeter", wheel_perimeter, 37.9);
  nh_ns.param("axis_length", axis_length, 28.0);
  nh_ns.param("turning_adaptation", turning_adaptation, 0.69);
  nh_ns.param("feedforward_turn", feedforward_turn, 0.35);
  nh_ns.param("ki", ki, 3.4);
  nh_ns.param("kp", kp, 0.4);
  nh_ns.param("ticks_per_turn_of_wheel", ticks_per_turn_of_wheel, 21950);
  nh_ns.param("use_microcontroller", use_microcontroller, false);
  nh_ns.param("use_gyro", use_gyro, false);
  if(!nh_ns.getParam("speedtable", speedPwmLeerlaufTable)) {
    ROS_ERROR("speedtable not set, aborting");
    return 1;
  }

  ros::Rate loop_rate(100);
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, velCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  //ros::Publisher ultrasound_pub = n.advertise<sensor_msgs::Range>("usound", 10);
  tf::TransformBroadcaster odom_broadcaster;

  ki_l = ki_r = ki;
  kp_l = kp_r = kp;

  int nr;
  double *v_pwm_l, *v_pwm_r;
  if(!read_speed_to_pwm_leerlauf_tabelle(speedPwmLeerlaufTable, &nr, &v_pwm_l, &v_pwm_r)) {
    return 2;
  }
  make_pwm_v_tab(nr, v_pwm_l, v_pwm_r, nr_v, &pwm_v_l, &pwm_v_r, &vmax);
  free(v_pwm_l);
  free(v_pwm_r);

  step_max = vmax * 0.5;

  long wheel_a = 0, wheel_b = 0;
  if (k_can_init() > 0) {
    ROS_ERROR("can not init can");
    return 1;
  }

  can_gyro_reset();
  can_gyro_calibrate();
  // adjust gyro, i.e. execute loop for a while
  // without executing any behavior

  if(!k_read_wheel_encoder(&wheel_a, &wheel_b)) {
    ROS_ERROR("error starting base (power on?)");
    return 1;
  }

  //TODO move to gyro()
  if(use_gyro) {
    ROS_INFO("Initializing gyroscope");
    for (int i = 0; i < 100; i++) { //TODO change 100 to 500?
      while (!odometry()) {
        usleep(100);
      };
      gyro();
      //infrared_sonar(); //TODO not needed?
      //get_tilt(); //TODO not needed?
      sleep(10);
    }
  }

  signal(SIGINT, quit);
  ROS_INFO("started");

  ros::Time current_time;
  double x, y, th;

  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  /*sensor_msgs::Range ultrasound;
  ultrasound.radiation_type = sensor_msgs::Range::ULTRASOUND;
  //TODO add sensor message
  ultrasound.field_of_view = 0;
  ultrasound.min_range = 0;
  ultrasound.max_range = 30;*/

  while (ros::ok()) {
    if (odometry()) {
      current_time = ros::Time::now();
      gyro();
      infrared_sonar();
      //get_tilt(); //TODO not needed?

      if(!use_gyro) {
        x = z_from_encoder/100.0;
        y = -x_from_encoder/100.0;
        th = -theta_from_encoder;
      } else {
        x = z_from_gyro/100.0;
        y = -x_from_gyro/100.0;
        th = -theta_from_gyro;
      }

      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

      odom_trans.header.stamp = current_time;
      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      odom_broadcaster.sendTransform(odom_trans);

      odom.header.stamp = current_time;
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //TODO change to real speed information
      odom.twist.twist.linear.x = v_r_ist;
      odom.twist.twist.linear.y = v_l_ist;
      odom.twist.twist.angular.z = (v_r_ist - v_l_ist) / (2.0 * 100.0);

      odom_pub.publish(odom);

      /*ultrasound.range = (1.0*IRdist[5])/100.0;
      ultrasound_pub.publish(ultrasound);*/

    }
    ros::spinOnce();
    //set_values();
    loop_rate.sleep();
  }

  return 0;
}

void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  v_l_soll = ((msg->linear.x - (axis_length / 100.0) * msg->angular.x) /*/ wheelRadius*/);
  v_r_soll = ((msg->linear.x + (axis_length / 100.0) * msg->angular.x)/*/wheelRadius*/);
  if(msg->linear.x == 0 && msg->angular.x == 0) {
    AntiWindup = 0.0;
  }
  set_values();
}

void set_values()
{
  /*
     v_l_soll = 0.1;
     cout << v_l_soll << " " << v_r_soll << endl;
     set_wheel_speed1(v_l_soll, v_r_soll, 0, 0);
     return;
     */
  if (use_microcontroller == 0) {
    set_wheel_speed2(v_l_soll, v_r_soll,
        v_l_ist, v_r_ist,
        0, Get_mtime_diff(), AntiWindup); //TODO use ros timer
    AntiWindup = 1.0;
  } else {
    set_wheel_speed2_mc(v_l_soll, v_r_soll,
        v_l_ist, v_r_ist,
        0, Get_mtime_diff(), AntiWindup); //TODO use ros timer
    AntiWindup = 1.0;
  }
}

void get_tilt()
{
  double tilt_lr, tilt_fb;
  unsigned long curr_time;
  int comp1_dummy, comp2_dummy;
  can_tilt_comp(&tilt_lr, &tilt_fb, &comp1_dummy, &comp2_dummy, &curr_time);
  tilt_lr = tilt_lr * 180.0 / M_PI;
  tilt_fb = tilt_fb * 180.0 / M_PI;
  roll = tilt_lr;
  pitch = tilt_fb;
}

void gyro()
{
  // beim allerersten Durchlauf Initialwerte einlesen
  if(!gyro_offset_read) {
    can_gyro_mc1(&theta_alt_from_gyro, &sigma_from_gyro, NULL);
    gyro_offset_read = true;
    sleep(10);
    can_gyro_mc1(&theta_neu_from_gyro, &sigma_from_gyro, NULL);
    return;
  }
  // Daten vom Gyro holen
  can_gyro_mc1(&theta_neu_from_gyro, &sigma_from_gyro, NULL);

  double dt_odo = theta_from_encoder - theta_alt_from_encoder;
  double fehler_gyro = gyro_fehler;
  // ist |dt_gyro| > |grenze_gyro| stellt der gyro eine kurve fest
  double grenze_gyro = 0.015;
  // ist |dt_odo| > |grenze_odo| stellt odometrie eine kurve fest
  double grenze_odo = 0.0006;
  double dt_gyro = theta_alt_from_gyro - theta_neu_from_gyro;

  // machen Raeder eine Kurve?
  if(fabs(dt_odo) > grenze_odo) {
    // Fehler rausrechnen
    dt_gyro -= fehler_gyro;
    // Gyro zur winkelbestimmung nutzen
    theta_from_gyro += dt_gyro;
  } // macht gyro eine kurve?
  else if (fabs(dt_gyro - fehler_gyro) > grenze_gyro) {
    // Fehler rausrechnen
    dt_gyro -= fehler_gyro;
    // Gyro zur Winkelbestimmung nutzen
    theta_from_gyro += dt_gyro;
  } // sonst relativ geradeaus, korrigiere gyro_fehler
  else {
    double fehler_gyro_neu = dt_gyro - dt_odo;
    // alter Fehler wird gewichtet mit eingerechnet
    fehler_gyro = 0.6 * fehler_gyro_neu + 0.4 * fehler_gyro;
    gyro_fehler = fehler_gyro;
    theta_from_gyro += dt_odo;
  }
  // Phasenkorrektur
  if(theta_from_gyro > M_PI) theta_from_gyro -= 2*M_PI;
  if(theta_from_gyro < -M_PI) theta_from_gyro += 2*M_PI;
  theta_alt_from_gyro = theta_neu_from_gyro;
  theta_alt_from_encoder = theta_from_encoder;

  //cout << gyro_fehler << " " << theta_from_gyro << endl;
}

bool odometry()
{
  // speed from odometry
  double v_encoder = 0.0, v_encoder_left = 0.0, v_encoder_right = 0.0;
  // "the" pose of the robot
  double dx = 0.0, dz = 0.0, dtheta_y = 0.0;
  // covered distance of wheels (left/right) -- resetted with each encoder update
  double wheel_L = 0.0, wheel_R = 0.0;

  if (!local_odometry(v_encoder, v_encoder_left, v_encoder_right, dx, dz, dtheta_y, wheel_L, wheel_R)) {
    return false;
  }

  theta_from_encoder += dtheta_y;
  if (theta_from_encoder > M_PI )
    theta_from_encoder -= 2.0*M_PI ;
  if (theta_from_encoder < -M_PI )
    theta_from_encoder += 2.0*M_PI;

  x_from_encoder += dx;
  z_from_encoder += dz;

  v_l_ist = v_encoder_left;
  v_r_ist = v_encoder_right;

  // x, z from gyro setzen
  x_from_gyro += (local_dx * cos(theta_from_gyro)) +
    (local_dz * sin(theta_from_gyro));
  z_from_gyro += (-local_dx * sin(theta_from_gyro)) +
    (local_dz * cos(theta_from_gyro));

  return true;
}

bool local_odometry(double &v_encoder, double &v_encoder_left, double &v_encoder_right, double &dx, double &dz, double &dtheta_y, double &wheel_L, double &wheel_R)
{
  long wheel_a = 0, wheel_b = 0;
  double angular_velocity = 0.0;
  double distance = 0.0;
  double hypothenuse= 0.0;

  // read wheel encoder here
  if (!k_read_wheel_encoder(&wheel_a, &wheel_b)) {
    return false;
  }

  Time_Diff *= 0.001;

  // Now we cal distance speed and position
  wheel_L = wheel_perimeter * (wheel_a)
    / ticks_per_turn_of_wheel;
  wheel_R = wheel_perimeter * (wheel_b)
    / ticks_per_turn_of_wheel;

  // distance is in cm negativ or positiv
  distance = (wheel_L + wheel_R) * 0.5;

  // calc different speeds in meter pro sec
  // distance is in cm so * 0.01
  v_encoder_left = wheel_L / Time_Diff * 0.01;
  v_encoder_right = wheel_R / Time_Diff * 0.01;
  v_encoder = (v_encoder_right + v_encoder_left) * 0.5;

  // mehr Bahn als Winkelgeschwindigkeit
  angular_velocity = (v_encoder_right - v_encoder_left) * 0.5;

  // resets the timer
  Time_Diff = 0.0;

  // calc position deltas
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
    hypothenuse = 0.5 * (wheel_L + wheel_R);

    dtheta_y = (wheel_L - wheel_R) / axis_length * turning_adaptation;

    local_dx = hypothenuse * sin(dtheta_y);
    local_dz = hypothenuse * cos(dtheta_y);
  }

  // Odometrie : Koordinatentransformation in Weltkoordinaten
  dx = (local_dx * cos(theta_from_encoder))
    + (local_dz * sin(theta_from_encoder));
  dz = (-(local_dx) * sin(theta_from_encoder))
    + (local_dz * cos(theta_from_encoder));

  return true;
}

/*****************************************************************************
 *          Auslesen der Zaehlerstaende auf der Encoder-Karte                *
 *****************************************************************************/
// This function has to be called 100 times per second
bool k_read_wheel_encoder (long *channel_1, long *channel_2)
{
  char *retext; // return text of CAN interface functions
  int nr_msg = 0; // msg count since the last readout of encoder ticks
  if ((retext = can_encoder(channel_1, channel_2, &nr_msg))) {
    ROS_WARN("error Can read encodervals (%s)", retext);
  }
  //TODO change to timing of CAN (or ROS?)
  // we hope kurt is precise ?? !! and sends every 10 ms
  Time_Diff = (double)nr_msg * 10.0;

  return nr_msg > 0;
}

void transmit_speed(void)
{
  char *retext; // return text of CAN interface functions
  /*
     printf("### trans: %d %d %d %d %d %d\n",kurt2_ctrl.pwm_left,kurt2_ctrl.pwm_right,
     kurt2_ctrl.brake_left, kurt2_ctrl.brake_right,
     kurt2_ctrl.dir_left, kurt2_ctrl.dir_right);
     fflush(stdout);
     */
  if ((retext = can_motor(kurt2_ctrl.pwm_left, kurt2_ctrl.dir_left,
          kurt2_ctrl.brake_left, kurt2_ctrl.pwm_right,
          kurt2_ctrl.dir_right, kurt2_ctrl.brake_right))) {
    ROS_WARN("transmit_speed %s", retext);
  }
}

// stoppen sofort
void k_hard_stop(void)
{
  kurt2_ctrl.control_mode = 0;
  kurt2_ctrl.pwm_left = 1023;
  kurt2_ctrl.dir_left = 0;
  kurt2_ctrl.brake_left = 1;
  kurt2_ctrl.pwm_right = 1023;
  kurt2_ctrl.dir_right = 0;
  kurt2_ctrl.brake_right = 1;
  transmit_speed();
}

int k_can_init(void)
{
  int can_version;
  char *retext; // return text of CAN interface functions

  if ((retext = can_init(&can_version))) {
    ROS_INFO("k_can_init %s", retext);
    return 1;
  }
  ROS_INFO("CAN interface version %3d.%02d",
      can_version / 100, can_version % 100);
  return 0;
}

void k_can_close(void)
{
  char *retext; // return text of CAN interface functions
  // be sure the robot ist stopped before closing the connection
  do {
    kurt2_ctrl.control_mode = 0;
    kurt2_ctrl.pwm_left = 1023;
    kurt2_ctrl.dir_left = 0;
    kurt2_ctrl.brake_left = 1;
    kurt2_ctrl.pwm_right = 1023;
    kurt2_ctrl.dir_right = 0;
    kurt2_ctrl.brake_right = 1;
  } while ((retext = can_motor(kurt2_ctrl.pwm_left, kurt2_ctrl.dir_left,
          kurt2_ctrl.brake_left, kurt2_ctrl.pwm_right,
          kurt2_ctrl.dir_right, kurt2_ctrl.brake_right)));

  if ((retext = can_close())) {
    ROS_WARN("k_can_close %s", retext);
  }
}

// reads init data from pmw to speed experiment
bool read_speed_to_pwm_leerlauf_tabelle(string &filename, int *nr, double **v_pwm_l, double **v_pwm_r)
{
  int i;
  FILE *fpr_fftable = NULL;
  double t, v, vl, vr;
  int pwm;

  ROS_INFO("Open FeedForwardTabelle: %s", filename.c_str());
  fpr_fftable = fopen(filename.c_str(),"r");

  if (fpr_fftable == NULL) {
    ROS_ERROR("ERROR opening speedtable.");
    return false;
  }

  *nr = 1025; //TODO use global nr_v
  *v_pwm_l = (double *)calloc(*nr, sizeof(double));
  *v_pwm_r = (double *)calloc(*nr, sizeof(double));
  for (i = 0; i < *nr; i++) {
    //TODO ignoring return value
    fscanf(fpr_fftable, "%lf %lf %lf %lf %d",&t, &v, &vl, &vr, &pwm);
    (*v_pwm_l)[pwm] = vl;
    (*v_pwm_r)[pwm] = vr;
  }
  fclose(fpr_fftable);
  return true;
}

// generate reverse speedtable
void make_pwm_v_tab(int nr, double *v_pwm_l, double *v_pwm_r, int nr_v, int **pwm_v_l, int **pwm_v_r, double *v_max)
{
  int i, index_l, index_r;
  int last_l, last_r;
  double v_max_l = 0.0, v_max_r = 0.0;
  *v_max = 0.0;

  *pwm_v_l = (int *)calloc(nr_v+1, sizeof(int));
  *pwm_v_r = (int *)calloc(nr_v+1, sizeof(int));

  // check max wheel speed left right wheel
  for (i = 0; i < nr; i++) {
    if (v_pwm_l[i] > v_max_l) v_max_l = v_pwm_l[i];
    if (v_pwm_r[i] > v_max_r) v_max_r = v_pwm_r[i];
    *v_max = min(v_max_l,v_max_r);
  }

  // diskretisieren
  for (i = 0; i < nr; i++)
  {
    index_l = min(nr_v,max(0,(int)((v_pwm_l[i]) * (nr_v) / *v_max)));
    (*pwm_v_l)[index_l] = i;
    index_r = min(nr_v,max(0,(int)((v_pwm_r[i]) * (nr_v) / *v_max)));
    (*pwm_v_r)[index_r] = i;
  }
  (*pwm_v_l)[0] = 0; // for a stop
  (*pwm_v_r)[0] = 0;

  // interpolate blank entries
  last_l = (*pwm_v_l)[0];
  last_r = (*pwm_v_r)[0];
  for (i = 1; i < nr_v; i++) {
    // interpolation simple methode, pwm must be increasing
    if ((*pwm_v_l)[i] < (*pwm_v_l)[i-1]) (*pwm_v_l)[i] = (*pwm_v_l)[i-1];
    if ((*pwm_v_r)[i] < (*pwm_v_r)[i-1]) (*pwm_v_r)[i] = (*pwm_v_r)[i-1];
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

void print_v_pwm_tab(int nr, double *v_pwm_l, double *v_pwm_r)
{
  int i;

  printf("########## speed pwm tabelle #############\n");
  for (i = 0; i < nr; i++) {
    printf("%d %f %f\n",i,v_pwm_l[i],v_pwm_r[i]);
  }
  printf("##########################################\n");
}

void print_pwm_v_tab(int nr)
{
  int i;

  printf("######### pwm speed tabelle #############\n");
  for (i = 0; i < nr; i++) {
    printf("%d %f %d %d\n",i,vmax*i/nr,pwm_v_l[i],pwm_v_r[i]);
  }
  printf("vmax: %f ################################\n",vmax);
}

// PWM Lookup
// basis zuordnung
// darueber liegt ein PID geschwindigkeits regler
// Leerlauf adaption fuer den Teppich boden und geradeaus fahrt
void set_wheel_speed1(double v_l, double v_r, int integration_l, int integration_r)
{
  int index_l, index_r; // point in speed_pwm tabelle

  // calc pwm values form speed array
  index_l = (int)(fabs(v_l) / vmax * nr_v);
  index_r = (int)(fabs(v_r) / vmax * nr_v);

  index_l = min(nr_v-1,index_l);
  index_r = min(nr_v-1,index_r);

  // 1023 = zero, 0 = maxspeed
  if (fabs(v_l) > 0.01)
    kurt2_ctrl.pwm_left = max(0,min(1023,1024-pwm_v_l[index_l] - leerlauf_adapt - integration_l));
  else
    kurt2_ctrl.pwm_left = 1023;
  if (fabs(v_r) > 0.01)
    kurt2_ctrl.pwm_right = max(0,min(1023,1024-pwm_v_r[index_r] - leerlauf_adapt - integration_r));
  else
    kurt2_ctrl.pwm_right = 1023;

  //  printf(" pwm %d %d\n",kurt2_ctrl.pwm_left,kurt2_ctrl.pwm_right);

  if (v_l >= 0.0) kurt2_ctrl.dir_left = 0; // forward
  else kurt2_ctrl.dir_left = 1; // backward
  if (v_r >= 0.0) kurt2_ctrl.dir_right = 0; // s.o right wheel
  else kurt2_ctrl.dir_right = 1;

  kurt2_ctrl.control_mode = 0;
  // relase breaks
  kurt2_ctrl.brake_left = 0;
  kurt2_ctrl.brake_right = 0;

  transmit_speed();
}

void set_wheel_speed2_mc(double _v_l_soll, double _v_r_soll,
    double _v_l_ist, double _v_r_ist,
    double _omega, long double Time, double _AntiWindup)
{
  // solldaten an controller uebermitteln
  char *retext; // return text of CAN interface functions
  const long factor = 100; // faktor um nachkommastellen mitzu übertragen, gesendet werden
  // nämlich nur integer werte ([+-]32e3 !)
  if ((retext = can_speed_cm((int)(_v_l_soll * factor),
          (int)(_v_r_soll * factor),
          (int)(factor * _omega / M_PI),
          (int)_AntiWindup))) {
    ROS_INFO("set_wheel_speed2_mc %s", retext);
  }
}

// pid geschwindigkeits regler fuers linke und rechte rad
// omega wird benoetig um die integration fuer den darunterstehenden regler
// zu berechnen
void set_wheel_speed2(double _v_l_soll, double _v_r_soll,
    double _v_l_ist, double _v_r_ist,
    double _omega, long double Time, double _AntiWindup)
{
  int holdl = 0, holdr = 0; // delete errors

  // stellgroessen v=speed, l= links, r= rechts
  static double zl = 0.0, zr = 0.0;
  static double last_zl = 0.0, last_zr = 0.0;
  // regelabweichung e = soll - ist;
  static double el = 0.0, er = 0.0;
  static double last_el = 0.0, last_er= 0.0;
  // integral
  static double int_el = 0.0, int_er = 0.0;
  // differenzieren
  static double del = 0.0, der = 0.0;
  static double last_del = 0.0, last_der = 0.0;
  // zeitinterval
  static long double last_time = 0.0;
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

  if ((dt = (Time - last_time) < 10.0))
    dt = 0.01;
  else
    dt = (double)((int)((Time - last_time) * 0.1) * 0.01); // runden auf 10 20 ...

  int_el *= _AntiWindup; int_er *= _AntiWindup;

  //@@@ TODO already in the kurt software
  if ((dt > 5.0)) {
    dt = 0.0;
  }
  last_v_l_ist *= _AntiWindup;
  last_v_r_ist *= _AntiWindup;

  double turn_feedforward_l = -_omega / M_PI * feedforward_turn;
  double turn_feedforward_r = _omega / M_PI * feedforward_turn;

  // filtern: grosser aenderungen deuten auf fehlerhafte messungen hin
  if (fabs(_v_l_ist - last_v_l_ist) < 0.19) {
    // filter glaettung werte speichern
    v_l_list[vl_index] = _v_l_ist;
    f_v_l_ist = (v_l_list[vl_index]+v_l_list[vl_index-1]+v_l_list[vl_index-2]+v_l_list[vl_index-3]) / 4.0;
    vl_index++; // achtung auf ueberlauf
    if (vl_index >= MAX_V_LIST) {
      vl_index = 3; // zum schutz vor ueberlauf kopieren bei hold einen mehr kopieren
      for (i = 0; i < 3; i++)
        v_l_list[2-i] = v_l_list[MAX_V_LIST-i-1];
    }

    el = _v_l_soll - f_v_l_ist;
    del = (el - last_el) / dt;
    int_el += el * dt;

    zl = kp_l * el + kd_l * del + ki_l * int_el + _v_l_soll + turn_feedforward_l;

    last_el = el; // last e
    last_del = del; // last de
    holdl = 0;
  }
  else {
    holdl = 1;
  }

  // filtern: grosser aenderungen deuten auf fehlerhafte messungen hin
  if (fabs(_v_r_ist - last_v_r_ist) < 0.19) {
    // filter glaettung werte speichern
    v_r_list[vr_index] = _v_r_ist;
    f_v_r_ist = (v_r_list[vr_index]+v_r_list[vr_index-1]+v_r_list[vr_index-2]) / 3.0;
    vr_index++; // achtung auf ueberlauf
    if (vr_index >= MAX_V_LIST) {
      vr_index = 3; // zum schutz vor ueberlauf kopieren bei hold einen mehr kopieren
      for (i = 0; i < 3; i++) {
        v_r_list[2-i] = v_r_list[MAX_V_LIST-i-1];;
      }
    }

    er = _v_r_soll - f_v_r_ist;
    der = (er - last_er) / dt;
    int_er += er * dt;

    zr = kp_r * er + kd_r * der + ki_r * int_er + _v_r_soll + turn_feedforward_r;

    last_er = er; // last e
    last_der = der; // last de
    holdr = 0;
  } else {
    holdr = 1;
  }

  last_time = Time;

  // range check und antiwindup stellgroessenbeschraenkung
  // verhindern das der integrier weiter hochlaeuft
  // deshalb die vorher addierten werte wieder abziehen
  if (zl > vmax) { zl = vmax; int_el -= el * dt; }
  if (zr > vmax) { zr = vmax; int_er -= er * dt; }
  if (zl < -vmax) { zl = -vmax; int_el -= el * dt; }
  if (zr < -vmax) { zr = -vmax; int_er -= er * dt; }

  // reduzieren

  /* kraft begrenzung damit die Kette nicht springt bzw
     der Motor ein wenig entlastet wird. bei vorgabe von max
     geschwindigkeit braucht es so 5 * 10 ms bevor die Maximale
     Kraft anliegt */
  if ((zl - last_zl) > step_max) {
    zl = last_zl + step_max;
  }
  if ((zl - last_zl) < -step_max) {
    zl = last_zl - step_max;
  }
  if ((zr - last_zr) > step_max) {
    zr = last_zr + step_max;
  }
  if ((zr - last_zr) < -step_max) {
    zr = last_zr - step_max;
  }

  // store old val for deviation plotting
  last_v_l_ist = _v_l_ist;
  last_v_r_ist = _v_r_ist;
  last_zl = zl;
  last_zr = zr;

  set_wheel_speed1(zl, zr, 0, 0);
}

// IR related stuff
// ----------------

// gemessen 3. Mai 2005
#define IR_MIN 440
#define IR_MAX 70
#define SONAR_MIN 0
#define SONAR_MAX 800

void get_ir(int *ir1, int *ir2, int *ir3, int *ir4, int *ir5, int *ir6, int *ir7)
{
  int dummy;
  unsigned long time;
  can_sonar0_3(ir1, ir2, ir3, &dummy, &time);
  can_sonar4_7(ir4, &dummy, ir5, ir6, &time);
  can_sonar8_9(&dummy, ir7, &time);

  return;
}

void get_sonar(int *s)
{
  int dummy;
  unsigned long time;
  can_sonar4_7(&dummy, s, &dummy, &dummy, &time);
  return;
}

void normalize_ir(int *ir)
{
  if (*ir > IR_MIN || *ir < IR_MAX) {
    *ir = -1;
    return;
  }
  //cout << (pow(30000.0 / ((double)*ir - 10.0), 1.0/1.4)- 10.0) << " ";
  *ir = (int)(pow(30000.0 / ((double)*ir - 10.0), 1.0/1.4) - 10.0);
  //cout << *ir << endl;
}

void normalize_sonar(int *s)
{
  if (*s < SONAR_MIN || *s > SONAR_MAX) {
    *s = -1;
    return;
  }
  *s = (int)((double)*s * 0.110652 + 11.9231);
}

#define EDWZ 0.7071067811865475
bool infrared_sonar()
{
  int ir[7]; // infrared sensors
  int s; // sonar sensor
  IRnr = 0;
  double IRx[10], IRz[10];

  get_ir(&ir[0], &ir[1], &ir[2], &ir[3], &ir[4], &ir[5], &ir[6]);
  get_sonar(&s);

  for (int i = 0; i < 7; i++) {
    normalize_ir(&ir[i]);
  }
  normalize_sonar(&s);

  // back
  if (ir[0] > 0) {
    IRdist[IRnr] = ir[0]; IRx[IRnr] = 0.0; IRz[IRnr] = -(double)ir[0] - 30.0;
  } else {
    IRdist[IRnr] = 0; IRx[IRnr] = 0.0; IRz[IRnr] = 0.0;
  }
  // cout << "back " << IRdist[IRnr] << " " << IRx[IRnr] << " " << IRz[IRnr] << endl;
  IRnr += 1;

  // right-back
  if (ir[1] > 0) {
    IRdist[IRnr] = ir[1]; IRx[IRnr] = EDWZ * (double)ir[1] + 15.0; IRz[IRnr] = -EDWZ * (double)ir[1] - 30.0;
  } else {
    IRdist[IRnr] = 0; IRx[IRnr] = 0.0; IRz[IRnr] = 0.0;
  }
  //cout << "right-back " << IRdist[IRnr] << " " << IRx[IRnr] << " " << IRz[IRnr] << endl;
  IRnr += 1;

  // right
  if (ir[2] > 0) {
    IRdist[IRnr] = ir[2]; IRx[IRnr] = (double)ir[2] + 15.0; IRz[IRnr] = -10.0;
  } else {
    IRdist[IRnr] = 0; IRx[IRnr] = 0.0; IRz[IRnr] = 0.0;
  }
  //cout << "right " << IRdist[IRnr] << " " << IRx[IRnr] << " " << IRz[IRnr] << endl;
  IRnr += 1;

  // right-front
  if (ir[3] > 0) {
    IRdist[IRnr] = ir[3]; IRx[IRnr] = EDWZ * (double)ir[3] + 15.0; IRz[IRnr] = EDWZ * (double)ir[3] + 5.0;
  } else {
    IRdist[IRnr] = 0; IRx[IRnr] = 0.0; IRz[IRnr] = 0.0;
  }
  IRnr += 1;
  //cout << "right-front " << IRdist[IRnr] << " " << IRx[IRnr] << " " << IRz[IRnr] << endl;

  // front
  if (s > 0) {
    IRdist[IRnr] = s; IRx[IRnr] = 0.0; IRz[IRnr] = s + 5.0;
  } else {
    IRdist[IRnr] = 0; IRx[IRnr] = 0.0; IRz[IRnr] = 0.0;
  }
  // cout << "front " << IRdist[IRnr] << " " << IRx[IRnr] << " " << IRz[IRnr] << endl;
  IRnr += 1;

  // left-front
  if (ir[4] > 0) {
    IRdist[IRnr] = ir[4]; IRx[IRnr] = -EDWZ * (double)ir[4] - 15.0; IRz[IRnr] = EDWZ * (double)ir[4] + 5.0;
  } else {
    IRdist[IRnr] = 0; IRx[IRnr] = 0.0; IRz[IRnr] = 0.0;
  }
  // cout << "left-front " << IRdist[IRnr] << " " << IRx[IRnr] << " " << IRz[IRnr] << endl;
  IRnr += 1;

  // left
  if (ir[5] > 0) {
    IRdist[IRnr] = ir[5]; IRx[IRnr] = -(double)ir[5] - 15.0; IRz[IRnr] = -10.0;
  } else {
    IRdist[IRnr] = 0; IRx[IRnr] = 0.0; IRz[IRnr] = 0.0;
  }
  // cout << "left " << IRdist[IRnr] << " " << IRx[IRnr] << " " << IRz[IRnr] << endl;
  IRnr += 1;

  // left-back
  if (ir[6] > 0) {
    IRdist[IRnr] = ir[6]; IRx[IRnr] = -EDWZ * (double)ir[6] - 15.0; IRz[IRnr] = -EDWZ * (double)ir[6] - 30.0;
  }  else {
    IRdist[IRnr] = 0; IRx[IRnr] = 0.0; IRz[IRnr] = 0.0;
  }
  //cout << "left-back " << IRdist[IRnr] << " " << IRx[IRnr] << " " << IRz[IRnr] << endl;
  IRnr += 1;

  if (IRnr > 0)
    return 1;
  else
    return 0;
}
