#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "kurt.h"
#include "stdoutcomm.h"
#include "mytime.h"

void usage(char *pgrname)
{
  printf("%s: <configfile> <start-pwm-value>\n",pgrname);
}

int main(int argc, char **argv)
{
  FILE *fpr = NULL;
  long double Time_Diff1 = 1000.0; // time
  long double Time_Diff2 = 0.0; // time
  int i,j, k;
  double v1=0.0, prev_v1=0.0; // position update
  double v1_left = 0.0, v1_right=0.0;  // current speed
  double prev_v1_left=0.0, prev_v1_right=0.0;  // previous  speed
  int finish = 0;
  int startpwm = 1;

  if (argc == 3) {
    startpwm = atoi(argv[2]);
  }
  else {
    usage(argv[0]);
    return 0;
  }

  fpr = fopen(argv[1], "w");

  // fill with zeros until startpwm
  for (i = 1; i < startpwm;i++) {
    fprintf(fpr,"%Lf 0.0 0.0 0.0 %d\n",Get_mtime_diff(9), i);
  }

  //Odometry parameter (defaults for kurt2 indoor)
  double wheel_perimeter = 0.379;
  double axis_length = 0.28;

  double turning_adaptation = 0.69;
  int ticks_per_turn_of_wheel = 21950;

  STDoutComm stdoutcomm;
  Kurt kurt(stdoutcomm, wheel_perimeter, axis_length, turning_adaptation, ticks_per_turn_of_wheel);

  i = j = startpwm;
  Get_mtime_diff(3);
  Get_mtime_diff(2);
  int pwm_left, pwm_right;
  do {
    Time_Diff1 -= Get_mtime_diff(3);
    Time_Diff2 -= Get_mtime_diff(2);
    //printf("Hallo %Lf %Lf\n",Time_Diff1,Time_Diff2);
    if (Time_Diff2 <= 0.0) {
      Time_Diff2 = 100; // alle 100 ms senden sonst ausfall
      // speed auf die motoren
      pwm_left = 1024-i;
      pwm_right = 1024-j;
      kurt.can_motor(pwm_left, 0, 0, pwm_right, 0, 0);
      //	get_multimeter(0);
      if (i > 1024) {j  = i = 1024;finish = 1;}
    }
    // werte von den winkel encodern
    while (kurt.can_read_fifo() != CAN_ENCODER) ;
    v1 = stdoutcomm.v1();
    v1_left = stdoutcomm.v1_left();
    v1_right = stdoutcomm.v1_right();

    //  get_current_old(0);
    if (Time_Diff1 <= 0.0) { // jede sekunde neuer wert

      if ((v1_left < prev_v1_left)   || (prev_v1_left > 0.0  && 2.0 * prev_v1_left < v1_left)) {
        v1_left = prev_v1_left;
        if ((prev_v1_left > 0.0  && 2.0 * prev_v1_left < v1_left)) printf("ticks error left\n");
      }

      if ((v1_right < prev_v1_right) || (prev_v1_right > 0.0 && 2.0 * prev_v1_right < v1_right)) {
        v1_right = prev_v1_right;
        if ((prev_v1_right > 0.0 && 2.0 * prev_v1_right < v1_right)) printf("ticks error right \n");
      }

      if ((v1 < prev_v1) || (prev_v1 > 0.0 && 2.0 * prev_v1 < v1))
        v1 = prev_v1;

      //  get_multimeter(1);
      //  get_current_old(1);
      printf("t1: %Lf, %Lf, %Lf, v: %lf, vl: %lf, vr: %lf, pwm: %d\n",
          Get_mtime_diff(9), Time_Diff1, Time_Diff2,
          v1,
          v1_left,
          v1_right,
          i);
      fprintf(fpr,"%Lf %lf %lf %lf %d\n",
          Get_mtime_diff(9),
          v1,
          v1_left,
          v1_right,
          i);
      Time_Diff1 += 1000.0;
      Time_Diff2 = 0.0;
      i++; j++;
      prev_v1_left = v1_left;
      prev_v1_right = v1_right;
      prev_v1 = v1;
    }
  } while (finish == 0);

  i--;
  j--;

  // langsames stoppen ueber 1024 * 20 ms
  for (k = 0; k < i; k+=1 ) {
    pwm_left = 1024-i+k;
    pwm_right = 1024-j+k;
    kurt.can_motor(pwm_left, 0, 0, pwm_right, 0, 0);

    /*
    // werte von den winkel encodern
    if ((K_update_pose_from_encoder (0, &dx, &dy, &dtheta,
    &v1, &v1_left, &v1_right, &wL, &wR,
    Get_mtime_diff(9))) > 0) {
    fprintf(fpr,"%Lf %Lf %Lf %Lf %d\n",
    Get_mtime_diff(9),
    v1,
    v1_left,
    v1_right,
    i
    );
    }
    */
    mydelay(20);
  }
  fclose(fpr);
  printf("Closing the can connection\n");
  printf("Bye Bye Spoki\n");
  return (0);
}
