#include <cstdio>
#include <cstdlib>

#include "kurt.h"
#include "stdoutcomm.h"
#include "mytime.h"

void usage(char *pgrname)
{
  printf("%s: <pwml> <pwmr> <nr_turns> <nr_ticks> (radumdrehungen zaehlen)\n",pgrname);
  printf("e.g. %s 400 400 10 2050\n",pgrname);
}

int main(int argc, char **argv)
{
  FILE *fpr = NULL;
  long double Time_Diff1 = 101.0; // time
  long double Time_Diffa = 0.0; // time
  long double Time_Diffb = 0.0; // time
  int i,j, k;
  int startpwml = 400, startpwmr = 400, nr_turns = 10, nr_ticks = 2050;

  if (argc < 5) {
    usage(argv[0]);
    return 0;
  }
  else {
    startpwml = atoi(argv[1]);
    startpwmr = atoi(argv[2]);
    nr_turns = atoi(argv[3]);
    nr_ticks = atoi(argv[4]);
  }

  fpr = fopen("distance-test.dat","w");

  mydelay(200);

  //Odometry parameter (defaults for kurt2 indoor)
  double wheel_perimeter = 0.379;
  double axis_length = 0.28;

  double turning_adaptation = 0.69;
  int ticks_per_turn_of_wheel = 21950;

  STDoutComm stdoutcomm;
  Kurt kurt(stdoutcomm, wheel_perimeter, axis_length, turning_adaptation, ticks_per_turn_of_wheel);

  Get_mtime_diff(2);
  Get_mtime_diff(3);
  Get_mtime_diff(4);

  i = startpwml;
  j = startpwmr;

  int pwm_left, pwm_right;
  do {

    // speed auf die motoren
    pwm_left = 1024-i;
    pwm_right = 1024-j;
    Time_Diff1 += Get_mtime_diff(3);
    if (Time_Diff1 > 100.0) {
      kurt.can_motor(pwm_left, 0, 0, pwm_right, 0, 0);
      Time_Diff1 = 0.0;
    }
    //printf("out %d\n",i);
    // werte von den winkel encodern
    while (kurt.can_read_fifo() != CAN_ENCODER) ;

    fprintf(fpr,"%Lf %lf %lf %lf %lld %lld\n",
        Get_mtime_diff(9),
        stdoutcomm.v1(),stdoutcomm.v1_left(),stdoutcomm.v1_right(),
        stdoutcomm.K_get_sum_ticks_a(),
        stdoutcomm.K_get_sum_ticks_b());

    // xx umdrehungen pro rad zaehlen
    if (abs(stdoutcomm.K_get_sum_ticks_a()) > nr_ticks*nr_turns-200) {
      Time_Diffa += Get_mtime_diff(2);
      if ( i > 0) {
        i = 0;
        pwm_left = 1024-i;
        kurt.can_motor(pwm_left, 0, 0, pwm_right, 0, 0);
      }

    }
    else {
      Get_mtime_diff(2);
      Time_Diffa = 0.0;
    }
    // fuer kurt2 mit 90 watt motoren und 1:14 getriebe
    // encoder 500 oder 1000 / umdrehung
    // umsetzung kette ??
    if (abs(stdoutcomm.K_get_sum_ticks_b()) > nr_ticks*nr_turns-200) {
      Time_Diffb += Get_mtime_diff(4);
      if (j > 0) {
        j = 0;
        pwm_right = 1024-j;
        kurt.can_motor(pwm_left, 0, 0, pwm_right, 0, 0);
      }

    }
    else {
      Get_mtime_diff(4);
      Time_Diffb = 0.0;
    }

  } while ((Time_Diffa < 3000.0) || (Time_Diffb < 3000.0));
  fclose(fpr);

  // langsames stoppen ueber 1024 / 2 * 10 ms
  for (k = 0; k < i; k+=2 ) {
    pwm_left = 1024-i+k;
    pwm_right = 1024-j+k;
    kurt.can_motor(pwm_left, 0, 0, pwm_right, 0, 0);
    mydelay(10);
  }

  printf("Closing the can connection\n");
  printf("Bye Bye Spoki\n");
  return (0);
}
