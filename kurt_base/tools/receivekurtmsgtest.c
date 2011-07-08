// ./cansend can0 001#0000640064

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define CANINTERFACE "can0"

int main (void) {
  int left_pwm = 100;
  char left_dir = 0;
  char left_brake = 0;
  int right_pwm = 100;
  char right_dir = 0;
  char right_brake = 0;

  char left_dir_brake =  (left_dir << 1) + left_brake;
  char right_dir_brake = (right_dir << 1) + right_brake;

  int i;


  int s; /* can raw socket */ 
  int nbytes;
  struct sockaddr_can addr;
  struct can_frame frame;
  struct ifreq ifr;

  frame.can_id = 1;
  frame.can_dlc = 8;

  frame.data[0] = 0 >> 8;
  frame.data[1] = 0;
  frame.data[2] = (left_dir_brake);
  frame.data[3] = (left_pwm >> 8);
  frame.data[4] = (left_pwm);
  frame.data[5] = (right_dir_brake);
  frame.data[6] = (right_pwm >> 8);
  frame.data[7] = (right_pwm);

  printf("%s ", CANINTERFACE);
  printf("%X#", frame.can_id);

  for (i = 0; i < 8; i++) {
    printf("%X", frame.data[i]);
  }

  printf("\n");


  /* open socket */
  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("socket");
    return 1;
  }

  addr.can_family = AF_CAN;

  strcpy(ifr.ifr_name, CANINTERFACE);
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
    perror("SIOCGIFINDEX");
    return 1;
  }
  addr.can_ifindex = ifr.ifr_ifindex;

  /* disable default receive filter on this RAW socket */
  /* This is obsolete as we do not read from the socket at all, but for */
  /* this reason we can remove the receive list in the Kernel to save a */
  /* little (really a very little!) CPU usage.                          */
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("bind");
    return 1;
  }

  /* send frame */
  if ((nbytes = write(s, &frame, sizeof(frame))) != sizeof(frame)) {
    perror("write");
    return 1;
  }

  //fprint_long_canframe(stdout, &frame, "\n", 0);
  close(s);
  return 0;
}
