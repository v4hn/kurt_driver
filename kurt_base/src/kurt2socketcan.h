#ifndef __KURT2CAN_H__
#define __KURT2CAN_H__

/*
 * The contents of this file are subject to the Mozilla Public
 * License Version 1.1 (the "License"); you may not use this file
 * except in compliance with the License. You may obtain a copy of
 * the License at http://www.mozilla.org/MPL/
 *
 * Software distributed under the License is distributed on an "AS
 * IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
 * implied. See the License for the specific language governing
 * rights and limitations under the License.
 *
 * The Original Code is KURT2 Open Source Firmware and Win32 Software,
 * released March 15, 2001.
 *
 * The Initial Developer of the Original Code is GMD National Research
 * Center for Information Technology.  Portions created by GMD are
 * Copyright (C) 2000 - 2001 GMD National Research Center for
 * Information Technology.  All Rights Reserved.
 *
 * As of July 11, 2001, GMD has been integrated into the Fraunhofer-
 * Gesellschaft. As the new legal entity, Fraunhofer-Gesellschaft has thus
 * taken over all legal relationships involving GMD. Portions created by 
 * Fraunhofer-Gesellschaft are Copyright (C) 2002 Fraunhofer-Gesellschaft.
 * All Rights Reserved.
 *
 * Contributor(s):
 */
/******************************************************************************/
/* KURT2CAN.H: KURT2's CAN interface                                          */
/* $Id                                                                        */
/******************************************************************************/

typedef struct {
  int sonar0, sonar1, sonar2, sonar3, sonar4, sonar5, sonar6, sonar7, sonar8, 
	  sonar9, tilt1, tilt2, cur_left, cur_right, compass1, compass2, 
	  enc_left, enc_right; 
  char bumper, rc;
  int bdc[16];
  unsigned long time_adc00_03, time_adc04_07, time_adc08_11, time_adc12_15,
	time_encoder, time_bumper, 
	time_bdc00_03, time_bdc04_07, time_bdc08_11, time_bdc12_15;
} SENSOR_STATE;

typedef struct {
  short control_mode;
  unsigned short pwm_left, pwm_right; 
  short speed_left, speed_right;
  unsigned char dir_left, dir_right, brake_left, brake_right;
} MOTOR_CTRL;

typedef struct {
  unsigned char osbyte[8];
} OS_DATA;

char *can_init(int *version);
char *can_sonar0_3(int *sonar0, int *sonar1, int *sonar2, int *sonar3,
  unsigned long *time);
char *can_sonar4_7(int *sonar4, int *sonar5, int *sonar6, int *sonar7,
  unsigned long *time);
char *can_sonar8_9(int *sonar8, int *sonar9, unsigned long *time);
char *can_tilt(int *tilt0, int *tilt1, unsigned long *time);
char *can_current(int *left, int *right, unsigned long *time);
char *can_tilt_comp(double *tilt0, double *tilt1, int *comp1, int *comp2, unsigned long *time);
char *can_gyro_mc1(double *gyro_mc1_angle, double *gyro_mc1_sigma, unsigned long *time);
char *can_gyro_mc2(double *gyro_mc2_angle, double *gyro_mc2_sigma, unsigned long *time);
char *can_encoder(long *left, long *right, int *nr_msg);
char *can_rc(char *value, unsigned long *time);
char *can_bumper(char *value, unsigned long *time);
char *can_adc2nd0_3(int *adc2nd0, int *adc2nd1, int *adc2nd2, 
  int *adc2nd3, unsigned long *time);
char *can_adc2nd4_7(int *adc2nd4, int *adc2nd5, int *adc2nd6, 
  int *adc2nd7, unsigned long *time);
char *can_adc2nd8_11(int *adc2nd8, int *adc2nd9, int *adc2nd10, 
  int *adc2nd11, unsigned long *time);
char *can_adc2nd12_15(int *adc2nd12, int *adc2nd13, int *adc2nd14, 
  int *adc2nd15, unsigned long *time);
char *can_sensor_state1(SENSOR_STATE *sensor_state1);
char *can_sensor_state2(SENSOR_STATE *sensor_state2);
char *can_motor(int left_pwm,  char left_dir,  char left_brake,
			 int right_pwm, char right_dir, char right_brake);
char *can_speed(int left_speed, int right_speed);
char *can_speed_cm(int left_speed, int right_speed, int omega, int AntiWindup);
char *can_float(int mode, float f1, float f2);
char *can_time(unsigned long *time);
char *can_close(void);

char *can_sonar0_3(int *sonar0, int *sonar1, int *sonar2, int *sonar3, 
			    unsigned long *time);
char *can_sonar4_7(int *sonar4, int *sonar5, int *sonar6, int *sonar7, 
			    unsigned long *time);
char *can_sonar8_9(int *sonar8, int *sonar9, unsigned long *time);
char *can_gyro_reset();
char *can_gyro_calibrate();
char *can_mc_reset();
char *can_ssc_reset();
char *can_rotunit_send(int speed);
char *can_getrotunit(int *rot);

char *can_read_fifo(void);

#endif


