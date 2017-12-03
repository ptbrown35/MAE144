/*******************************************************************************
* File: hw2.c
* Author: Parker Brown
* Date: 12/3/2017
* Course: MAE 144, Fall 2017
* Description:
*******************************************************************************/

// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
// Nice to have for TWO_PI
#include <rc_usefulincludes.h>
// main roboticscape API header
#include <roboticscape.h>
#include "hw2_config.h"

// Struct for angle stuff
typedef struct angles_t{
	float theta_a_raw;
	float theta_g_raw;
	float last_theta_g_raw;

	float theta_a;
	float theta_g;
	float theta_f;
}angles_t;

typedef struct filter_t{
	float lp_coeff[2];
	float hp_coeff[3];
} filter_t;

// function declarations
void on_pause_pressed(); // do stuff when paused button is pressed
void on_pause_released(); // do stuff when paused button is released
void complimentary_filter();
void zero_filers(); // Zero out filters

// Global Variables
filter_t filter;
angles_t angles;

/*******************************************************************************
* int main()
*
* hw1 main function contains these critical components
* - call to rc_initialize() at the beginning
* - main while loop that checks for EXITING condition
*		- do stuff
* - rc_cleanup() at the end
*******************************************************************************/
int main(){

	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	// initialize stuff here
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);

	// Initialize variables used in the while loop
	int sleep_time = DT * 1e6; // Sleep time to set rough loop rate
	rc_imu_data_t data; // imu struct to hold new data
	rc_imu_config_t conf = rc_default_imu_config(); // Set imu config struct to defaults

	// Initialize anlges to zero
	zero_filers();

	// Initialize filter coefficients
	filter.lp_coeff[0] = -(WC*DT-1);
	filter.lp_coeff[1] =  WC*DT;
	filter.hp_coeff[0] = -(WC*DT-1);
	filter.hp_coeff[1] = 1;
	filter.hp_coeff[2] = -1;

	// Initialize imu
	if(rc_initialize_imu(&data, conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}

	// Print header for standard output
	printf("Data Output\n");
	printf("| theta_a_raw |");
	printf(" theta_g_raw |");
	printf(" theta_a |");
	printf(" theta_g |");
	printf(" theta_f |");
	printf(" \n");

  // Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// If RUNNING, run feedback loop
		if(rc_get_state()==RUNNING){
			rc_set_led(GREEN, ON); // GREEN when on
			rc_set_led(RED, OFF); // RED when paused

			// Get accel data and convert to angle
			if(rc_read_accel_data(&data)<0){
				printf("read accel data failed\n");
			}
			angles.theta_a_raw = -1 * atan2(data.accel[2], data.accel[1]);

			// Get gyro data and integrate
			if(rc_read_gyro_data(&data)<0){
				printf("read gyro data failed\n");
			}
			angles.theta_g_raw = angles.last_theta_g_raw  + (data.gyro[0] * DT * DEG_TO_RAD);

			// Complimentary Filter
			complimentary_filter();

			// Update values
			angles.last_theta_g_raw = angles.theta_g_raw;

			// Print raw angles
			printf("\r|"); // carriage return because it looks pretty
			printf(" %11.3f |", angles.theta_a_raw);
			printf(" %11.3f |", angles.theta_g_raw);
			printf(" %7.3f |", angles.theta_a);
			printf(" %7.3f |", angles.theta_g);
			printf(" %7.3f |", angles.theta_f);
			fflush(stdout);

		}
		else if(rc_get_state()==PAUSED){
			// Set everything to an off state when paused
			rc_set_led(GREEN, OFF); // GREEN when on
			rc_set_led(RED, ON); // RED when paused
			zero_filers(); // Reset filters when paused
		}

		rc_usleep(sleep_time); // Sleep for DT in microseconds
	}

	// Shutdown procedures
	rc_power_off_imu();

	// exit cleanly
	rc_cleanup();
	return 0;
}

/*******************************************************************************
* void zero_filers()
*
* Zero out filter inputs.
*******************************************************************************/
void zero_filers(){
	angles.last_theta_g_raw = 0;
	angles.theta_a = 0;
	angles.theta_g = 0;
	angles.theta_f = 0;
}

/*******************************************************************************
* void complimentary_filter()
*
* Complimentary filter built by summing high and low pass fitlers applied to
* raw theta values from accel and gyro data, respectively.
*******************************************************************************/
void complimentary_filter(){
	// First order Low Pass filter of theta from raw accel data
	angles.theta_a = (filter.lp_coeff[0] * angles.theta_a) \
								 + (filter.lp_coeff[1] * angles.theta_a_raw);
  // First order high pass filter of theta from raw gyro data
	angles.theta_g = (filter.hp_coeff[0] * angles.theta_g) \
	 							 + (filter.hp_coeff[1] * angles.theta_g_raw) \
								 + (filter.hp_coeff[2] * angles.last_theta_g_raw);
  // Sum of Low and High pass filters of theta
	angles.theta_f = angles.theta_a + angles.theta_g;
}

/*******************************************************************************
* void on_pause_released()
*
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void on_pause_released(){
	// toggle betewen paused and running modes
	if(rc_get_state()==RUNNING)		rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/*******************************************************************************
* void on_pause_pressed()
*
* If the user holds the pause button for 2 seconds, set state to exiting which
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
void on_pause_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}
