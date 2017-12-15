/*******************************************************************************
* File: balance.c
* Author: Parker Brown
* Date: 12/15/2017
* Course: MAE 144, Fall 2017
* Description: Balance program.
*******************************************************************************/

// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
// Nice to have for TWO_PI
#include <rc_usefulincludes.h>
// main roboticscape API header
#include <roboticscape.h>
#include "balance_config.h"

/*******************************************************************************
* arm_state_t
*
* ARMED or DISARMED to indicate if the controller is running
*******************************************************************************/
typedef enum arm_state_t{
	ARMED,
	DISARMED
}arm_state_t;

// Struct for angles
typedef struct angles_t{
	float theta_a_raw;
	float theta_g_raw;
	float last_theta_g_raw;
	float theta_a;
	float theta_g;
	float theta;
	float last_theta;
	float last_last_theta;

	float phi;
	float last_phi;
	float last_last_phi;

}angles_t;

// Struct for filters
typedef struct filter_t{
	float lp_coeff[2];
	float hp_coeff[3];
} filter_t;

// function declarations
void on_pause_pressed(); // do stuff when paused button is pressed
void on_pause_released(); // do stuff when paused button is released
void complimentary_filter(); // Complimentary filter
void zero_controllers(); // Zero out controllers and filters
void inner_loop(); // IMU interrupt routine
void* printf_data(void* ptr); // printf_thread function ot print data

// Global Variables
filter_t filter; // filter struct to hold filter coefficients
angles_t angles; // angles struct to hold theta angles
rc_imu_data_t data; // imu struct to hold new data

/*******************************************************************************
* int main()
*
* hw1 main function contains these critical components
* - call to rc_initialize() at the beginning
* - Initialize IMU
* - Initialize DMP for interrupt
* - Start and schedule printf_thread
* - Initialize filters
*	- Set RUNNING and start IMU isr
* - main while loop that checks for EXITING condition
*		- do nothing, just sleep
* - shutdown procedures
* - rc_cleanup() at the end
*******************************************************************************/
int main(){

	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	// Initialize imu
	rc_imu_config_t conf = rc_default_imu_config(); // imu config to defaults
	if(rc_initialize_imu(&data, conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}

	// Initialize imu for dmp interrupt operation
	if(rc_initialize_imu_dmp(&data, conf)){
		printf("rc_initialize_imu_failed\n");
		return -1;
	}

	// initialize pause functions
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);

	// Check min/max sched_priority
	printf("Valid priority range for SCHED_FIFO: %d - %d\n",
		sched_get_priority_min(SCHED_FIFO),
		sched_get_priority_max(SCHED_FIFO));

	// Start printf_thread
	pthread_t printf_thread;
	struct sched_param params;
	params.sched_priority = 10; // Reasonably low priority
	pthread_create(&printf_thread, NULL, printf_data, (void*) NULL);
	pthread_setschedparam(printf_thread, SCHED_FIFO, &params);

	// Initialize filter variables
	zero_controllers(); // Initialize controllers to zero
	filter.lp_coeff[0] = -(WC*DT_INNER-1);
	filter.lp_coeff[1] =  WC*DT_INNER;
	filter.hp_coeff[0] = -(WC*DT_INNER-1);
	filter.hp_coeff[1] = 1;
	filter.hp_coeff[2] = -1;

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);
	rc_set_led(GREEN, ON);  // GREEN when running
	rc_set_led(RED, OFF);  // RED when paused

	rc_set_imu_interrupt_func(&inner_loop); // IMU isr to get data

  // Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		rc_usleep(100000); // Sleep ocassionally
	}

	// Shutdown procedures
	pthread_join(printf_thread, NULL);
	rc_power_off_imu();

	// exit cleanly
	rc_cleanup();
	return 0;
}

/*******************************************************************************
* void zero_controllers()
*
* Zero out filter inputs nad integration values.
*******************************************************************************/
void zero_controllers(){
	angles.last_theta_g_raw = 0; // Zero out for gyro integration
	angles.theta_a = 0;
	angles.theta_g = 0;
	angles.theta = 0;
}

/*******************************************************************************
* void complimentary_filter()
*
* Complimentary filter built by summing high and low pass fitlers applied to
* raw theta values from accel and gyro data, respectively.
*******************************************************************************/
void complimentary_filter(){
	angles.theta_a_raw = -1.0 * atan2(data.accel[2], data.accel[1]); // theta [rad]
	angles.theta_g_raw = angles.last_theta_g_raw \
										+ (data.gyro[0] * DT_INNER * DEG_TO_RAD); // theta [rad]
	// First order Low Pass filter of theta from raw accel data
	angles.theta_a = (filter.lp_coeff[0] * angles.theta_a) \
								 + (filter.lp_coeff[1] * angles.theta_a_raw);
  // First order high pass filter of theta from raw gyro data
	angles.theta_g = (filter.hp_coeff[0] * angles.theta_g) \
	 							 + (filter.hp_coeff[1] * angles.theta_g_raw) \
								 + (filter.hp_coeff[2] * angles.last_theta_g_raw);
  // Sum of Low and High pass filters of theta
	angles.theta = angles.theta_a + angles.theta_g;
	// Correct for BBBlue mount angle
	angles.theta += CAPE_MOUNT_ANGLE;
	// Update integration value
	angles.last_theta_g_raw = angles.theta_g_raw;
}

/*******************************************************************************
* float tf(const float[3] a, const float[3] b, const float[3] u, const float[3] y)
*
* Difference equation for second order transfer function.
* Input: coefficients a[3] and b[3], inputs u[3], outputs y[3]
*******************************************************************************/

float tf(const float[3] a, const float[3] b, const float[3] u, const float[3] y){
	// Assume a nd b are normalized by a[0]
	// Compute y[0] for b
  for(int j = 0; j < 3; j++){
    y[0] += b[j] * u[j];
  }
  // Compute y[0] for a
  for(int k = 1; k < 3; k++){
    y[0] -= a[k] * y[k];
  }
  // return newest output
  return y[0];
}

/*******************************************************************************
* void d1_controller()
*
* D1 controller.
*******************************************************************************/
void d1_controller(){
	d1_u[0] = tf(D1_DEN, D1_NUM, theta_error, d1_u);
	// update values
	d1_u[2] = d1_u[1];
	d1_u[1] = d1_u[0];
}

/*******************************************************************************
* void inner_loop()
*
* Gets imu data using rc_set_imu_interrupt_func(&inner_loop)
*******************************************************************************/
void inner_loop(){
	// If RUNNING, run Complimentary Filter
	if(rc_get_state()==RUNNING){
		complimentary_filter(); // Complimentary Filter
		d1_controller();

	}
	else if(rc_get_state()==PAUSED){
		zero_controllers(); // Reset filters when paused
	}
}

/*******************************************************************************
* void* printf_data(void* ptr)
*
* printf_thread function prints data.
*******************************************************************************/
void* printf_data(void* ptr){
	rc_state_t last_rc_state, new_rc_state; // keep track of last state
	last_rc_state = rc_get_state();
	while(rc_get_state()!=EXITING){
		new_rc_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_rc_state==RUNNING && last_rc_state!=RUNNING){
			printf("\nRUNNING: Complimentary Filter, theta in [rad].\n");
			// Print header for standard output
			printf("| theta_a_raw |");
			printf(" theta_g_raw |");
			printf(" theta_a |");
			printf(" theta_g |");
			printf(" theta |");
			printf(" \n");
		}
		else if(new_rc_state==PAUSED && last_rc_state!=PAUSED){
			// first time sonce being paused
			printf("\nPAUSED: Press pause again to start.\n");
		}
		last_rc_state = new_rc_state; // update last_rc_state

		if(new_rc_state == RUNNING){
			// Print raw angles
			printf("\r|"); // carriage return because it looks pretty
			printf(" %11.3f |", angles.theta_a_raw);
			printf(" %11.3f |", angles.theta_g_raw);
			printf(" %7.3f |", angles.theta_a);
			printf(" %7.3f |", angles.theta_g);
			printf(" %7.3f |", angles.theta);
			fflush(stdout);
		}

		rc_usleep(1000000 / PRINTF_HZ); // Sleep to set 10HZ print rate
	}
	return NULL;
}

/*******************************************************************************
* void on_pause_released()
*
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void on_pause_released(){
	// toggle betewen paused and running modes
	if(rc_get_state()==RUNNING){
		rc_set_state(PAUSED);
		rc_set_led(GREEN, OFF); // GREEN when running
		rc_set_led(RED, ON); // RED when paused
	}
	else if(rc_get_state()==PAUSED){
		rc_set_state(RUNNING);
		rc_set_led(GREEN, ON); // GREEN when running
		rc_set_led(RED, OFF); // RED when paused
	}
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
