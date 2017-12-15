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
	float theta_g_raw[2];
	float theta_a;
	float theta_g;
	float theta[3];
	float theta_error[3];

	float phi[3];
	float phi_error[3];
	float phi_ref;
}angles_t;

// Struct for filters
typedef struct filter_t{
	float lp_coeff[2];
	float hp_coeff[3];
} filter_t;

// Struct for D1 controller
typedef struct controllers_t{
	float d1_u[3];
	float d2_u[3];
}

// function declarations
void on_pause_pressed(); // do stuff when paused button is pressed
void on_pause_released(); // do stuff when paused button is released
void angles_manager(); // Complimentary filter
void zero_out(); // Zero out controllers and filters
float tf2(const float[3] a, const float[3] b, const float[3] u, const float[3] y, float k);
void d1_ctrl();
void d2_ctrl();
void inner_loop(); // IMU interrupt routine
void* outer_loop(void* ptr);
void* printf_data(void* ptr); // printf_thread function ot print data

// Global Variables
filter_t filter; // filter struct to hold filter coefficients
angles_t angles; // angles struct to hold theta angles
controllers_t ctrl; // d1, d2 controller struct
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
	conf.dmp_sample_rate = INNER_RATE;
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
	pthread_t outer_thread;
	struct sched_param outer_params;
	params.sched_priority = 60; // Reasonably low priority
	pthread_create(&outer_thread, NULL, outer_loop, (void*) NULL);
	pthread_setschedparam(outer_thread, SCHED_FIFO, &outer_params);

	// Start printf_thread
	pthread_t printf_thread;
	struct sched_param printf_params;
	params.sched_priority = 20; // Reasonably low priority
	pthread_create(&printf_thread, NULL, printf_data, (void*) NULL);
	pthread_setschedparam(printf_thread, SCHED_FIFO, &printf_params);

	// Initialize filter variables
	zero_out(); // Initialize filters and controllers to zero
	filter.lp_coeff[0] = -(WC*DT_INNER-1);
	filter.lp_coeff[1] =  WC*DT_INNER;
	filter.hp_coeff[0] = -(WC*DT_INNER-1);
	filter.hp_coeff[1] = 1;
	filter.hp_coeff[2] = -1;

	// Enable motors, put this in arm controller
	rc_enable_motors();

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
	pthread_join(outer_thread, NULL);
	rc_disable_motors();
	rc_power_off_imu();

	// exit cleanly
	rc_cleanup();
	return 0;
}

/*******************************************************************************
* void zero_out()
*
* Zero out filter inputs nad integration values.
*******************************************************************************/
void zero_out(){
	// A whole bunch more things need to be zeroed!!!
	angles.theta_a_raw = 0;
	angles.theta_g_raw = [0,0];
	angles.theta_a = 0;
	angles.theta_g = 0;
	angles.theta = [0,0,0];
	angles.theta_error = [0,0,0];

	angles.phi = [0,0,0];
	angles.phi_error = [0,0,0];
	angles.phi_ref = 0;

	ctrl.d1_u = [0,0,0];
	ctrl.d1_u = [0,0,0];

	rc_set_encoder_pos(ENCODER_CHANNEL_L,0);
	rc_set_encoder_pos(ENCODER_CHANNEL_R,0);
}

/*******************************************************************************
* void angles_manager()
*
* Complimentary filter built by summing high and low pass fitlers applied to
* raw theta values from accel and gyro data, respectively.
*******************************************************************************/
void angles_manager(){
	float wheelAngleL = 0;
	float wheelAngleR = 0;

	// Complimentary Filter start
	angles.theta_a_raw = -1.0 * atan2(data.accel[2], data.accel[1]); // theta [rad]
	angles.theta_g_raw[0] = angles.theta_g_raw[1] \
										+ (data.gyro[0] * DT_INNER * DEG_TO_RAD); // theta [rad]
	// First order Low Pass filter of theta from raw accel data
	angles.theta_a = (filter.lp_coeff[0] * angles.theta_a) \
								 + (filter.lp_coeff[1] * angles.theta_a_raw);
  // First order high pass filter of theta from raw gyro data
	angles.theta_g = (filter.hp_coeff[0] * angles.theta_g) \
	 							 + (filter.hp_coeff[1] * angles.theta_g_raw[0]) \
								 + (filter.hp_coeff[2] * angles.theta_g_raw[1]);
  // Sum of Low and High pass filters of theta
	angles.theta[0] = angles.theta_a + angles.theta_g;
	// Correct for BBBlue mount angle
	angles.theta[0] += CAPE_MOUNT_ANGLE;
	// Complimentary Filter end

	// Theta error for D1
	angles.theta_error[0] = angles.theta_ref - angles.theta[0];

	// Update theta values
	angles.theta_g_raw[1] = angles.theta_g_raw[0]; // Integration value
	angles.theta[2] = angles.theta[1]; // Hold on for d1
	angles.theta[1] = angles.theta[0]; // Hold on for d1
	angles.theta_error[2] = angles.theta_error[1]; // Hold on for d1
	angles.theta_error[1] = angles.theta_error[0]; // Hold on for d1

	// Get phi for d2 outer controller
	wheelAngleL = ((rc_get_encoder_pos(ENCODER_CHANNEL_L) * TWO_PI) \
								/ (ENCODER_POLARITY_L * GEAR_RATIO * ENCODER_RES));
	wheelAngleR = ((rc_get_encoder_pos(ENCODER_CHANNEL_R) * TWO_PI) \
								/ (ENCODER_POLARITY_R * GEAR_RATIO * ENCODER_RES));
	angles.phi[0] = ((wheelAngleL + wheelAngleR)/2) + angles.theta[0];
	angles.phi_error[0] = angles.phi_ref - angles.phi[0];
}

/*******************************************************************************
* float tf(const float[3] a, const float[3] b, const float[3] u, const float[3] y, float k)
*
* Difference equation for second order transfer function.
* Input: coefficients numerator a[3] and denominator b[3],
* old inputs u[3], old outputs y[3]
*******************************************************************************/

float tf2(const float[3] a, const float[3] b, const float[3] u, const float[3] y, float k){
	// Assume a nd b are normalized by a[0]
	// Compute y[0] for denominator b
  for(int j = 0; j < 3; j++){
    y[0] += k * b[j] * u[j];
  }
  // Compute y[0] for a
  for(int k = 1; k < 3; k++){
    y[0] -= a[k] * y[k];
  }
  // return newest output
  return y[0];
}

/*******************************************************************************
* void d1_ctrl()
*
* D1 controller.
*******************************************************************************/
void d1_ctrl(){
	float duty = 0;

	// Second order tf for D1 Controller
	ctrl.d1_u[0] = tf2(D1_DEN, D1_NUM, angles.theta_error, ctrl.d1_u, D1_GAIN);

	// Saturate D1 Controller output
	if (ctrl.d1_u[0] > 1.0){
		ctrl.d1_u[0] = 1.0;
	}	else if (ctrl.d1_u[0] < -1.0){
		ctrl.d1_u[0] = -1.0;
	}

	duty = ctrl.d1_u[0]; // Set duty cycle to write to motors
	// rc_set_motor(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * duty);
	// rc_set_motor(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * duty);

	// update values
	ctrl.d1_u[2] = ctrl.d1_u[1];
	ctrl.d1_u[1] = ctrl.d1_u[0];
}

/*******************************************************************************
* void d2_ctrl()
*
* D2 controller.
*******************************************************************************/
void d2_ctrl(){
	// Second order tf for D2 Contoller
	ctrl.d2_u[0] = tf2(D2_DEN, D2_NUM, angles.phi_error, ctrl.d2_u, D2_GAIN);

	// Saturate d2 controller output
	if (ctrl.d2_u[0] > THETA_REF_MAX){
		ctrl.d2_u[0] = THETA_REF_MAX;
	}	else if (ctrl.d2_u[0] < -THETA_REF_MAX){
		ctrl.d2_u[0] = -THETA_REF_MAX;
	}

	// Theta reference set by d2 passed to
	// angles.theta_ref = ctrl.d2_u[0]; // uncomment to close outer loop

	// update values
	ctrl.d1_u[2] = ctrl.d1_u[1];
	ctrl.d1_u[1] = ctrl.d1_u[0];
	angles.phi[2] = angles.phi[1];
	angles.phi[1] = angles.phi[0];
	angles.phi_error[2] = angles.phi_error[1];
	angles.phi_error[1] = angles.phi_error[0];
}

/*******************************************************************************
* void inner_loop()
*
* Inner (fast) loop at 200hz.
*******************************************************************************/
void inner_loop(){
	// If RUNNING, run Complimentary Filter
	if(rc_get_state()==RUNNING){
		angles_manager(); // Complimentary Filter
		d1_ctrl();
	}
	else if(rc_get_state()==PAUSED){
		rc_set_motor_free_spin_all(); // Set motors to free spin while paused
		zero_out(); // Zero out everything
	}
}

/*******************************************************************************
* void outer_loop()
*
* Gets imu data using rc_set_imu_interrupt_func(&inner_loop)
*******************************************************************************/
void* outer_loop(void* ptr){

	while(rc_get_state()!=EXITING){
		// If RUNNING, run Complimentary Filter
		if(rc_get_state()==RUNNING){
			d2_ctrl();
		}
		else if(rc_get_state()==PAUSED){
			// zero_out(); // Zero out everything
		}

		rc_usleep(1000000 / OUTER_RATE); // Sleep to set 20HZ print rate
	}
	return NULL;
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
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("    θ    |");
			printf("  θ_ref  |");
			printf("    φ    |");
			printf("  φ_ref  |");
			printf("  d1_u   |");
			printf("  d2_u   |");
			printf("arm_state|");
			printf("\n");
		}
		else if(new_rc_state==PAUSED && last_rc_state!=PAUSED){
			// first time sonce being paused
			printf("\nPAUSED: Press pause again to start.\n");
		}
		last_rc_state = new_rc_state; // update last_rc_state

		if(new_rc_state == RUNNING){
			// Print raw angles
			printf("\r|"); // carriage return because it looks pretty
			printf(" %7.3f |", angles.theta[0]);
			printf(" %7.3f |", angles.theta_ref);
			printf(" %7.3f |", angles.phi[0]);
			printf(" %7.3f |", angles.phi_ref);
			printf(" %7.3f |", angles.d1_u[0]);
			printf(" %7.3f |", angles.d2_u[0]);.

			// if(setpoint.arm_state == ARMED) printf("  ARMED  |");
			// else printf("DISARMED |");
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
