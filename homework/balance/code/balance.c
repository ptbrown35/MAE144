/*******************************************************************************
* File: balance.c
* Author: Parker Brown
* Date: 12/15/2017
* Course: MAE 144, Fall 2017
* Description: Balance program estimates MIP state, evaluates D1 and D2
* controllers, checks out of bounds conditions to disarm.
*******************************************************************************/

// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
// Nice to have for TWO_PI
#include <rc_usefulincludes.h>
// main roboticscape API header
#include <roboticscape.h>
#include "balance_config.h"

// Controller arming enumerated type
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
	float theta;
	float theta_error[3];
	float theta_ref;

	float phi;
	float phi_error[3];
	float phi_ref;
}angles_t;

// Struct for filters
typedef struct filter_t{
	float lp_coeff[2];
	float hp_coeff[3];
} filter_t;

// Struct for controller
typedef struct controllers_t{
	float d1_u[3];
	float d2_u[3];
	arm_state_t arm_state;
} controllers_t;

// Function declarations
void angle_mananger(); // MIP state estimation
float tf2(float a[3], float b[3], float u[3], float y[3], float k);
void d1_ctrl(); // D1 Controller
void inner_loop(); // IMU ISR func with arming checks and motor driving
void d2_ctrl(); // D2 Controller
void* outer_loop(void* ptr); // outer_thread func for D2 Controller
int arm_controller(); // Ser controller state to ARMED
int disarm_controller(); // Set contrller state to DISARMED
int start_condition(); // Start with upright condition
void zero_out(); // Zero out controllers and filters
void* printf_data(void* ptr); // printf_thread func to print data
void on_pause_pressed(); // do stuff when paused button is pressed
void on_pause_released(); // do stuff when paused button is released

// Global Variables
filter_t filter; // filter struct to hold filter coefficients
angles_t angles; // angles struct to hold theta angles
controllers_t ctrl; // d1, d2 controller struct
rc_imu_data_t data; // imu struct to hold new data

// Initialize Controller coefficients
float d1_num[] = D1_NUM;
float d1_den[] = D1_DEN;
float d2_num[] = D2_NUM;
float d2_den[] = D2_DEN;

/*******************************************************************************
* int main()
*
* balance main function contains these critical components
* - call to rc_initialize() at the beginning
* - UNINITIALIZED and DISARMED
* - Initialize DMP for interrupt
* - Start and schedule outer_thread
* - Start and schedule printf_thread
* - Initialize filters
*	- Set RUNNING and start IMU ISR
* - main while loop that checks for EXITING condition
*		- checks start condition that arms controllers
* - shutdown procedures
* - rc_cleanup() at the end
*******************************************************************************/
int main(){

	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	// Set UNINITIALIZED while setting up
	rc_set_led(RED,1);
	rc_set_led(GREEN,0);
	rc_set_state(UNINITIALIZED);

	// make sure controller state starts DISARMED
	ctrl.arm_state = DISARMED;

	// Initialize imu
	rc_imu_config_t conf = rc_default_imu_config(); // imu config to defaults
	conf.dmp_sample_rate = INNER_RATE;
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
	outer_params.sched_priority = 60; // Reasonably low priority
	pthread_create(&outer_thread, NULL, outer_loop, (void*) NULL);
	pthread_setschedparam(outer_thread, SCHED_FIFO, &outer_params);

	// Start printf_thread
	pthread_t printf_thread;
	struct sched_param printf_params;
	printf_params.sched_priority = 20; // Reasonably low priority
	pthread_create(&printf_thread, NULL, printf_data, (void*) NULL);
	pthread_setschedparam(printf_thread, SCHED_FIFO, &printf_params);

	// Initialize filter variables
	filter.lp_coeff[0] = -(WC*DT_INNER-1);
	filter.lp_coeff[1] =  WC*DT_INNER;
	filter.hp_coeff[0] = -(WC*DT_INNER-1);
	filter.hp_coeff[1] = 1;
	filter.hp_coeff[2] = -1;

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);
	rc_set_led(GREEN, ON);  // GREEN when running
	rc_set_led(RED, OFF);  // RED when paused

	rc_set_imu_interrupt_func(&inner_loop); // IMU ISR for D1 Controller

  // Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		rc_usleep(1000000 / 100); // 100hz

		// nothing to do if paused, go back to beginning of loop
		if(rc_get_state() != RUNNING) continue;

		// Wait for start condition (upright) to pass, then arm controllers
		if(ctrl.arm_state == DISARMED){
			if(start_condition()==0){
				zero_out();
				arm_controller();
			}
			else continue; // do nothing if start condition fails
		}

	}

	// Shutdown procedures
	printf("Joining printf_thread... ");
	pthread_join(printf_thread, NULL);
	printf("joined.\n");
	printf("Joining outer_thread... ");
	pthread_join(outer_thread, NULL);
	printf("joined.\n");
	disarm_controller(); // Disarm controller after closing all threads
	rc_power_off_imu();

	// exit cleanly
	rc_cleanup();
	return 0;
}

/*******************************************************************************
* void angle_mananger()
*
* MIP state estimation, theta, phi, and their outputs calculated.
*******************************************************************************/
void angle_mananger(){
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
	angles.theta = angles.theta_a + angles.theta_g;
	// Correct for BBBlue mount angle
	angles.theta += CAPE_MOUNT_ANGLE;
	// Complimentary Filter end

	// Theta error for D1
	angles.theta_error[0] = angles.theta_ref - angles.theta;

	// Update theta values
	angles.theta_g_raw[1] = angles.theta_g_raw[0]; // Hold on for integration
	angles.theta_error[2] = angles.theta_error[1]; // Hold on for D1 input
	angles.theta_error[1] = angles.theta_error[0]; // Hold on for D1 input

	// Get phi [rad] for D2 Controller
	wheelAngleL = ((rc_get_encoder_pos(ENCODER_CHANNEL_L) * TWO_PI) \
								/ (ENCODER_POLARITY_L * GEAR_RATIO * ENCODER_RES));
	wheelAngleR = ((rc_get_encoder_pos(ENCODER_CHANNEL_R) * TWO_PI) \
								/ (ENCODER_POLARITY_R * GEAR_RATIO * ENCODER_RES));
	angles.phi = ((wheelAngleL + wheelAngleR)/2) + angles.theta;
	angles.phi_error[0] = angles.phi_ref - angles.phi;
}

/*******************************************************************************
* float tf(const float a[3], const float b[3], const float u[3], const float y[3], float k)
*
* Computes difference equation for second order transfer function.
* Input: denominator and numerator coefficients a[3] and b[3], old inputs uk's
* u[3], old outputs yk's y[3]
* Output: float y_new from evaluation of tf
*******************************************************************************/
float tf2(float a[3], float b[3], float u[3], float y[3], float gain){
	float y_new;
	y_new = 0;
	// Assume a nd b are normalized by a[0]
	// Compute y_new for numerator coefficients b[i]
	int i;
  for(i = 0; i < 3; i++){
    y_new += b[i] * u[i];
  }
  // Compute y_new for denominator coefficients a[j]
	int j;
  for(j = 1; j < 3; j++){
    y_new -= a[j] * y[j];
  }

	// scale by gain
	y_new = y_new * gain;

  // return newest output
  return y_new;
}

/*******************************************************************************
* void d1_ctrl()
*
* D1 controller evaluation with tf2 and saturation check.
*******************************************************************************/
void d1_ctrl(){
	// Second order tf for D1 Controller
	ctrl.d1_u[0] = tf2(d1_den, d1_num, angles.theta_error, ctrl.d1_u, D1_GAIN);

	// Saturate D1 Controller output
	if (ctrl.d1_u[0] > 1.0){
		ctrl.d1_u[0] = 1.0;
	}	else if (ctrl.d1_u[0] < -1.0){
		ctrl.d1_u[0] = -1.0;
	}

	// update values
	ctrl.d1_u[2] = ctrl.d1_u[1]; // Hold on for tf2
	ctrl.d1_u[1] = ctrl.d1_u[0]; // Hold on for tf2
}

/*******************************************************************************
* void inner_loop()
*
* Inner (fast) loop run in interrupt service routine. Gets data, angles, errors.
* Checks tipping and loop saturation, disarms on failure. Runs D1 Controller.
* Drives motors if everyting passes.
*******************************************************************************/
void inner_loop(){
	static int sat_counter = 0;
	float duty = 0;

	/*************************************************************
	* MIP state estimation: phi and theta angles
	***************************************************************/
	angle_mananger();

	/*************************************************************
	* check for various exit conditions AFTER state estimate
	***************************************************************/
	//DISARM if EXITING
	if(rc_get_state() == EXITING){
		rc_disable_motors();
		return;
	}
	// DISARM if not RUNNING (i.e. PAUSED)
	if((rc_get_state() != RUNNING) && (ctrl.arm_state == ARMED)){
		disarm_controller();
		return;
	}
	// Return out of loop if DISARMED
	if(ctrl.arm_state == DISARMED){
		return;
	}
	// DISARM if tip over detected
	if(fabs(angles.theta) > TIP_ANGLE){
		disarm_controller();
		printf("tip detected \n");
		return;
	}

	/*************************************************************
	* Run inner loop if checks pass.
	*************************************************************/
	d1_ctrl();

	/*************************************************************
	* Check if the inner loop saturated. If it saturates for over
	* the timout, DISARM the controller.
	*************************************************************/
	if(fabs(ctrl.d1_u[0]) > 0.95) sat_counter++;
	else sat_counter = 0;
	// if saturate for a second, disarm for safety
	if(sat_counter > (INNER_RATE * D1_SATURATION_TIMEOUT)){
		printf("inner loop controller saturated\n");
		disarm_controller();
		sat_counter = 0;
		return;
	}

	/**********************************************************
	* Drive motors.
	***********************************************************/
	duty = ctrl.d1_u[0]; // Set duty cycle to write to motors
	rc_set_motor(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * duty);
	rc_set_motor(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * duty);

	return;
}

/*******************************************************************************
* void d2_ctrl()
*
* D1 controller evaluation with tf2 and saturation check.
*******************************************************************************/
void d2_ctrl(){
	// Second order tf for D2 Controller
	ctrl.d2_u[0] = tf2(d2_den, d2_num, angles.phi_error, ctrl.d2_u, D2_GAIN);

	// Saturate D2 controller output
	if (ctrl.d2_u[0] > THETA_REF_MAX){
		ctrl.d2_u[0] = THETA_REF_MAX;
	}	else if (ctrl.d2_u[0] < -THETA_REF_MAX){
		ctrl.d2_u[0] = -THETA_REF_MAX;
	}

	// Theta reference set by D2 passed to D1 theta error inputs
	angles.theta_ref = ctrl.d2_u[0]; // uncomment to close outer loop

	// update values
	ctrl.d1_u[2] = ctrl.d1_u[1]; // Hold on for tf2
	ctrl.d1_u[1] = ctrl.d1_u[0]; // Hold on for tf2
	angles.phi_error[2] = angles.phi_error[1]; // Hold on for tf2
	angles.phi_error[1] = angles.phi_error[0]; // Hold on for tf2

	return;
}

/*******************************************************************************
* void outer_loop()
*
* Runs D2 controller in outer_loop thread.
*******************************************************************************/
void* outer_loop(void* ptr){
	while(rc_get_state()!=EXITING){
		// Just run D2 Controller and wait
		d2_ctrl();
		rc_usleep(1000000 / OUTER_RATE); // Sleep to set outer loop rate
	}
	return NULL;
}

/*******************************************************************************
* int disarm_controller()
*
* Disable motors and set arming state to DISARMED
*******************************************************************************/
int disarm_controller(){
	rc_disable_motors();
	ctrl.arm_state = DISARMED;
	return 0;
}

/*******************************************************************************
* int arm_controller()
*
* Zero out the controllers and encoders. Enable motors and arm the controllers.
*******************************************************************************/
int arm_controller(){
	zero_out();
	rc_set_encoder_pos(ENCODER_CHANNEL_L,0);
	rc_set_encoder_pos(ENCODER_CHANNEL_R,0);
	ctrl.arm_state = ARMED;
	rc_enable_motors();
	return 0;
}

/*******************************************************************************
* int start_condition()
*
* Wait for MiP to be held upright long enough to initiate arming.
* Returns -1 on fail.
*******************************************************************************/
int start_condition(){
	int count = 0;
	const int count_hz = 20;	// check 20 times per second
	int count_needed = round(START_DELAY*count_hz);
	int wait_us = 1000000/count_hz;

	// Wait for MIP to be tipped out of START_ANGLE range
	while(rc_get_state() == RUNNING){
		// if within range, start counting
		if(fabs(angles.theta) > START_ANGLE) count++;
		// fell out of range, restart counter
		else count = 0;
		// waited long enough, return
		if(count >= count_needed) break;
		rc_usleep(wait_us);
	}
	// Wait for MIP to be within START_ANGLE range
	count = 0;
	while(rc_get_state() == RUNNING){
		// If within range, start counting
		if(fabs(angles.theta) < START_ANGLE) count++;
		// Else out of range and restart count
		else count = 0;
		// Return if waited long enough
		if(count >= count_needed) return 0;
		rc_usleep(wait_us);
	}
	return -1;
}

/*******************************************************************************
* void zero_out()
*
* Zero out filter inputs nad integration values.
*******************************************************************************/
void zero_out(){
	// Complimentary filter values
	angles.theta_a_raw = 0;
	angles.theta_g_raw[0] = 0;
	angles.theta_g_raw[1] = 0;
	angles.theta_a = 0;
	angles.theta_g = 0;
	// D1 Controller feedback
	angles.theta = 0;
	// D1 Controller inputs
	angles.theta_ref = 0;
	angles.theta_error[0] = 0;
	angles.theta_error[1] = 0;
	angles.theta_error[2] = 0;
	// D1 Controller outputs
	ctrl.d1_u[0] = 0;
	ctrl.d1_u[1] = 0;
	ctrl.d1_u[2] = 0;
	// D2 Controller feedback
	angles.phi = 0;
	// D2 Controller inputs
	angles.phi_ref = 0;
	angles.phi_error[0] = 0;
	angles.phi_error[1] = 0;
	angles.phi_error[2] = 0;
	// D2 Controller outputs
	ctrl.d2_u[0] = 0;
	ctrl.d2_u[1] = 0;
	ctrl.d2_u[2] = 0;
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
		// First time in RUNNING, print header
		if((new_rc_state == RUNNING) && (last_rc_state != RUNNING)){
			printf("\nRUNNING: Hold upright to balance.\n|");
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
			// First time being PAUSED, print pause statement
			printf("\nPAUSED: Press pause again to start.\n");
		}
		last_rc_state = new_rc_state; // update last_rc_state

		// Print data while RUNNING
		if(new_rc_state == RUNNING){
			// Print raw angles
			printf("\r|"); // carriage return because it looks pretty
			printf(" %7.3f |", angles.theta);
			printf(" %7.3f |", angles.theta_ref);
			printf(" %7.3f |", angles.phi);
			printf(" %7.3f |", angles.phi_ref);
			printf(" %7.3f |", ctrl.d1_u[0]);
			printf(" %7.3f |", ctrl.d2_u[0]);

			if(ctrl.arm_state == ARMED) printf("  ARMED  |");
			else printf("DISARMED |");
			fflush(stdout);
		}

		rc_usleep(1000000 / PRINTF_HZ); // Sleep to set print rate
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
		disarm_controller(); // Always set DISARMED on PAUSE change
		rc_set_led(GREEN, OFF); // GREEN when running
		rc_set_led(RED, ON); // RED when paused
	}
	else if(rc_get_state()==PAUSED){
		rc_set_state(RUNNING);
		disarm_controller(); // Always set DISARMED on PAUSE change
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
