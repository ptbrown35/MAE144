/*******************************************************************************
* hw1_config.h
*
* Contains the settings for configuration of hw1.c
*******************************************************************************/

#ifndef HW1_CONFIG
#define HW1_CONFIG

// Set constants
#define		GEAR_RATIO		35.555		// Motor gear ratio
#define		ENCODER_RES		60		// Encoder resolution
#define		MOTOR_CHANNEL_L		3		// Left motor channel
#define		MOTOR_CHANNEL_R		2		// Right motor channel
#define		MOTOR_POLARITY_L		1		// Left motor polarity
#define		MOTOR_POLARITY_R		-1		// Right motor polarity
#define		ENCODER_CHANNEL_L		3		// Left encoder channel
#define		ENCODER_CHANNEL_R		2		// Right encoder channel
#define		ENCODER_POLARITY_L		1		// Left encoder polarity
#define		ENCODER_POLARITY_R		-1		// Right encoder polarity

#define		D_GAIN		0.9		// Proportional loop gain
#define		SETPOINT_ZERO		0		// Zero set point for stationary wheel

#define		SAMPLE_RATE_HZ		100    // Loop rate
#define		DT		0.01   // 1/SAMPLE_RATE_HZ

#endif	//HW1_CONFIG
