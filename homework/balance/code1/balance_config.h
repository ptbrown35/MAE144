/*******************************************************************************
* balance_config.h
*
* Contains the settings for configuration of balance.c
*******************************************************************************/

#ifndef BALANCE_CONFIG
#define BALANCE_CONFIG

// Set loop rates
#define	INNER_RATE 100 // Inner oop rate
#define	OUTER_RATE 20 // Outer loop rate
#define	DT_INNER 0.005 // 1/SAMPLE_RATE_HZ
#define	DT_OUTER 0.05 // 1/SAMPLE_RATE_HZ

// Set hardware constants
#define CAPE_MOUNT_ANGLE 0.49 // increase if mip tends to roll forward
#define GEAR_RATIO 35.555 // Motor gear ratio
#define ENCODER_RES 60 // Encoder resolution
#define MOTOR_CHANNEL_L 3 // Left motor channel
#define MOTOR_CHANNEL_R 2 // Right motor channel
#define MOTOR_POLARITY_L 1 // Left motor polarity
#define MOTOR_POLARITY_R -1 // Right motor polarity
#define ENCODER_CHANNEL_L 3 // Left encoder channel
#define ENCODER_CHANNEL_R 2 // Right encoder channel
#define ENCODER_POLARITY_L 1 // Left encoder polarity
#define ENCODER_POLARITY_R -1 // Right encoder polarity

// inner loop controller: 100hz
#define D1_GAIN 1.0
#define D1_ORDER 2
#define D1_NUM {-4.9500,  8.8709, -3.9709}
#define D1_DEN { 1.0000, -1.4810,  0.4812}
#define D1_SAT 1
#define D1_SATURATION_TIMEOUT	0.5

// outer loop controller: 20hz
#define D2_GAIN 1.0
#define	D2_ORDER 1
#define D2_NUM {1.0000, -0.9961}
#define D2_DEN {1.0000, -0.6065}
#define D2_SAT 0.3

// Arming conditions
#define TIP_ANGLE 0.85
#define START_ANGLE 0.3
#define START_DELAY 0.4
#define PICKUP_DETECTION_TIME	0.6

// Filter constants
#define TAU 0.5 // Filter time constant
#define WC 2 // 1/TAU
#define PRINTF_HZ 100 // printf_data rate

#endif	//BALANCE_CONFIG
