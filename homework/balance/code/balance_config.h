/*******************************************************************************
* balance_config.h
*
* Contains the settings for configuration of balance.c
*******************************************************************************/

#ifndef BALANCE_CONFIG
#define BALANCE_CONFIG

// Set constants
#define	INNER_RATE 200 // Loop rate
#define	OUTER_RATE 20 // Loop rate
#define	DT_INNER 0.005 // 1/SAMPLE_RATE_HZ
#define	DT_OUTER 0.05 // 1/SAMPLE_RATE_HZ

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

// inner loop controller: 200hz
#define D1_GAIN 0.8
#define D1_ORDER 2
#define D1_NUM {-6.289, 11.910, -5.634}
#define D1_DEN { 1.000, -1.702,  0.702}
#define D1_SATURATION_TIMEOUT	0.5

// outer loop controller: 20hz
#define D2_GAIN 0.7
#define	D2_ORDER 1
#define D2_NUM {0.18856,  -0.37209,  0.18354}
#define D2_DEN {0.18856,  -0.37209,  0.18354}
#define THETA_REF_MAX 0.37

#define TAU 2 // Filter time constant
#define WC 0.5 // 1/TAU
#define PRINTF_HZ 10 // printf_data rate

#endif	//BALANCE_CONFIG
