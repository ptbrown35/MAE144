/*******************************************************************************
* hw2_isr_config.h
*
* Contains the settings for configuration of hw2_isr.c
*******************************************************************************/

#ifndef HW2_ISR_CONFIG
#define HW2_ISR_CONFIG

// Set constants
#define		SAMPLE_RATE_HZ		100    // Loop rate
#define		DT		0.01   // 1/SAMPLE_RATE_HZ
#define   TAU   0.5   // Filter time  constant
#define   WC    2   // 1/TAU
#define   PRINTF_HZ   10    // printf_data rate

#endif	//HW2_ISR_CONFIG
