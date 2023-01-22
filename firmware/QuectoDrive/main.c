/*
* GccApplication1.c
*
* Created: 26/10/2022 00:33:08
* Author : Edward
*/

#include <avr/io.h>
#include <avr/interrupt.h>

#define PER_CELL_CUTOFF 3.0

float startup_voltage = 0.0;

ISR(ADC0_WCOMP_vect) {
	while (1) {
		// Cutoff voltage
	}
}

void set_clk(void) {
	CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSC20M_gc;	// Select 20MHz internal oscillator
	CLKCTRL.MCLKCTRLB = CLKCTRL_PEN_bm				// Enable prescaler
					| CLKCTRL_PDIV_2X_gc;			// Set /2 prescaler for 10MHz
}

void init_pwm(void) {
	TCA0.SINGLE.CTRLD = TCA_SINGLE_SPLITM_bm;		// Enable split mode
	TCA0.SPLIT.HPER = 0xff;							// Set high timer period of 255
	TCA0.SPLIT.LPER = 0xff;							// Set low timer period of 255
	TCA0.SPLIT.HCNT = 0;							// Reset high timer initial count
	TCA0.SPLIT.LCNT = 0;							// Reset low timer initial count
	TCA0.SPLIT.CTRLB = TCA_SPLIT_HCMP0EN_bm			// Enable high timer output 0
					| TCA_SPLIT_HCMP1EN_bm			// Enable high timer output 1
					| TCA_SPLIT_HCMP2EN_bm			// Enable high timer output 2
					| TCA_SPLIT_LCMP0EN_bm			// Enable low timer output 0
					| TCA_SPLIT_LCMP1EN_bm			// Enable low timer output 1
					| TCA_SPLIT_LCMP2EN_bm;			// Enable low timer output 2
	TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV1_gc		// Set /1 clock prescaler for 39.2KHz
					| TCA_SPLIT_ENABLE_bm;			// Start timer
	
	TCA0.SPLIT.HCMP0 = 0;							// Set initial value to 0
	TCA0.SPLIT.HCMP1 = 0;							// Set initial value to 0
	TCA0.SPLIT.HCMP2 = 0;							// Set initial value to 0
	TCA0.SPLIT.LCMP0 = 0;							// Set initial value to 0
	TCA0.SPLIT.LCMP1 = 0;							// Set initial value to 0
	TCA0.SPLIT.LCMP2 = 0;							// Set initial value to 0
	
	PORTMUX.CTRLC = PORTMUX_TCA02_bm;				// Map WO2 to alternate PB5
	PORTB.DIRSET = 1 << 0;							// Set PB0 as output
	PORTB.DIRSET = 1 << 1;							// Set PB1 as output
	PORTB.DIRSET = 1 << 5;							// Set PB5 as output
	PORTA.DIRSET = 1 << 3;							// Set PA3 as output
	PORTA.DIRSET = 1 << 4;							// Set PA4 as output
	PORTA.DIRSET = 1 << 5;							// Set PA5 as output
}

void init_adc_and_cutoff(void) {
	// Configured to get 94 readings per second
	ADC0.CTRLB = ADC_SAMPNUM_ACC64_gc;			// Take 64 samples per reading
	ADC0.CTRLC = ADC_REFSEL_VDDREF_gc			// Select VDD as reference
				| ADC_PRESC_DIV128_gc;			// Select /128 prescaler for 78KHz
	ADC0.CTRLA = ADC_RESSEL_10BIT_gc			// Set 10 bit resolution
				| ADC_FREERUN_bm				// Set free-running
				| ADC_ENABLE_bm;				// Enable
	ADC0.MUXPOS = ADC_MUXPOS1_bm;				// Set PA1 as input
	ADC0.COMMAND = ADC_STCONV_bm;				// Start running
	
	// Wait for first conversion to complete
	while (ADC0.INTFLAGS & ADC_RESRDY_bm) continue;
	
	// Figure out cell count and appropriate window threshold
	float conversion_factor = (3.3/(1 << 10))*(10.0/37.0);
	startup_voltage = ADC0.RES * conversion_factor;
	uint8_t cell_count = startup_voltage / PER_CELL_CUTOFF;
	uint16_t threshold = (PER_CELL_CUTOFF * cell_count) / conversion_factor;
	
	ADC0.WINLTH = threshold;					// Set window low threshold
	ADC0.INTCTRL = ADC_WCMP_bm;					// Enable interrupt
	ADC0.CTRLE = ADC_WINCM_BELOW_gc;			// Set mode to below and enable
	
}

int main(void) {
	set_clk();
	init_pwm();
	init_adc_and_cutoff();
	
	while (1) {
		TCA0.SPLIT.HCMP0 = 127;
		TCA0.SPLIT.HCMP1 = 127;
		TCA0.SPLIT.HCMP2 = 127;
		TCA0.SPLIT.LCMP0 = 127;
		TCA0.SPLIT.LCMP1 = 127;
		TCA0.SPLIT.LCMP2 = 127;
	}
}

