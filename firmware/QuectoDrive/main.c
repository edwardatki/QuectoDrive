/*
* GccApplication1.c
*
* Created: 26/10/2022 00:33:08
* Author : Edward
*/

#include <avr/io.h>
#include <avr/interrupt.h>

// pymcuprog write -f QuectoDrive.hex  --erase --verify -d attiny1616 -t uart -u com8

#define F_CPU 3300000
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#define PER_CELL_CUTOFF 3.0

#define PROTOCOL_LENGTH 0x20
#define PROTOCOL_OVERHEAD 3             // packet is <len><cmd><data....><chkl><chkh>, overhead=cmd+chk bytes
#define PROTOCOL_TIMEGAP 3              // Packets are received every ~7ms so use ~half that for the gap
#define PROTOCOL_CHANNELS 14
#define PROTOCOL_COMMAND_CHANNELS 0x40

float startup_voltage;
float cutoff_voltage;

uint16_t channels [PROTOCOL_CHANNELS] = {1500};
uint32_t last_packet_ms;

volatile uint32_t millis = 0;

ISR(TCB0_INT_vect) {
	TCB0.INTFLAGS = TCB_CAPT_bm;
	millis += 1;
}

// TODO: Implement low voltage cutoff with continuous conversions
ISR(ADC0_WCOMP_vect) {
	
}

// TODO: Fix clock setting, or maybe 3.3MHz is enough
void set_clk(void) {
	//CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSC20M_gc;	// Select 20MHz internal oscillator
	//CLKCTRL.MCLKCTRLB = CLKCTRL_PEN_bm			// Enable prescaler
	//				| CLKCTRL_PDIV_2X_gc;			// Set /2 prescaler for 10MHz
}

// Set up an interrupt to fire at 1Khz aka 1ms
void init_systick(void) {
	TCB0.CCMP = 3300;								// Set timer period
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc				// Select 3.3MHz
			   | TCB_ENABLE_bm						// Enable timer
			   | TCB_RUNSTDBY_bm;					// Start running
	TCB0.CTRLB = TCB_CNTMODE_INT_gc;				// Set mode
	TCB0.INTCTRL = TCB_CAPT_bm;						// Enable interrupt
}

void init_motors(void) {
	TCA0.SINGLE.CTRLD = TCA_SINGLE_SPLITM_bm;		// Enable split mode
	TCA0.SPLIT.HPER = 250;							// Set high timer period of 255
	TCA0.SPLIT.LPER = 250;							// Set low timer period of 255
	TCA0.SPLIT.HCNT = 0;							// Reset high timer initial count
	TCA0.SPLIT.LCNT = 0;							// Reset low timer initial count
	TCA0.SPLIT.CTRLB = TCA_SPLIT_LCMP0EN_bm			// Enable low timer output 0
					 | TCA_SPLIT_LCMP1EN_bm			// Enable low timer output 1
					 | TCA_SPLIT_LCMP2EN_bm			// Enable low timer output 2
					//| TCA_SPLIT_HCMP0EN_bm		// Enable high timer output 0
					//| TCA_SPLIT_HCMP1EN_bm		// Enable high timer output 1
					 | TCA_SPLIT_HCMP2EN_bm;		// Enable high timer output 2
	TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV1_gc		// Set /1 clock prescaler for 3.3MHz
					 | TCA_SPLIT_ENABLE_bm;			// Start timer
	
	// 3.3MHz/250, final period is 13.2KHz I think
	// DRV8837 says 250KHz max, frequency doesn't seems effect performance
	// Higher frequency has less whine so we've gone highest possible
	
	TCA0.SPLIT.LCMP0 = 0;							// Set initial value to 0
	TCA0.SPLIT.LCMP1 = 0;							// Set initial value to 0
	TCA0.SPLIT.LCMP2 = 0;							// Set initial value to 0
	//TCA0.SPLIT.HCMP0 = 0;							// Set initial value to 0
	//TCA0.SPLIT.HCMP1 = 0;							// Set initial value to 0
	TCA0.SPLIT.HCMP2 = 0;							// Set initial value to 0
	
	//PORTMUX.CTRLC |= PORTMUX_TCA02_bm;			// Map WO2 to alternate PB5
	PORTB.DIRSET = PIN0_bm;							// Set PB0 as output
	PORTB.DIRSET = PIN1_bm;							// Set PB1 as output
	PORTB.DIRSET = PIN2_bm;							// Set PB2 as output
	PORTB.DIRSET = PIN5_bm;							// Set PB5 as output
	PORTA.DIRSET = PIN5_bm;							// Set PA5 as output
}

void init_weapons(void) {
	TCD0.CTRLB = TCD_WGMODE_ONERAMP_gc;				// Single ramp mode
	
	CPU_CCP = CCP_IOREG_gc;							// Enable write protected register
	TCD0.FAULTCTRL = TCD_CMPCEN_bm;					// Enable WOC
	
	TCD0.CMPACLR = 2062;							// Period of 2062 for 20ms period
	TCD0.CMPBCLR = 2062;
	TCD0.CMPASET = 2062-103;							// 103 to 206 for 1.03ms to 2.06ms
	TCD0.CMPBSET = 2062-103;
	
	while(!(TCD0.STATUS & TCD_ENRDY_bm)) {}			// Ensure ENRDY bit is set
	
	TCD0.CTRLA = TCD_CLKSEL_SYSCLK_gc				// Choose 3.3MHz clock
			   | TCD_SYNCPRES_DIV1_gc
			   | TCD_CNTPRES_DIV32_gc				// Choose /32 prescaler for 103.1KHz
			   | TCD_ENABLE_bm;						// Enable timer
	
	PORTC.DIRSET = PIN0_bm;							// Set PC0 as output
}

// THIS IS ALL BROKEN I THINK
void init_adc_and_cutoff(void) {
	// Configured to get 94 readings per second
	ADC0.CTRLB = ADC_SAMPNUM_ACC64_gc;				// Take 64 samples per reading
	ADC0.CTRLC = ADC_REFSEL_VDDREF_gc				// Select VDD as reference
				| ADC_PRESC_DIV128_gc;				// Select /128 prescaler for 78KHz
	ADC0.CTRLA = ADC_RESSEL_10BIT_gc				// Set 10 bit resolution
				| ADC_FREERUN_bm					// Set free-running
				| ADC_ENABLE_bm;					// Enable
	ADC0.MUXPOS = ADC_MUXPOS4_bm;					// Set PA1 as input
	ADC0.COMMAND = ADC_STCONV_bm;					// Start running
	
	// Wait for first conversion to complete
	while (ADC0.INTFLAGS & ADC_RESRDY_bm) continue;
	
	// Figure out cell count and appropriate window threshold
	float conversion_factor = (3.3/(1 << 10))*(10.0/37.0);
	startup_voltage = ADC0.RES * conversion_factor;
	uint8_t cell_count = startup_voltage / PER_CELL_CUTOFF;
	cutoff_voltage = PER_CELL_CUTOFF * cell_count;
	
	// TODO: setup interrupts for conversion complete
}

void init_uart(void) {
	PORTMUX.CTRLB |= PORTMUX_USART0_bm;				// Set USART0 alternate pins
	PORTA.DIRCLR = PIN2_bm;							// Set PA2 as input
	
	USART0.BAUD = (uint16_t)USART0_BAUD_RATE(115200);
	USART0.CTRLB = USART_RXEN_bm;					// Enable receive pin
	
	USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc
				| USART_CHSIZE_8BIT_gc
				| USART_PMODE_DISABLED_gc
				| USART_SBMODE_1BIT_gc;
}

// TODO if transmitter turned off and on then controls stutter and lag
void ibus_update(void) {
	enum {GET_LENGTH, GET_DATA, GET_CHKSUML, GET_CHKSUMH, DISCARD};

	static uint32_t last_byte_ms;					// Timestamp of last byte
	static uint8_t state = DISCARD;					// Receive state
	static uint8_t buffer[PROTOCOL_LENGTH];			// Message buffer
	static uint8_t ptr;								// Pointer in buffer
	static uint8_t length;							// Message length
	static uint16_t chksum;							// Calculated checksum
	static uint8_t lchksum;							// Received checksum lower byte
	
	if (USART0.STATUS & USART_RXCIF_bm) {
		// Only consider a new data packet if we have not heard anything for >3ms
		uint32_t now = millis;
		if ((now - last_byte_ms) >= PROTOCOL_TIMEGAP) {
			state = GET_LENGTH;
		}
		last_byte_ms = now;

		uint8_t byte =  USART0.RXDATAL;
		switch (state) {
			case GET_LENGTH:
				if ((byte <= PROTOCOL_LENGTH) & (byte > PROTOCOL_OVERHEAD)) {
					ptr = 0;
					length = byte - PROTOCOL_OVERHEAD;
					chksum = 0xFFFF - byte;
					state = GET_DATA;
				} else {
					state = DISCARD;
				}
				break;
			
			case GET_DATA:
				buffer[ptr++] = byte;
				chksum -= byte;
				if (ptr == length) {
					state = GET_CHKSUML;
				}
				break;
			
			case GET_CHKSUML:
				lchksum = byte;
				state = GET_CHKSUMH;
				break;
			
			case GET_CHKSUMH:
				// Validate checksum
				if (chksum == ((byte << 8) + lchksum)) {
					// Checksum is all fine so execute command
					if (buffer[0] == PROTOCOL_COMMAND_CHANNELS) {
						// Channel data command
						for (uint8_t i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2) {
							channels[i / 2] = buffer[i] | (buffer[i + 1] << 8);
						}
					}
					
					// Update last valid packet timestamp
					last_packet_ms = millis;
				}
				state = DISCARD;
				break;

			default:
				break;
		}
	}
}

void set_led(uint8_t value) {
	TCA0.SPLIT.LCMP0 = value;
}

void set_motor_a(int16_t value) {
	if (value > 250) value = 250;
	if (value < -250) value = -250;
	
	if (value > 0) {
		TCA0.SPLIT.LCMP1 = 250-value;
		TCA0.SPLIT.LCMP2 = 250;
	} else if (value < 0) {
		TCA0.SPLIT.LCMP1 = 250;
		TCA0.SPLIT.LCMP2 = 250+value;
	} else {
		TCA0.SPLIT.LCMP1 = 250;
		TCA0.SPLIT.LCMP2 = 250;
	}
}

void set_motor_b(int16_t value) {
	if (value > 250) value = 250;
	if (value < -250) value = -250;
	
	if (value > 0) {
		PORTB.OUTCLR = 1 << 5;
		TCA0.SPLIT.HCMP2 = 0+value;
	} else if (value < 0) {
		PORTB.OUTSET = 1 << 5;
		TCA0.SPLIT.HCMP2 = 250+value;
	} else {
		PORTB.OUTSET = 1 << 5;
		TCA0.SPLIT.HCMP2 = 250;
	}
}

void set_weapon_a(uint16_t value) {
		// Need to map 1000-2000 to 103-206
		// Only managed 100-200
		TCD0.CMPASET = 2062-(value/10);
		TCD0.CTRLE = TCD_SYNCEOC_bm;
}

int main(void) {
	//set_clk();
	init_motors();
	init_weapons();
	//init_adc_and_cutoff();
	init_uart();
	init_systick();

	sei();

	while (1) {
		// Read data from receiver
		ibus_update();
		
		if (millis-last_packet_ms < 100) {
			set_led(5);
			
			// Update motor speeds
			int16_t fb = ((int16_t)channels[0]-1500)/2;
			int16_t lr = ((int16_t)channels[1]-1500)/2;
			set_motor_a(fb+lr);
			set_motor_b(fb-lr);
			set_weapon_a(channels[2]);
		} else {
			set_led(0);
			
			// Stop motors
			set_motor_a(0);
			set_motor_b(0);
			set_weapon_a(0);
		}
	}
}

