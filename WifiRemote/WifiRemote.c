/*
 * AirBoat.c
 *
 * Created: 2018-03-20 15:26:52
 *  Author: Equipe 24
 */ 

/* ============ Include all required libraries ============= */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "driver.h"
#include "uart.h"
#include "fifo.h"
#include "lcd.h"

/* =========== Define the reception states ============= */
#define WAIT_OPEN_BRACKET 1 // Wait for beginning of data transmission
#define READ_DATA 2 // Read data transmission
#define PROCESS_DATA 3 // Data processing

/* ==================== REMOTE VARIABLES ===================== */
char* aText;
char mOutputText[16];
char output[3];
uint8_t mHorizontal;
uint8_t mVertical;
uint8_t mLiftMemory;
uint8_t mBattery;
uint8_t loopCount;
bool mButtonState;
bool mFlyMode;
bool mOldFlyState;
bool brakeEngine;

/* =================== All methods used by the microcontroller ============== */
uint8_t getMaxBatteryValue(uint8_t);
uint8_t getBatteryUsagePercentage(uint8_t,uint8_t,uint8_t);
uint8_t getRealBatteryTension(uint8_t,uint8_t);
void initializeRemote();

int main(void)
{
	/* ================ Initialize all the values and microcontroller ports on the ATMEGA32 ================ */
	initializeRemote();

	/* ================ Microcontroller loop ================ */	
	while(1){			
		// Adjust the horizontal value with an offset of 2 (can be removed, calibration)
		if(adc_read(PA1) == 126){
			mHorizontal = adc_read(PA1)  - 2;
		}else{
			mHorizontal = adc_read(PA1);
		}
		mVertical = adc_read(PA2);
		mVertical = (255 - mVertical);
		if(mVertical == 0){
			mVertical++;
		}
		// Read the battery tension
		mBattery = adc_read(PA3);
		if(mFlyMode){
			mLiftMemory = mVertical;
		}
		// Read the joystick button state to change the fly mode
		if(loopCount == 1 || loopCount == 50){
			mButtonState = read_bit(PIND, PD3);
			if(mButtonState != mOldFlyState && mButtonState == 0)
			{
				mFlyMode = !mFlyMode;
				mOldFlyState = 0;
			}
			else{
				mOldFlyState = 1;
			}
		}
		
		/* ============== Send data to the airboat ============ */
		if(uart_is_tx_buffer_empty())
		{
			mOutputText[0] = '[';
			uint8_to_string(mOutputText + 1,mHorizontal);
			uint8_to_string(mOutputText + 4, mVertical);
			if(brakeEngine){
				mOutputText[7] = 'B';
			}else{
				if(!mFlyMode){
					mOutputText[7] = 'L';
				}
				else{
					mOutputText[7] = 'S';
				}
			}			
			mOutputText[8] = ']';
			mOutputText[9] = '\0';			
			uint8_to_string(output,getBatteryUsagePercentage(mBattery,9,6));
			loopCount++;
			if(loopCount == 1){
				lcd_clear_display();
				lcd_write_string(mOutputText);
				lcd_write_string(" M:");
				lcd_write_string(output);
				lcd_write_string("%");
				if(adc_read(PA0) == 255){
					lcd_write_string("BRAKE");
					brakeEngine = 1;
				}else{
					brakeEngine = 0;
				}
				uart_put_string(mOutputText);				
			}
			
			if(loopCount == 100){
				loopCount = 0;
			}					
		}
	}	
}
	
/* ================= Methods for the microcontroller ================== */	

/************************************************************************/
/* Initialize the remote                                                */
/************************************************************************/
void initializeRemote(){
	lcd_init();
	adc_init();
	pwm_init(1,1);
	uart_init();
	uart_clean_rx_buffer();
	servo_init();
	SREG = set_bit(SREG, 7);
	uart_set_baudrate(BAUDRATE_9600);
	DDRD = clear_bit(DDRD,PD6);
	mHorizontal = 0;
	mVertical = 0;
	mLiftMemory = 0;
	mButtonState = 0;
	mFlyMode = 0;
	mOutputText[16] = NULL;
	mBattery = 0;
	loopCount = 0;
	brakeEngine = 0;
	DDRB = set_bit(DDRB,PB0);
	DDRB = set_bit(DDRB,PB1);
	DDRB = set_bit(DDRB,PB2);
	DDRB = set_bit(DDRB,PB3);
	DDRB = set_bit(DDRB,PB4);
	DDRD = clear_bit(DDRD, PD3);
	PORTD = set_bit(PORTD, PD3);
	mOldFlyState = read_bit(PIND, PD3);	
	lcd_write_string("connecting....");	
	OSCCAL = OSCCAL +4;
	DDRD = set_bit(DDRD, PD2);
	PORTD = clear_bit(PORTD,PD2);
	_delay_ms(500);
	PORTD = set_bit(PORTD,PD2);
	_delay_ms(1000);	
	uart_put_string("AT+CIPMODE=1\r\n\0");
	_delay_ms(2500);	
	uart_put_string("AT+CIPSTART=\"UDP\",\"192.168.4.1\",456,123\r\n\0");
	_delay_ms(5000);	
	uart_put_string("AT+CIPSEND\r\n\0");
	lcd_clear_display();
	lcd_write_string("Connection initialized");
	_delay_ms(500);
}	

/**************************************************************************/
/* Returns the maximum value between 0 and 255 that the battery can output*/
/**************************************************************************/
uint8_t getMaxBatteryValue(uint8_t maxTension){
	return  maxTension * 0.232558f / 3.3f * 255;
}

/************************************************************************/
/* Returns the battery usage percentage                                 */
/************************************************************************/
uint8_t getBatteryUsagePercentage(uint8_t adcValue, uint8_t maxTension, uint8_t minTension){
	return (int)((((float)adcValue / (float)getMaxBatteryValue(maxTension) - ((float)minTension/(float)maxTension)) *100) /(100-(100*minTension/maxTension)) * 100) ;
}

/************************************************************************/
/* Returns the real battery tension                                     */
/************************************************************************/
uint8_t getRealBatteryTension(uint8_t adcValue, uint8_t maxTension){
	return  (int)((float)adcValue * 10 / (float)getMaxBatteryValue(maxTension) * maxTension ) ; // For more precision, output the value x10
}


