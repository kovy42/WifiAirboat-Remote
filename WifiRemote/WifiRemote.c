/*
 * AirBoat.c
 *
 * Created: 2018-03-20 15:26:52
 *  Author: Equipe 24
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "driver.h"
#include "uart.h"
#include "fifo.h"
#include "lcd.h"

#define WAIT_DATA 0 // Attente d'une transmission
#define WAIT_OPEN_BRACKET 1 // Début de la transmission
#define READ_DATA 2 // Lecture de la transmission
#define DISPLAY_DATA 3 // Affichage

bool isOnBoard;
uint8_t getMaxBatteryValue(uint8_t);
uint8_t getBatteryUsagePercentage(uint8_t,uint8_t,uint8_t);
uint8_t getRealBatteryTension(uint8_t,uint8_t);
void DelBattery(uint8_t);


/* ==================== REMOTE VARIABLES ===================== */
uint8_t mHorizontale;
uint8_t mVerticale;
uint8_t mLiftMemory;
uint8_t mBatterie;
bool mButtonState;
bool mFlyMode;
char mOutputText[16];
bool mOldFlyState;
void initializeRemote();
char* aText;
uint8_t loopCount;
bool brakeBoi;


int main(void)
{
	/* ================== INITIALISATION GLOBALE ================== */
	lcd_init();
	adc_init();
	pwm_init(1,1);
	uart_init();
	uart_clean_rx_buffer();
	servo_init();
	
	SREG = set_bit(SREG, 7);
	uart_set_baudrate(BAUDRATE_9600);
	DDRD = clear_bit(DDRD,PD6);
	
	initializeRemote();
	//Lecture de la tension de la batterie
	
	
	while(1){
		
		
		
		// Conversion de la direction pour limiter à 45 degrés, et ajustement de 2/255 vers la droite car le servomoteur est décalibré (props à David).
		if(adc_read(PA1) == 126){
			mHorizontale = adc_read(PA1)  - 2;
		}else{
			mHorizontale = adc_read(PA1);
		}
		
		mVerticale = adc_read(PA2);
		mVerticale = (255 - mVerticale);
		if(mVerticale == 0){
			mVerticale++;
		}
		
		mBatterie = adc_read(PA3);
		if(mFlyMode)
		mLiftMemory = mVerticale;
		
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
		
		if(uart_is_tx_buffer_empty())
		{
			mOutputText[0] = '[';
			uint8_to_string(mOutputText + 1,mHorizontale);
			uint8_to_string(mOutputText + 4, mVerticale);
			if(brakeBoi){
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
			
			
			
			
			char output[3];
			uint8_to_string(output,getBatteryUsagePercentage(mBatterie,9,6));
			loopCount++;
			if(loopCount == 1){
				lcd_clear_display();
				lcd_write_string(mOutputText);
				lcd_write_string(" M:");
				lcd_write_string(output);
				lcd_write_string("%");
				if(adc_read(PA0) == 255){
					lcd_write_string("BRAKE BOI");
					brakeBoi = 1;
				}else{
					brakeBoi = 0;
				}
				
				
			}
			
			if(loopCount == 100){
				loopCount = 0;
			}
			
			if(loopCount == 1 ){
				uart_put_string(mOutputText);
				
			}
		
			
			
			
			
		}
	}
		
	
	
	
	
}
	
/* ================= MÉTHODES POUR LA MANETTE ================== */	

/************************************************************************/
/* Initialiser la télécommande                                          */
/************************************************************************/
void initializeRemote(){
	mHorizontale = 0;
	mVerticale = 0;
	mLiftMemory = 0;
	mButtonState = 0;
	mFlyMode = 0;
	mOutputText[16] = NULL;
	mBatterie = 0;
	loopCount = 0;
	brakeBoi = 0;
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

/* ================== MÉTHODES POUR L'AÉROGLISSEUR ================== */




/**************************************************************************/
/* Returns the maximum value between 0 and 255 that the battery can output*/
/**************************************************************************/
uint8_t getMaxBatteryValue(uint8_t maxTension){
	return  maxTension * 0.232558f / 3.3f * 255;
}

/************************************************************************/
/* Returns the battery tension                                          */
/************************************************************************/
uint8_t getBatteryUsagePercentage(uint8_t adcValue, uint8_t maxTension, uint8_t minTension){
	return (int)((((float)adcValue / (float)getMaxBatteryValue(maxTension) - ((float)minTension/(float)maxTension)) *100) /(100-(100*minTension/maxTension)) * 100) ;
}

uint8_t getRealBatteryTension(uint8_t adcValue, uint8_t maxTension){
	return  (int)((float)adcValue * 10 / (float)getMaxBatteryValue(maxTension) * maxTension ) ; // Pour plus de précision, il affiche la valeur fois 100
}

void DelBattery(uint8_t battValue)
{
	if(battValue <= 100 && battValue > 80){
		PORTB = 0b00011111;	
	}
	
	
	if(battValue <= 80 && battValue > 60){
		PORTB = 0b00011111;
	}
	
	if(battValue <= 60 && battValue > 40){
		PORTB = 0b00011111;
	}
	
	if(battValue <= 40 && battValue > 20){
		PORTB = 0b00011111;
	}
	
	if(battValue <= 20 && battValue >= 0){
		PORTB = 0b00011111;	
	}
	
}

