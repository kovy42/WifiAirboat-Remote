#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "driver.h"
#include "uart.h"
#include "fifo.h"
#include "lcd.h"

int main(void)
{
	lcd_init();
	adc_init();
	//pwm_init(1,1);
	//uint8_t valeurMoteur;
	//servo_init();
	//uint8_t valeurServo;
	
	uart_init();
	SREG = set_bit(SREG, 7);
	uart_set_baudrate(BAUDRATE_9600);
	
	//slide

		
	
	uint8_t horizontale = 0;
	uint8_t verticale = 0;	
	uint8_t lift_memory = 0;
	bool button_state = 0;
	bool mode = 0;
	DDRD = clear_bit(DDRD, PD3);
	PORTD = set_bit(PORTD, PD3);
	
	
	
	
	lcd_write_string("connecting....");
	OSCCAL = OSCCAL +4;
	DDRD = set_bit(DDRD, PD2);
	PORTD = clear_bit(PORTD,PD2);	
	_delay_ms(500);
	PORTD = set_bit(PORTD,PD2);
	_delay_ms(5000);
	
	uart_put_string("AT+CIPMODE=1\r\n\0");
	_delay_ms(250);
	uart_put_string("AT+CIPSTART=\"UDP\",\"192.168.4.1\",456,123\r\n\0");
	_delay_ms(250);	
	uart_put_string("AT+CIPSEND\r\n\0");
	 lcd_clear_display();
	lcd_write_string("connect :)");
	_delay_ms(500);	
	
	char text[16] = "";
	
	bool old_state = read_bit(PIND, PD3);
	
    while(1)
    {
		
		button_state = read_bit(PIND, PD3);
		if(button_state != old_state && button_state == 0)
		{
			mode = !mode;
			old_state = 0;						
		}
		else
		old_state = 1;
		
		/*
		
		if(button_state == 0 && _done == 0)
		{
			compteur++;
			_done = 1;
		}
		if(button_state == 1)
		{
			_done = 0;
		}*/
		
		horizontale = adc_read(PA1);				
		verticale = adc_read(PA2);		
		verticale = (255 - verticale);
		
		if(mode)
		lift_memory = verticale;
		
		if(!mode && lift_memory < 128)
		verticale = 0;				
				
				
		 lcd_clear_display();
		if(uart_is_tx_buffer_empty())
		{		
			text[0] = '[';	
			uint8_to_string(text + 1,horizontale);
			uint8_to_string(text + 4, verticale);
			if(mode)
			text[7] = 'A';
			else
			text[7] = 'B';
			
			text[8] = ']';
	        text[9] = '\0';
			
			
		uart_put_string(text);
		lcd_write_string(text);
		if(mode)
		lcd_write_string(" LIFT");
		else
		lcd_write_string(" SPEED");
		
		if(lift_memory < 128 && !mode)
		{
			lcd_set_cursor_position(0,1);
			lcd_write_string(" LIFT TO LOW");
		}
		
		
		_delay_ms(100);
		}
		/*		
       
		if(adc_read(PA0)< 128){
			valeurMoteur = 0;
		}else{
			valeurMoteur =(adc_read(PA0) - 128) * 2;	
		}
		valeurServo = adc_read(PA1);
		uint8_to_string(text,valeurMoteur); 
		pwm_set_a(valeurMoteur);
		pwm_set_b(valeurMoteur);
		servo_set_a(valeurServo);
		
		lcd_write_string(text);
		_delay_ms(100);
		*/
		
		
    }
}
