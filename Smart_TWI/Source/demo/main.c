/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org

BSD New License
See LICENSE file for license details.
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "config.h"
#include "../library/atm_twis/twis.h"
#include "../library/smart/smart.h"

// Constants
// Device  description
const PROGMEM uint8_t  device_id[] = {"Uhr_V0.1"};   // Device ID
// Variables description
const PROGMEM s_SMART_CONFIG_t cfg_data[] =
{
	// Variables description
	{ 5,                                    // Record length
		(accWrite | objInt64),              // variable type + access
		0,                                  // Offset
		{"NOW"}},                           // Description
		{ 7, (accRead | objUInt16),  1, {"light"}},
		{ 9, (accWrite | accRead | objUInt8), 0x10, {"Address"}},
	};
	// Please don't change
	const PROGMEM uint8_t smart_config[] = {sizeof(device_id) - 1,
	sizeof(cfg_data)/sizeof(s_SMART_CONFIG_t)};

	static uint8_t line;
	static uint8_t buf[16];
	static uint8_t time_psec;
	static int16_t br_sum;
	static uint8_t br_cnt;
	static uint8_t flags;
	

	static uint8_t now[8];
	static void setSec(uint8_t s, uint8_t v);
	static void setDigit(uint8_t pos, uint8_t v);
	static void refresh();

	__attribute__((OS_main)) int main(void)
	{
		cli();
		wdt_reset();
		wdt_disable();
		DDRB =0b00000110; // nu, nu, nu, nu, nu, Load, ~OE, nu
		PORTB=0b11111011;

		DDRC =0b00000000; //na, na, SCL, SDA, nu, nu, ADC1, ADC0
		PORTC=0b00111110;

		DDRD =0b00010010; //nu, nu, nu, XSC, nu, nu, TXD, nu
		PORTD=0b00000101;

		OCR1A=1245;
		OCR1B=100;
		ICR1=1250;
		TCNT1=1020;
		TCCR1A=(3<<COM1A0) | (3<<COM1B0) | (2<<WGM10);  // Fast PWM, ICR1
		TCCR1B=(3<<WGM12) | (2<<CS10);					// CLK/8
		TIFR1=(1<<TOV1);
		TIMSK1=(1<<TOIE1);		// enable timer 1 overflow interrupt

		UCSR0C=(1<<UMSEL01) | (1<<UMSEL00); // SPI mode
		UCSR0B=(1<<TXEN0);
		UBRR0H=0;
		UBRR0L=16;

		ADMUX = (1<<REFS0) | (1<<REFS1) | 0;	// Ai0
		ADCSRA = (1<<ADEN) | (1<<ADIE) | (6<<ADPS0);  // 125KHz
		
		// Init RTC
		
		ASSR = (1<<AS2);
		TCCR2A = (1<<WGM21);
		TCNT2 = 0;
		OCR2A = 15;
		TIFR2 = (1<<OCF2A);
		TIMSK2 = (1<<OCIE2A);
		TCCR2B = (1<<WGM22) | (7 << CS20); // prescaling /1024
		while(ASSR & 0x1F);
		

		ADMUX = 0x0F;
		
		{
			uint8_t tmp;

			tmp = eeprom_read_byte((uint8_t *)EE_ADDR_I2CADDR);
			if(tmp == 0xFF)
			tmp = DEF_I2C_ADDR;

			InitTWI(tmp);
		}
		line=0;
		now[0]=0;
		now[1]=0;
		now[2]=0;
		flags=1;
		sei();

		while(1)
		{
			if((flags&1)==1){
				flags&=~1;
				refresh();
			}
			if(smart_status() == SM_STATUS_FREE)
			{
				if((flags&2)==2){
				  smart_set_reg(1);
				}
			}
		}
	}

	uint8_t GetUserDataLen(uint8_t reg)
	{
		if(reg == 1){
			return 2;
		}
		return 0;
	}

	uint8_t GetUserData(uint8_t reg, uint8_t offset)
	{
		if(reg == 1){
			flags&=~2;
			return (offset==0?OCR1BL:OCR1BH);
		}else if(reg == 0){
			return now[offset];
		}else if(reg == 0x10){
		  return TWI_addr();
		}

		return 0xFF;
	}

	uint8_t WriteUserData(uint8_t reg, uint8_t offset, uint8_t data)
	{
		if(reg == 0 && offset<8){
			now[offset]=data;
			if(offset<7){
			  return 0;
			}else{
			  flags|=1;
			}
		} else if(reg == 0x10){
			eeprom_write_byte((uint8_t *)EE_ADDR_I2CADDR, data);
		}
		return 1;
	}

	ISR(TIMER2_COMPA_vect){
		time_psec++;
		if(time_psec<2){
			if(time_psec==1){
				if(now[0]<59){
					setSec(now[0]+1, 1);
					}else{
					setSec(0, 1);
				}
				setSec(60, 0);
				setSec(61, 0);
			}
			return;
		}
		time_psec=0;
		setSec(now[0], 0);
		setSec(60, 1);
		setSec(61, 1);
		now[0]++;
		ADMUX = (1<<REFS0) | (1<<REFS1) | 0;	// Ai0
		ADCSRA = (1<<ADEN) | (1<<ADIE) | (6<<ADPS0);  // 125KHz
		ADCSRA |= (1<<ADSC);
		if(now[0]>59)	{
			now[0]=0;
			now[1]++;
			if( (now[1] & 0x0F)>9){
				now[1]=(now[1]+0x10) & 0xF0;
				if(now[1]>0x50){
					now[1]=0;
					now[2]++;
					if((now[2] & 0x0F)>9){
						now[2]=(now[2] & 0x30)+0x10;
					}
					if(now[2]>0x23){
						now[2]=0;
					}
					setDigit(2, now[2] & 0x0F);
					if(now[2]<10){
						setDigit(3, 10);
					}else{
						setDigit(3, now[2]>>4);
					}
				}
				setDigit(1, now[1]>>4);
			}
			setDigit(0, now[1] & 0x0F);
		}
	}
	ISR(TIMER1_OVF_vect){
		line++;
		if(line>7){
			line=0;
		}
		UDR0=buf[line+8];
		UDR0=buf[line];
	}
	ISR(ADC_vect){
		int16_t val = ADC;
		ADMUX=0x0F;
		ADCSRA=0;
		if(val<1020){
			br_cnt++;
			br_sum+=val;
			if(br_cnt>7){
				val=(br_sum>>2)+OCR1B*3;
				br_sum=0;
				br_cnt=0;
				val=val<7?1:(val/4);
				if(val>1240){
					val=1240;
				}
				if(val!=OCR1B){
					flags|=2;
				  OCR1B=val;
				}
			}
		}
	}	
	static void refresh(){
		uint8_t tmp;
		for(tmp=0;tmp<8;tmp++){
			buf[tmp]=0x0;
			buf[tmp+8]=((tmp+1)<<4) ^ 0xF0;
		}
		setDigit(0, now[1] & 0x0F);
		setDigit(1, now[1]>>4);
		setDigit(2, now[2] & 0x0F);
		if(now[2]<10){
			setDigit(3, 10);
			}else{
			setDigit(3, now[2]>>4);
		}
	}
	static void setSec(uint8_t s, uint8_t v){
		uint8_t idx=s>>3;
		uint8_t mask;
		if((idx & 1)==1){
			mask=(0x80>>(s & 0x07));
		}else{
			mask=(1<<(s & 0x07));
		}
		if(v==0){
			buf[idx]&=~mask;
		}else{
			buf[idx]|=mask;
		}
	}
	static uint8_t font[11]={0xEE, 0x24, 0xBA, 0xB6, 0x74, 0xD6, 0xDE, 0xA4, 0xFE, 0xF6, 0x00};
	static void setDigit(uint8_t pos, uint8_t v){
		if(v>10 || pos>3){  // 10 - empty
			return;
		}
		pos=pos*2+8;
		buf[pos]=(buf[pos] & 0xF0) | (font[v] & 0x0F);
		pos++;
		buf[pos]=(buf[pos] & 0xF0) | ((font[v]>>4) & 0x0F);
	}
