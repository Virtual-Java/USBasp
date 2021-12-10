/*
 * USBasp - USB in-circuit programmer for Atmel AVR controllers
 *
 * Thomas Fischl <tfischl@gmx.de>
 *
 * License........: GNU GPL v2 (see Readme.txt)
 * Target.........: ATMega8 at 12 MHz
 * Creation Date..: 2005-02-20
 * Last change....: 2009-02-28
 *
 * PC2 SCK speed option.
 * GND  -> slow (8khz SCK),
 * open -> software set speed (default is 375kHz SCK)
 */

#define AVR_SPI_SPEED_SEARCH

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include "usbasp.h"
#include "usbdrv/usbdrv.h"
#include "isp.h"
#include "clock.h"
#include "tpi.h"
#include "tpi_defs.h"
#include "uart.h"
#include "i2c.h"
#include "microwire.h"

static uchar replyBuffer[8];

static uchar prog_state = PROG_STATE_IDLE;
static uchar prog_sck = USBASP_ISP_SCK_AUTO;

static uchar prog_address_newmode = 0;
static unsigned long prog_address;
static unsigned int prog_nbytes = 0;
static unsigned int prog_pagesize;
static uchar prog_blockflags;
static uchar prog_pagecounter;

#include <util/delay.h>
static uchar spi_cs_hi = 1;
static uchar mw_cs_lo = 1;
static uchar mw_bitnum = 0;
static uchar i2c_stop_aw = 0;


usbMsgLen_t usbFunctionSetup(uchar data[8]) {

	usbMsgLen_t len = 0;
	
	if (data[1] == USBASP_FUNC_CONNECT) {
		uart_disable(); // make it not interfere.

		/* set SCK speed */
		if ((PINC & (1 << PC2)) == 0) {
			ispSetSCKOption(USBASP_ISP_SCK_8);
		} else {
			ispSetSCKOption(prog_sck);
		}

		/* set compatibility mode of address delivering */
		prog_address_newmode = 0;

		ledRedOn();
		ispConnect();

								
//spi --------------------------------------------------------------
	} else if (data[1] == USBASP_FUNC_SPI_CONNECT) {
		/* set SCK speed */
		if ((PINC & (1 << PC2)) == 0) {
			ispSetSCKOption(USBASP_ISP_SCK_8);
		} else {
			ispSetSCKOption(prog_sck);
		}

		ledRedOn();
		isp25Connect();
			
	} else if (data[1] == USBASP_FUNC_SPI_READ) {
		CS_LOW();
		spi_cs_hi = data[2]; //
		prog_nbytes = (data[7] << 8) | data[6]; //Длинна буфера данных
		prog_state = PROG_STATE_SPI_READ;
		len = USB_NO_MSG;
		
	} else if (data[1] == USBASP_FUNC_SPI_WRITE) {
		CS_LOW();
		spi_cs_hi = data[2]; //
		prog_nbytes = (data[7] << 8) | data[6]; //Длинна буфера данных
		prog_state = PROG_STATE_SPI_WRITE;
		len = USB_NO_MSG;

//i2c 24xx ---------------------------------------------------------			

	} else if (data[1] == USBASP_FUNC_I2C_INIT) {
	
		ledRedOn();
		i2c_init();
		
	} else if (data[1] == USBASP_FUNC_I2C_START) {
		i2c_start();
		
	} else if (data[1] == USBASP_FUNC_I2C_STOP) {
		i2c_stop();
		
	} else if (data[1] == USBASP_FUNC_I2C_WRITEBYTE) {
		replyBuffer[0] = i2c_send_byte(data[2]);
		len = 1;
		
	} else if (data[1] == USBASP_FUNC_I2C_READBYTE) {
		replyBuffer[0] = i2c_read_byte(data[2]);
		len = 1;
		
	} else if (data[1] == USBASP_FUNC_I2C_READ) {
		i2c_address(data[2], I2C_READ);
		prog_nbytes = (data[7] << 8) | data[6]; //Размер куска данных
		prog_state = PROG_STATE_I2C_READ;
		len = USB_NO_MSG;

	} else if (data[1] == USBASP_FUNC_I2C_WRITE) {
		i2c_start();
		i2c_address(data[2], I2C_WRITE); //Адрес устройства
		i2c_stop_aw = data[4]; //Команда стоп(1) или старт(0)
		prog_nbytes = (data[7] << 8) | data[6]; //Размер куска данных
		prog_state = PROG_STATE_I2C_WRITE;
		len = USB_NO_MSG;
		
//microwire 93xx ---------------------------------------------------------		

	} else if (data[1] == USBASP_FUNC_MW_WRITE) {
		CS_HI();
		mw_cs_lo = data[2];
		
		//data[4] lo(index)= сколько бит передавать
		mw_bitnum = data[4];
	
		prog_nbytes = (data[7] << 8) | data[6]; //Размер куска данных
		prog_state = PROG_STATE_MW_WRITE;
		len = USB_NO_MSG;
		
	} else if (data[1] == USBASP_FUNC_MW_READ) {
		mw_cs_lo = data[2];
		
		prog_nbytes = (data[7] << 8) | data[6]; //Размер куска данных
		prog_state = PROG_STATE_MW_READ;
		len = USB_NO_MSG;
	
			
	} else if (data[1] == USBASP_FUNC_MW_BUSY) {
		if (mwBusy() == 1) 
		{
			replyBuffer[0] = 1; //Линия занята
		}
		else
		{
			replyBuffer[0] = 0; 
		}
		
		len = 1;
		
//------------------------------------------------------------------------
	
	} else if (data[1] == USBASP_FUNC_DISCONNECT) {
		ispDisconnect();
		ledRedOff();

	} else if (data[1] == USBASP_FUNC_TRANSMIT) {
		replyBuffer[0] = ispTransmit(data[2]);
		replyBuffer[1] = ispTransmit(data[3]);
		replyBuffer[2] = ispTransmit(data[4]);
		replyBuffer[3] = ispTransmit(data[5]);
		len = 4;

	} else if (data[1] == USBASP_FUNC_READFLASH) {

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_READFLASH;
		len = USB_NO_MSG; /* multiple in */

	} else if (data[1] == USBASP_FUNC_READEEPROM) {

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_READEEPROM;
		len = USB_NO_MSG; /* multiple in */

	} else if (data[1] == USBASP_FUNC_ENABLEPROG) {
		replyBuffer[0] = ispEnterProgrammingMode();
		
		#ifdef AVR_SPI_SPEED_SEARCH 
		if (replyBuffer[0] != 0){ //target don't answer
			uint8_t i, speed;
			
			if (prog_sck == USBASP_ISP_SCK_AUTO)
				speed = USBASP_ISP_SCK_187_5;
			else
				speed = prog_sck-1;
			//trying lower speeds		
			for (i = speed; i >= USBASP_ISP_SCK_2; i--){
				ispSetSCKOption(i);
				replyBuffer[0] = ispEnterProgrammingMode();
				if (replyBuffer[0] == 0){
					ispSetSCKOption(i-1);
					break; 
				}	
			}
		}
		#endif
		len = 1;

	} else if (data[1] == USBASP_FUNC_WRITEFLASH) {

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_pagesize = data[4];
		prog_blockflags = data[5] & 0x0F;
		prog_pagesize += (((unsigned int) data[5] & 0xF0) << 4);
		if (prog_blockflags & PROG_BLOCKFLAG_FIRST) {
			prog_pagecounter = prog_pagesize;
		}
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_WRITEFLASH;
		len = USB_NO_MSG; /* multiple out */

	} else if (data[1] == USBASP_FUNC_WRITEEEPROM) {

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_pagesize = 0;
		prog_blockflags = 0;
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_WRITEEEPROM;
		len = USB_NO_MSG; /* multiple out */

	} else if (data[1] == USBASP_FUNC_SETLONGADDRESS) {

		/* set new mode of address delivering (ignore address delivered in commands) */
		prog_address_newmode = 1;
		/* set new address */
		prog_address = *((unsigned long*) &data[2]);

	} else if (data[1] == USBASP_FUNC_SETISPSCK) {

		/* set sck option */
		prog_sck = data[2];
		replyBuffer[0] = 0;
		len = 1;

	} else if (data[1] == USBASP_FUNC_TPI_CONNECT) {
		uart_disable(); // make it not interefere.
		tpi_dly_cnt = data[2] | (data[3] << 8);

		/* RST high */
		ISP_OUT |= (1 << ISP_RST);
		ISP_DDR |= (1 << ISP_RST);

		clockWait(3);

		/* RST low */
		ISP_OUT &= ~(1 << ISP_RST);
		ledRedOn();

		clockWait(16);
		tpi_init();
	
	} else if (data[1] == USBASP_FUNC_TPI_DISCONNECT) {

		tpi_send_byte(TPI_OP_SSTCS(TPISR));
		tpi_send_byte(0);

		clockWait(10);

		/* pulse RST */
		ISP_OUT |= (1 << ISP_RST);
		clockWait(5);
		ISP_OUT &= ~(1 << ISP_RST);
		clockWait(5);

		/* set all ISP pins inputs */
		ISP_DDR &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));
		/* switch pullups off */
		ISP_OUT &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));

		ledRedOff();
	
	} else if (data[1] == USBASP_FUNC_TPI_RAWREAD) {
		replyBuffer[0] = tpi_recv_byte();
		len = 1;
	
	} else if (data[1] == USBASP_FUNC_TPI_RAWWRITE) {
		tpi_send_byte(data[2]);
	
	} else if (data[1] == USBASP_FUNC_TPI_READBLOCK) {
		prog_address = (data[3] << 8) | data[2];
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_TPI_READ;
		len = USB_NO_MSG; /* multiple in */
	
	} else if (data[1] == USBASP_FUNC_TPI_WRITEBLOCK) {
		prog_address = (data[3] << 8) | data[2];
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_TPI_WRITE;
		len = USB_NO_MSG; /* multiple out */
	
	}	else if(data[1]==USBASP_FUNC_UART_CONFIG){
		uint16_t baud=(data[3]<<8)|data[2];
		uint8_t par  = data[4] & USBASP_UART_PARITY_MASK;
		uint8_t stop = data[4] & USBASP_UART_STOP_MASK;
		uint8_t bytes= data[4] & USBASP_UART_BYTES_MASK;
		uart_config(baud, par, stop, bytes);
	}
	else if(data[1]==USBASP_FUNC_UART_FLUSHTX){
		uart_flush_tx();
	}
	else if(data[1]==USBASP_FUNC_UART_FLUSHRX){
		uart_flush_rx();
	}
	else if(data[1]==USBASP_FUNC_UART_DISABLE){
		uart_disable();
	}
	else if(data[1]==USBASP_FUNC_UART_TX){
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_UART_TX;
		len = USB_NO_MSG; // multiple out
	}
	else if(data[1]==USBASP_FUNC_UART_RX){
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_UART_RX;
		len = USB_NO_MSG; // multiple in
	}
	else if(data[1]==USBASP_FUNC_UART_TX_FREE){
		uint16_t places=uart_tx_freeplaces();
		replyBuffer[0]=places>>8;
		replyBuffer[1]=places&0xFF;
		len=2;
	}
	else if (data[1] == USBASP_FUNC_GETCAPABILITIES) {
		replyBuffer[0] = USBASP_CAP_0_TPI|USBASP_CAP_6_UART; // TODO: check for issues
		replyBuffer[1] = 0;
		replyBuffer[2] = 0;
		replyBuffer[3] = USBASP_CAP_3_FLASH; // TODO: check for issues
		len = 4;
	}

	usbMsgPtr = replyBuffer;

	return len;
}

uchar usbFunctionRead(uchar *data, uchar len) {

	uchar i;

	/* check if programmer is in correct read state */
	if ((prog_state != PROG_STATE_READFLASH) && (prog_state
			!= PROG_STATE_READEEPROM) && (prog_state != PROG_STATE_TPI_READ) &&
			(prog_state != PROG_STATE_UART_RX) &&
			(prog_state != PROG_STATE_SPI_READ) && (prog_state != PROG_STATE_MW_READ) && (prog_state != PROG_STATE_I2C_READ)) {
		return 0xff;
	}
	
	// TODO: Improve to be consistent
	if(prog_state == PROG_STATE_UART_RX)
	{
		if(len > prog_nbytes) 
			len = prog_nbytes;
		for (i = 0; i < len; i++)
		{
			if(!uart_getc(data+i)){
				len = i; // Emptied whole buffer.
				break;
			}
		}
		if(len < 8)
		{
			prog_state = PROG_STATE_IDLE;
		}
		return len; // Whole data buffer written.
	}

	if(prog_state == PROG_STATE_SPI_READ)
	{
		if(len > prog_nbytes)
		len = prog_nbytes;
		
		for (i = 0; i < len; i++)
		{
			data[i] = ispTransmit(0);
			prog_nbytes -= 1;
		}
		
		if(prog_nbytes <= 0)
		{
			if (spi_cs_hi) CS_HI();
			prog_state = PROG_STATE_IDLE;
		}

		return len;
	}

	if(prog_state == PROG_STATE_I2C_READ)
	{
		if(len > prog_nbytes) 
			len = prog_nbytes;
		
		for (i = 0; i < len; i++)
		{
			prog_nbytes -= 1;
			
			if (prog_nbytes == 0)
			{
				data[i] = i2c_read_byte(I2C_NACK);
				i2c_stop();
				prog_state = PROG_STATE_IDLE;				
			}
			else
			{
				data[i] = i2c_read_byte(I2C_ACK);	
			}
		}
		
		return len;
	}
	
	if(prog_state == PROG_STATE_MW_READ)
	{
		if(len > prog_nbytes)
			len = prog_nbytes;
		
		for (i = 0; i < len; i++)
		{
			data[i] = mwReadByte();
			prog_nbytes -= 1;
		}
		
		if(prog_nbytes <= 0)
		{
			if(mw_cs_lo) mwEnd();
			prog_state = PROG_STATE_IDLE;
		}

		return len;
	}
	
	/* fill packet TPI mode */
	if(prog_state == PROG_STATE_TPI_READ)
	{
		tpi_read_block(prog_address, data, len);
		prog_address += len;
		return len;
	}

	/* fill packet ISP mode */
	for (i = 0; i < len; i++) {
		if (prog_state == PROG_STATE_READFLASH) {
			data[i] = ispReadFlash(prog_address);
		} else {
			data[i] = ispReadEEPROM(prog_address);
		}
		prog_address++;
	}

	/* last packet? */
	if (len < 8) {
		prog_state = PROG_STATE_IDLE;
	}

	return len;
}

uchar usbFunctionWrite(uchar *data, uchar len) {
	if(prog_state == PROG_STATE_UART_TX){
		if(len){
			uart_putsn(data, len);
			// This function should succeed, since computer should
			// request correct number of bytes. If request is bad,
			// return anything.
		}

		prog_nbytes -= len;
		if(prog_nbytes <= 0){
			prog_state = PROG_STATE_IDLE;
			return 1;
		}
		return 0;
	}

	uchar retVal = 0;
	uchar i;

	/* check if programmer is in correct write state */
	// Note: this is done after checking for UART_TX, because we want
	// super small delay here.
	if ((prog_state != PROG_STATE_WRITEFLASH) && (prog_state
			!= PROG_STATE_WRITEEEPROM) && (prog_state != PROG_STATE_TPI_WRITE) &&
				(prog_state != PROG_STATE_SPI_WRITE) && (prog_state != PROG_STATE_MW_WRITE) && (prog_state != PROG_STATE_I2C_WRITE)) {
		return 0xff;
	}

	if (prog_state == PROG_STATE_I2C_WRITE)
	{
		if(len > prog_nbytes)
		    len = prog_nbytes;

		for (i = 0; i < len; i++)
		{
			i2c_send_byte(data[i]);
			prog_nbytes -= 1;
		}
		
		if(prog_nbytes <= 0)
		{
			if(i2c_stop_aw == 1) i2c_stop();
			  else i2c_start();
			  
			prog_state = PROG_STATE_IDLE;
			return 1;
		}

		return 0;
	}

	if (prog_state == PROG_STATE_SPI_WRITE)
	{
		if(len > prog_nbytes)
		    len = prog_nbytes;

		for (i = 0; i < len; i++) 
		{
			ispTransmit(data[i]);
			prog_nbytes -= 1;
		}
		
		if(prog_nbytes <= 0)
		{
			if (spi_cs_hi) CS_HI();
			return 1;
		}

		return 0;
	}

	if (prog_state == PROG_STATE_MW_WRITE)
	{
		if(len > prog_nbytes)
		    len = prog_nbytes;

		for (i = 0; i < len; i++)
		{
			//Пишем биты
			if(mw_bitnum > 0){
				if(mw_bitnum < 8)
				{
					mwSendData(data[i], mw_bitnum);
					mw_bitnum = 0;
				}
				else
				{ 
					mwSendData(data[i], 8);
					mw_bitnum -= 8;	
				}
			}
				
			prog_nbytes -= 1;
		}
			
		if(prog_nbytes <= 0)
		{
			if(mw_cs_lo) mwEnd();
			prog_state = PROG_STATE_IDLE;
			return 1;
		}

		return 0;
	}
		
	if (prog_state == PROG_STATE_TPI_WRITE)
	{
		tpi_write_block(prog_address, data, len);
		prog_address += len;
		prog_nbytes -= len;
		if(prog_nbytes <= 0)
		{
			prog_state = PROG_STATE_IDLE;
			return 1;
		}
		return 0;
	}

	for (i = 0; i < len; i++) {

		if (prog_state == PROG_STATE_WRITEFLASH) {
			/* Flash */

			if (prog_pagesize == 0) {
				/* not paged */
				ispWriteFlash(prog_address, data[i], 1);
			} else {
				/* paged */
				ispWriteFlash(prog_address, data[i], 0);
				prog_pagecounter--;
				if (prog_pagecounter == 0) {
					ispFlushPage(prog_address, data[i]);
					prog_pagecounter = prog_pagesize;
				}
			}

		} else {
			/* EEPROM */
			ispWriteEEPROM(prog_address, data[i]);
		}

		prog_nbytes--;

		if (prog_nbytes == 0) {
			prog_state = PROG_STATE_IDLE;
			if ((prog_blockflags & PROG_BLOCKFLAG_LAST) && (prog_pagecounter
					!= prog_pagesize)) {

				/* last block and page flush pending, so flush it now */
				ispFlushPage(prog_address, data[i]);
			}

			retVal = 1; // Need to return 1 when no more data is to be received
		}

		prog_address++;
	}

	return retVal;
}


int main(void) {
	uchar i, j;

	/* no pullups on USB and ISP pins */
	PORTD = 0;
	PORTB = 0;
	
	/* all outputs except PD2 = INT0 */
	DDRD = ~(1 << 2);
	PORTD |= (1<<0); // pullup on Rx pin.
	DDRD &= ~(1<<0); // Rx as input too.[m

	/* output SE0 for USB reset */
	DDRB = ~0;
	j = 0;
	/* USB Reset by device only required on Watchdog Reset */
	while (--j) {
		i = 0;
		/* delay >10ms for USB reset */
		while (--i)
			;
	}
	/* all USB and ISP pins inputs */
	DDRB = 0;

	/* all inputs except PC0, PC1 */
	DDRC = 0x03;
	PORTC = 0xfe;

	/* init timer */
	clockInit();

	
	/*
	uart_config(155, 
			USBASP_UART_PARITY_NONE,
			USBASP_UART_STOP_1BIT,
			USBASP_UART_BYTES_8B);
	sei();
	while(1){
		_delay_ms(100);
		for(int i=0;i<10;i++){
			uart_putc('a');
		}
	}*/
	
	
	/* main event loop */
	usbInit();

/* not supported by usbasp-uart yet
#ifdef TEST_TARGET_AS_SPI_MASTER
	DDRB = (1 << PB2 | 1 << PB3 | 1 << PB5);
	PORTB &= (1 << PB2);
	SPCR = (1 << SPE | 1 << MSTR);
	sei();
	for (;;) {
		clockWait(30); // wait 9,6 ms
		toggleLedRed();
		SPDR = j++;
		clockWait(1); // wait 320 us
		SPDR = j++;
		if (j >= 'Z')
			j = 'A';
	}
#endif*/

	//ledGreenOn();[m
	sei();
	for (;;) {
		usbPoll();
/* disabled due to time-critical usb interface
		if (prog_state == PROG_STATE_SERIAL) {
			if (--blink_counter == 0) {
				toggleLedRed();
				blink_counter = (unsigned int)(1UL << 15);
			}
		}*/
	}
	return 0;
}

