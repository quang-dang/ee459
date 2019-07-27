
#include "seven_seg.h"


void spi_init(){
    //From SPI, set MOSI and SCK output, all others input
    DDRB |= (1 << PIN_SCK) | (1 << PIN_MOSI) | (1 << PIN_SS);
    //Enable SPI, Master, set clock rate fck/16 ~ 10MHtz
    SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    
}

void spi_sendByte(char databyte){
    //Start transmission
    SPDR = databyte; //equals data
    //Wait for transmission complete
    while (!(SPSR & (1 << SPIF)));
}

void max7219_writeData(char data_register, char data){
    MAX7219_LOAD0;
    // Send the register where the data will be stored
    spi_sendByte(data_register);
    // Send the data to be stored
    spi_sendByte(data);
    MAX7219_LOAD1;
}

void max7219_clearDisplay(){
    char i = 8;
    // Loop until 0, but don't run for zero
    do {
        // Set each display in use to blank
        max7219_writeData(i, SEVENSEG_CHAR_BLANK);
    } while (--i);
}

void sevenSeg_init(){
    spi_init();
    
    // Decode mode to "Font Code-B"
    max7219_writeData(MAX7219_MODE_DECODE, 0xFF);
    
    // Scan limit runs from 0.
    max7219_writeData(MAX7219_MODE_SCAN_LIMIT, 7);
    max7219_writeData(MAX7219_MODE_INTENSITY, 8);
    max7219_writeData(MAX7219_MODE_POWER, ON);
    
    max7219_clearDisplay();
}

void sevenSeg_display(){
    max7219_writeData(SEVENSEG_DIGIT_HOUR1, 1);
    max7219_writeData(SEVENSEG_DIGIT_HOUR2, 2 | 0xF0); // add dot
    max7219_writeData(SEVENSEG_DIGIT_MIN1, 3);
    max7219_writeData(SEVENSEG_DIGIT_MIN2, 5);
    max7219_writeData(SEVENSEG_DIGIT_TEMP_NEG, SEVENSEG_CHAR_NEGATIVE);
    max7219_writeData(SEVENSEG_DIGIT_TEMP1, 7);
    max7219_writeData(SEVENSEG_DIGIT_TEMP2, 1);
    max7219_writeData(SEVENSEG_DIGIT_TEMP_F, SEVENSEG_CHAR_E);
    
}


