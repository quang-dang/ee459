#define PIN_MOSI                  PORTB3 //DIN //PURPLE WIRE TESTING
#define PIN_SS                    PORTB2 //CS //YELLOW WIRE TESTING
#define PIN_SCK                   PORTB5 //CLK //WHITE WIRE TESTING

#define ON                        1
#define OFF                       0

#define MAX7219_LOAD1             PORTB |= (1 << PIN_SS)
#define MAX7219_LOAD0             PORTB &= ~(1 << PIN_SS)

#define MAX7219_MODE_DECODE       0x09
#define MAX7219_MODE_INTENSITY    0x0A
#define MAX7219_MODE_SCAN_LIMIT   0x0B
#define MAX7219_MODE_POWER        0x0C
#define MAX7219_MODE_TEST         0x0F
#define MAX7219_MODE_NOOP         0x00

#define SEVENSEG_DIGIT_HOUR1      0x08
#define SEVENSEG_DIGIT_HOUR2      0x07
#define SEVENSEG_DIGIT_MIN1       0x06
#define SEVENSEG_DIGIT_MIN2       0x05
#define SEVENSEG_DIGIT_TEMP_NEG   0x04
#define SEVENSEG_DIGIT_TEMP1      0x03
#define SEVENSEG_DIGIT_TEMP2      0x02
#define SEVENSEG_DIGIT_TEMP_F     0x01

#define SEVENSEG_CHAR_NEGATIVE    0xA
#define SEVENSEG_CHAR_E           0xB
#define SEVENSEG_CHAR_H           0xC
#define SEVENSEG_CHAR_L           0xD
#define SEVENSEG_CHAR_P           0xE
#define SEVENSEG_CHAR_BLANK       0xF

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>


void spi_init();

void spi_sendByte(char databyte);

void max7219_writeData(char data_register, char data);

void max7219_clearDisplay();

void sevenSeg_init();

void sevenSeg_display();


