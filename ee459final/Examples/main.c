//initialize the USART port
#include <avr/io.h>
#include <util/delay.h>
#include <string.h> 
#include "light_ws2812.h"
#include <stdint.h>
#include <stdio.h>
#include "seven_seg.h"

//Global variables
struct cRGB led[1];
float calculate_led[3];
int color_mode;
int sensor_green, sensor_red, sensor_blue;


void i2c_init(uint8_t);
uint8_t i2c_io(uint8_t, uint8_t *, uint16_t,
               uint8_t *, uint16_t, uint8_t *, uint16_t);

// Find divisors for the UART0 and I2C baud rates
#define FOSC 9830400            // Clock frequency = Oscillator freq.
#define BAUD 9600               // UART0 baud rate
#define MYUBRR FOSC/16/BAUD-1   // Value for UBRR0 register
#define BDIV (FOSC / 100000 - 16) / 2 + 1    // Puts I2C rate just below 100kHz

#define PRESET_SUN_R 1
#define PRESET_SUN_G 1
#define PRESET_SUN_B 0.85

#define PRESET_INDOOR_R 1
#define PRESET_INDOOR_G 0.83
#define PRESET_INDOOR_B 0.66

#define PRESET_NIGHT_R 0.78
#define PRESET_NIGHT_G 0.88
#define PRESET_NIGHT_B 1




/*
 i2c_init - Initialize the I2C port
 */
void i2c_init(uint8_t bdiv)
{
    TWSR = 0;                           // Set prescalar for 1
    TWBR = bdiv;                        // Set bit rate register
}


uint8_t i2c_io(uint8_t device_addr, uint8_t *ap, uint16_t an,
               uint8_t *wp, uint16_t wn, uint8_t *rp, uint16_t rn)
{
    uint8_t status, send_stop, wrote, start_stat;
    
    status = 0;
    wrote = 0;
    send_stop = 0;
    
    if (an > 0 || wn > 0) {
        wrote = 1;
        send_stop = 1;
        
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);  // Send start condition
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != 0x08)                 // Check that START was sent OK
            return(status);
        
        TWDR = device_addr & 0xfe;          // Load device address and R/W = 0;
        TWCR = (1 << TWINT) | (1 << TWEN);  // Start transmission
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != 0x18) {               // Check that SLA+W was sent OK
            if (status == 0x20)             // Check for NAK
                goto nakstop;               // Send STOP condition
            return(status);                 // Otherwise just return the status
        }
        
        // Write "an" data bytes to the slave device
        while (an-- > 0) {
            TWDR = *ap++;                   // Put next data byte in TWDR
            TWCR = (1 << TWINT) | (1 << TWEN); // Start transmission
            while (!(TWCR & (1 << TWINT))); // Wait for TWINT to be set
            status = TWSR & 0xf8;
            if (status != 0x28) {           // Check that data was sent OK
                if (status == 0x30)         // Check for NAK
                    goto nakstop;           // Send STOP condition
                return(status);             // Otherwise just return the status
            }
        }
        
        // Write "wn" data bytes to the slave device
        while (wn-- > 0) {
            TWDR = *wp++;                   // Put next data byte in TWDR
            TWCR = (1 << TWINT) | (1 << TWEN); // Start transmission
            while (!(TWCR & (1 << TWINT))); // Wait for TWINT to be set
            status = TWSR & 0xf8;
            if (status != 0x28) {           // Check that data was sent OK
                if (status == 0x30)         // Check for NAK
                    goto nakstop;           // Send STOP condition
                return(status);             // Otherwise just return the status
            }
        }
        
        status = 0;                         // Set status value to successful
    }
    
    if (rn > 0) {
        send_stop = 1;
        
        // Set the status value to check for depending on whether this is a
        // START or repeated START
        start_stat = (wrote) ? 0x10 : 0x08;
        
        // Put TWI into Master Receive mode by sending a START, which could
        // be a repeated START condition if we just finished writing.
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
        // Send start (or repeated start) condition
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != start_stat)           // Check that START or repeated START sent OK
            return(status);
        
        TWDR = device_addr  | 0x01;         // Load device address and R/W = 1;
        TWCR = (1 << TWINT) | (1 << TWEN);  // Send address+r
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != 0x40) {               // Check that SLA+R was sent OK
            if (status == 0x48)             // Check for NAK
                goto nakstop;
            return(status);
        }
        
        // Read all but the last of n bytes from the slave device in this loop
        rn--;
        while (rn-- > 0) {
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // Read byte and send ACK
            while (!(TWCR & (1 << TWINT))); // Wait for TWINT to be set
            status = TWSR & 0xf8;
            if (status != 0x50)             // Check that data received OK
                return(status);
            *rp++ = TWDR;                   // Read the data
        }
        
        // Read the last byte
        TWCR = (1 << TWINT) | (1 << TWEN);  // Read last byte with NOT ACK sent
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != 0x58)                 // Check that data received OK
            return(status);
        *rp++ = TWDR;                       // Read the data
        
        status = 0;                         // Set status value to successful
    }
    
nakstop:                                    // Come here to send STOP after a NAK
    if (send_stop)
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);  // Send STOP condition
    
    return(status);
}



// ls/dev/tty.usb*
// screen/dev/tty.usberial-A9015MC8 9600 
void lcd_init(unsigned short ubrr)	{
    /* calculate from clock freq and buad see handout
     UBRR = [f_osc / (16*BUAD)] - 1
     UBRR = 47
     */
    DDRC |= 1 << DDC0;
	UBRR0 = ubrr;
	UCSR0B |= (1 << TXEN0);
	UCSR0B |= (1 << RXEN0);
	UCSR0C = (3 << UCSZ00);
    lcd_out(0xFE);
    lcd_out(128);
    lcd_outputString("                                   ");
    _delay_ms(20);
    lcd_out(0xFE);
    lcd_out(128);
    lcd_outputString("Sun | Indoor | Night ");
    lcd_out(0b11111111);
}
//output a byte to the USART0 port
void lcd_out(char ch)	{
	while ((UCSR0A & (1 << UDRE0)) == 0);
	UDR0 = ch;
}

void lcd_outputString (char* in)
{
	char i = 0; 
	for (i = 0; i < strlen(in) ; i ++)
	{
		lcd_out(in[i]);
	}

}
void buttons_init(){
    DDRD &= ~(1<<PD4);
    //PORTD |= (1<<DD4);
    DDRD &= ~(1<<PD5);
    //PORTD |= (1<<DD5);
    DDRD &= ~(1<<PD6);
    //PORTD |= (1<<DD6);
    DDRD &= ~(1<<PD7);
    //PORTD |= (1<<DD7);
    DDRB &= ~(1<<PB0);
    //PORTB |= (1<<PB0);
}
void lcd_update_option(int mode){
    lcd_out(0xFE);
    lcd_out(192);
    lcd_outputString("               ");
    switch(mode){
        case 1:
            lcd_out(0xFE);
            lcd_out(193);
            lcd_out(0b11111111);
            break;
        case 2:
            lcd_out(0xFE);
            lcd_out(199);
            lcd_out(0b11111111);
            break;
        case 3:
            lcd_out(0xFE);
            lcd_out(205);
            lcd_out(0b11111111);
            break;
    }
}

int read_buttons(){
    if(PIND & (1<<PD4)){
        return 1;
    }
    else if(PIND & (1<<PD5)){
        return 2;
    }
    else if(PIND & (1<<PD6)){
        return 3;
    }
    else if(PIND & (1<<PD7)){
        return 4;
    }
    else if(PINB & (1<<PB0)){
        return 5;
    }
    else{
        return 0;
    }

}
void rgb_sensor_init(){
    unsigned char status;
    unsigned char bufConfig1[3] = {0x01, 0x1d, 0xbf}; //10000 lux, 12 bits resolution of ADC
    //IR compensation
    //set three color interrupts
    status = i2c_io(0x88, NULL, 0, bufConfig1, 3, NULL, 0);

}

void rgb_read_color(int *red, int *green, int *blue){
    unsigned char storeBuff;
    unsigned char status;
    int temp_low, temp_high;
    unsigned char buf1 = 0x09; 
    status = i2c_io(0x88, NULL, 0, &buf1, 1, &storeBuff, 1);
    temp_low = storeBuff;
    buf1 = 0x0a;
    status = i2c_io(0x88, NULL, 0, &buf1, 1, &storeBuff, 1);
    temp_high = storeBuff;
    *green = temp_high*256 + temp_low;
    buf1 = 0x0b;
    status = i2c_io(0x88, NULL, 0, &buf1, 1, &storeBuff, 1);
    temp_low = storeBuff;
    buf1 = 0x0c;
    status = i2c_io(0x88, NULL, 0, &buf1, 1, &storeBuff, 1);
    temp_high = storeBuff;
    *red = temp_high*256 + temp_low;
    buf1 = 0x0d;
    status = i2c_io(0x88, NULL, 0, &buf1, 1, &storeBuff, 1);
    temp_low = storeBuff;
    buf1 = 0x0e;
    status = i2c_io(0x88, NULL, 0, &buf1, 1, &storeBuff, 1);
    temp_high = storeBuff;
    *blue = temp_high*256 + temp_low;
}

void calculate_led_light(float *led, int sensor_red, int sensor_green, int sensor_blue){
    float temp_red = sensor_red/500.0;
    float temp_green = sensor_green/500.0;
    float temp_blue = sensor_blue/500.0;
    if(color_mode == 1){
        if(temp_red <= PRESET_SUN_R){
            led[0] = PRESET_SUN_R-temp_red;
        }else{
            led[0] = 0;
        }
        if(temp_green <= PRESET_SUN_G){
            led[1] = PRESET_SUN_G - temp_green;
        }else{
            led[1]= 0;
        }
        if(temp_blue <= PRESET_SUN_B){
            led[2] = PRESET_SUN_B - temp_blue;
        }else{
            led[2]= 0;
        }
    }
    else if(color_mode == 2){
        if(temp_red <= PRESET_INDOOR_R){
            led[0] = PRESET_INDOOR_R-temp_red;
        }else{
            led[0]= 0;
        }
        if(temp_green <= PRESET_INDOOR_G){
            led[1]= PRESET_INDOOR_G - temp_green;
        }else{
            led[1] = 0;
        }
        if(temp_blue <= PRESET_INDOOR_B){
            led[2] = PRESET_INDOOR_B - temp_blue;
        }else{
            led[2] = 0;
        }
    }else{
        if(temp_red <= PRESET_NIGHT_R){
            led[0]= PRESET_NIGHT_R-temp_red;
        }else{
            led[0] = 0;
        }
        if(temp_green <= PRESET_NIGHT_G){
            led[1] = PRESET_NIGHT_G - temp_green;
        }else{
            led[1] = 0;
        }
        if(temp_blue <= PRESET_NIGHT_B){
            led[2] = PRESET_NIGHT_B - temp_blue;
        }else{
            led[2] = 0;
        }
    }
}
int main ()	{
    color_mode = 1;
    int brightness = 50;

    int prev_button = 0;
    
    TWSR = 0;                           // Set prescalar for 1
    TWBR = BDIV;
    _delay_ms(1000);                        // Set bit rate register
    i2c_init(BDIV);
    lcd_init(47);
    buttons_init();
    rgb_sensor_init();
    sevenSeg_init();

    while(1){
        sevenSeg_display();
        int current_button = read_buttons();
//        char temp_demo = '0'+current_button;
//        lcd_out(temp_demo);
        if(current_button!= 0 && current_button!=prev_button){
            switch(current_button){
                case 1:
                    color_mode = 1;
                    lcd_update_option(1);
                    break;
                case 2:
                    color_mode = 2;
                    lcd_update_option(2);
                    break;
                case 3:
                    color_mode = 3;
                    lcd_update_option(3);
                    break;
                case 4:
                    if(brightness<=95){
                        brightness-=5;
                    }
                    break;
                case 5:
                    if(brightness>=5){
                        brightness+=5;
                    }
                    break;
            }
        }
        prev_button = current_button;
    //     // //Button logic ends here
        rgb_read_color(&sensor_red,&sensor_green,&sensor_blue);
        calculate_led_light(calculate_led,sensor_red,sensor_green,sensor_blue);
        led[0].r = calculate_led[0]*brightness;
        led[0].g = calculate_led[1]*brightness;
        led[0].b = calculate_led[2]*brightness;
        ws2812_sendsinglearray(led,3);
    }
	return 0;
}
