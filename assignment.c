#include <stdint.h>
#include <stdio.h>
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

//Defining functions for UART
void uart_setup(void);
void uart_put_byte(unsigned char byte_val);
void uart_printf(const char * fmt, ...);

//Defining arduino constants
#define BAUD (9600)
#define MYUBRR (F_CPU/16/BAUD-1)
#define Freq (16000000)
#define Prescale (256)

//Defining useful functions for bit manipulation
#define SET_BIT(reg, pin)		    (reg) |= (1 << (pin))
#define CLEAR_BIT(reg, pin)		  (reg) &= ~(1 << (pin))
#define WRITE_BIT(reg, pin, value)   (reg) = (((reg) & ~(1 << (pin))) | ((value) << (pin)))
#define BIT_VALUE(reg, pin)		  (((reg) >> (pin)) & 1)
#define BIT_IS_SET(reg, pin)	     (BIT_VALUE((reg),(pin))==1) 

//Defining the LCD to be in 4-pin mode
#define LCD_USING_4PIN_MODE (1)

//Setting up all of the data registers to be in DDRD
#define LCD_DATA4_DDR (DDRD)
#define LCD_DATA5_DDR (DDRD)
#define LCD_DATA6_DDR (DDRD)
#define LCD_DATA7_DDR (DDRD)

//Defining the LCD ports
#define LCD_DATA4_PORT (PORTD)
#define LCD_DATA5_PORT (PORTD)
#define LCD_DATA6_PORT (PORTD)
#define LCD_DATA7_PORT (PORTD)

//Defining the pin that each data pin is connected to in the
//arduino
#define LCD_DATA4_PIN (3)
#define LCD_DATA5_PIN (4)
#define LCD_DATA6_PIN (5)
#define LCD_DATA7_PIN (7)

#define LCD_RS_DDR (DDRB)
#define LCD_ENABLE_DDR (DDRB)

#define LCD_RS_PORT (PORTB)
#define LCD_ENABLE_PORT (PORTB)

#define LCD_RS_PIN (1)
#define LCD_ENABLE_PIN (0)

//Commands for the LCD
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

//Flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

//Flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

//Flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

//Flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

//Defining functions that will be used for the LCD
void lcd_init(void);
void lcd_write_string(uint8_t x, uint8_t y, char string[]);
void lcd_write_char(uint8_t x, uint8_t y, char val);
void lcd_clear(void);
void lcd_home(void);
void lcd_createChar(uint8_t, uint8_t[]);
void lcd_setCursor(uint8_t, uint8_t); 
void lcd_noDisplay(void);
void lcd_display(void);
void lcd_noBlink(void);
void lcd_blink(void);
void lcd_noCursor(void);
void lcd_cursor(void);
void lcd_leftToRight(void);
void lcd_rightToLeft(void);
void lcd_autoscroll(void);
void lcd_noAutoscroll(void);
void scrollDisplayLeft(void);
void scrollDisplayRight(void);
size_t lcd_write(uint8_t);
void lcd_command(uint8_t);
void lcd_send(uint8_t, uint8_t);
void lcd_write4bits(uint8_t);
void lcd_write8bits(uint8_t);
void lcd_pulseEnable(void);
uint8_t _lcd_displayfunction;
uint8_t _lcd_displaycontrol;
uint8_t _lcd_displaymode;

//Generating bitmaps for the LCD
uint8_t bmp0[8] = { 0b00010010, 
                    0b00001010, 
                    0b00001001, 
                    0b00000100, 
                    0b00000100, 
                    0b00010010, 
                    0b00001010, 
                    0b00001001};
uint8_t bmp1[8] = { 0b00010101, 
                    0b00010101, 
                    0b00010101, 
                    0b00010101, 
                    0b00010101, 
                    0b00010101, 
                    0b00010101, 
                    0b00010101};
uint8_t bmp2[8] = { 0b00001001, 
                    0b00001010, 
                    0b00010010, 
                    0b00000100, 
                    0b00000100, 
                    0b00001001, 
                    0b00001010, 
                    0b00010010};
uint8_t bmp3[8] = { 0b00000000, 
                    0b00011111, 
                    0b00000000, 
                    0b00011111, 
                    0b00000000, 
                    0b00011111, 
                    0b00000000, 
                    0b00011111};

void setup_lcd(void);
void loop(void);




void setup_time(void) {
    //  Initialise Timer 0 in normal mode so that it overflows 
    //  with a period of approximately 0.004 seconds.
	UBRR0 = (F_CPU/8/BAUD) - 1;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);
	UBRR0H = (unsigned char)(UBRR0>>8);
    UBRR0L = (unsigned char)(UBRR0);
		
	TCCR0A = 0;
	TCCR0B = 4;

    //Enable timer overflow interrupt for Timer 0.
	TIMSK0 = 1;

    //Turning on interrupts
	sei();

    //Enable the I/O pin labelled A2 for digital input.
	CLEAR_BIT(DDRC, 2);

    //Debugging message for UART (student number and prescaler)
	uart_printf("n10469265,256\r\n");
  	
  	
  	//Setup LED 1
  	SET_BIT(DDRB, 3); //Corresponds to pin 11
}

//Volatile unsigned 8-bit int for the bit count
volatile uint8_t bit_count = 0;

//Volatile unsigned 8-bit int for the current switch state
volatile uint8_t switch_closed = 0;

//Entering code for timer 0 to get the simulation time
volatile int counter = 0;

//Interrupt service to process timer0 overflow
	ISR(TIMER0_OVF_vect) {
		uint8_t mask = 0b00111111;
		
		bit_count = ((bit_count << 1) & mask) | BIT_IS_SET(PINC,2);
		
		if (bit_count == 0) switch_closed = 0;
		else if (bit_count == mask) switch_closed = 1;
        
        //Incrimenting the counter by one so we can get the time
        counter++;
	}

char buffer[100];

//Setting up the UART
void uart_setup(void) {
#define BAUD (9600)
#define UBRR (F_CPU / 16 / BAUD - 1)
    UBRR0H = UBRR >> 8;
    UBRR0L = UBRR & 0b11111111;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (3 << UCSZ00);
}

//Setting up a function to print to the serial monitor
void uart_printf(const char * fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    for (int i = 0; buffer[i]; i++) {
        uart_put_byte(buffer[i]);
    }
}

#ifndef __AMS__
void uart_put_byte(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0))) { /* Wait */ }
    UDR0 = data;
}
#endif

//Setting up the PWM
void setup_pwm(uint16_t division_factor) {
    //Configure the digital I/O pin corresponding to OCR0A for output. 
    //Modify at most one pin in the DDR.
	OCR0A = 0b0000000;

    //Update the value of TCCR0A so that register OC0A will clear on compare 
    //match. 
	TCCR0A |= (1<<COM0A1);

    //Update TCCR0B to disable Force Output Compare functionality.
	TCCR0B |= (1<<FOC0B);

    //Update TCCR0B to ensure that the clock pre-scaler matches the 
    //designated division factor.
	if(division_factor == 1){
		TCCR0B = 0b00000001;
	}
	else if(division_factor == 8){
		TCCR0B = 0b00000010;
	}
	else if(division_factor == 16){
	TCCR0B = 0b00000000;
	}
	else if(division_factor == 64){
		TCCR0B = 0b00000011;
	}
	else if(division_factor == 256){
		TCCR0B = 0b00000100;
	}
	else if(division_factor == 1024){
		TCCR0B = 0b00000101;
	}
    //Update the values of TCCR0A and TCCR0B so that the Waveform Generation 
    //Mode corresponds to Fast PWM with a Top value equal to 255.
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
}


void write_pwm(uint8_t duration) {
    OCR0A = duration; 	 	
}

uint16_t adc_read(uint8_t channel) {
    ADMUX |= channel;

    //start single conversion
    //write '1' to ADSC
    ADCSRA |= (1 << ADSC);

    // wait for conversion to complete
    // ADSC becomes '0' again
    // till then, run loop continuously
#if !__AMS__
    while (ADCSRA & (1 << ADSC));
#endif

    return (ADC);
}

//Setting up the analogue to digital converter
void adc_setup(uint8_t division_factor) {
	ADMUX = 0b01000000;
	if (division_factor == 4){
		SET_BIT(ADCSRA, 7);
		CLEAR_BIT(ADCSRA, 6);
		CLEAR_BIT(ADCSRA, 5);
		CLEAR_BIT(ADCSRA, 4);
		CLEAR_BIT(ADCSRA, 3);
		CLEAR_BIT(ADCSRA, 2);
		SET_BIT(ADCSRA, 1);
		CLEAR_BIT(ADCSRA, 0);
	}
	else if(division_factor == 8){
		SET_BIT(ADCSRA, 7);
		CLEAR_BIT(ADCSRA, 6);
		CLEAR_BIT(ADCSRA, 5);
		CLEAR_BIT(ADCSRA, 4);
		CLEAR_BIT(ADCSRA, 3);
		CLEAR_BIT(ADCSRA, 2);
		SET_BIT(ADCSRA, 1);
		SET_BIT(ADCSRA, 0);
	}
	else if(division_factor == 16){
		SET_BIT(ADCSRA, 7);
		CLEAR_BIT(ADCSRA, 6);
		CLEAR_BIT(ADCSRA, 5);
		CLEAR_BIT(ADCSRA, 4);
		CLEAR_BIT(ADCSRA, 3);
		SET_BIT(ADCSRA, 2);
		CLEAR_BIT(ADCSRA, 1);
		CLEAR_BIT(ADCSRA, 0);
	}
	else if(division_factor == 32){
		SET_BIT(ADCSRA, 7);
		CLEAR_BIT(ADCSRA, 6);
		CLEAR_BIT(ADCSRA, 5);
		CLEAR_BIT(ADCSRA, 4);
		CLEAR_BIT(ADCSRA, 3);
		SET_BIT(ADCSRA, 2);
		CLEAR_BIT(ADCSRA, 1);
		SET_BIT(ADCSRA, 0);
	}
	else if(division_factor == 64){
		SET_BIT(ADCSRA, 7);
		CLEAR_BIT(ADCSRA, 6);
		CLEAR_BIT(ADCSRA, 5);
		CLEAR_BIT(ADCSRA, 4);
		CLEAR_BIT(ADCSRA, 3);
		SET_BIT(ADCSRA, 2);
		SET_BIT(ADCSRA, 1);
		CLEAR_BIT(ADCSRA, 0);
	}
	else if(division_factor == 128){
		SET_BIT(ADCSRA, 7);
		CLEAR_BIT(ADCSRA, 6);
		CLEAR_BIT(ADCSRA, 5);
		CLEAR_BIT(ADCSRA, 4);
		CLEAR_BIT(ADCSRA, 3);
		SET_BIT(ADCSRA, 2);
		SET_BIT(ADCSRA, 1);
		SET_BIT(ADCSRA, 0);
	}
	
}

//This buffer may be any size from 2 to 256 bytes.
#define  TX_BUFFER_SIZE  64

//uart definitions
static volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
static volatile uint8_t tx_buffer_head;
static volatile uint8_t tx_buffer_tail;

void pwm_process(void);
//uart functions
void uart_putchar(uint8_t c);
void uart_putstring(char* s);

//Defining a global volatile 8-bit int to hold the value of the
//potentiometer
volatile uint8_t pot;

uint8_t prevState = 0;

void pwm_process(void) {
    char temp_buf[64];
	
  	//Dividing the value from adc_read by 4 to give an 8-bit
  	//value (0 to 255) instead of a 10-bit value
    pot = adc_read(0)/4;

    // convert uint16_t to string
    snprintf(temp_buf, sizeof(temp_buf), "%d", pot);

    //send serial data
    uart_putstring(temp_buf);
    uart_putstring("\r\n");
  	
  	//  Desired clock scale factor for timer0
    uint16_t divider = 256;

    //  Duration: number of timer ticks required for 25% duty cycle.
    volatile uint16_t duration = pot;
      	
  	setup_pwm(divider);
    write_pwm(duration);
}



//Function to get the elapsed time
double elapsed_time(void){
	double time = (counter * 65536.0 + TCNT0) * 1 / Freq;
	return time;
}

volatile int time_now;

int main() {
  	//Calling the function to setup UART
    uart_setup();
  	//Calling the function to setup Timer0
    setup_time();
  	//Calling the function to setup the ADC
  	adc_setup(4);  

    for (;;) {
      	//Calling the function to process the PWM
      	pwm_process();
      	//Setting the time_now to elapsed_time to be able to
      	//read the simulation time
      	time_now = elapsed_time();
      	_delay_ms(50);
      	setup_lcd();
        if (switch_closed != prevState) {
          	prevState = switch_closed;
          	//Printing the state of the button when it is
          	//pressed to the serial monitor
            uart_printf("Switch is %s.\r\n",  prevState ? "closed" : "open");
          	if (prevState != 1) {
            	//Toggle LED 1 to show weather the motor is on
              	//or not
              	PORTB ^= (1<<3);
              	//Toggle the output of pin 6 to output the PWM
              	//generated or not (outputs to LED 2, Motor 1,
              	//Oscilloscope, and voltmeter)
              	DDRD ^= (1<<6);
            }
        }
    }

    return 0;
}


// Transmit a byte
void uart_putchar(uint8_t c) {
    uint8_t i;

    i = tx_buffer_head + 1;
    if (i >= TX_BUFFER_SIZE) i = 0;
    while (tx_buffer_tail == i); // wait until space in buffer
    //cli();
    tx_buffer[i] = c;
    tx_buffer_head = i;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0);
    //sei();
}

// Transmit a string
void uart_putstring(char* s) {
    // transmit character until NULL is reached
    while (*s > 0) uart_putchar(*s++);
}

// Transmit Interrupt
ISR(USART_UDRE_vect) {
    uint8_t i;

    if (tx_buffer_head == tx_buffer_tail) {
        // buffer is empty, disable transmit interrupt
        UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    }
    else {
        i = tx_buffer_tail + 1;
        if (i >= TX_BUFFER_SIZE) i = 0;
        UDR0 = tx_buffer[i];
        tx_buffer_tail = i;
    }
}




//More LCD setup
void setup_lcd(void) {
  // set up the LCD in 4-pin or 8-pin mode
  lcd_init();


  // Print a message to the LCD 
  lcd_write_string(0, 0, "Duty Cycle:");
  lcd_write_string(0, 1, "Sim Time:");
  //lcd_write_string(10, 1, "005(s)");
  char str [3];
  volatile uint8_t pots = (pot*100)/255;
  sprintf(str, "%d", pots);
  lcd_write_string(12, 0, str);
  lcd_write_string(15, 0, "%");
  
  char str2[4];
  sprintf(str2, "%d", time_now);
  lcd_write_string(10, 1, str2);
  lcd_write_string(15, 1, "s");
  
  //_delay_ms(1000);


  //register 4 new character bitmaps as character codes 0-3
  lcd_createChar(0, bmp0);
  lcd_createChar(1, bmp1);
  lcd_createChar(2, bmp2);
  lcd_createChar(3, bmp3);

  lcd_blink();

}


void loop(void) {
  static uint8_t frame = 0;

  //write the custom character bitmaps one at a time to make an animation
  if (frame == 0){
    lcd_setCursor(1,0);
    lcd_write(0);
    lcd_write(1);
    lcd_setCursor(1,1);
    lcd_write(2);
    lcd_write(3);
  } else if (frame == 1) {
    lcd_setCursor(1,0);
    lcd_write(1);
    lcd_write(2);
    lcd_setCursor(1,1);
    lcd_write(3);
    lcd_write(0);
  } else if (frame == 2){
    lcd_setCursor(1,0);
    lcd_write(2);
    lcd_write(3);
    lcd_setCursor(1,1);
    lcd_write(0);
    lcd_write(1);
  } else if (frame == 3){
    lcd_setCursor(1,0);
    lcd_write(3);
    lcd_write(0);
    lcd_setCursor(1,1);
    lcd_write(1);
    lcd_write(2);
  }

  if(frame % 2 == 0){
    scrollDisplayLeft();
  } else {
    scrollDisplayRight();
  }
  frame = (frame + 1) % 4;
}

//Function for the initialisation of the LCD
void lcd_init(void){
  //dotsize
  if (LCD_USING_4PIN_MODE){
    _lcd_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  } else {
    _lcd_displayfunction = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;
  }
  
  _lcd_displayfunction |= LCD_2LINE;

  // RS Pin
  LCD_RS_DDR |= (1 << LCD_RS_PIN);
  // Enable Pin
  LCD_ENABLE_DDR |= (1 << LCD_ENABLE_PIN);
  
  #if LCD_USING_4PIN_MODE
    //Set DDR for all the data pins
    LCD_DATA4_DDR |= (1 << LCD_DATA4_PIN);
    LCD_DATA5_DDR |= (1 << LCD_DATA5_PIN);
    LCD_DATA6_DDR |= (1 << LCD_DATA6_PIN);    
    LCD_DATA7_DDR |= (1 << LCD_DATA7_PIN);

  #else
    //Set DDR for all the data pins
    LCD_DATA0_DDR |= (1 << LCD_DATA0_PIN);
    LCD_DATA1_DDR |= (1 << LCD_DATA1_PIN);
    LCD_DATA2_DDR |= (1 << LCD_DATA2_PIN);
    LCD_DATA3_DDR |= (1 << LCD_DATA3_PIN);
    LCD_DATA4_DDR |= (1 << LCD_DATA4_PIN);
    LCD_DATA5_DDR |= (1 << LCD_DATA5_PIN);
    LCD_DATA6_DDR |= (1 << LCD_DATA6_PIN);
    LCD_DATA7_DDR |= (1 << LCD_DATA7_PIN);
  #endif 

  // SEE PAGE 45/46 OF Hitachi HD44780 DATASHEET FOR INITIALIZATION SPECIFICATION!

  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way before 4.5V so we'll wait 50
  _delay_us(50000); 
  // Now we pull both RS and Enable low to begin commands (R/W is wired to ground)
  LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
  LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
  
  //put the LCD into 4 bit or 8 bit mode
  if (LCD_USING_4PIN_MODE) {
    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    lcd_write4bits(0b0111);
    _delay_us(4500); // wait min 4.1ms

    // second try
    lcd_write4bits(0b0111);
    _delay_us(4500); // wait min 4.1ms
    
    // third go!
    lcd_write4bits(0b0111); 
    _delay_us(150);

    // finally, set to 4-bit interface
    lcd_write4bits(0b0010); 
  } else {
    // this is according to the hitachi HD44780 datasheet
    // page 45 figure 23

    // Send function set command sequence
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
    _delay_us(4500);  // wait more than 4.1ms

    // second try
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
    _delay_us(150);

    // third go
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
  }

  // finally, set # lines, font size, etc.
  lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);  

  // turn the display on with no cursor or blinking default
  _lcd_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
  lcd_display();

  // clear it off
  lcd_clear();

  // Initialize to default text direction (for romance languages)
  _lcd_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}


/********** high level commands, for the user! */
void lcd_write_string(uint8_t x, uint8_t y, char string[]){
  lcd_setCursor(x,y);
  for(int i=0; string[i]!='\0'; ++i){
    lcd_write(string[i]);
  }
}

void lcd_write_char(uint8_t x, uint8_t y, char val){
  lcd_setCursor(x,y);
  lcd_write(val);
}

void lcd_clear(void){
  lcd_command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  _delay_us(2000);  // this command takes a long time!
}

void lcd_home(void){
  lcd_command(LCD_RETURNHOME);  // set cursor position to zero
  _delay_us(2000);  // this command takes a long time!
}


// Allows us to fill the first 8 CGRAM locations
// with custom characters
void lcd_createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  lcd_command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    lcd_write(charmap[i]);
  }
}


void lcd_setCursor(uint8_t col, uint8_t row){
  if ( row >= 2 ) {
    row = 1;
  }
  
  lcd_command(LCD_SETDDRAMADDR | (col + row*0x40));
}

// Turn the display on/off (quickly)
void lcd_noDisplay(void) {
  _lcd_displaycontrol &= ~LCD_DISPLAYON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}
void lcd_display(void) {
  _lcd_displaycontrol |= LCD_DISPLAYON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// Turns the underline cursor on/off
void lcd_noCursor(void) {
  _lcd_displaycontrol &= ~LCD_CURSORON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}
void lcd_cursor(void) {
  _lcd_displaycontrol |= LCD_CURSORON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// Turn on and off the blinking cursor
void lcd_noBlink(void) {
  _lcd_displaycontrol &= ~LCD_BLINKON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}
void lcd_blink(void) {
  _lcd_displaycontrol |= LCD_BLINKON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// These commands scroll the display without changing the RAM
void scrollDisplayLeft(void) {
  lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void scrollDisplayRight(void) {
  lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

/*********** mid level commands, for sending data/cmds */

inline void lcd_command(uint8_t value) {
  //
  lcd_send(value, 0);
}

inline size_t lcd_write(uint8_t value) {
  lcd_send(value, 1);
  return 1; // assume sucess
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void lcd_send(uint8_t value, uint8_t mode) {
  //RS Pin
  LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
  LCD_RS_PORT |= (!!mode << LCD_RS_PIN);

  if (LCD_USING_4PIN_MODE) {
    lcd_write4bits(value>>4);
    lcd_write4bits(value);
  } else {
    lcd_write8bits(value); 
  } 
}

void lcd_pulseEnable(void) {
  //Enable Pin
  LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
  _delay_us(1);    
  LCD_ENABLE_PORT |= (1 << LCD_ENABLE_PIN);
  _delay_us(1);    // enable pulse must be >450ns
  LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
  _delay_us(100);   // commands need > 37us to settle
}

void lcd_write4bits(uint8_t value) {
  //Set each wire one at a time

  LCD_DATA4_PORT &= ~(1 << LCD_DATA4_PIN);
  LCD_DATA4_PORT |= ((value & 1) << LCD_DATA4_PIN);
  value >>= 1;

  LCD_DATA5_PORT &= ~(1 << LCD_DATA5_PIN);
  LCD_DATA5_PORT |= ((value & 1) << LCD_DATA5_PIN);
  value >>= 1;

  LCD_DATA6_PORT &= ~(1 << LCD_DATA6_PIN);
  LCD_DATA6_PORT |= ((value & 1) << LCD_DATA6_PIN);
  value >>= 1;

  LCD_DATA7_PORT &= ~(1 << LCD_DATA7_PIN);
  LCD_DATA7_PORT |= ((value & 1) << LCD_DATA7_PIN);

  lcd_pulseEnable();
}

void lcd_write8bits(uint8_t value) {
  //Set each wire one at a time

  #if !LCD_USING_4PIN_MODE
    LCD_DATA0_PORT &= ~(1 << LCD_DATA0_PIN);
    LCD_DATA0_PORT |= ((value & 1) << LCD_DATA0_PIN);
    value >>= 1;

    LCD_DATA1_PORT &= ~(1 << LCD_DATA1_PIN);
    LCD_DATA1_PORT |= ((value & 1) << LCD_DATA1_PIN);
    value >>= 1;

    LCD_DATA2_PORT &= ~(1 << LCD_DATA2_PIN);
    LCD_DATA2_PORT |= ((value & 1) << LCD_DATA2_PIN);
    value >>= 1;

    LCD_DATA3_PORT &= ~(1 << LCD_DATA3_PIN);
    LCD_DATA3_PORT |= ((value & 1) << LCD_DATA3_PIN);
    value >>= 1;

    LCD_DATA4_PORT &= ~(1 << LCD_DATA4_PIN);
    LCD_DATA4_PORT |= ((value & 1) << LCD_DATA4_PIN);
    value >>= 1;

    LCD_DATA5_PORT &= ~(1 << LCD_DATA5_PIN);
    LCD_DATA5_PORT |= ((value & 1) << LCD_DATA5_PIN);
    value >>= 1;

    LCD_DATA6_PORT &= ~(1 << LCD_DATA6_PIN);
    LCD_DATA6_PORT |= ((value & 1) << LCD_DATA6_PIN);
    value >>= 1;

    LCD_DATA7_PORT &= ~(1 << LCD_DATA7_PIN);
    LCD_DATA7_PORT |= ((value & 1) << LCD_DATA7_PIN);
    
    lcd_pulseEnable();
  #endif
}