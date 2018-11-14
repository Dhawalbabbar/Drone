/*
▪ * Team Id: 1326
▪ * Author List: Arun Malik, Shubham Sharma, Dhawal Babbar, Tushar Purang
▪ * Filename: main.c
▪ * Theme: Chaser Drone
▪ * Functions: buzzer_pin_config, buzzer_on, buzzer_off, lcd_port_config, adc_pin_config, motion_pin_config, port_init, timer1_init, adc_init, velocity,
    ADC_Conversion, print_sensor, motion_set, forward, stop, init_devices, in_cave, main
▪ * Global Variables: ADC_Conversion, ADC_Value, Left_white_line , Center_white_line, Right_white_line
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"
#include <stdbool.h>
#define		THRESHOLD		15        // set the pots such that all three sensor
#include <math.h>                     // calibrated to show its min value on LCD.
                                      // i.e on LCD Sensor values are betwn 168 to 172
			 						  // on black line
#define		VELOCITY_MAX	210
#define		VELOCITY_MIN	50
#define 	VELOCITY_LOW	20

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char irSensor = 0;

void buzzer_pin_config (void)
{
/*
▪ * Function Name:buzzer_pin_config
▪ * Logic: Function to configure buzzer pin
▪ * Example Call: buzzer_pin_config()
▪ */
 DDRC = DDRC | 0x08;
 PORTC = PORTC & 0xF7;
}

void buzzer_on (void)
{
    /*
▪ * Function Name:buzzer_on
▪ * Logic: Function to turn on the buzzer
▪ * Example Call: buzzer_on()
▪ */
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{/*
▪ * Function Name:buzzer_off
▪ * Logic: Function to turn off the buzzer
▪ * Example Call: buzzer_off()
▪ */
unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

void lcd_port_config (void)
{/*
▪ * Function Name:lcd_port_config
▪ * Logic: Function to configure LCD port
▪ * Example Call: lcd_port_config()
▪ */
 DDRC = DDRC | 0xF7;
 PORTC = PORTC & 0x80;
}

void adc_pin_config (void)
{/*
▪ * Function Name:adc_pin_config
▪ * Logic: ADC pin configuration
▪ * Example Call: adc_pin_config()
▪ */
 DDRA = 0x00;
 PORTA = 0x00;
}

void motion_pin_config (void)
{/*
▪ * Function Name:motion_pin_config
▪ * Logic: Function to configure ports to enable robot's motion
▪ * Example Call: motion_pin_config()
▪ */
 DDRB = DDRB | 0x0F;
 PORTB = PORTB & 0xF0;
 DDRD = DDRD | 0x30;
 PORTD = PORTD | 0x30;
}

void port_init()
{
/*
▪ * Function Name:port_init
▪ * Logic: Function to Initialize PORTS
▪ * Example Call: port_init()
▪ */
 adc_pin_config();
 motion_pin_config();
 buzzer_pin_config();
}

//TIMER1 initialize - prescale:64
// WGM: 5) PWM 8bit fast, TOP=0x00FF
// desired value: 450Hz
// actual value: 450.000Hz (0.0%)
void timer1_init(void)
{/*
▪ * Function Name:timer1_init
▪ * Logic: <Description of the function performed and the logic used
▪ * in the function>
▪ * Example Call: timer1_init()
▪ */
 TCCR1B = 0x00;
 TCNT1H = 0xFF;
 TCNT1L = 0x01;
 OCR1AH = 0x00;
 OCR1AL = 0xFF;
 OCR1BH = 0x00;
 OCR1BL = 0xFF;
 ICR1H  = 0x00;
 ICR1L  = 0xFF;
 TCCR1A = 0xA1;
 TCCR1B = 0x0D;
}

void adc_init()
{/*
▪ * Function Name:adc_init
▪ * Logic: Function to Initialize ADC
▪ * Example Call: adc_init()
▪ */
 ADCSRA = 0x00;
 ADMUX = 0x20;
 ACSR = 0x80;
 ADCSRA = 0x86;
}


void velocity (unsigned char left_motor, unsigned char right_motor )
{/*
▪ * Function Name:velocity
▪ * Input:left_motor(speed of the respective motor), right_motor(speed of the respective motor)
▪ * Logic: Function to set velocity of left and right motors
▪ * Example Call: velocity(40,40)
▪ */
 OCR1AH = 0x00;
 OCR1AL = left_motor;
 OCR1BH = 0x00;
 OCR1BL = right_motor;


}

unsigned char ADC_Conversion(unsigned char Ch)
{/*
▪ * Function Name:ADC_Conversion
▪ * Input: Ch (channel number)
▪ * Output: analog value generated from channel number
▪ * Logic: This Function accepts the Channel Number and returns the corresponding Analog Value
▪ * Example Call: ADC_Conversion(3)
▪ */
 unsigned char a;       //generated analog value, to be returned
 Ch = Ch & 0x07;
 ADMUX= 0x20| Ch;
 ADCSRA = ADCSRA | 0x40;
 while((ADCSRA&0x10)==0);
 a=ADCH;
 ADCSRA = ADCSRA|0x10;
 return a;
}

void print_sensor(char row, char coloumn,unsigned char channel)
{/*
▪ * Function Name:print_sensor
▪ * Input: row, column (row and column values of the display), channel(channel for the display)
▪ * Logic: This Function prints the Analog Value Of Corresponding Channel No. at required Row and Column Location.
▪ * Example Call: print_sensor(2,10,3)
▪ */
 ADC_Value = ADC_Conversion(channel);

}

void motion_set (unsigned char Direction)
{/*
▪ * Function Name:motion_set
▪ * Input: Direction(Direction of motor)
▪ * Logic: Function used for setting motor's direction
▪ * Example Call: motion_set()
▪ */
 unsigned char PortBRestore = 0;

 Direction &= 0x0F;
 PortBRestore = PORTB;
 PortBRestore &= 0xF0;
 PortBRestore |= Direction;
 PORTB = PortBRestore;
}

void forward (void)
{/*
▪ * Function Name:forward
▪ * Logic: Both wheels forward
▪ * Example Call: forward()
▪ */
  motion_set(0x06);
}

void stop (void)
{/*
▪ * Function Name:stop
▪ * Logic: Hard stop
▪ * Example Call: stop()
▪ */
  motion_set(0x00);
}

void init_devices (void)
{/*
▪ * Function Name:init_devices
▪ * Logic: Function for velocity control
▪ * Example Call:init_devices()
▪ */
 cli();          //Clears the global interrupts
 port_init();
 timer1_init();
 adc_init();
 sei();          //Enables the global interrupts
}

void in_cave(){
    /*
▪ * Function Name:in_cave
▪ * Logic: <Description of the function performed and the logic used
▪ * in the function>
▪ * Example Call: in_cave()
▪ */
 velocity(VELOCITY_LOW,VELOCITY_LOW);
 _delay_ms(5000);
velocity(VELOCITY_MIN,VELOCITY_MIN);
}


int main(void)
{
/*
▪ * Function Name:main
▪ * Logic: Main Function
*/

int runnerpath[]={3,1,2,1,3,2,1,2,1,3,0},dec_point=0 ;  //runnerpath - path to be followed by the runner, dec_point - predicted decision point
int lflag=0;        //checks the direction the robot was last diverted from, off the line
init_devices();
velocity(255,255);    // Set the speed to max velocity
forward();
while(true){
    Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(4);	//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(5);	//Getting data of Right WL Sensor
	irSensor=ADC_Conversion(7);

    if(dec_point!=1 && irSensor<100)
    {
        velocity(0,0);
        buzzer_on();
        _delay_ms(100);
        buzzer_off();
        break;
    }

if ((Left_white_line>60 && Center_white_line>60) || (Right_white_line>60 && Center_white_line>60)){
	if (runnerpath[dec_point]==1){
        if(dec_point!=1)
        {
            velocity(VELOCITY_MAX,VELOCITY_MAX);
            _delay_ms(50);
            velocity(0,0);
            _delay_ms(1950);
            velocity(VELOCITY_MAX,VELOCITY_MAX);
        }
        else
        {
            velocity(VELOCITY_MAX,VELOCITY_MAX);
            _delay_ms(100);
            velocity(0,0);
            for(int sad=0;sad<14;sad++){
                velocity(0,0);
                _delay_ms(1000);
            }

            velocity(VELOCITY_MAX,VELOCITY_MAX);
        }
	}
	if (runnerpath[dec_point]==2){
	velocity(VELOCITY_MAX,VELOCITY_MAX);
	_delay_ms(200);
	 motion_set(0x05);
	velocity(VELOCITY_MAX,VELOCITY_MAX);
	_delay_ms(440);
    motion_set(0x06);
    velocity(0,0);
    _delay_ms(1360);
    velocity(VELOCITY_MAX,VELOCITY_MAX);
	}

	if (runnerpath[dec_point]==3){
	velocity(VELOCITY_MAX,VELOCITY_MAX);
	_delay_ms(200);
	 motion_set(0x0A);
	velocity(VELOCITY_MAX,VELOCITY_MAX);
	_delay_ms(410);
    motion_set(0x06);
    velocity(0,0);
    _delay_ms(1360);
    velocity(VELOCITY_MAX,VELOCITY_MAX);
	}

	if (runnerpath[dec_point]==4){
	 motion_set(0x0A);
	velocity(VELOCITY_MAX,VELOCITY_MAX);
	_delay_ms(820);
    motion_set(0x06);
    velocity(0,0);
    _delay_ms(1300);
    velocity(VELOCITY_MAX,VELOCITY_MAX);
	}
	if (runnerpath[dec_point]==0){
	velocity(0,0);
	for(int sad=0;sad<10;sad++)
        _delay_ms(2000);
    buzzer_on();
    _delay_ms(1000);
    buzzer_off();
    break;
	}
	dec_point++;
}

if (Left_white_line>15 && Right_white_line>15){
    if (lflag==1) {
		velocity(VELOCITY_MAX-(((0.010588*Left_white_line*Left_white_line)+(0.8422*Left_white_line))*1.5),VELOCITY_MAX);
    }
    else {
		velocity(VELOCITY_MAX,VELOCITY_MAX-(((0.010588*Right_white_line*Right_white_line)+(0.8422*Right_white_line))*1.5));
    }
}
    else{
 	if((Left_white_line>THRESHOLD) )  // Is left Whiteline is not within threshold limit
 	{
		lflag=1;
		velocity(VELOCITY_MAX-(((0.010588*Left_white_line*Left_white_line)+(0.8422*Left_white_line))*1.5),VELOCITY_MAX);
	}

	if((Right_white_line>THRESHOLD) ) // Is right Whiteline is not within threshold limit
 	{
		lflag=0;
		velocity(VELOCITY_MAX,VELOCITY_MAX-(((0.010588*Right_white_line*Right_white_line)+(0.8422*Right_white_line))*1.5));
	}

	if(Center_white_line>THRESHOLD)               // Is middle Whiteline is within threshold limit
	{
		velocity(VELOCITY_MAX,VELOCITY_MAX);      // Run robot at max velocity
	}

}
}
}


