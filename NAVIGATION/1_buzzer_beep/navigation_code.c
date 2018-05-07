/*
Team Id:2143
Author List:Tushar K.S. Chauhan
Filename:navigation
Theme:Transporter Bot
Functions:send(unsigned char),moveforward(int),rotate_clockwise(unsigned int),rotate_anticlockwise(unsigned int),count_black_dots(),
		  pickup_and_place(int),sense(),get_back_on_track(),place_container_on_rotating_structure(int),initialize_servo(),calcdots(int,int),
		  return_rotating_structure(int),start(),main()
Global Variables:int count,int object_sequence[],int colour_sequence[],int total_rounds,int current_object,int position_while_placing_containers_in_rotating_structure,
				 int empty_current_container,int i,int flag for starting position,int ShaftCountLeft,int ShaftCountRight,int Degrees,char ADC_Value,char flag,char Left_white_line
				 char Center_white_line,char Right_white_line,char sharp_ir3,char data 
*/
 /*no. defines the initial direction of the colour in the crate, 
 1 represents north 
 2 defines west
 3 defines south
 4 defines east */
#define empty_containers 4 //no. of containers present in the firebird
#define object_sequence_length 6 //no. of pickup places where objects are placed
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.h"
int count=0;//counts the no. of black dots passed for moveforward()
int object_sequence[]={12,3,5,7,9,12};//placement sequence
char colour_sequence[]={'b','y','b','r','r','g'};//colour corresponding to the blocks
int total_rounds=0;//defines no. of rounds of robot from the pickup places to the container.
int current_object=0;//position of the object where bot has to go currently
int position_while_placing_containers_in_rotating_structure=0;//it will be initialized some value after bot has completed one round back to the rotating structure,we are using three ways to come back and the variable will be initialized according to the path chosen
int empty_current_container=0;//no. of containers empty at present
int i=0;//total no. of objects picked up
int flag_for_starting_position=0;//specifies whether the robot is at starting position or at standing at the position where he picked the previous block
//variables for position encoders
volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
//end of variables for position encoders

//variables of white line following
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
//end of variables for white line following

//variables for IR sensorss
unsigned char sharp_ir3=0;
//end

//variable for communication
unsigned char data=0; //to store received data from UDR1
//end 

unsigned char ADC_Conversion(unsigned char);
void sense(); //function to check repeatedly for the white line sensor values.


//functions for zigbee communication
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	// UBRR0L = 0x47; //11059200 Hz
	UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}

//servo motor functions
//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
	DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
	PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}
//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
	float PositionServo = 0;
	PositionServo = ((float)degrees / 1.86) + 35.0;
	OCR1CH = 0x00;
	OCR1CL = (unsigned char) PositionServo;
}

//servo_free functions unlocks the servo motors from the any angle
//and make them free by giving 100% duty cycle at the PWM. This function can be used to
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}

void servo_3_free (void) //makes servo 3 free rotating
{
	OCR1CH = 0x03;
	OCR1CL = 0xFF; //Servo 3 off
}


//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}


//functions to configure position encoder
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}
//Function to initialize Buzzer 
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void port_init (void)
{
 buzzer_pin_config();
 lcd_port_config();
 adc_pin_config();
 motion_pin_config(); //robot motion pins config
 left_encoder_pin_config(); //left encoder pin config
 right_encoder_pin_config(); //right encoder pin config
 servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
 servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
 servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation
}
//interrupt function for postion encoders
void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

//function for timer5 used in PWM
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}


void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}


void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}
void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void)
{
	motion_set(0x00);
}
//Function used for turning robot by specified degrees

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	velocity(220,220);
	linear_distance_mm(DistanceInMM);
}

void init_devices (void)
{
 cli(); //Clears the global interrupts
 port_init();
 uart0_init();
 adc_init();
 timer5_init();
 timer1_init();//for servo motors
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 sei(); //Enables the global interrupts
}
/*
Function Name: send
Input: character to be sent via zigbee
Output:no output
logic:sends the signal multiple times since it may be possible zigbee on the other hand might not recieve the signal send from here
Example call:send('A')
*/
void send(unsigned char data_to_be_sent)//function to send data
{
		UDR0=data_to_be_sent;
}

/*
Function Name: moveforward
Input: no. of dots to be skipped
Output:no output
logic:this function will follow the white line until the specified no. of dots are not skipped
Example call:moveforward(2)
*/
void moveforward(int no_of_dots_to_be_skipped)
{
	//move forward and dont stop  till skipped lines !=0 
	int count=0;//will count the no. of black dots passed.
	while(1)//code to follow black line
	{
		sense();
		if(Center_white_line>0x28)
		{
			forward();
			velocity(255,255);
		}
		else if(Left_white_line<0x28)
		{
			forward();
			velocity(255,0);
		}
		else if(Right_white_line<0x28)
		{
			forward();
			velocity(0,255);
		}
		if(Center_white_line+Left_white_line+Right_white_line>0xB4)
		{
			/*initially when bot will be at the starting position it will rotate towards its
			first position and then will continue its path,after reaching the edge,bot has to rotate 
			a little more on 1,4,5,8,9 and 12 to calibrate that the below if condition is used
			*/
			if((current_object%4==0||(current_object-1)%4==0)&& current_object!=0)
			{
				forward_mm(10);
			}
			else
			{
				forward_mm(50);
			}
			if(count>=no_of_dots_to_be_skipped)
			{
				
				current_object=0;//to null the effect of above  if((current_object%4==0||(current_object-1)%4==0)&& current_object!=0) condition after the edge rotation is performed,it is set to 0.current object will again be given new value when bot will come from starting position to any of the object positions   
				stop();
				velocity(0,0);
				_delay_ms(500);
				break;
			}
			count++;
		}			
	}
}
/*
Function Name: rotate_clockwise
Input:no. of lines to be skipped while rotation-1
Output:no output
logic:this function will rotate the robot in clockwise direction till the specified no. of lines are not skipped 
Example call:rotate_clockwise(1)
*/
void rotate_clockwise(unsigned int lines_to_be_skipped)
{
	//rotate clockwise and dont stop  till skipped lines !=0 
	right();
	velocity(150,150);
	while(lines_to_be_skipped>0)
	{
		_delay_ms(300);
		lines_to_be_skipped--;
		sense();
		while(Right_white_line<0x20)
		{
			sense();
		}
	}
	stop();
	_delay_ms(500);
	sense();
	//condition if by chance the robot has moved more than desired
	while(Center_white_line<0x20 && Left_white_line>0x20)
	{
		sense();
		left();
		velocity(150,150);
	}
	stop();
	_delay_ms(500);
	
}
/*
Function Name: rotate_anticlockwise
Input:no. of lines to be skipped-1
Output:no output
logic:this function will rotate the robot in anticlockwise direction till the specified no. of lines are not skipped 
Example call:rotate_anticlockwise(1)
*/
void rotate_anticlockwise(unsigned int lines_to_be_skipped)
{
	left();
	velocity(150,150);
	while(lines_to_be_skipped>0)
	{
		_delay_ms(300);
		lines_to_be_skipped--;
		sense();
		while(Left_white_line<0x20)
		{
			sense();
		}
	}
	stop();
	_delay_ms(500);
	sense();
	//condition if by chance the robot has moved more than desired
	while(Center_white_line<0x20 && Right_white_line>0x20)
	{
		sense();
		right();
		velocity(150,150);
	}
	stop();
	_delay_ms(500);
}
/*
Function Name: count_black_dots
Input:no input
Output:no output
logic:this function will increment the value of global variable count by 1
		this function is mainly made to count the no. of dots robot passes in at run time
Example call:count_black_dots
*/
void count_black_dots()
{
	count++;
	//increments the value of count each time called
}

/*
Function Name:initialize_servo()
Input:no input
Output:no output
logic:initializes arm to come in its default position
Example call:initialize_servo()
*/
void initialize_servo()
{
	//initialize the servos to its initial position
	_delay_ms(1500);
	servo_2(100);
	_delay_ms(1500);
	servo_1(180);
	_delay_ms(1500);
	servo_3(0);
	_delay_ms(1500);
	servo_1_free();
	servo_2_free();
	servo_3_free();
}
/*
Function Name:pickup_and_place
Input:integer specifying which  first container is empty in the priority order been set for them
Output:no output
logic:this function will pick the block up and place it in the (empty_container)th container
Example call:pickup_and_place(2)
*/
void pickup_and_place(int empty_container)//pickups the block and places them in the container
{
	char signal_to_blender='A';//taken an initial value of signal
	signal_to_blender+=object_sequence[i]-1;//incremented the value by the no. at which object is placed - 1 i.e. now for position 1 signal is A, for 2 signal is B and so on
	send(signal_to_blender);//send the signal to blender so that robot in blender is also placed infront of the object their also
	//picking up the object
	
	_delay_ms(150);
	servo_1(180);//double checking the initial position
	_delay_ms(1500);
	servo_1_free();
	servo_2(7);//arm comes down
	_delay_ms(1500);
	servo_2_free();
	servo_3(77);//arm grabs the object
	_delay_ms(1500);
	servo_3_free();
	servo_2(100);//arm again goes up
	_delay_ms(1500);
	servo_2_free();
	//one switch condition for every container
	switch(empty_container)
	{
		case 4: servo_1(86);//arm rotates to the specified angle
				_delay_ms(1500);
				servo_1_free();
				servo_2(60);//arm comes down to drop the object in container
				_delay_ms(1500);
				servo_2_free();
				servo_3(0);//flap opens and object is dropped
				_delay_ms(1500);
				servo_3_free();
				initialize_servo();
				
				break;
		case 3: servo_1(55);//arm rotates to the specified angle
			_delay_ms(1500);
			servo_1_free();
			servo_2(57);//arm comes down to drop the object in container
			_delay_ms(1500);
			servo_2_free();
			servo_3(0);//flap opens and object is dropped
			_delay_ms(1500);
			servo_3_free();
			initialize_servo();
			break;
		case 2: servo_1(23);//arm rotates to the specified angle
			_delay_ms(1500);
			servo_1_free();
			servo_2(60);//arm comes down to drop the object in container
			_delay_ms(1500);
			servo_2_free();
			servo_3(0);//flap opens and object is dropped
			_delay_ms(1500);
			servo_3_free();
			initialize_servo();
			break;
		case 1: servo_1(0);//arm rotates to the specified angle
			_delay_ms(1500);
			servo_1_free();
			servo_2(50);//arm comes down to drop the object in container
			_delay_ms(1500);
			servo_2_free();
			servo_3(0);//flap opens and object is dropped
			_delay_ms(1500);
			servo_3_free();
			initialize_servo();
			break;			 
	}
	servo_1_free();
	servo_2_free();
	servo_3_free();

}

/*
Function Name:sense
Input:no input
Output:no output
logic:will change the value of global variables left_white_line,center_white_line,right_white_line accordingly as specified by the colour sensors and the value of sharp_ir3 as specified by the sharp ir sensor
Example call:sense()
*/
void sense()
{
	//checks for the ir sensor values and calls count_black_dots whenever a black dot is seen
	Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
	sharp_ir3=ADC_Conversion(11);			//getting data of sharp ir sensor in the front
}
/*
Function Name:get_back_on_track
Input:no input
Output:no output
logic:after picking up the block this function will move the bot back on the main line to complete the remaining task
Example call:get_back_on_track()
*/
void get_back_on_track()
{
	//gets the bot back on track when the object is picked up
	rotate_clockwise(1);//since the robot is picking the object in ascending order so everytime will rotate clockwise to go to next bigger position 
}
/*
Function Name:place_container_on_rotating_structure
Input:no input
Output:no output
logic:this function will place the bot back on the rotating structure 
Example call:place_container_on_rotating_structure
*/
void place_container_on_rotating_structure(int empty_container_position)
{
	
	send('M');//message to place containers on the truck
	_delay_ms(150);
	int i_copy=i-1;//copy of the initial position
	//place the blocks on rotating structure,refer to the array for any required information
	
	if(empty_container_position<=0)//i.e. if one object is held in arm only
	{
		send(colour_sequence[i_copy]);
		i_copy--;
		_delay_ms(5000);
		servo_2(70);//arm comes at an angle to drop the object in rotating structure contained in his hand
		_delay_ms(1500);
		servo_3(0);//arm drops the object
		_delay_ms(1500);
		
	}
		initialize_servo();
		while(empty_container_position<=4)//empty_container_position will give the last container position where object was placed and then with fall through in switch arm will pickup object from that container as well as all the other previously filled 
		{
			
			if(empty_container_position==4)
			{
				send(colour_sequence[i_copy]);
				i_copy--;
				_delay_ms(1500);
				servo_1(86);//arm rotates at specified angle
				_delay_ms(1500);
				servo_2(50);//arm comes down to collect the object
				_delay_ms(1500);
				servo_3(80);//arm grabs the object
				_delay_ms(1500);
				servo_2(100);//arm comes up again
				_delay_ms(1500);
				servo_1(180);//arm rotates to front
				_delay_ms(1500);
				servo_2(70);//arm comes to an angle to drop the object
				_delay_ms(1500);
				servo_3(0);//flap opens and the object is droped
				initialize_servo();
			}
			if(empty_container_position==3)
			{
				send(colour_sequence[i_copy]);
				i_copy--;
				_delay_ms(1500);
				servo_1(55);//arm rotates at specified angle
				_delay_ms(1500);
				servo_2(50);//arm comes down to collect the object
				_delay_ms(1500);
				servo_3(80);//arm grabs the object
				_delay_ms(1500);
				servo_2(100);//arm comes up again
				_delay_ms(1500);
				servo_1(180);//arm rotates to front
				_delay_ms(1500);
				servo_2(70);//arm comes to an angle to drop the object
				_delay_ms(1500);
				servo_3(0);//flap opens and the object is droped
				initialize_servo();
				
			}
			if(empty_container_position==2)
			{
				send(colour_sequence[i_copy]);
				i_copy--;
				_delay_ms(1500);
				servo_1(23);//arm rotates at specified angle
				_delay_ms(1500);
				servo_2(50);//arm comes down to collect the object
				_delay_ms(1500);
				servo_3(80);//arm grabs the object
				_delay_ms(1500);
				servo_2(100);//arm comes up again
				_delay_ms(1500);
				servo_1(180);//arm rotates to front
				_delay_ms(1500);
				servo_2(70);//arm comes to an angle to drop the object
				_delay_ms(1500);
				servo_3(0);//flap opens and the object is droped
				initialize_servo();
			}
			if(empty_container_position==1)
			{
				send(colour_sequence[i_copy]);
				i_copy--;
				_delay_ms(1500);
				servo_1(0);//arm rotates at specified angle
				_delay_ms(1500);
				servo_2(50);//arm comes down to collect the object
				_delay_ms(1500);
				servo_3(80);//arm grabs the object
				_delay_ms(1500);
				servo_2(100);//arm comes up again
				_delay_ms(1500);
				servo_1(180);//arm rotates to front
				_delay_ms(1500);
				servo_2(70);//arm comes to an angle to drop the object
				_delay_ms(1500);
				servo_3(0);//flap opens and the object is droped
				initialize_servo();
			}
				empty_container_position++;
		}
	
}
/*
Function Name:calcdots()
Input:two integers i.e. final and initial postions
Output:no. of dots between the two positions(position refers to nodes where the blocks are to placed)
logic:total dots will be equal to difference of two positions + no. of even no. present between them as their is an extra after every integer position
Example call:calcdots(2,6)
*/
int calcdots(int initial_position,int final_position)
{
	int dots=0;
	dots=final_position-initial_position-1;
	while(initial_position<final_position)
	{
		if(initial_position % 2==0)
		{
			dots++;
		}
		initial_position++;
	}
	return dots;	
}
/*
Function Name:return_rotating_structure
Input:integer specifying current position where the bot is standing
Output:no output
logic:the function will navigate the bot from the place where currently he is standing to the rotating structure
Example call:pickup_and_place
*/
void return_rotating_structure(int current_position)
{
	//we will be using three return paths bot will choose the one closest to it.
	total_rounds++;
	if(current_position<=4)
	{
		//this return path is the one you can find nearest to object position 1
		send('1');//send message that we are coming from this path
		position_while_placing_containers_in_rotating_structure=1;
		rotate_anticlockwise(1);
		moveforward(calcdots(1,current_position)+1);
		rotate_anticlockwise(1);
	}
	else if(current_position<=8)
	{
		//this return path is the one you can find nearest to starting position
		//since the rotating structure is initially at this position only so no need to send message 
		position_while_placing_containers_in_rotating_structure=2;
		if(current_position<7)
		{
			rotate_clockwise(1);
			moveforward(calcdots(current_position,7)-1);
			rotate_clockwise(1);
		}
		else
		{
			rotate_anticlockwise(1);
			moveforward(calcdots(6,current_position)-1);
			rotate_anticlockwise(1);
		}
		moveforward(2);
	}
	else
	{
				//this return path is the one you can find nearest to object position 3
				send('3');//send message that we are coming from this path
				position_while_placing_containers_in_rotating_structure=2;
				rotate_clockwise(1);
				moveforward(calcdots(current_position,12)+1);
				rotate_clockwise(1);
	}
	while(sharp_ir3<0x80)//robot will move forward till a required nearness is not maintained from the robot
	{
		sense();
		forward();
		velocity(220,220);
	}
	stop();
	_delay_ms(500);
}
/*
Function Name:start
Input:no input
Output:no output
logic:navigates the robot from start position to the next object position or form the previous object position to the next object position
Example start
*/
void start()
{
	while(i<object_sequence_length && empty_current_container!=-1)
	{
		if(flag_for_starting_position==0)//flag_for_starting_position==0 signifies robot is at starting position else the robot will be at previous object position
		{
			current_object=object_sequence[i];//it will be used while rotation at the edge when the robot will move from starting position to the outer square and make a turn
			flag_for_starting_position=1;//specifies now object will be found at its previous object location
			//navigate from the starting postion to the (object_sequence[i])th position
			rotate_clockwise(object_sequence[i]/2+1);//rotate  to the line shortest for its object to reach from starting position
			if(object_sequence[i]==1 || object_sequence[i]==12)//since the lines for 1 and 12 from starting position contain a extra dot so given two dot skip statement
			{
				moveforward(2);
			}
			else
			{
				moveforward(1);
			}
			if(object_sequence[i]%2==0)//if the object is at even position the robot will take a turn in anticlockwise direction else in clockwise direction 
			{
				rotate_anticlockwise(1);
			}
			else
			{
				rotate_clockwise(1);
			}
			moveforward(0);//robot now will stop at the next dot,at this position the required object is placed 
			stop();
			_delay_ms(500);
			if(object_sequence[i]%2==0)//if the object is at even position the robot will take a turn in anticlockwise direction else in clockwise direction
			{
				//in this case arm will not go down as it may not support but interfere in the pickup
				rotate_clockwise(1);//robot will turn towards the object,our navigation for the first object is complete
			}
			else
			{
				servo_2(7);//arm goes down
				_delay_ms(1500);
				rotate_anticlockwise(1);//robot will turn towards the object,our navigation for the first object is complete
			}
			//bot is standing at the position to pickup the block
		}
		else
		{
			//navigate the robot from (object_sequence[i-1])th to (object_sequence[i])th)position
			//i.e from previous position to next position
			get_back_on_track();//move the robot clockwise to ready the robot to go to the next placement sequence
			if(object_sequence[i-1]<=4 && object_sequence[i]>4 && object_sequence[i]<=8)// if previous position is less than 5 and next position is 5,6,7,8 it will be executed  
			{
				moveforward(calcdots(object_sequence[i-1],4)+1);
				rotate_clockwise(1);
				moveforward(calcdots(5,object_sequence[i]));
			}
			else if(object_sequence[i-1]>4 && object_sequence[i-1]<=8 && object_sequence[i]>8)//if previous postion is less than 9 and next position is 9,10,11,12 it will be executed
			{
				moveforward(calcdots(object_sequence[i-1],8)+1);
				rotate_clockwise(1);
				moveforward(calcdots(9,object_sequence[i])+1);
				
			}
			else if(object_sequence[i-1]<=4 && object_sequence[i]>8)//if previous position is less than 5 and next position is greator than 8 it will be executed
			{
				moveforward(calcdots(object_sequence[i-1],4)+1);
				rotate_clockwise(1);
				moveforward(5);
				rotate_clockwise(1);
				moveforward(calcdots(9,object_sequence[i])+1);
				
			}
			else//for all the other cases it will be executed
			{
				moveforward(calcdots(object_sequence[i-1],object_sequence[i]));
				
			}
			servo_2(7);//arm goes down
			_delay_ms(1500);
			rotate_anticlockwise(1);
		}
		//pick the block and place in the (empty_current_container)th container
		pickup_and_place(empty_current_container);//picking up the object and placing it in the container
		empty_current_container--;i++;//incrementing i for next object to reach and decrementing container as by now that position will be filled
	}
}
/*
Function Name:main
Input:no input
Output:returns 0
logic:contains the main code
Example call:called automatically when the program is executed
*/
//Main Function
int main(void)
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	//initializing servo motors
	
	initialize_servo();//intialize the arm for its default position
	while(i<object_sequence_length)//i.e till all the elements are not picked
	{
		empty_current_container=empty_containers;//specifies no. of containers empty
		if(total_rounds!=0)//implies the robot is at starting position if total_rounds=0
		{
			back();
			velocity(220,220);
			linear_distance_mm(300);
			stop();
			_delay_ms(500);
			//if the robot is at position 1 after placing the containers total no. of object left to pickup will be zero as bot is capable of holding more than 4 objects at a time and 1 position is specified only for first 4 objects so no is condition for position 1
			if(position_while_placing_containers_in_rotating_structure==2)
			{
				rotate_clockwise(2);
				moveforward(0);
				rotate_clockwise(4);
				initialize_servo();
				_delay_ms(1500);
				//object will reach the starting position
				flag_for_starting_position=0;
				start();
			}
			else
			{
				rotate_clockwise(1);
				moveforward(0);
				rotate_anticlockwise(1);
				//if this block is executed blocks will only be left at 10 11 and 12
				if(object_sequence[i]==10)
				{
					send('1');
					moveforward(3);//navigate to 10 position
					
				}
				else//for position 11 and 12
				{
					moveforward(12-object_sequence[i]);//navigate to 11 or 12
					
				}
				servo_2(7);
				_delay_ms(1500);
				rotate_clockwise(1);
				pickup_and_place(empty_current_container);
				empty_current_container--;i++;
				flag_for_starting_position=1;
				start();
				
			}
		}
		else
		{
			flag_for_starting_position=0;
			start();
		}
		
		//return to the rotating structure
		return_rotating_structure(object_sequence[i-1]);
		//place the objects in the rotating structure
		place_container_on_rotating_structure(empty_current_container); 
	}
	stop();
	send('e');
	_delay_ms(150);
	buzzer_on();
	_delay_ms(6000);
	buzzer_off();
	_delay_ms(200000);
	//send message to end the game
	//stop the bot
	//buzzer on for 5 seconds
	
}


