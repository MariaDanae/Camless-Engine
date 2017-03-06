#include<avr/pgmspace.h>	//flash memory
#include <LiquidCrystal.h>   // include the LCD library
#include "TimerThree.h"

LiquidCrystal lcd(28,26, 30, 32, 34 , 36, 38);  

#include "SPI.h"		// for SPI
double result = 0; 
unsigned int result1 = 0;
unsigned int result2 = 0;

int Exhaust_Solenoid = 22; 	// Declare pins
int Intake_Solenoid = 24;
int blink_pin = 13;		// for testing timing
int SScounter1 = 48;
int SScounter2 = 46;

int servo_pin = 3;
int TDC_pin = 14 ;

signed long encoder1count = 0; //Hold count for encoder
signed long encoder2count = 0;

double rpm = 5.0;           // initialize variables of inputs
double rpm_crank_init = 0;
double rpm_time_init = 0;
double MaxL = 0.0; 		// lcd
double Mo = 0.0;		// lcd mode
unsigned long lastnow=0, now=0, timing=0;    // timing
volatile int crankpos=0;
volatile double last_crankpos=0.0;
volatile double crank_diff;
volatile bool rollover = false; //checks if crankpos >360	
const int TDC = 310;	
int Crankpos_offset = 310 - 215;// offset for engine crankpos
const int exop = 99 -20; 	//exhaust valve open
const int excl = 331 -0 ;  	// exhaust valve close -22
 int intop = 307 + 0 ;  	// intake valve open
 int intcl = 554 - 70; 	// intake valve close -7

unsigned long RPM_START_TIME; // in check_crankpos() to 
unsigned long RPM_START_TIME_LAST; //calculate rpm 
int INSTANT_RPM = 0;
int AVERAGE_RPM = 0;
double crank_position_last = 0.0;

long offset_on; 		// calculating offsets
long offset_off;
int intop_ACT;
int intcl_ACT;
int exop_ACT;
int excl_ACT;
bool added720Exhaustopen=0;
bool added720Exhaustclose=0;

volatile unsigned long time_now=0, last_time;

volatile bool update_720 = 0;
bool rpm_zero = 0;

// flash memory
/*
PROGMEM prog_uint32_t time[2000];
PROGMEM prog_uint16_t crank_pos[2000];
PROGMEM prog_int32_t exhaust_solenoid_pos[2000];
PROGMEM prog_uchar exhaust_solenoid_cmd[2000];
PROGMEM prog_int32_t intake_solenoid_pos[2000];
PROGMEM prog_uchar intake_solenoid_cmd[2000];
*/


int mode_toggle_SW1 = 7;	// Declare pins for BUTTONS
int mode_toggle_SW2 = 6;

int throttle_pin = 8;
int rpm_CMD;

int start_init; 		// SWITCH flags
int run_init;
int cam_init;

bool right_pressed = 0; // setting cursor with interrupt
bool left_pressed = 0;
bool up_pressed = 0;
bool down_pressed = 0;
bool enter_pressed = 0;

int cursor_position_col = 0;
int cursor_position_row = 0;
int ROW = 0;

int manual_exop = 0;
int manual_excl = 0;
int manual_inop = 0;
int manual_incl = 0;


int previous_ROW = 0;
int previous_curs_col = 0;
int previous_curs_row = 0;

//serial print stuff
//String serialprintstuff;

void setup() 
{ 	Serial.begin(115200); // for testing 500000 works
	//  LCD_setup();
// pinMode(A1,INPUT);      // analog inputting the potentiometer at the moment. We want this to be the crankshaft position sensor. This is what we're going to be using for testing
	spi_setup();

	pinMode(Exhaust_Solenoid, OUTPUT);
	pinMode(Intake_Solenoid, OUTPUT);

	clearEncoderCount();
	//Serial.println("starting test245");
	
	pinMode(mode_toggle_SW1, INPUT); 	// buttons
	pinMode(mode_toggle_SW2, INPUT);
	
	pinMode(throttle_pin, INPUT); 	//throttle
	pinMode(servo_pin, OUTPUT);
	pinMode(TDC_pin, OUTPUT);
	
	attachInterrupt(2,right_fun, RISING); // interupt	
	attachInterrupt(5, left_fun, RISING);		
	attachInterrupt(4, up_fun, RISING);			
	attachInterrupt(3, down_fun, RISING);		
	attachInterrupt(0, enter_fun, FALLING);				
	
	start_init = 1;
	run_init = 1;
	cam_init = 1;
	
// START-UP MODE
while (digitalRead(mode_toggle_SW1) == HIGH && digitalRead(mode_toggle_SW2) == LOW)   
	{		
		while (start_init) // display start lcd once
		{
			LCD_setup_start();
			start_init = 0;
		}
		LCD_START_UPDATE();

	//Serial.print("Inside_start,");			//Serial.print(digitalRead(mode_toggle_SW1));
	//Serial.print(",");
	//Serial.println(digitalRead(mode_toggle_SW2));
			
		if (down_pressed == 1)
{	digitalWrite(Intake_Solenoid,
!digitalRead(Intake_Solenoid));
down_pressed = 0;
			Serial.println("intake loop");
		}
		if (up_pressed == 1)
{	digitalWrite(Exhaust_Solenoid, 
				!digitalRead(Exhaust_Solenoid));
			up_pressed = 0;
			Serial.println("exhaust loop");

		}
		if (enter_pressed == 1)
{	clearEncoderCount();
			enter_pressed = 0;
			Serial.println("zeroed loop");
		}	



		static int throttle_pos = 0;
		//analogWrite(servo_pin, 
(analogRead(throttle_pin)/4));
//Serial.println((analogRead(throttle_pin)
/ 4));
		// include setting throttle to zero 
		throttle_pos = analogRead(throttle_pin);	
		throttle_pos = map(throttle_pos, 0, 1023, 
145,255 );    // input range from 0 to 1023 is converted to rpm scale from 0 to 2500
analogWrite(servo_pin, throttle_pos);

		// open valves					
		//Serial.println("high");
		//digitalWrite(Exhaust_Solenoid, HIGH);
		//digitalWrite(Intake_Solenoid, HIGH);
		delay(100);	
	}

// RUN MODE
	while (digitalRead(mode_toggle_SW1) == HIGH  && 
digitalRead(mode_toggle_SW2) == HIGH)  

		static unsigned long last_time_1 = 0;
		static unsigned long last_time_2 = 0;
		static unsigned long last_time_3 = 0;
		static int throttle_pos = 0;
		
		while (run_init)
			{	LCD_setup();
				run_init = 0;		
			}

		throttle_pos = analogRead(throttle_pin);

intop = map(throttle_pos, 0, 1023, 260, 400);    
// input range from 0 to 1023 is converted to rpm scale from 0 to 2500

		// analogWrite(servo_pin, throttle_pos);

		if ((RPM_START_TIME - (last_time_1)) > 500000)
{	// lcd.setCursor(11, 0);
			//lcd.print((int)result); // Start at 11 
			//lcd.print("   ");			
			// Serial.println(result);
			//Serial.print("last time:");
			//Serial.println((last_time_1));
			//last_time_1 = RPM_START_TIME;
			//Serial.print("rpm time:");
			//Serial.println(RPM_START_TIME);
			//Serial.println(crankpos);
			
update_LCD();

	// 2ms loop time -- need more data
		for (int x = 0; x <= 10; x++)
{	calc_crank();
			CALC_RPM();
			//Serial.println(INSTANT_RPM);
			valve_offset();
			//Serial.print("crankpos");
			//Serial.println(crankpos);
			valve_event();		
			//if (x == 0)	Serial.print(micros());
			//if (x == 1) Serial.print(',');
			//if (x== 2) Serial.print(crankpos);
			//if (x == 3) Serial.print(',');
			//if (x == 4) encoder1count = 
//			readEncoder(1);
			//if (x == 5) 
			//	Serial.println((encoder1count));
			//if (x == 6) Serial.print(',');
			//if (x == 7) encoder2count = 
			//	readEncoder(2);
			//if (x == 8) 
			//	Serial.println(encoder2count);
			//if (x == 9) Serial.print(',');
			//if (x == 10){
			// Serial.println(digitalRead(
Intake_Solenoid));		
			//	}
			//	*/		
		}

		if (enter_pressed == 1)
{	rollover = !rollover;
			enter_pressed = 0;
		}	

   /*  if ( update_720 = 1) 	// add = here 
		{	static int rpm_counter = 0;
			update_LCD();
			//Serial.println(crankpos);
			if (right_pressed || left_pressed || 
up_pressed || down_pressed)
			{	update_button_status();
				Serial.print("row: ");
      Serial.print(cursor_position_row);
			    Serial.print(", col ");
			    Serial.println(cursor_position_col);
			    update_cursor_location();
			}
		if (cursor_position_row >= 3 && enter_pressed)
		{ 	update_button_functions();	
}
//valve_offset(); // add rpm based offset for valve timing
		if (rpm_counter == 20)
{	throttle_pos();
			rpm_counter = 0;
		}
		rpm_counter++;
	}	*/		
	//count++;
	}
	
/// RUNNING ENGINE WITH CAMSHAFT
	while (digitalRead(mode_toggle_SW1) == LOW && 
digitalRead(mode_toggle_SW2) == HIGH
	{
		static unsigned long last_time_1=0;
		static unsigned long last_time_2=0;
		static unsigned long last_time_3=0;
		
		while (cam_init)
		{	LCD_setup_cam();
			cam_init = 0;
		}
	
		check_crank_pos();
		CALC_RPM();
		
		static int throttle_pos = 0;

//analogWrite(servo_pin, 
//	(analogRead(throttle_pin)/4));
		//Serial.println((analogRead(throttle_pin)/ 
//	4));		

		throttle_pos = analogRead(throttle_pin);


		throttle_pos = map(throttle_pos, 0, 1023, 145, 
255);    // input range from 0 to 1023 is converted to rpm scale from 0 to 2500

		analogWrite(servo_pin, throttle_pos);

		if ((RPM_START_TIME - (last_time_1)) > 500000)
{	lcd.setCursor(11, 0);               
			lcd.print((int)result); // start at 11 
			lcd.print("   ");			
			// Serial.println(result);
			Serial.print("last time:");
			Serial.println((last_time_1));
			last_time_1 = RPM_START_TIME;
			Serial.print("rpm time:");
			Serial.println(RPM_START_TIME);	
		}

		if (RPM_START_TIME > (last_time_2 + 570000))
{	lcd.setCursor(9, 1);               
			lcd.print(INSTANT_RPM); //instant rpm
			lcd.print("   ");
			//Serial.print("inst:");
			last_time_2 = RPM_START_TIME;
			//Serial.println(CALC_RPM(1));
		}

		if (RPM_START_TIME > (last_time_3 + 
			600000))
{	lcd.setCursor(9, 2); //PRINT AVERAGE RPM               
			lcd.print(AVERAGE_RPM); 
			lcd.print("   ");
			last_time_3 = RPM_START_TIME;
			//Serial.print("avg:");
			//Serial.println(CALC_RPM(0));
		}
	}
	
}		// end of main loop


//Functions

int loop_time()
{	static long last_time;
	static  long time = 0;
	static int count = 0;
	
	last_time = time;
	time = micros();

	Serial.println(time - last_time);	
}

double calc_crank()
{	last_crankpos = crankpos;
	crankpos = check_crank_pos();
			
	if (rollover) crankpos = crankpos + 360.0;
	if (((crankpos - last_crankpos) < -2))
{	if (rollover ) // if rollover from 720 to 360
{	crankpos = crankpos - 360;
			//calculate rpm	
rpm = (((720 + crankpos – 
rpm_crank_init)*500000.0) / 3.0)
 / ( - rpm_time_init);
//store initial time
			rpm_time_init = micros();
//store initial crankpos
			rpm_crank_init = crankpos;
	
			update_720 = 1;
			//update_LCD();			
				
		}
		if (!rollover)  // if rollover from 360 to 0
{ 	crankpos = crankpos + 360; 
}
		
rollover = !rollover;
	}
	/*
	if (crankpos > (TDC-2) && (crankpos < (TDC +2)))
{	digitalWrite(TDC_pin, HIGH);
		//Serial.print(crankpos);
		//Serial.println("high");
	}
	else {	digitalWrite(TDC_pin, LOW);
		//Serial.print(crankpos);
		//Serial.println("low");
	      }
	*/

return crankpos;
} 

int valve_offset() // calculate offset outside of valve event
{	CALC_RPM();
	
	added720Exhaustopen=0;
	added720Exhaustclose=0;

	offset_on = ((double)INSTANT_RPM * 360.0 * 13.0) / 
60000.0; // rpm in deg/ms with 16 ms delay 
	offset_off = ((double)INSTANT_RPM * 360.0 * 18.0 ) / 
60000.0; // rpm in deg/ms with 20 ms delay  

	/*
	Serial.print("rpm:");
	Serial.println(INSTANT_RPM);
	Serial.print("offset:");
	Serial.println(offset_off);
	*/

	// add offset to intake and exhaust
	intop_ACT = (intop + manual_inop) - offset_on;
// check if offset is negative
	if (intop_ACT < 0) {intop_ACT= 720 + intop_ACT;}

	intcl_ACT = (intcl + manual_incl)- offset_off;
	if (intcl_ACT < 0) {intcl_ACT= 720 + intcl_ACT;}

	exop_ACT = (exop + manual_exop) - offset_on;
	if (exop_ACT < 0)
{	exop_ACT= 720 + exop_ACT;
	added720Exhaustopen =1;
	}

	excl_ACT = (excl + manual_excl) - offset_off;
	if (excl_ACT < 0)
{	excl_ACT= 720 + excl_ACT;
		added720Exhaustclose =1;
	}

	return 0;
}

int valve_event()
{
	if ( (crankpos > intop_ACT) && (crankpos < intcl_ACT)) 
{ digitalWrite(Intake_Solenoid, HIGH); }
	else 	{ digitalWrite(Intake_Solenoid, LOW); }

	if (added720Exhaustopen && !added720Exhaustclose)
{	if ( crankpos > (exop_ACT) || crankpos < 
(excl_ACT) )
{digitalWrite(Exhaust_Solenoid, HIGH);}
		else 	{digitalWrite(Exhaust_Solenoid, LOW);}
	}
	else
{	if ( crankpos > (exop_ACT) && crankpos < 
(excl_ACT) )
{digitalWrite(Exhaust_Solenoid, HIGH);}
		else 	{digitalWrite(Exhaust_Solenoid, LOW); }
	}

	return 0;
}


void LCD_setup(){
		lcd.begin(20, 4);    // LCD 20 cols and 4 rows
		lcd.setCursor(0, 0);     
		lcd.print("RPM:");
		lcd.setCursor(11, 0);
		lcd.print("CMD:");
		lcd.setCursor(0, 1);                 
		lcd.print("EO:");
		lcd.print(exop);			
		lcd.setCursor(7, 1);		
		lcd.print("+0");// changes with interrupt	
		// manual change of exhaust opening set to 0 
manual_exop = 0;			 
		lcd.setCursor(10,1);
		lcd.print("EC:");
		lcd.print(excl);
		lcd.setCursor(17, 1);
		lcd.print("+0");				
		manual_excl = 0;
		lcd.setCursor(0, 2);               
		lcd.print("IO:");
		lcd.print(intop);				
		lcd.setCursor(7, 2); 
		lcd.print("+0");
		manual_inop = 0;		
		lcd.setCursor(10, 2);
		lcd.print("IC:");
		lcd.print(intcl);
		lcd.setCursor(17, 2);
		lcd.print("+0");
		manual_incl = 0;
		lcd.setCursor(0, 3);						lcd.print("   Functions   ");
}

void LCD_setup_cam()
{	lcd.begin(20, 4);          
	lcd.setCursor(0, 0);                
	lcd.print("CRANK POS:"); 
	lcd.setCursor(0, 1);
	lcd.print("INST RPM:");
	lcd.setCursor(0, 2);                 
	lcd.print("AVG RPM:");
}

void LCD_setup_start()
{	lcd.begin(20, 4);                    	lcd.setCursor(0, 0);                
	lcd.print("CRANK POS:");
	lcd.setCursor(0, 1);
	lcd.print("EX LIFT:");
	lcd.setCursor(0, 2);                 
	lcd.print("IN LIFT:");
	lcd.setCursor(0, 3);
	lcd.print("  Functions   ");
}

int update_LCD()
{	static unsigned int update = 0;	
	if(update = 1)
{	lcd.setCursor(4, 0);               
		lcd.print(INSTANT_RPM);
		lcd.print(" ");
	}
	if (update = 1)
	{	lcd.setCursor(15, 0);
		lcd.print((int)intop);
		lcd.print(" ");
	}
	//Serial.println(update % 4);
	update++;
	return 0;
}


int LCD_START_UPDATE()
{
	lcd.setCursor(10, 0);                
	lcd.print((int)check_crank_pos());
	lcd.print("   ");
	lcd.setCursor(8, 1);
	lcd.print(readEncoder(1));
	lcd.print("     ");
	lcd.setCursor(8, 2);                 
	lcd.print(readEncoder(2));
	lcd.print("     ");
	lcd.setCursor(0, 3);
	lcd.print("  Functions    ");
}


int max_lift(int lift)
{	if (lift > MaxL) { MaxL= lift; }
	return MaxL;
}


// setup for SPI communication -- used for angle sensor
void spi_setup()
{	// Set slave selects as outputs
	pinMode(SScounter1, OUTPUT);
	pinMode(SScounter2, OUTPUT);
	// Raise select pins
	// Communication begins when you drop the individual 
select signsl


SPI.begin();
SPI.setBitOrder(MSBFIRST);
SPI.setDataMode (SPI_MODE0) ;

// Initialize encoder 1
		// Clock division factor: 0
		// Negative index input
		// free-running count mode
		// x4 quatrature count mode (four counts per 
quadrature cycle)
		// NOTE: For info on commands, see datasheet
	digitalWrite(SScounter1,LOW); // Begin SPI convo
	SPI.transfer(0x88); // Write to MDR0
	SPI.transfer(0x03); // Configure to 4 byte mode
	digitalWrite(SScounter1,HIGH); // Terminate SPI convo
	// Initialize encoder 2
	// Clock division factor: 0
	// Negative index input
	// free-running count mode
	// x4 quatrature count mode (four counts per 
quadrature cycle)
	// NOTE: For info on commands, see datasheet
	digitalWrite(SScounter2,LOW); // Begin SPI convo
	SPI.transfer(0x88); // Write to MDR0
	SPI.transfer(0x03); // Configure to 4 byte mode
	digitalWrite(SScounter2,HIGH); // Terminate SPI convo

SPI.setDataMode (SPI_MODE1) ;
}


// read SPI signal for angle sensor
double check_crank_pos()
{ 	SPI.setDataMode (SPI_MODE1);
	//crank_position_last = result;
	RPM_START_TIME = micros();
	digitalWrite(SS, LOW);	

 	//Reading 8 bit frame 
// (ie. the first half of 16-bit SPI transfer)
 	result1 = SPI.transfer(0b00000000);
 	//Serial.print("byte 1: ");
  	//Serial.println(result1, BIN);
 
  	// removing (masking) first 2 bit
 	result1 &= 0b00111111;
// Serial.print("byte 1 masked: ");
 	// Serial.println(result1, BIN);
  
 	//shifting 8 bit to left to create empty space for 
//last 8 bit transfer
result1 = result1 << 8;
// Serial.print("byte 1 shifted: ");
  	//Serial.println(result, BIN);
  
  	// getting last 8 bits 
// (ie.the last half of 16-bit SPI transfer)
  	result2 = SPI.transfer(0b00000000);
  	//Serial.print("byte 2: ");
  	//Serial.println(result2, BIN);
  
  	// merging
  	result = result1 | result2;
 	// Serial.print("Result: ");
  	//Serial.print(result, BIN);
  	//Serial.print(" = ");
  	//Serial.println(result, DEC);
  	//Serial.println();
    
digitalWrite(SS, HIGH);

result = (abs((result*360.0 / 16384) - 360)+ 
Crankpos_offset);
//Serial.println(result);

if (result > 360) {result = result - 360;}
if (result > (TDC - 5) && (result < (TDC + 5)))
{	digitalWrite(TDC_pin, HIGH);
		//Serial.print(crankpos);
		//Serial.println("high");
}
else
{	digitalWrite(TDC_pin, LOW);
		//Serial.print(crankpos);
		//Serial.println("low");
}

return result; // convert to degrees
}


//RPM_TYPE 1 = INSTANT	
//RPM_TYPE 0 = AVERAGE
int CALC_RPM()
{	static double crank_position_last_avg = 0;
	static double RPM_START_TIME_LAST_AVG = 0;	

	if (RPM_START_TIME > (RPM_START_TIME_LAST +5000))
{ 	// IF 5MS HAVE PASSED

		static double INSTANT_RPM_STORE = 0;
		double delta_crank;

		delta_crank = result - crank_position_last;

		// CHECK FOR ROLLOVER
		if (delta_crank < 0) delta_crank += 360;
		if (delta_crank < 0.4)
{	INSTANT_RPM = 0;
			return 0; 
//if the crankshaft moved less than 10 
degrees, output 0 rpm
		}	

		INSTANT_RPM_STORE = INSTANT_RPM;
		INSTANT_RPM = ((delta_crank)*500000.0 / 3.0) /
      ((RPM_START_TIME - RPM_START_TIME_LAST));
		

		if ((abs(INSTANT_RPM - INSTANT_RPM_STORE))>200) 
{INSTANT_RPM = INSTANT_RPM_STORE;}
		
		//Serial.println(INSTANT_RPM);
		crank_position_last = result;
		RPM_START_TIME_LAST = RPM_START_TIME;

		return INSTANT_RPM;
	}
	
	if (RPM_START_TIME > (RPM_START_TIME_LAST_AVG + 
10000)) // for average rpm check every 10 ms
{ 	 double delta_crank;

		 delta_crank = result -crank_position_last_avg;
		 
		 // CHECK FOR ROLLOVER
		if (delta_crank < 0) delta_crank += 360; 
		if (delta_crank < 0.2) 
{	AVERAGE_RPM = 0;
			return 	0; 
//if the crankshaft moved less than 10 
degrees, output 0 rpm
		}

		 RPM_START_TIME_LAST_AVG = RPM_START_TIME;
		 crank_position_last_avg = result;

		 return AVERAGE_RPM;
	 }

	 return 0;
}


long readEncoder(int encoder) 
{	SPI.setDataMode (SPI_MODE0) ;

	// Initialize temporary variables for SPI read
	unsigned int count_1, count_2, count_3, count_4;
	long count_value;



	if (encoder == 1) // Read encoder 1 
{	digitalWrite(SScounter1,LOW); //Begin SPI convo
		SPI.transfer(0x60); // Request count
		count_1 = SPI.transfer(0x00); //Read highest 
order byte
		count_2 = SPI.transfer(0x00);
		count_3 = SPI.transfer(0x00);
		count_4 = SPI.transfer(0x00); // Read lowest 
							order byte
		digitalWrite(SScounter1,HIGH);//Term. SPI convo
	}	
	else if (encoder == 2) // Read encoder 2
{	digitalWrite(SScounter2,LOW); 
		SPI.transfer(0x60); 
		count_1 = SPI.transfer(0x00); 
		count_2 = SPI.transfer(0x00);
		count_3 = SPI.transfer(0x00);
		count_4 = SPI.transfer(0x00); 
		digitalWrite(SScounter2,HIGH); 
	}

	// Calculate encoder count
	count_value = (count_1 << 8) + count_2;
	count_value = (count_value << 8) + count_3;
	count_value = (count_value << 8) + count_4;
	return count_value;
}


void clearEncoderCount() 
{	SPI.setDataMode (SPI_MODE0) ;
	// Set encoder1's data register to 0
	digitalWrite(SScounter1,LOW); // Write to DTR
	SPI.transfer(0x98); // Load data
	SPI.transfer(0x00); 
	SPI.transfer(0x00);
	SPI.transfer(0x00);
	SPI.transfer(0x00); 
	digitalWrite(SScounter1,HIGH); 
// provides some breathing room between SPI convos
	delayMicroseconds(100); 
	// Set encoder1's current data register to center
	digitalWrite(SScounter1,LOW); // Begin SPI convo
	SPI.transfer(0xE0);
	digitalWrite(SScounter1,HIGH); 	
	// Set encoder2's data register to 0
	digitalWrite(SScounter2,LOW); 
	SPI.transfer(0x98);
	SPI.transfer(0x00); 
	SPI.transfer(0x00);
	SPI.transfer(0x00);
	SPI.transfer(0x00); 
	digitalWrite(SScounter2,HIGH);
	delayMicroseconds(100); 
	// Set encoder2's current data register to center
	digitalWrite(SScounter2,LOW); 
	SPI.transfer(0xE0);
	digitalWrite(SScounter2,HIGH); 
}

void right_fun() { right_pressed = 1; }

void left_fun()  { left_pressed = 1; }

void up_fun()    { up_pressed = 1; }

void down_fun()  { down_pressed = 1; } 
 
void enter_fun() { enter_pressed = 1; }

void update_button_status()
{
if (right_pressed)
	{ 	previous_curs_row = cursor_position_row;
	 	previous_curs_col = cursor_position_col;
		cursor_position_col += 1;
		if (cursor_position_col == 2)
			{cursor_position_col = 0; }		
right_pressed = 0;
	}
	else if (left_pressed)
	{	previous_curs_row = cursor_position_row;
		previous_curs_col = cursor_position_col;
		cursor_position_col -= 1;
		if (cursor_position_col < 0)
			{ cursor_position_col = 0;}				left_pressed = 0;}

else if (up_pressed)
	{	if (enter_pressed) {update_button_valve_up();}
else
{	previous_curs_row = cursor_position_row;
			previous_curs_col = cursor_position_col;
			cursor_position_row -= 1;
			if (cursor_position_row < 0)
			{cursor_position_row = 0;}
}
up_pressed = 0; 	
	}
	else if (down_pressed)
	{	if (enter_pressed){update_button_valve_down();}
else
		{	previous_curs_row = cursor_position_row;
			previous_curs_col = cursor_position_col;
			cursor_position_row += 1;
			if (cursor_position_row > 6)
				{cursor_position_row = 6;}
		}
		down_pressed = 0;
	}

	if (!enter_pressed)
	{	if (cursor_position_row == 3)
		{	lcd.setCursor(0, 3);
			lcd.print(" Test valve         ");
		}
		else if (cursor_position_row == 4)
		{	lcd.setCursor(0, 3);
			lcd.print("  function 2    ");
		}
		else if (cursor_position_row == 5)
		{	lcd.setCursor(0, 3);
			lcd.print("   function 3       ");
		}
	}
}
	


void update_cursor_location()
{
	static int arrow_pos[6][2] = { {6,6}, {6,16}, {6,16},					{0,0}, {0,0}, {0,0},};
	
	previous_ROW = ROW;
	ROW = cursor_position_row;
	if (ROW > 3)	{ ROW = 3; }
	lcd.setCursor((arrow_pos[previous_curs_row]
[previous_curs_col]), previous_ROW);
	lcd.print(' ');

	if (ROW >= 1)
	{	lcd.setCursor((arrow_pos[cursor_position_row]
[cursor_position_col]), ROW);
		lcd.print((char)126);
	}
}


void update_button_valve_up()
{	
	if (cursor_position_col==0 && cursor_position_row==1)
	{	manual_exop += 1; 	// EVO
		if (manual_exop < 0)	// negative sign			{	lcd.setCursor(7, 1);
			lcd.print(manual_exop);
		}
		else					
		{	lcd.setCursor(7, 1);
			lcd.print("+");
			lcd.print(manual_exop);
		}
	}
	else if(cursor_position_col==1 && 
cursor_position_row==1)
	{	manual_excl += 1;
	
		if (manual_excl < 0)
		{	lcd.setCursor(17, 1);
			lcd.print(manual_excl);
		}
		else
		{
			lcd.setCursor(17, 1);
			lcd.print("+");
			lcd.print(manual_excl);
		}
	}
	else if (cursor_position_col == 0 && 
cursor_position_row == 2)
	{	manual_inop += 1;

		if (manual_inop < 0)
		{	lcd.setCursor(7, 2);
			lcd.print(manual_inop);
		}
		else
		{	lcd.setCursor(7, 2);
			lcd.print("+");
			lcd.print(manual_inop);
		}
	}
	else if (cursor_position_col == 1 && 
cursor_position_row == 2)
	{	manual_incl += 1;

		if (manual_incl < 0)
		{	lcd.setCursor(17, 2);
			lcd.print(manual_incl);
		}
		else
		{	lcd.setCursor(17, 2);
			lcd.print("+");
			lcd.print(manual_incl);
		}
	}

	enter_pressed = 0;	
}


void update_button_valve_down()
{
	if (cursor_position_col==0 && cursor_position_row==1)
	{	manual_exop -= 1;

		if (manual_exop < 0)				
		{	lcd.setCursor(7, 1);
			lcd.print(manual_exop);
		}
		else						
		{	lcd.setCursor(7, 1);
			lcd.print("+");
			lcd.print(manual_exop);
		}
	}
	else if (cursor_position_col == 1 && 
cursor_position_row == 1)
	{	manual_excl -= 1;

		if (manual_excl < 0)
		{	lcd.setCursor(17, 1);
			lcd.print(manual_excl);
		}
		else
		{	lcd.setCursor(17, 1);
			lcd.print("+");
			lcd.print(manual_excl);
		}
	}
	else if (cursor_position_col == 0 && 
cursor_position_row == 2)
	{	manual_inop -= 1;

		if (manual_inop < 0)
		{	lcd.setCursor(7, 2);
			lcd.print(manual_inop);
		}
		else
		{	lcd.setCursor(7, 2);
			lcd.print("+");
			lcd.print(manual_inop);
		}
	}

	else if (cursor_position_col == 1 && 
cursor_position_row == 2)
	{	manual_incl -= 1;

		if (manual_incl < 0)
		{	lcd.setCursor(17, 2);
			lcd.print(manual_incl);
		}
		else
		{	lcd.setCursor(17, 2);
			lcd.print("+");
			lcd.print(manual_incl);
		}
	}
	
	enter_pressed = 0;
}


void update_button_functions()
{
	if (cursor_position_row == 3) // do function 1
	{	Serial.println(" entered 3");
		lcd.setCursor(0, 3);
		lcd.print(" Running Test ");

		test_valve_pos();
		lcd.setCursor(0, 3);
		lcd.print(" Test valve   ");
		}
	else if (cursor_position_row == 4) // do function 2
	{	Serial.println(" entered 4 ");
		lcd.setCursor(0,3); 
		lcd.print("     Team 21     ");
	}
	else if (cursor_position_row == 5) // do function 3
	{	Serial.print(" entered 5 ");
		lcd.setCursor(0, 3);
		lcd.print("     Team 21     ");
	}

	enter_pressed = 0;
}

void test_valve_pos() 	// TESTING
{
	long* Time_us = 0;
	int* Crank_deg = 0;
	int* Intake_valve_pos_ACT = 0;
	bool* Intake_valve_pos_CMD = 0;
	int* Exhaust_valve_pos_ACT = 0;
	bool* Exhaust_valve_pos_CMD = 0;
	
	int x = 0; 		// clear encoder count
	long start_time;

	const int Data_Size = 500;
	Time_us = new long[Data_Size]; 
// cast some items to int to reduce size
	Crank_deg = new int[Data_Size];
	Intake_valve_pos_ACT = new int[Data_Size];
	Intake_valve_pos_CMD = new bool[Data_Size];
	Exhaust_valve_pos_ACT = new int[Data_Size];
	Exhaust_valve_pos_CMD = new bool[Data_Size];


	// check allocation of memory
	if (Time_us == NULL) 
{ 	Serial.println("1memory allocatiopn error");
return;
}
	if (Crank_deg == NULL)  
{ 	Serial.println("2memory allocatiopn error"); 
return; 
}
	if (Intake_valve_pos_ACT == NULL) 
{ 	Serial.println("3memory allocatiopn error"); 
return; 
}
	if (Intake_valve_pos_CMD == NULL)  
{ 	Serial.println("4memory allocatiopn error"); 
return; 
}
	if (Exhaust_valve_pos_ACT == NULL)  
{ 	Serial.println("5memory allocatiopn error"); 
return; 
}

	if (Exhaust_valve_pos_CMD == NULL)  
{ 	Serial.println("6memory allocatiopn error"); 
return; 
}
	
	while(crankpos != 650) //check size
{	calc_crank();
		valve_offset();
		valve_event();
	}

	clearEncoderCount();

	calc_crank();
	valve_offset();
	valve_event();
	
	start_time = micros();
	Time_us[x] = micros() - start_time;
	Crank_deg[x] = crankpos;
	Intake_valve_pos_ACT[x] = (int)readEncoder(2); 	Intake_valve_pos_CMD[x]=
(bool)digitalRead(Intake_Solenoid);
	Exhaust_valve_pos_ACT[x] = (int)readEncoder(1);
	Exhaust_valve_pos_CMD[x] = 
(bool)digitalRead(Exhaust_Solenoid);
	x++;

	for ( x ; x < Data_Size; )
{	calc_crank();
		valve_offset();
		valve_event();
		
		if ( abs((crankpos - (Crank_deg[x - 1]))) > 1)
{	Time_us[x] = micros() - start_time;
			Crank_deg[x] = crankpos;
			Intake_valve_pos_ACT[x] = 
     (int)readEncoder(2);
			Intake_valve_pos_CMD[x] = 
     (bool)digitalRead(Intake_Solenoid);
Exhaust_valve_pos_ACT[x] =    
     (int)readEncoder(1);
			Exhaust_valve_pos_CMD[x] = 
    (bool)digitalRead(Exhaust_Solenoid);
			x++;
		}	
	}
	
	for (x = 0; x < Data_Size; x++)
{	for ( int j = 0; j <= 10 ; j++)
{  calc_crank();
		   valve_offset();
		   valve_event();
		   if (j == 0)	Serial.print(Time_us[x]);
		   if (j == 1) Serial.print(',');
		   if (j == 2) Serial.print(Crank_deg[x]);
		   if (j == 3) Serial.print(',');
		   if (j == 4) 
                    {Serial.print(Exhaust_valve_pos_ACT[x]);}
		   if (j == 5) Serial.print(',');
	   	   if (j == 6) 
     {Serial.print(Exhaust_valve_pos_CMD[x]);}
		   if (j == 7) Serial.print(',');
		   if (j == 8) 
     {Serial.print(Intake_valve_pos_ACT[x]);}
		   if (j == 9) Serial.print(',');
		   if (j == 10)
     {Serial.println(Intake_valve_pos_CMD[x]);}
}	
	}

	calc_crank();
	valve_offset();
	valve_event();

	Serial.print(" , , ,");
	Serial.println(INSTANT_RPM);

	if (Time_us!=NULL){delete Time_us; Time_us = NULL;}
	else{ Serial.println("delete error");}

	if (Crank_deg!=NULL){delete Crank_deg;Crank_del=NULL;}
	else{ Serial.println("delete error"); }

	if (Intake_valve_pos_ACT != NULL) 
{	delete Intake_valve_pos_ACT; 
Intake_valve_pos_ACT = NULL; 
}
	else{ Serial.println("delete error"); }

	if (Intake_valve_pos_CMD != NULL)  
{ 	delete Intake_valve_pos_CMD; 
	Intake_valve_pos_CMD = NULL; 
}
	else{ Serial.println("delete error");}

	if (Exhaust_valve_pos_ACT != NULL) 
{ 	delete Exhaust_valve_pos_ACT;
Exhaust_valve_pos_ACT = NULL; 
}
	else{ Serial.println("delete error"); }

	if (Exhaust_valve_pos_CMD != NULL)  
{ 	delete Exhaust_valve_pos_CMD; 
	Exhaust_valve_pos_CMD = NULL; 
}
	else{ Serial.println("delete error"); }

	calc_crank();
	valve_offset();
	valve_event();
}


void throttle_pos()
{
	rpm_CMD = analogRead(throttle_pin); 
	static int servo_angle = 145;

	rpm_CMD = map(rpm_CMD, 0, 1023, 0, 2500); 

	if(rpm < rpm_CMD)	servo_angle--;	
	if (rpm > rpm_CMD)	servo_angle++;  

	if (servo_angle < 145) servo_angle = 145;
	if (servo_angle > 255) servo_angle = 255;
	
	analogWrite(servo_pin, servo_angle);	

	//Serial.print("servo:");
	//Serial.println(servo_angle);
}
