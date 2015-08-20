// STARTRACKER MOTOR CONTROL: STEPPER MOTOR CONTROL FOR JJROBOTS POV DISPLAY
// This code is designed for JJROBOTS arDusplay Stepper Motor Control board
// Author: JJROBOTS.COM (Jose Julio & Juan Pedro)
// Licence: GNU GPL
// Stepper : NEMA17
// Driver : A4988 or DRV8825
// Microstepping : configured to 1/16
// Arduino board: Pro micro (leonardo equivalent)

// STEPPER DRIVER CONNECTED TO:
// ENABLE pin: D10   (PB6)
// DIR pin:  D9      (PB5) 
// STEP pin: D8      (PB4)
// Button1 : START/STOP  D4
// Button2 : DEC   D5
// Button3 : CHANGE ROTATION DIRECTION (go back to original position)  D6

//DO NO TAKE PHOTOS WITH AN EXPOSITION LONGER THAN 5 MINUTES. THE DRIFT WOULD BE NOTICEABLE.

//Theory behind this CODE
//-----------------------------------------
// 360º (rotation of the Earth every 1436min)
// *Using a M8 rod coming up 1.25mm every complete rotation

#define COMINGUPSPEED 1.25  //milimeters that the rod comes up every complete rotation (360º). In a M8 rod/bolt is usually 1.25 mm. In a M6, only 1.00mm


//other info needed:
//ratio between the large gear and the small one=0.2549

//MEASURE THIS VALUE AS GOOD AS YOU CAN AND SET THE LENGHT BELOW
#define LENGTH 228 //distance from the centre of the hinge to the centre of the hole for the rod in milimiters


// Calculus here:
#define STEP ((2*3.14159)/1436)*LENGTH //rotational velocity of the small gear

#define RPS (STEP/(60*0.2549))/COMINGUPSPEED //rotational velocity of the large gear


#define ZERO_SPEED 65535
#define STEPS_PER_REV 3200     // 200 steps motor with 1/16 microstepping
#define MAX_RPM (RPS*60.0)   

// BIT functions
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

//uint16_t rpm;
float rpm;
uint16_t period;
uint16_t userCommand=0;
uint8_t motor_enable;

// TIMER 1: STEP INTERRUPT
ISR(TIMER1_COMPA_vect)
{
  if (motor_enable)
    {
    SET(PORTB,4);
    delayMicroseconds(2);
    CLR(PORTB,4);
    }
}

void setRpm()
{
  float temp;
  if (rpm == 0)
  {
    ICR1 = ZERO_SPEED;
    digitalWrite(10,HIGH);  // Disable motor
  }
  else
  {
    digitalWrite(10,LOW);  // Enable motor
 /*   if (rpm<8)
      rpm = 8;*/
    if (rpm>MAX_RPM)
      rpm = MAX_RPM;
    temp = (rpm/60.0)*STEPS_PER_REV;
    temp = 2000000 / temp;          //  2000000 = (16000000/8) timer1 16Mhz with 1/8 preescaler
    if (period<600000)
      period=60000;
    period = temp;
    while (TCNT1 < 30);   // Wait until a pulse to motor has finished
    //cli();
    ICR1 = period; //+ userCommand;
    if (TCNT1 > ICR1)     // Handle when we need to reset the timer
      TCNT1=0;
    //sei();
  }
}

void setup()
{
 
  pinMode(7,OUTPUT);    // LED pin
  pinMode(8,OUTPUT);    // STEP pin
  pinMode(9,OUTPUT);    // DIR pin
  pinMode(10,OUTPUT);   // ENABLE pin
  
  // Button input with pullups enable
  pinMode(4,INPUT_PULLUP);
  pinMode(5,INPUT_PULLUP);
  pinMode(6,INPUT_PULLUP);

  digitalWrite(10,HIGH);  // Disable motor
  digitalWrite(9,HIGH);    // Motor direction

  Serial.begin(115200);

  digitalWrite(7,HIGH);
  delay(200);    // Initial delay
  Serial.println("ArduPOV MOTOR Stepper motor driver v1.0");
  digitalWrite(7,LOW);

  motor_enable = 0;
  
  // PWM SETUP
  // Fast PWM mode => TOP:ICR1
  TCCR1A =(1<<WGM11);           
//  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS10);   //No Prescaler, Fast PWM
   TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);   // Prescaler 1:8, Fast PWM
  ICR1 = ZERO_SPEED;
  TIMSK1 = (1<<OCIE1A);  // Enable Timer interrupt
  
  rpm = 0;
  
  while (digitalRead(4)==HIGH);    // Wait until START button is pressed
  motor_enable = 1;
  delay(250);
  while (digitalRead(4)==LOW);
}

void loop()
{ 
  if (digitalRead(4)==LOW)   // START/STOP Button pressed?
    {
    rpm = 0;
    userCommand=0;
    setRpm();
    if (motor_enable == 1)
      motor_enable = 0;
    else
      motor_enable = 1;
    while (digitalRead(4)==LOW);   // Wait until botton release
    }
    
  if (digitalRead(6)==LOW)   //  Button 3 pressed?
	{
         Serial.println("Coming back");
	
        digitalWrite(9,HIGH);    // Motor direction

          rpm=50; 
        setRpm();
	
        if (motor_enable == 1)
		motor_enable = 0;
	else
		motor_enable = 1;
         
	while (digitalRead(6)==LOW);   // Wait until botton release
	}
    
    
  if (motor_enable)
    {
    rpm++;
    digitalWrite(7,HIGH);
    }
  else
    {
    rpm = 0;
    digitalWrite(7,LOW);
    }
  Serial.print("RPM:");
  Serial.print(rpm);
  Serial.print(" ");
  Serial.println(period+userCommand);
  setRpm();
    Serial.print("100xRPS large gear:");
    Serial.print(RPS/0.2549);
    Serial.print(" ");
    Serial.print("STEP:");
    Serial.print(STEP);
    Serial.print(" ");
  delay(10);
  
  if (digitalRead(5)==LOW)   // Decrease button
    {
    digitalWrite(7,LOW);
    userCommand--;
    while (digitalRead(5)==LOW);  // Wait until released
    }
  if (digitalRead(6)==LOW)   // Increase button
    {
    digitalWrite(7,LOW);
    userCommand++;
    while (digitalRead(6)==LOW);  // Wait until released
    }
}

