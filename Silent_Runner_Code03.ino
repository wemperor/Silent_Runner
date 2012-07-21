//02 Smoothing embedded
//03 flying curves by the motor thrust included
//03 blinking LED on Pin 8 included

#include <Servo.h>

Servo MotorL;  //Motor left
Servo MotorR;  //Motor right
Servo ServoL;  //Servo fin stern left
Servo ServoR;  //Servo fin stern right
Servo ServoU;  //Servo fin stern up

#define CH1pin 2 //CH1 = Thrust Channel
#define CH2pin 4 //CH2 = Left/Right Channel
#define CH3pin 7 //CH3 = Up/Down Channel

#define MotorLpin 3 //Motor left
#define MotorRpin 5 //Motor right
#define ServoLpin 6 //Servo fin stern left
#define ServoRpin 9 //Servo fin stern right
#define ServoUpin 10 //Servo fin stern up

#define LEDpin 8 //define the pin for blinking LED´s

// stuff for value correction via averaging
const int numReadings = 4; //chose high values for smoother but slower control
double readings_CH1[numReadings];      // the readings from the analog input
double readings_CH2[numReadings];
double readings_CH3[numReadings];
int index = 0;             

//variables for the maneuver calculations
double Thrust;
double Rotation;
double Xaxis; 

//variables for the LED Blinking
long previousMillis = 0; 
long interval = 1000; 
int ledState = LOW;

int MotorLout = 10; //Motor left
int MotorRout = 10; //Motor right
int ServoLout = 90; //Servo fin stern left
int ServoRout = 90; //Servo fin stern right
int ServoUout = 90; //Servo fin stern up

void setup()
{

  MotorL.attach(MotorLpin);//attaches the Servos and ESC´s
  MotorR.attach(MotorRpin);
  ServoL.attach(ServoLpin);
  ServoR.attach(ServoRpin);
  ServoU.attach(ServoUpin);

  pinMode(CH1pin, INPUT); //defines the input pins
  pinMode(CH2pin, INPUT);
  pinMode(CH3pin, INPUT);
  
  pinMode(LEDpin, OUTPUT);
  
  //setting the smoothing arrays to zero
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings_CH1[thisReading] = 0.0;
    readings_CH2[thisReading] = 0.0;
    readings_CH3[thisReading] = 0.0;
  }

}

void loop()
{

  Thrust -= readings_CH1[index]; //smoothing stuff
  Rotation -= readings_CH2[index];
  Xaxis -= readings_CH3[index];

  double Thrust_tmp = (((long)pulseIn(CH1pin, HIGH))-1500) / 500.0; //retrieves the receiver signals and maps to -1 to 1
  double Rotation_tmp = (((long)pulseIn(CH2pin, HIGH))-1500) / 500.0; 
  double Xaxis_tmp = (((long)pulseIn(CH3pin, HIGH))-1500) / 500.0; 

  Thrust += Thrust_tmp;
  Rotation += Rotation_tmp;
  Xaxis += Xaxis_tmp;

  readings_CH1[index] = Thrust_tmp;
  readings_CH2[index] = Rotation_tmp;
  readings_CH3[index] = Xaxis_tmp;
  index = (index + 1) % numReadings;

  
  //flying curves including the motorthrust
  double mLeft = 0;
  double mRight = 0;
  
  if (Rotation > 0.1) {mLeft = Rotation;}
  if (Rotation < -0.1) {mRight = Rotation;}
  
  MotorLout = ((Thrust - mLeft) * 90);
  MotorRout = ((Thrust + mRight) * 90);

  /* 
   
   Control Matrix for inverted Y fins
   
               | Rotation  |  Xaxis  |
   ServoLeft   |    +      |    +    |
   ServoRight  |    +      |    -    |
   ServoUp     |    -      |   N/A   |
   
   */

  ServoLout = (((0.5 * Rotation) + Xaxis) * 90);

  ServoRout = (((0.5 * Rotation) - Xaxis) * 90);

  ServoUout = (-Rotation * 90);


  MotorL.write(forServo(MotorLout));
  MotorR.write(forServo(MotorRout));
  ServoL.write(forServo(ServoLout));
  ServoR.write(forServo(ServoRout));
  ServoU.write(forServo(ServoUout));

//led blinking
unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(LEDpin, ledState);
  }
}

inline long forServo(int s) {

  byte MIN_A = 10;
  byte MAX_A = 170;
  byte MAX_DEG = 90;  //max range of the servos

  if (s >= MAX_DEG) {
    s = MAX_DEG;
  } //Setzt 90Â° als Obergrenze fÃ¼r den Servoausschlag
  if (s <= -MAX_DEG) {
    s = -MAX_DEG;
  } //Setzt -90Â° als Untergrenze fÃ¼r den Servoausschlag
  return map(s,-MAX_DEG,MAX_DEG,MIN_A,MAX_A); //bringt die Werte in den korrekten Bereich von -90 bis 90 zu 10 bis 170

}




