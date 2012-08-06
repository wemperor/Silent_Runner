#include <Servo.h>

//defines the servo object
Servo Motors;  //Motor left
Servo ServoL;  //Servo fin stern left
Servo ServoR;  //Servo fin stern right
Servo ServoU;  //Servo fin stern up

#define CH1pin 2 //CH1 = Thrust Channel
#define CH2pin 4 //CH2 = Left/Right Channel
#define CH3pin 7 //CH3 = Up/Down Channel

//defines the servo pins, they must be PWM
#define Motorspin 3 //Motor left
#define ServoLpin 5 //Servo fin stern left
#define ServoRpin 6 //Servo fin stern right
#define ServoUpin 9 //Servo fin stern up

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

int Motorsout = 10; //Motor left
int ServoLout = 90; //Servo fin stern left
int ServoRout = 90; //Servo fin stern right
int ServoUout = 90; //Servo fin stern up

//max range of the servos
#define MIN_A 10
#define MAX_A 170
#define MAX_DEG 90  

#define A500TO90FACT 0.18

void setup()
{

  Motors.attach(Motorspin);//attaches the Servos and ESC´s
  ServoL.attach(ServoLpin);
  ServoR.attach(ServoRpin);
  ServoU.attach(ServoUpin);

  pinMode(CH1pin, INPUT); //defines the input pins
  pinMode(CH2pin, INPUT);
  pinMode(CH3pin, INPUT);

  //setting the smoothing arrays to zero
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings_CH1[thisReading] = 0.0;
    readings_CH2[thisReading] = 0.0;
    readings_CH3[thisReading] = 0.0;
  }
} //setup end

void loop()
{
  Thrust -= readings_CH1[index]; //smoothing stuff
  Rotation -= readings_CH2[index];
  Xaxis -= readings_CH3[index];

  double Thrust_tmp = (((long)pulseIn(CH1pin, HIGH))-1500) * A500TO90FACT; //retrieves the receiver signals and maps to -90 to 90
  double Rotation_tmp = (((long)pulseIn(CH2pin, HIGH))-1500) * A500TO90FACT; 
  double Xaxis_tmp = (((long)pulseIn(CH3pin, HIGH))-1500) * A500TO90FACT; 

  Thrust += Thrust_tmp;
  Rotation += Rotation_tmp;
  Xaxis += Xaxis_tmp;

  readings_CH1[index] = Thrust_tmp;
  readings_CH2[index] = Rotation_tmp;
  readings_CH3[index] = Xaxis_tmp;
  index = (index + 1) % numReadings;

  Motorsout = (br(Thrust));

  /* Control Matrix for inverted Y fins
   
               | Rotation  |  Xaxis  |
   ServoLeft   |    +      |    +    |
   ServoRight  |    +      |    -    |
   ServoUp     |    -      |   N/A   |     */

  ServoLout = ((0.5 * br(Rotation)) + br(Xaxis)); //calculation of maneuvers
  ServoRout = ((0.5 * br(Rotation)) - br(Xaxis));
  ServoUout = (-br(Rotation));

  Motors.write(forServo(Motorsout)); //sends commando to the servos
  ServoL.write(forServo(ServoLout));
  ServoR.write(forServo(ServoRout));
  ServoU.write(forServo(ServoUout));

} //loop end


// by readings 
inline double br(double in) {
  return in / numReadings;
}


inline long forServo(int s) {
  if (s >= MAX_DEG) {
    s = MAX_DEG;
  } //set´s MAX_DEG as maximum for the servo movement
  if (s <= -MAX_DEG) {
    s = -MAX_DEG;
  } //set´s -MAX_DEG as minimum for the servos
  return map(s,-MAX_DEG,MAX_DEG,MIN_A,MAX_A); //maps the values from -90 to 90 to the necessary output 10 to 170
}




