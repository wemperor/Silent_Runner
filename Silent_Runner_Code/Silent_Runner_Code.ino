#include <Servo.h>

//defines the servo object
Servo ServoL;  //Servo fin stern left
Servo ServoR;  //Servo fin stern right
Servo ServoU;  //Servo fin stern up

#define CH2pin 4 //CH2 = Left/Right Channel
#define CH3pin 7 //CH3 = Up/Down Channel

//defines the servo pins, they must be PWM
#define ServoLpin 5 //Servo fin stern left
#define ServoRpin 6 //Servo fin stern right
#define ServoUpin 9 //Servo fin stern up

           

//variables for the maneuver calculations
double Rotation;
double Xaxis; 


int ServoLout = 90; //Servo fin stern left
int ServoRout = 90; //Servo fin stern right
int ServoUout = 90; //Servo fin stern up

//max range of the servos
#define MIN_A 10
#define MAX_A 170
#define MAX_DEG 500  
#define MAX_DEG_SQUARE 250000
#define MIN_PWM 1500

// divisor for correcting rudder 
#define YCORRECTDIV 2

// stuff for value correction via averaging
#define NUMREADINGS 8 //chose high values for smoother but slower control

int readings_CH2[ NUMREADINGS ] ;
int readings_CH3[ NUMREADINGS ] ;
int index = 0;  

void setup()
{

  ServoL.attach(ServoLpin);
  ServoR.attach(ServoRpin);
  ServoU.attach(ServoUpin);

  pinMode(CH2pin, INPUT);
  pinMode(CH3pin, INPUT);

  //setting the smoothing arrays to zero
  for (byte thisReading = 0; thisReading < NUMREADINGS; thisReading++) {
    readings_CH2[thisReading] = 0.0;
    readings_CH3[thisReading] = 0.0;
  }
} //setup end

void loop()
{
  Rotation -= readings_CH2[index];
  Xaxis -= readings_CH3[index];

  // values range from -500 to 500
  int Rotation_tmp = -(((long)pulseIn(CH2pin, HIGH))-MIN_PWM); 
  int Xaxis_tmp = (((long)pulseIn(CH3pin, HIGH))-MIN_PWM); 

  Rotation += Rotation_tmp;
  Xaxis += Xaxis_tmp;

  readings_CH2[index] = Rotation_tmp;
  readings_CH3[index] = Xaxis_tmp;
  index = (index + 1) % NUMREADINGS;

   /* Control Matrix for inverted Y fins
   
               | Rotation  |  Xaxis  |
   ServoLeft   |    +      |    +    |
   ServoRight  |    +      |    -    |
   ServoUp     |    -      |   N/A   |     */
  
  ServoLout = ((br(Rotation) / YCORRECTDIV) + cubic(br(Xaxis))); //calculation of maneuvers
  ServoRout = ((br(Rotation) / YCORRECTDIV) - cubic(br(Xaxis)));
  ServoUout = (-br(Rotation));

  ServoL.write(forServo(ServoLout));
  ServoR.write(forServo(ServoRout));
  ServoU.write(forServo(ServoUout));

} //loop end

// cubic function to fit into [MIN_DEG, MAX_DEG]
inline int cubic(double in) {
 return (in * in * in) / MAX_DEG_SQUARE;
}
// by readings 
inline int br(double in) {
  return in / NUMREADINGS;
}


inline long forServo(int s) {
  if (s >= MAX_DEG) {
    s = MAX_DEG;
  } //setÂ´s MAX_DEG as maximum for the servo movement
  if (s <= -MAX_DEG) {
    s = -MAX_DEG;
  } //setÂ´s -MAX_DEG as minimum for the servos
  return map(s,-MAX_DEG,MAX_DEG,MIN_A,MAX_A); //maps the values from -90 to 90 to the necessary output 10 to 170
}
