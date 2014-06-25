#include <Servo.h>
// download at https://github.com/GreyGnome/PinChangeInt/archive/master.zip
#include <PinChangeInt.h>

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
int Rotation;
int Xaxis; 

// interrupt stuff
volatile uint8_t updateFlagsShared;

volatile uint16_t vRotation;
volatile uint16_t vXaxis;

uint32_t ulRotationStart;
uint32_t ulXaxisStart;

#define ROTATION_FLAG 1
#define XAXIS_FLAG 2


uint8_t updateChannel;

int ServoLout = 1500; //Servo fin stern left
int ServoRout = 1500; //Servo fin stern right
int ServoUout = 1500; //Servo fin stern up

//max range of the servos

#define MAX_DEG 500  
#define MIN_PWM 1500

// divisor for correcting rudder 
#define YCORRECTDIV 2
#define UPCORRECTDIV 0.7


void setup()
{

  ServoL.attach(ServoLpin);
  ServoR.attach(ServoRpin);
  ServoU.attach(ServoUpin);

  pinMode(CH2pin, INPUT);
  pinMode(CH3pin, INPUT);
  
  Xaxis = 0;
  Rotation = 0;
  

  PCintPort::attachInterrupt(CH2pin, calcRotation,CHANGE);
  PCintPort::attachInterrupt(CH3pin, calcXaxis,CHANGE);
  
} //setup end

void loop()
{
  static uint16_t vRotationIn;
  static uint16_t vXaxisIn;
   
  static uint8_t updateFlags;

  if(updateFlagsShared)
  {
    noInterrupts(); 

    
    updateFlags = updateFlagsShared;
    if(updateFlags & ROTATION_FLAG)
    {
      vRotationIn = vRotation;
    }

    if(updateFlags & XAXIS_FLAG)
    {
      vXaxisIn = vXaxis;
    }
  
    updateFlagsShared = 0;

    interrupts(); 
  }

  if(updateFlags & ROTATION_FLAG) {
    Rotation = -(vRotationIn-MIN_PWM); 
    
    updateChannel |= ROTATION_FLAG;
  }

  if(updateFlags & XAXIS_FLAG) {
    Xaxis = (vXaxisIn-MIN_PWM);

    updateChannel |= XAXIS_FLAG;
  }
  
  updateFlags = 0;
  
  updateChannel = 0;

    /* Control Matrix for inverted Y fins
     
     | Rotation  |  Xaxis  |
     ServoLeft   |    +      |    +    |
     ServoRight  |    +      |    -    |
     ServoUp     |    -      |   N/A   |     */

    ServoLout = (Rotation / YCORRECTDIV) + Xaxis; //calculation of maneuvers
    ServoRout = (Rotation / YCORRECTDIV) - Xaxis;
    ServoUout = -(Rotation / UPCORRECTDIV);

    ServoL.writeMicroseconds(forServo(ServoLout));
    ServoR.writeMicroseconds(forServo(ServoRout));
    ServoU.writeMicroseconds(forServo(ServoUout));
  

} //loop end


// interrupt routines
void calcRotation() {
  if(digitalRead(CH2pin) == HIGH)
  { 
    ulRotationStart = micros();
  }
  else
  { 
    vRotation = (uint16_t)(micros() - ulRotationStart);
    updateFlagsShared |= ROTATION_FLAG;
  }
}

void calcXaxis() {
  if(digitalRead(CH3pin) == HIGH)
  { 
    ulXaxisStart = micros();
  }
  else
  { 
    vXaxis = (uint16_t)(micros() - ulXaxisStart);
    updateFlagsShared |= XAXIS_FLAG;
  }
}

inline long forServo(int s) {
  if (s >= MAX_DEG) {
    s = MAX_DEG;
  } //setÂ´s MAX_DEG as maximum for the servo movement
  if (s <= -MAX_DEG) {
    s = -MAX_DEG;
  } //setÂ´s -MAX_DEG as minimum for the servos
  return s + MIN_PWM; //maps the values from -90 to 90 to the necessary output 10 to 170
}

