#include <AFMotor.h>      //add Adafruit Motor Shield library
#include <Servo.h>        //add Servo Motor library            
#include <NewPing.h>      //add Ultrasonic sensor library

#define TRIG_PIN 11 // Pin A0 on the Motor Drive Shield soldered to the ultrasonic sensor
#define ECHO_PIN 10 // Pin A1 on the Motor Drive Shield soldered to the ultrasonic sensor

#define MAX_DISTANCE 300 // sets maximum useable sensor measuring distance to 300cm

#define COLL_DIST 15 // sets distance at which robot stops and reverses to 30cm
#define TURN_DIST COLL_DIST+20 // sets distance at which robot veers away from object

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // sets up sensor library to use the correct pins to measure distance.
//Mert Arduino https://bit.ly/MertArduino

/// MOTOR A 
  const int motorA1  = 2;  
  const int motorA2  = 3;  

//// MOTOR B  
  const int motorB1  = 4; 
  const int motorB2  = 5;  

/// MOTOR C
  const int motorC1  = 6;  
  const int motorC2  = 7;  

//// MOTOR D  
  const int motorD1  = 8; 
  const int motorD2  = 9; 
  
Servo myservo;  // create servo object to control a servo 

int leftDistance, rightDistance; //distances on either side
int curDist = 0;


//-------------------------------------------- SETUP LOOP ----------------------------------------------------------------------------
void setup() { //Mert Arduino https://bit.ly/MertArduino
  myservo.attach(12);  // attaches the servo on pin 10 (SERVO_1 on the Motor Drive Shield to the servo object 
  myservo.write(90); // tells the servo to position at 90-degrees ie. facing forward.
  delay(1000); // delay for one seconds


      //Set pins as outputs
    pinMode(motorA1, OUTPUT);
    pinMode(motorA2, OUTPUT);
    
    pinMode(motorB1, OUTPUT);
    pinMode(motorB2, OUTPUT);

    pinMode(motorC1, OUTPUT);
    pinMode(motorC2, OUTPUT);

    pinMode(motorD1, OUTPUT);
    pinMode(motorD2, OUTPUT);
    Serial.begin(9600);
 }
//------------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------MAIN LOOP ------------------------------------------------------------------------------
void loop() {
   myservo.write(90);  // move eyes forward
   delay(90);
  curDist = readPing();   // read distance
  if (curDist < COLL_DIST) {changePath();}  // if forward is blocked change direction
  moveForward();  // move forward
  Serial.println (curDist);
 }
//-------------------------------------------------------------------------------------------------------------------------------------

void changePath() { //Mert Arduino https://bit.ly/MertArduino
  moveStop();   // stop forward movement
  myservo.write(36);  // check distance to the right
    delay(500);
    rightDistance = readPing(); //set right distance
    delay(500);
    myservo.write(144);  // check distace to the left
    delay(700);
    leftDistance = readPing(); //set left distance
    delay(500);
    myservo.write(90); //return to center
    delay(100);
    compareDistance();
  }

  
void compareDistance()   // CAUTA CEA MAI LUNGA DISTANTA DE LA SENZOR
{
  if (leftDistance>rightDistance) // DACA E AI OK LEFT
  {
    turnLeft();
  }
  else if (rightDistance>leftDistance) // DACA E MAI OK RIGHT
  {
    turnRight();
  }
   else // DACA SUNT LA FEL DE FUCKED
  {
    turnAround();
  }
}


//-------------------------------------------------------------------------------------------------------------------------------------

int readPing() { // read the ultrasonic sensor distance
  delay(70);   
  unsigned int uS = sonar.ping();
  int cm = uS/US_ROUNDTRIP_CM;
  return cm;
}
//-------------------------------------------------------------------------------------------------------------------------------------
void moveStop() {

  analogWrite(motorA1, 0); analogWrite(motorA2, 0);
  analogWrite(motorB1, 0); analogWrite(motorB2, 0); 
        
  analogWrite(motorC1, 0); analogWrite(motorC2, 0);
  analogWrite(motorD1, 0); analogWrite(motorD2, 0); 
}

//-------------------------------------------------------------------------------------------------------------------------------------
void moveForward() { //Mert Arduino https://bit.ly/MertArduino

      analogWrite(motorA1, 0); analogWrite(motorA2, 255);
      analogWrite(motorB1, 0); analogWrite(motorB2, 255); 
        
      analogWrite(motorC1, 0); analogWrite(motorC2, 255);
      analogWrite(motorD1, 0); analogWrite(motorD2, 255);

}
//-------------------------------------------------------------------------------------------------------------------------------------
void moveBackward() { //Mert Arduino https://bit.ly/MertArduino


      analogWrite(motorA1, 255); analogWrite(motorA2, 0);
      analogWrite(motorB1, 255); analogWrite(motorB2, 0); 
        
      analogWrite(motorC1, 255); analogWrite(motorC2, 0);
      analogWrite(motorD1, 255); analogWrite(motorD2, 0); 

      
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnRight() { //Mert Arduino https://bit.ly/MertArduino

  analogWrite(motorA1, 0); analogWrite(motorA2, 255);
  analogWrite(motorB1, 255); analogWrite(motorB2, 0); 
        
  analogWrite(motorC1, 0); analogWrite(motorC2, 255);
  analogWrite(motorD1, 255); analogWrite(motorD2, 0); 
  delay(2000);
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnLeft() { //Mert Arduino https://bit.ly/MertArduino

  analogWrite(motorA1, 255); analogWrite(motorA2, 0);
  analogWrite(motorB1, 0); analogWrite(motorB2, 255); 
        
  analogWrite(motorC1, 255); analogWrite(motorC2, 0);
  analogWrite(motorD1, 0); analogWrite(motorD2, 255); 
  delay(2000); // run motors this way for 1500        

}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnAround() { //Mert Arduino https://bit.ly/MertArduino

  analogWrite(motorA1, 0); analogWrite(motorA2, 255);
  analogWrite(motorB1, 255); analogWrite(motorB2, 0); 
        
  analogWrite(motorC1, 0); analogWrite(motorC2, 255);
  analogWrite(motorD1, 0); analogWrite(motorD2, 255); 
  delay(2500); // run motors this way for 1500        

}
