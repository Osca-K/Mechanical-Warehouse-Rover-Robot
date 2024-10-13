#include <ArduinoMotorCarrier.h>
#include <ArduinoMotorCarrier.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// IMU SENSOR VARIABLES 
Adafruit_BNO055 myIMU = Adafruit_BNO055(55);
float AngleOrientation;
float NewOrientation;
float AngleAtPickUp;
float Differences;

  
//Servo variables
int InitialAngle = 180;
int PickAngle = 0;
int GripAngle=150;
int RestAngle=0;
int BayHeight=80;

 
// Motors variables
int InitialSpeed = 0;
int NormalSpeed = 30;
int SlowSpeed = 0;
int RotatingSpeed=20;
 
// Sensor variable
int RightSensor = A7;  //under motor 1
int LeftSensor = A2;   //under motor 2
int RightSensorReading;
int LeftSensorReading;
 
// Ultrasonic variable
int Triger = A3;
int Reciver = A6;
int SpeedOfSound = 343;
float Time;
float BoxDistance;
float BayDistance;

 
 
 //Running once conditions
 
bool PickBoxOnce = true;
bool DetectBayWallOnce = true;
bool RobotRotateOnce=true;
int k=0;
 
 
 
void setup() {
  // put your setup code here, to run once:
 
  //  Servos set up
  Serial.begin(9600);  
  controller.begin() ;
  myIMU.begin();

  servo1.setAngle(180);
  servo2.setAngle(0);
  SetUpForServos(InitialAngle,RestAngle);
  // Motor wheels set up
  SetUpForMotors(InitialSpeed);
 
  //Set up for sensor color sensorline
  SetUpForLineSensor(RightSensor, LeftSensor);
 
  //Set up for ultrasonic sensor
  SetUpForUltrasonicSensor(Triger, Reciver);
}
 
void loop()
{
  // put your main code here, to run repeatedly:
  BoxDistance = DistanceFromObject(Triger, Reciver, SpeedOfSound);  
  SensorReading(LeftSensor, RightSensor, &LeftSensorReading, &RightSensorReading);
  RobotMoving(NormalSpeed, SlowSpeed, LeftSensorReading, RightSensorReading); 
  
    
  if (BoxDistance <= 17 && PickBoxOnce==0) 
  {  
              Serial.print(" \nPicking up the box\n\n ")  ;
          
              StopMoving(SlowSpeed, SlowSpeed); 
              OpeningGripers(RestAngle,GripAngle);              
              LowerFork(InitialAngle, PickAngle);
            
              ClosingGripers(RestAngle,GripAngle) ;            
              delay(100);
              LiftFork(InitialAngle, PickAngle);
              delay(100);
            
              
              sensors_event_t event;
              myIMU.getEvent(&event);              
              AngleAtPickUp = (event.orientation.x);
              Serial.print(AngleAtPickUp) ;
              
              if (AngleAtPickUp > 180.00) 
              {

                  Serial.println("Angle is  more 180 \n\n");
                    NewOrientation = AngleAtPickUp - 175.00; 
                    while (AngleOrientation >= NewOrientation)
                    {
                      sensors_event_t event;
                      myIMU.getEvent(&event);
                      AngleOrientation = (event.orientation.x);

                      Serial.print("Angle 1 = "); Serial.print(AngleAtPickUp);
                      
                      Serial.print("\tAngle At Moment = "); Serial.print(AngleOrientation); 
                      
                      Serial.print("\tAngle 2 = ");    Serial.print(NewOrientation); 
                      Serial.println("") ;                                                             
                      
                      M1.setDuty(-RotatingSpeed);
                      M2.setDuty(RotatingSpeed);

                      if(AngleOrientation<NewOrientation)
                         {
                           break;
                         }                    
                                      
                    }  
                    StopMoving(SlowSpeed, SlowSpeed);                    
               }      
              else
              {      Serial.println("Angle is less than 180 \n\n");
                      NewOrientation = AngleAtPickUp + 175.00;
                      while ((AngleOrientation < NewOrientation) && (AngleOrientation>AngleAtPickUp))  
                      {

                          sensors_event_t event;
                          myIMU.getEvent(&event);
                          
                          AngleOrientation = (event.orientation.x);

                          Serial.print("Angle 1 = "); Serial.print(AngleAtPickUp);
                          Serial.print("\tAngle At Moment = "); Serial.print(AngleOrientation); 
                          Serial.print("\tAngle 2 = ");    Serial.print(NewOrientation);
                          Serial.println("") ;                     
                          M1.setDuty(RotatingSpeed);
                          M2.setDuty(-RotatingSpeed);
                         
                         if(AngleOrientation>NewOrientation)
                         {
                           break;
                           
                         }                                            
                      }
                      
                      StopMoving(SlowSpeed, SlowSpeed);                           
              } 
                  
      PickBoxOnce = false;             
  }

  
  if (PickBoxOnce==0 && DetectBayWallOnce) {
     BayDistance= DistanceFromObject(Triger, Reciver, SpeedOfSound);    
    if(BayDistance<=15 && DetectBayWallOnce && k==0)
    {
          StopMoving(SlowSpeed, SlowSpeed);
          OpeningGripers(RestAngle,GripAngle);
          LowerFork(InitialAngle, BayHeight);
          RobotReversing(NormalSpeed, SlowSpeed, LeftSensorReading, RightSensorReading);  
          delay(650) ;                
          DetectBayWallOnce=false;

         k++;       
    }  
 
       
  } 

 if(DetectBayWallOnce==0){
  
 } 
    
 
}

//===========SETUPS=================//
//Servo setup
void SetUpForServos(int initialangle,int restangle) {
  servo1.setAngle(initialangle);
  servo2.setAngle(restangle);
}
void SetUpForMotors(int initialspeed) {
  M1.setDuty(initialspeed);
  M2.setDuty(initialspeed);
}
void SetUpForLineSensor(int leftsensor, int rightSensor) {
  pinMode(leftsensor, INPUT);
  pinMode(rightSensor, INPUT);
}
void SetUpForUltrasonicSensor(int triger, int reciver) {
  pinMode(triger, OUTPUT);
  pinMode(reciver, INPUT);
}



//======================LOOPS=====================

float DistanceFromObject(int triger, int reciver, int speedofsound) {
 
  digitalWrite(triger, LOW);
  delayMicroseconds(10);
  digitalWrite(triger, HIGH);
  digitalWrite(triger, LOW);
  Time = pulseIn(reciver, HIGH);
 
  Time = Time * 0.000001;
  return 0.5 * Time * speedofsound * 100;
}
 
void SensorReading(int leftsensor, int rightsensor, int *leftsensorreading, int *rightsensorreading) {
 
  *leftsensorreading = analogRead(leftsensor);
  *rightsensorreading = analogRead(rightsensor);
}
 
void RobotMoving(int normalspeed, int slowspeed, int leftsensorreading, int rightsensorreading) {
  if (leftsensorreading < 60 && rightsensorreading < 60) {
    Straightpath(normalspeed, slowspeed);
  } else if (leftsensorreading < 60 && rightsensorreading > 60) {
    TurnToRight(normalspeed, slowspeed);
  } else if (leftsensorreading > 60 && rightsensorreading < 60) {
    TurnToLeft(normalspeed, slowspeed);
  } else {
    StopMoving(normalspeed, slowspeed);
  }
}
void RobotReversing(int normalspeed, int slowspeed, int leftsensorreading, int rightsensorreading) {
  if (leftsensorreading < 60 && rightsensorreading < 60) {
    Straightpath(-normalspeed, -slowspeed);
  } else if (leftsensorreading < 60 && rightsensorreading > 60) {
    TurnToRight(-normalspeed, -slowspeed);
  } else if (leftsensorreading > 60 && rightsensorreading < 60) {
    TurnToLeft(-normalspeed, -slowspeed);
  } else {
    StopMoving(normalspeed, slowspeed);
  }
}
void Straightpath(int normalspeed, int slowspeed) {
 
  M1.setDuty(normalspeed);
  M2.setDuty(normalspeed);
}
void TurnToRight(int normalspeed, int slowspeed) {
  M1.setDuty(slowspeed);
  M2.setDuty(normalspeed);
}
void TurnToLeft(int normalspeed, int slowspeed) {
  M1.setDuty(normalspeed);
  M2.setDuty(slowspeed);
}
void StopMoving(int normalspeed, int slowspeed) {
  M1.setDuty(slowspeed);
  M2.setDuty(slowspeed);
}

 
void LowerFork(int initialangle, int pickangle) {
  for (int i = initialangle; i >= pickangle; i--) {
    servo1.setAngle(i);
    delay(10);
  }
}

void LiftFork(int initialangle, int pickangle) {
 
  for (int i = pickangle; i <= initialangle; i++) {
    servo1.setAngle(i);
    delay(10);
  }
}
void OpeningGripers(int restangle,int gripangle)
{
    
for (int i=gripangle; i>=restangle; i--) {
  servo2.setAngle(i);
  delay(10);
}   
  
}
void ClosingGripers(int restangle,int gripangle)
{
  for (int i=restangle; i<=gripangle; i++) {
   servo2.setAngle(i);
  delay(10);
  }
  
}

long DeltaTime(long previoustime) {
  long currenttime = millis() * 0.001;
  long deltatime = currenttime - previoustime;
  if (deltatime != 0) { previoustime = currenttime; }
  return deltatime;
}

