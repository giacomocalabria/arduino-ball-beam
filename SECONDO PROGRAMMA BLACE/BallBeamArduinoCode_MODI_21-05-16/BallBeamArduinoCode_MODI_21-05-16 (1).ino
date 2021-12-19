#include<Servo.h>
#include<PID_v1.h>
#include<NewPing.h>
Servo myServo;
const int servoPin = 11;
const int triggerPin = 7;                                             
const int echoPin = 8;                                                
const int massimaDistanza = 30;  
float Kp = 2.5;                                                    //Initial Proportional Gain
float Ki = 0;                                                      //Initial Integral Gain
float Kd = 1.1;                                                    //Intitial Derivative Gain
double Setpoint, Input, Output, ServoOutput;
unsigned int uS = 0;
NewPing sonar(triggerPin, echoPin, massimaDistanza);                                       
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);//Initialize PID object, which is in the class PID.

void setup() {
 Serial.begin(9600);                                                 //comunicazione seriale 
  myServo.attach(servoPin);                                          //pin del servomotore
  Input = readPosition();                                            //Calls function readPosition() and sets the balls
                                                                     //  position as the input to the PID algoritm
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(30,-30);                                     //Set Output limits to -80 and 80 degrees. 
}
void loop(){
  Setpoint = 15;                                                     // punto dove si tiene ferma la pallina
  Input = readPosition();                                         
  myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees
  ServoOutput=90+Output;                                            // 102 degrees is my horizontal 
  myServo.write(ServoOutput);                                        //Writes value of Output to servo
} 
float readPosition() {
  delay(36);                                        //Don't set too low or echos will run into eachother.   
  unsigned int OLDuS = uS;    
  uS = sonar.ping_median(4);                               //ping_median() function is in class NewPing. It is a digital filter that takes the 
                                                                    //  the average of 4 (in this case) pings and returns that distance value. This helps 
                                                                        //  to filter out the pesy noise in those $5.00 UltraSonic Sensors. You know which
                                                                        //  ones I am talking about....
  //return uS / US_ROUNDTRIP_CM;                                          //Returns distance value.
  uS = uS / US_ROUNDTRIP_CM;
  if (uS > massimaDistanza) {
    uS = OLDuS;
  }
 return uS;
}




