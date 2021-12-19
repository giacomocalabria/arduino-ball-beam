
// note: for my beam to be horizontal, Servo Motor angle should be 102 degrees.







#include<Servo.h>
#include<PID_v1.h>
#include<Ultrasonic.h>
#define TRIGGER_PIN  7
#define ECHO_PIN     8
Servo myServo;
Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
const int servoPin = 11;                                               //Servo Pin
float Kp = 2.5;                                                    //Initial Proportional Gain
float Ki = 0;                                                      //Initial Integral Gain
float Kd = 1.1;                                                    //Intitial Derivative Gain
double Setpoint, Input, Output, ServoOutput;                                       
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.
void setup() {
 Serial.begin(9600);                                                //Begin Serial 
  myServo.attach(servoPin);                                          //Attach Servo

  Input = readPosition();                                            //Calls function readPosition() and sets the balls
                                                                     //  position as the input to the PID algorithm
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(-80,80);                                     //Set Output limits to -80 and 80 degrees. 
}
void loop()
{
 
  Setpoint = 25;
  Input = readPosition();                                            
 
  myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees
  
  ServoOutput=102+Output;                                            // 102 degrees is my horizontal 
  myServo.write(ServoOutput);                                        //Writes value of Output to servo
  
  
}
float cmMsec;
  long microsec = ultrasonic.timing();
  cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
  Serial.print(", CM: ");
  Serial.println(cmMsec);      
if(cmMsec > 30)     // 30 cm is the maximum position for the ball
  {cmMsec=30;}
Serial.println(cmMsec);
return cmMsec;                                          //Returns distance value.
}



