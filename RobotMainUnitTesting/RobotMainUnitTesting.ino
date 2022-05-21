#include <Servo.h>
#include <Wire.h>
#include <LSM303.h>

Servo myservo;
LSM303 compass;

//Electrical Connections
int servoPin = 3;                               //Servomotor is connected to pin 3
int solenoidPin = 2;                            //Mosfet is connected to pin 2
int switchPin = 4;                              //Switch is connected to pin 4

//Physical Parameters
int numMagnets = 4;                             //Variable stores number of magnets on wheel
int tireCircumference = PI * 69;                //Variable stores tire circumference (mm)
int partialTurn = tireCircumference / numMagnets;                 //Variable stores travel distance for one reed switch tick
int servoRange = 64;                                     //Range of motion of servo for steering
int minServoAngle = 90 - 64 / 2;                         //Variable stores minimum angle servo should rotate to prevent breaking steering mechanicsm
int maxServoAngle = 90 + 64 / 2;                           //Variable stores maximum angle servo should rotate to prevent breaking steering mechanism


//Data Storage
int pos = 0;                                    //Stores the servo position
int switchState;                                //Stores the Reed switch state
int solenoidState = LOW;                        //Stores if solenoid is on or off         
unsigned long previousMillis = 0;               //Stores the last time the solenoid was updated
const long interval = 1000;                     //Stores the interval at which to turn solenoid on and off (milliseconds)
int previousSwitchState = 1;                    //Stores whether or not the switchState was 0 or 1 in the previous loop
int reedTicks = 0;

//Filter Settings
float Kp = 1.02;                                //Stores how much to scale the filtered signal
int previousSteer;                           //Stores the filteredsteer from the previous loop
int previousMag;                             //Stores the filteredMag

//Closed-Loop Settings
float goalPost[1][2];                           //Creates empty array of goal posts
int currentTarget = 0;                          //Sets current target to the first value of goalPost
float previousCoords[1][2] = {0, 0};            //Stores newCoords from previous loop
float newCoords[1][2] = {0, 0};                 //Stores the robot coordinates each loop
int steerPos;                                   //Creates empty variable to store the angle to send the servo to
int angleToTarget = 0;                          //Stores the angle between the current position vector and the target goalpost vector

void setup() {
  myservo.attach(servoPin);                     //Attaches the servo on pin 9 to the servo object
  pinMode(solenoidPin, OUTPUT);                 //Sets the pin as an output
  pinMode(switchPin, INPUT_PULLUP);             //Sets the pin as an input_pullup
  Serial.begin(9600);                           //Starts serial communication @ 9600 bps
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.writeReg(0x24, 0x74);                 //Sets a parameter on the magnetometer so it updates at a higher rate than the default rate of 7 Hz;  Has to occure after compass.enableDefault()

  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){ -1661,  +2121,  +3603};
  compass.m_max = (LSM303::vector<int16_t>){ +1729,  +5710,  +3953}; 

  compass.read();
  float heading = compass.heading();                                  //Records magnetometer heading value into pos
  pos = heading;                                                      //Sets starting angle wrt north
  filteredPrevious = pos;                                             //Sets starting filter value to pos (allows the filter to start closer to the right value and reduces wobbles at program start)
  goalPost[0][0] = -3000 * sin(pos / 57.2957795);                     //Set x-position of goalpost to x-distance from robot
  goalPost[0][1] = 3000 * cos(pos / 57.2957795);                      //Set y-position of goalpost to y-distance from robot
}
  


void loop() {

  
////////////// SERVO - Steering ////////////////////////////////////////////////////
  //myservo.write(pos); //Moves servo to position value stored in pos
  if(servoRange * -angleToTarget / 360 + 90 < minServoAngle){
    myservo.write(filterSignal(minServoAngle, 0.8, false));                                            //If the robot is facing too far away from the target, it sets the turn to the max servo angle for our mechanism
  }
  else if(servoRange * -angleToTarget / 360 + 90 > maxServoAngle){
    myservo.write(filterSignal(maxServoAngle, 0.8, false));                                            //If the robot is facing too far away from the target, it sets the turn to the max servo angle for our mechanism
  }
  else{
    myservo.write(filterSignal((servoRange * -angleToTarget / 360 + 90), 0.8, false));                   //If the robot is within the servo turn range, it sets the turn to angle between the current robot heading and the vector from the robot to the target
  }
  delay(10);

////////////// MAGNETOMETER - Steering ///////////////////////////////////////////////////
  compass.read();
  float heading = compass.heading();
  pos = Kp * filterSignal(heading, 0.95, true);                                        //filterSteering() removes interference from the magnetometer with the filterStrength and uses Kp to rescale the value so that it can range from 0 to 360

////////////// SOLENOID VALVE ///////////////////////////////////////////////////
unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (solenoidState == LOW) {                                             //If the interval between solenoid triggers has elapsed and the solenoid is in its LOW state, then set its state to high (extend piston)
      solenoidState = HIGH;
    } else {                                                                //If the interval between solenoid triggers has elapsed and the solenoid is in its HIGH state, then set its state to LOW (close piston)
      solenoidState = LOW;
    }
    digitalWrite(solenoidPin, solenoidState);    //Switch Solenoid ON/oFF
  }


////////////////MAGNETOMETER - Mapping/////////////////////////////////////////
  switchState = digitalRead(switchPin);                                   //Stores whether or not the reed switch has been activated this loop
  if(switchState == 0 && previousSwitchState== 1){                        //If a magnet passes the reed switch for the first time since the reed switch was inactive, record the robot's new position and store switchState in previousSwitchState for next loop 
    newCoords[0][0] = (previousCoords[0][0] + partialTurn * -sin(pos / 57.2957795));         //New robot x-position (after movement) is recorded in newCoords
    newCoords[0][1] = (previousCoords[0][1] + partialTurn * cos(pos / 57.2957795));          //New robot x-position (after movement) is recorded in newCoords
    previousCoords[0][0] = newCoords[0][0];                               //The new robot x-position is stored in previousCoords 
    previousCoords[0][1] = newCoords[0][1];                               //The new robot y-position is stored in previousCoords 
    
    float newSegmentVector[1][2] = {partialTurn * -sin(pos / 57.2957795), partialTurn * -cos(pos / 57.2957795)};                    //Creates vector that represents most recent movement vector
    float newErrorVector[1][2] = {(goalPost[currentTarget][0] - newCoords[0][0]), (goalPost[currentTarget][1] - newCoords[0][1])};                    //Creates vector that stretches from the current robot position to the target location
    angleToTarget = 57.2957795 * asin((newSegmentVector[0][0] * newErrorVector[0][1] - newSegmentVector[0][1] * newErrorVector[0][0])/(sqrt(sq(newSegmentVector[0][0]) + sq(newSegmentVector[0][1])) * sqrt(sq(newErrorVector[0][0]) + sq(newErrorVector[0][1]))));          //Uses cross product to calculate angle between newSegmentVector and the newErrorVector
    
    reedTicks++;            //Variable used to calculate distance travelled
    
    previousSwitchState = 0;                                //If the switch is ticked, it is now in the on position
  }
  else if(switchState == 0){                                          //If the switchState is active (magnet by reed switch), but it was active before, then the previousSwitchState is set to 0 (active) for next loop
    previousSwitchState = 0;
  }
  else{                                                               //If the switchState is inactive, set the previousSwitchState to inactive for next loop
    previousSwitchState = 1;
  }

////////////// Serial Print  ///////////////////////////////////////////////////
  
  Serial.print("Angle to target: ");
  Serial.print(angleToTarget);
  Serial.print("   Filtered Magnetometer: ");
  Serial.print(pos);
  Serial.print("       Current Coordinates: (");
  Serial.print(previousCoords[0][0]);
  Serial.print(" ");
  Serial.print(previousCoords[0][1]);
  Serial.print(")     Goal Coordinates: (");
  Serial.print(goalPost[0][0]);
  Serial.print(" ");
  Serial.print(goalPost[0][1]);
  Serial.print(")          ");  
  Serial.println("");

///////////// Loop Delay ///////////////////////////////////////////////////
  //delay(100);
}

float filterSignal(float deg, float filterStrength, boolean magnetometer){                                    //Recursive low-pass filter function returns filtered signal
  float currentFilter;
    if(magnetometer){
    currentFilter = ((1 - filterStrength) * deg + filterStrength * previousMag);
    previousMag = currentFilter;
    return currentFilter;
  }else{
    currentFilter = ((1 - filterStrength) * deg + filterStrength * previousSteer);
    previousSteer = currentFilter;
    return currentFilter;
  }
}
