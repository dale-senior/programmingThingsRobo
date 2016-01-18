#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LSM303.h>

ZumoReflectanceSensorArray reflectanceSensors;
SoftwareSerial XBee(0, 1); // RX, TX

// Define a bunch of settings.
//settings needs tweeking depending on various variables surface, battery charge ect
#define NUM_SENSORS 6 //number of sensors to use
#define LIGHT_THRESHOLD 10 //threshold for sensing a black line
#define FORWARD_DIS 10 //how far to go forward
#define CALIBRATION_SAMPLES 144 // Number of compass readings to take when calibrating
#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M value for magnetometer 220 Hz update rate
#define THRESHOLD 2 //threshhold for turning
#define SPEED           150// Maximum motor speed when going straight; variable speed when turning
#define TURN_BASE_SPEED 100 // Base speed when turning (added to variable speed)
#define ABOVE_LINE(sensor)((sensor) > LIGHT_THRESHOLD)
#define INCHES_TO_ZUNITS 16000.0
#define MOVEINCHES(inches)((((INCHES_TO_ZUNITS / 2) * (inches)) / SPEED))
#define MICROTOINCHES(microseconds) (microseconds / 74 / 2) //macro for changing the feedback from the sounds sensors to inches
#define MICROTOCM(microseconds) (microseconds / 29 / 2)//macro for doing the same as above but in cm
#define ZUMOMOVESIZE 5
#define echoPin 4 // Echo Pin
#define trigPin 3 // Trigger Pin
#define OBJECTDETECTDISTANCE 10
#define CORRIDORTURN 90
#define AVG 20.0

unsigned int sensorValues[NUM_SENSORS];
ZumoBuzzer buzzer;
ZumoMotors motors;
LSM303 compass;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
float currentForwardHeading;
enum direction {left, right, back, leftRoom, rightRoom};//will assume everything in between is a stright
int path[100];
int path_length = 0;
boolean continueToSolveMaze = true;
boolean logMoves = true;
//----HEADING TEMPLATE----//

template <typename T> float heading(LSM303::vector<T> v)
{
  float x_scaled =  2.0*(float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}

//Returns the average heading for the zumo
//average is needed as the compass values tend scew
float averageHeading()
{
  LSM303::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < AVG; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= AVG;
  avg.y /= AVG;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}

//----INITIAL SETUP----//

void setup() 
{
  button.waitForButton();
  //  //open up serial for communication
  Serial.begin(9600);
  //open XBee for communication
  XBee.begin(9600);

  iniCompass();
  button.waitForButton();
  //turn on led so show setup in progress
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  //initialise sensor array
  reflectanceSensors.init();
  delay(1000);
  //sensor calibration
  reflectanceSensors.calibrate();
  digitalWrite(13, LOW);
  button.waitForButton();
//  playOpeningNotes();
  moveDownCorridor(); //the initial maze solver
  mazeReturn();//return to beginning useing best path
}


//initialises the compass for use
void iniCompass()
{
  // The highest possible magnetic value to read in any direction is 2047
  // The lowest possible magnetic value to read in any direction is -2047
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};
  unsigned char index;
  Serial.println("Initializing Compass");
    // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();
  
  // Initiate LSM303
  compass.init();
  
  // Enables accelerometer and magnetometer
  compass.enableDefault();

  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate
  motors.setSpeeds(200, -200);

  for(index = 0; index < CALIBRATION_SAMPLES; index ++)
  {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);

//    Serial.println(index);

    delay(25);
  }
  motors.setSpeeds(0, 0);

  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;  
}

//----CODE LOOP----//

//main loop not needed as its being dealt with else where
void loop() 
{
}


//-----MAZE SOLVER LOGIC-----//

//Maze solver logic
//check left and right wall first
//the will move forward until the corridor ends
//will attempt to turn left and then right
//if a corrridor is found its logged and becomes the new heading and the loop continues until a dead end
void moveDownCorridor()
{
  //while not found a door or at a dead end
  Serial.println("Starting To solve Maze");
  currentForwardHeading = averageHeading();//sets the current direction we are heading
  while(continueToSolveMaze)
  {
    unsigned int pos = reflectanceSensors.readLine(sensorValues);
    boolean lineDetected = checkLeft();    
    if(!lineDetected)//not found a wall so maybe there is a door there
      checkForDoor(true);//check for a door then enter if there is one

    delay(500);
    lineDetected = checkRight();
    if(!lineDetected)//not found a wall so maybe there is a door there
      checkForDoor(false);

    delay(500);
    if(MoveForwardInInches(ZUMOMOVESIZE))//move forward, if a line has been detected then chances are wewe hit a dead end
    { 
      if((ABOVE_LINE(sensorValues[0]) || ABOVE_LINE(sensorValues[1])) && !ABOVE_LINE(sensorValues[3]))
        {
          reverseBack(ZUMOMOVESIZE - 1);//tad reverse
          turn(fmod(averageHeading() + 10, 360)); //turn slightly and try again
        }else if((ABOVE_LINE(sensorValues[5]) || ABOVE_LINE(sensorValues[4])) && !ABOVE_LINE(sensorValues[3]))
        {
          reverseBack(ZUMOMOVESIZE - 1);//tad reverse
          turn(fmod(averageHeading() - 10, 360)); //turn slightly and try again
        }
      if(MoveForwardInInches(ZUMOMOVESIZE))
        CheckForCorridors();
    }
  }
  //turn round here or in the maze return? 
  Serial.println("End of maze time to come back");
}


//checks if there is a corridor either side of a dead end
//turns 90 degrees left and right and attempts to move forward
void CheckForCorridors()
{
  reverseBack(ZUMOMOVESIZE - 1);
  boolean foundLeft = false;
  boolean foundRight = false;
  
  //checks left corridor
  Serial.println("Checking for left corridor");
  turn(fmod(currentForwardHeading - CORRIDORTURN, 360));//turn left 90
  turn(fmod(currentForwardHeading - CORRIDORTURN, 360));
  foundLeft = !MoveForwardInInches(ZUMOMOVESIZE - 1); //no wall so there is a left corridor
  float leftHeading = fmod(currentForwardHeading - CORRIDORTURN, 360);

  if(!foundLeft)
  { //no left corridor so check right
    reverseBack(ZUMOMOVESIZE);
    keepTurningTillStraight();//turn backstraight
    MoveForwardInInches(ZUMOMOVESIZE);//move back to the line
    reverseBack(ZUMOMOVESIZE - 1);//tad reverse
    Serial.println("Checking for right Corridor");
    turn(fmod(currentForwardHeading + CORRIDORTURN, 360));//turn back and 90 more degree's
    turn(fmod(currentForwardHeading + CORRIDORTURN, 360));
    foundRight = !MoveForwardInInches(ZUMOMOVESIZE - 1);
    float rightHeading = fmod(currentForwardHeading + CORRIDORTURN, 360);
    if(!foundRight)
    {
      reverseBack(2);
      continueToSolveMaze = false;//stop solving the maze
      Serial.println("found dead end");
      if(logMoves)
      {
        path[path_length] = back;
        path_length = path_length++; 
      }
      turn(currentForwardHeading);
      turn(currentForwardHeading);
    }
    else 
    {
      Serial.println("found right corridor");
      currentForwardHeading = rightHeading;
      if(logMoves)
      {
        path[path_length] = right;
        path_length++;
      }
    }
  }
  else
  {
    Serial.println("found left corridor");
    currentForwardHeading = leftHeading;
    if(logMoves)
    {
      path[path_length] = left;
      path_length++;
    }
  }
}
//------RETURN CODE-----/

//Returns back to the start of the course using the best path
//will stop once its back to the start
void mazeReturn()
{
  //creates best path
  optimiseReturnRoot();
  boolean backAtStart = false;
  logMoves = false;//stop logging moves
  turn(fmod(currentForwardHeading - 180, 360));
  currentForwardHeading = averageHeading();
  //Outputs the current command as a string
  //loops through a specific set of commands
  for(int i = 0; i <= path_length; i++)
  {
    Serial.print("Current Command: ");
    switch(path[i])
    {
      case left:
      Serial.println("left");
      break;
      case right:
      Serial.println("right");
      break;
      case leftRoom:
      Serial.println("leftroom");
      break;
      case rightRoom:
      Serial.println("rightroom");
      break;
      case back:
      Serial.println("back");
    }
    if(path[i] == left || path[i] == right || path[i] == back)
      {
        boolean atTheEnd = false;
        while(!atTheEnd)
        {
          atTheEnd = MoveForwardInInches(ZUMOMOVESIZE);
          //adjusts the zumo slightly to make sure we doent accidently get a wall as a dead end
          if((ABOVE_LINE(sensorValues[0]) || ABOVE_LINE(sensorValues[1])) && !ABOVE_LINE(sensorValues[3]))
          {
            reverseBack(ZUMOMOVESIZE - 1);//tad reverse
            turn(fmod(averageHeading() + 10, 360)); //turn slightly and try again
            atTheEnd = MoveForwardInInches(ZUMOMOVESIZE);
          }else if((ABOVE_LINE(sensorValues[5]) || ABOVE_LINE(sensorValues[4])) && !ABOVE_LINE(sensorValues[3]))
          {
            reverseBack(ZUMOMOVESIZE - 1);//tad reverse
            turn(fmod(averageHeading() - 10, 360)); //turn slightly and try again
            atTheEnd = MoveForwardInInches(ZUMOMOVESIZE);
          }    
        }
        reverseBack(ZUMOMOVESIZE - 1);//tad reverse
        if(path[i] == left)
        {
          turn(fmod(currentForwardHeading - CORRIDORTURN, 360));//turn back and 90 more degree's
          currentForwardHeading = fmod(currentForwardHeading - CORRIDORTURN, 360);
        }
        else if(path[i] == right) 
        {
          turn(fmod(currentForwardHeading + CORRIDORTURN, 360));//turn back and 90 more degree's
          currentForwardHeading = fmod(currentForwardHeading - CORRIDORTURN, 360);
        }
      }
      if(path[i] == leftRoom)
      {
        boolean lineDetected = true;
        while(lineDetected)
        {
          MoveForwardInInches(ZUMOMOVESIZE);
          lineDetected = checkLeft();
          if(!lineDetected)
            checkForDoor(true);
        } 
      }
       
      if(path[i] == rightRoom)
      {
        boolean lineDetected = true;
        while(lineDetected)
        {
          MoveForwardInInches(ZUMOMOVESIZE);
          lineDetected = checkRight();
          if(!lineDetected)
            checkForDoor(false);
        } 
      }
    }   
  motors.setSpeeds(0, 0);   
}

//converts the current path into the best path to take on the way back
//does this by reverseing the order (except back) and swapping right for left, leftDoor for rightDoor and vice versa
void optimiseReturnRoot()
{
  int newPath[100];
  int index = 0;
  Serial.println("path");
  for(int i = 0; i < path_length; i++)
  { 
    switch(path[i])
    {
      case left:
      Serial.println("left");
      break;
      case right:
      Serial.println("right");
      break;
      case leftRoom:
      Serial.println("leftroom");
      break;
      case rightRoom:
      Serial.println("rightroom");
      break;
      case back:
      Serial.println("back");
    }
  }
  for(int i = 0; i < path_length; i++)
    newPath[i] = path[path_length - 1 - i];

    newPath[path_length] = back; //append back to the end
  for(int i = 0; i <= path_length; i++)
  {
    switch(newPath[i])
    {
      case left:
        path[i] = right;
        break;
      case right:
        path[i] = left;
        break;
      case leftRoom:
        path[i] = rightRoom;
        break;
      case rightRoom:
        path[i] = leftRoom;
        break;
      case back:
        path[i] = back;
    } 
  }
  
  Serial.println("optimised path");
  for(int i = 0; i <= path_length; i++)
  { 
    switch(path[i])
    {
      case left:
      Serial.println("left");
      break;
      case right:
      Serial.println("right");
      break;
      case leftRoom:
      Serial.println("leftroom");
      break;
      case rightRoom:
      Serial.println("rightroom");
      break;
      case back:
      Serial.println("back");
    }
  }
}

//------TURN LOGIC-----///

//will turn zumo left by specified angle
void turn(float turnAngle)
{ 
  int speed;
  float heading = averageHeading();
  float relative = relativeHeading(heading, turnAngle);
  boolean notFinishedTurning = true;
  while(notFinishedTurning)
  {
    heading = averageHeading();
    relative = relativeHeading(heading, turnAngle);
    if(abs(relative) < THRESHOLD)
      notFinishedTurning = false;
    else 
    {
    speed = SPEED*relative/180;
    if (speed < 0)
      speed -= TURN_BASE_SPEED;
    else
      speed += TURN_BASE_SPEED;
    motors.setSpeeds(speed, -speed);  
    }
  }//gets us close enough
//  Serial.println("Exited While");
  motors.setSpeeds(0, 0);
  delay(500);
}

//----CHECK LOGIC----//

//attempts the turn a few times to straighten up, won't move if we are perfectly inline 
void keepTurningTillStraight()
{
  for(int i = 0; i < 5; i++)
  {
    turn(currentForwardHeading);  
  }
}

//slowly rotates and scanes for an object
//if at any point one has been detected it will stop and return true
boolean spinAndCheckForObject()
{ 
  boolean foundObject = false;
  Serial.println("searching for object");
  for(int i = 0; i < 18; i++)
  {
    turn(fmod(averageHeading() - 20, 360));
    delay(100);
    if(scanForObject())
    {
      foundObject = true;
      break;
    }
  }
  return foundObject;
}

//checks that the gap we have found is a door and not a corridor
//to do this it moves forward the length of the door
//and the reverses back half way
//onces it has does this it will enter the room
boolean checkForDoor(boolean chkLeft)
{//check for a door
  Serial.println("checking if its a door");

  //first move forward more than the length of the corridor (this is because the corridor we may have just found will have a dead end before that distance anyways)
  if(MoveForwardInInches(ZUMOMOVESIZE + 2))//slightly more that the corridor (zumo width plus half its width
    {
      reverseBack(ZUMOMOVESIZE - 1);
      return true;
    }

  //we have a door yay
  reverseBack(ZUMOMOVESIZE + 3); //reverse back so we are more or less in the door way
  //decide if we are going left or right into the room
  if(chkLeft)
  {//left
    turn(fmod(currentForwardHeading - 90, 360));
    MoveForwardInInches(ZUMOMOVESIZE + 2);
    if(spinAndCheckForObject())
    {
      Serial.println("object found in left hand room");
      if(logMoves)
      {
        path[path_length] = leftRoom;
        path_length++;
      }
    }//log
    delay(200);
    turn(fmod(currentForwardHeading + 90, 360));
  }
  else
  {//right
    turn(fmod(currentForwardHeading + 80, 360));
    MoveForwardInInches(ZUMOMOVESIZE + 4);
    if(spinAndCheckForObject())
    {
      Serial.println("object found in right hand room");
      if(logMoves)
      {
        path[path_length] = rightRoom;
        path_length++;
      }    
    }//log
    turn(fmod(currentForwardHeading - 90, 360));
  } 
  MoveForwardInInches(ZUMOMOVESIZE * 2);//carry on to will get to the wall 
  reverseBack(ZUMOMOVESIZE /2); //reverse back a tad
  turn(currentForwardHeading); //turn back to the correct heading
  MoveForwardInInches(ZUMOMOVESIZE * 2);//move rest of the door away so we dont accidently check it again 
}


//checks every sensors to see if it is above a line
//will return true if any are above a line
boolean checkSensors()
{
  for(int x = 0; x < NUM_SENSORS;x++)
  {//loop through all sensors and chech if there over a black line
    if(ABOVE_LINE(sensorValues[x]))
    {//one of the sensors is over a black line
      return true;
    }
  } 
  return false; 
}

//checks the left wall for a line, will return to were it was
//returns true if a wall has been detected
boolean checkLeft()
{
  boolean lineDetected = false;
  Serial.println("Checking left wall");
  turn(fmod(averageHeading() - 60, 360));//turn left
  lineDetected = moveAndReverse();
  turn(currentForwardHeading);
  turn(currentForwardHeading);
  return lineDetected;
}

//checks the right wall for a line, will return to were it was
//returns true if a wall has been detected
boolean checkRight()
{
  boolean lineDetected = false;
  Serial.println("Checking right wall");
  turn(fmod(averageHeading() + 60, 360));//turn right
  lineDetected = moveAndReverse();
  turn(currentForwardHeading);
  turn(currentForwardHeading);
  return lineDetected;
}

//uses the ultrasonci sensor to detect if an object is within range
//returns true if object is within range
boolean scanForObject()
{
  long duration, distance;
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
   //calculate the distance in cm 
  distance = MICROTOCM(duration);
  delay(100);
   if(distance < OBJECTDETECTDISTANCE)//we should more or less be in the center of the room when we perform the check so 10cm is plenty
      {
        return true;
      }
   return false;
}

//----MOVEMENT LOGIC----//

//moves forward and then reverses back to where it was
//returns true if line has been detected
boolean moveAndReverse()
{
  boolean lineDetected = MoveForwardInInches(ZUMOMOVESIZE);
  if(lineDetected)
    reverseBack(ZUMOMOVESIZE - 1);
  else
    reverseBack(ZUMOMOVESIZE * 2);
  motors.setSpeeds(0, 0);//
  delay(200);  
  return lineDetected;
}


//reverses the zumo by a given amount in cm
void reverseBack(int cm)//moves in cm
{
  motors.setSpeeds(-SPEED, -SPEED);
  delay(MOVEINCHES(cm));
  motors.setSpeeds(0, 0);
  delay(200);
}


//move forward till a line has been hit or until a blackLine has been reached
//returns true if line has been detected
boolean MoveForwardInInches(int inches)
{
  unsigned int pos = reflectanceSensors.readLine(sensorValues);
  int numCheck = inches * 3; //three checks per inch
  float delayTime = MOVEINCHES(inches) / numCheck;
  boolean lineDetected = checkSensors();//re-ini found line
  if(!lineDetected)//not detected one yet so go forward
  {//assume no line found yet
    motors.setSpeeds(SPEED, SPEED);
    for(int i = 0; i < numCheck; i++)
    {
      pos = reflectanceSensors.readLine(sensorValues);
      lineDetected = checkSensors();
      if(lineDetected)
        break;
      delay(delayTime * 2);//need to account for the delay intro duced by the previous code
    }
    motors.setSpeeds(0, 0);
  }
  delay(200);
  return lineDetected;//returns the total delay we used to go forward 
}

//----VARIOUS FUNCTIONS----//

void playOpeningNotes()
{
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 200, 15);
  }
  delay(1000);
  buzzer.playNote(NOTE_G(4), 500, 15);  
  delay(1000);
}

//----HEADING LOGIC----//

// Yields the angle difference in degrees between two headings
float relativeHeading(float heading_from, float heading_to)
{
  float relative_heading = heading_to - heading_from;

  // constrain to -180 to 180 degree range
  if (relative_heading > 180)
    relative_heading -= 360;
  if (relative_heading < -180)
    relative_heading += 360;

  return relative_heading;
}

//------DEBUGS------//

//spits out the debug values for the compass over the serial connection
void debugCompass()
{
  float heading, relative_heading;
  int speed;
  static float target_heading = averageHeading();

  // Heading is given in degrees away from the magnetic vector, increasing clockwise
  heading = averageHeading();

  // This gives us the relative heading with respect to the target angle
  relative_heading = relativeHeading(heading, target_heading);

  Serial.print("Target heading: ");
  Serial.print(target_heading);
  Serial.print("    Actual heading: ");
  Serial.print(heading);
  Serial.print("    Difference: ");
  Serial.print(relative_heading);
  Serial.println();
  Serial.println();
}

//outputs min and max values for light sensors
void minMaxDebug()
{
  for (byte i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(reflectanceSensors.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (byte i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(reflectanceSensors.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
}

//outputs the current values for the light senors over the serial connection
void sensorDebug(unsigned int pos)
{
  for (byte i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(i);
    Serial.print(":-");
    Serial.print(sensorValues[i]);
    Serial.print("    ");
  }
  Serial.print("    ");
  Serial.println(pos);
}
