/* Jacob Smith COSI 119A Arm Interfacing Project
    ArduinoArm Version 6, Presents Serial Interface designed to communicate
    between raspberry pi and arudino.
    Reads 
      ARM x y z w speed and parses to moveArm(x,w,z,y,speed) which doesn't move the arm
      MANIP state and parses to manip(state)     which moves the manipulator open close, and stop
     and regurlaly publishes DISTance sensor
    Bugs: I have to enter trailing space
    Please see //From https://forum.arduino.cc/index.php?topic=396450 for Serial
*/

//inlcude varSpeedServe library and set up named servos
#include <DistanceSensor.h>
#include <VarSpeedServo.h>

DistanceSensor distanceSensor;
VarSpeedServo fingerServo, base_servo, shoulder_servo, elbow_servo, wrist_servo, hand_servo;

//Serial data global variables
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
bool newData;

//parameter list global variables
int paramIndex=0;
int params[4];
int numParams=0;

//calculation global variables
const float BASE_HGT = 100;              //base hight
const float HUMERUS = 140 ;              //shoulder-to-elbow "bone"
const float ELBOW_TO_WRIST = 140;                  //elbow-to-wrist "bone"
const float GRIPPER = 100;              //gripper (incl.heavy duty wrist rotate mechanism) length
#define  ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5)) //float to long conversion
// Servo names/ pin numbers 
const int BAS_SERVO = 12    ;         // Base servo
const int SHL_SERVO = 11   ;          /* Shoulder Servo  */
const int ELB_SERVO = 10  ;           /* Elbow Servo  */
const int fingerPin = 7;              //set up servo pins

// Servo offsets
int shoulder_offset = 0;
int elbow_offset = 0;
int base_offset = 0;

/* pre-calculations */

float hum_sq = HUMERUS * HUMERUS;
float elbow_to_wrist_sq = ELBOW_TO_WRIST * ELBOW_TO_WRIST;

//runs once
void setup() {
  //intialize serial connection
  Serial.begin(9600);
  while (!Serial) {}
  //attach finger
  distanceSensor.begin(2, 3);
  fingerServo.attach(fingerPin);
  base_servo.attach( BAS_SERVO );    // attaches a servo to an I/O pin   , 544, 2400
  shoulder_servo.attach( SHL_SERVO );
  elbow_servo.attach( ELB_SERVO );
  //display welcome message
  Serial.println("Welcome to Arm Control");
  Serial.println("Enter ARM x y base speed : MANIP [0-2]");
}
 
//runs many times
void loop() {
  //read any serial input
  getToken();
  //if there is serial input, parse it
  if(newData){
    process();
    newData=false;
  }
}

//checks for Serial data separated by tokens
void getToken() {
  static byte ndx = 0;
  char endMarker = ' ';
  char endLineMarker = '\n';
  char rc;
  while (Serial.available() > 0 && newData ==false) {
    rc = Serial.read();

    if (rc != endMarker && rc != endLineMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData=true;
    }
  }
}

//Parse recieved commands in a non waiting fashion
void process() {
    //if command is arm, prepare to read 4 paremeters
    if (strcmp(receivedChars, "ARM") == 0) {
      acknowledgeCommand(4,"ARM");
    //if command is manip, prepare to read 1 paremeter
    } else if (strcmp(receivedChars, "MANIP") == 0) {
      acknowledgeCommand(1,"MANIP");
    //if there is a paremeter to read, read it and add to array
    }else if(paramIndex<numParams){
      int dataNumber = 0;           
      dataNumber = atoi(receivedChars);
      params[paramIndex]=dataNumber;
      paramIndex++;
    //if looking for four params and found them, move arm
    }else if(numParams==4){
      moveArm(params[0],params[1],params[2],params[3]); 
    //if looking for one parameter and found it, move manupulator
    }else if(numParams==1){
       moveManip(params[0]);
    }
}

//sets global variables and prints message to acknowledge start of header command
void acknowledgeCommand(int providedNumParams,char* command){
  numParams=providedNumParams;
  paramIndex=0;
  Serial.print("Recieved Command: ");
  Serial.print(command);
  Serial.print("\tDIST: ");
  Serial.println(distanceSensor.getDistance());

}

//move the manipulator to the specified state
void moveManip(int state) {
  //if state is set to close, open the finger
  if(state==0){
    fingerServo.write(35, 50, false);  
  //if state is set to open, open the finger
  }else if (state==1){
    fingerServo.write(160, 50, false);
  //if state is set to stop, stop the finger
  }else if (state==2){
    fingerServo.stop();
  //otherwise print error
  }else{
     Serial.println("ERROR: This Manipulator should be state [0,2]");
  }
}

/* arm positioning routine utilizing inverse kinematics *
  x is distance from base mm, y is height mm, base_rotation  is rotation around base in degrees, gripper_angle is hand rottion in degrees, servospeed is how fast is it to happen 0-300
  void set_arm( uint16_t x, uint16_t y, uint16_t z, uint16_t grip_angle )
  *****************************************************************************************************/
void moveArm( float x, float y, int base_rotation_d, int servoSpeed ) {
  
  /* Base angle and radial distance from x,y coordinates */
  /*
     This is the angle of the line shoulder to wrist (S>W) with respect to ground, expressed in radians.
  */
  /*
     radial distance
  */

  /* Wrist position */

  float wrist_y = y - BASE_HGT;        // wrist position y  is this used  spread sheet j9
  float wrist_x = x - GRIPPER;                       //wrist position x   spreadsheet k9

  /* Shoulder to wrist distance ( AKA sw ) */

  float s_w = ( wrist_y * wrist_y ) + ( wrist_x * wrist_x );  //s>w

  float s_w_sqrt = sqrt( s_w );                             // S>W spreadsheet I14  shoulder to wrist This is the length of a line between
  // the shoulder and the wrist.
  //If it is longer than the length of the humerus and ulna, then there is no solution.

  /* s_w angle to ground */

  float a1 = asin( wrist_y / s_w_sqrt);          // This is the angle of the line S>W with respect to ground, expressed in radians.

  /* s_w angle to humerus */

  float a2 = acos((( hum_sq - elbow_to_wrist_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt )); //Spreadsheet called A2 at I16
  //This is the angle of the line S>W with respect to the humerus, expressed in radians

  /* shoulder angle ***********************************************/

  float shl_angle_r = 1.571 - a1 - a2;                   // pi - sw wrt gnd - sw to humerus angle
  // spreadsheet  I17

  int shl_angle_dm = 101 + degrees( shl_angle_r ); //This is the required shoulder angle, expressed in degrees. Q17
  int shl_angle_d = degrees( shl_angle_r );    // spreadsheet  J17  modified by Q17  to adjust to servo alignment
  // value should be between 25 and 150

  /* elbow angle ****************************************************/

  float angle_w = acos(( hum_sq + elbow_to_wrist_sq - s_w ) / ( 2 * HUMERUS * ELBOW_TO_WRIST ));  //This is the required elbow angle, expressed in radians.
  // Spreadsheet I18
  float elb_angle_r = angle_w - shl_angle_r + a1;

  int elb_angle_dm = 0 + degrees( elb_angle_r );      //This is the required elbow angle, expressed in degrees  J18, Q18
 
  //check limits of positioning and go to set zero if it is not and print out error *******
  bool waits = false;
  int BR = base_rotation_d + base_offset;
  if (BR > 180) BR = 180;
  if (BR < 0) BR = 0;
  base_servo.write(BR, servoSpeed, waits);            // base angle from function call

  int SR = shl_angle_dm + shoulder_offset;
  if (SR > 150) SR = 150;
  if (SR < 25) SR = 25;
  shoulder_servo.write(SR, servoSpeed, waits);          // includes adjust for servo

  int ER = elb_angle_dm + elbow_offset;
  if (ER > 145) ER = 145;
  if (ER < 55) ER = 55;
  elbow_servo.write(ER, servoSpeed, waits);             // includes adj for servo

  waits = false;
}
