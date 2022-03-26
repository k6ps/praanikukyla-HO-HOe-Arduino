#include <SoftwareSerial.h>
#include <Servo.h>

// Bluetooth module baud rate. My module is BC-06, and it uses 9600.
#define BLUETOOTH_MODULE_BAUD_RATE 9600

// Pins for controlling two trains (actually just sets of blocks of railroad)
// independently of each other using a L298 DC motor controller.
// THe _ENA pins for controlling motor speeds via pulse width modulation (PWM)
// have to be pins where higher than default PWM frequency can be set.
#define TRAIN1_MOTOR_ENA 9
#define TRAIN1_MOTOR_IN1 8
#define TRAIN1_MOTOR_IN2 7

// Pins for switching on/off blocks of railroad.
// Block 1 is always on, there is no switch for it.
#define BLOCK2_ENA 12
#define BLOCK3_ENA 4
#define BLOCK4_ENA 22
#define BLOCK5_ENA 24

// Pins for controlling turnouts
#define TURNOUT_1_ENA 28
#define TURNOUT_1_SERVO 30
#define TURNOUT_1_SWITCH 31
#define TURNOUT_2_ENA 37
#define TURNOUT_2_SERVO 35
#define TURNOUT_2_SWITCH 36
#define TURNOUT_4_ENA 26
#define TURNOUT_4_SERVO 38
#define TURNOUT_4_SWITCH 39
#define TURNOUT_5_ENA 32
#define TURNOUT_5_SERVO 33
#define TURNOUT_5_SWITCH 34

// Turnout switch enc states. For the Weinert 74310 switch mechanisms, they are different wether the
// turnout is left or right. Here "LEFT" means the mechanism has moved to left if you look the mechanism
// from the switch side to the servo side, not that this is a left turnout.
#define TURNOUT_1_SWITCH_STATE_AT_LEFT HIGH
#define TURNOUT_1_SWITCH_STATE_AT_RIGHT LOW
#define TURNOUT_2_SWITCH_STATE_AT_LEFT LOW
#define TURNOUT_2_SWITCH_STATE_AT_RIGHT HIGH
#define TURNOUT_4_SWITCH_STATE_AT_LEFT LOW
#define TURNOUT_4_SWITCH_STATE_AT_RIGHT HIGH
#define TURNOUT_5_SWITCH_STATE_AT_LEFT LOW
#define TURNOUT_5_SWITCH_STATE_AT_RIGHT HIGH

// Turnout servo on times
#define MOVEMENT_TIME_FOR_TURNOUT_CHANGE 950

// Speeds of the turnout (continuous rotation) servos.
#define SERVO_SPEED_STOP 90
// There is some inprecision of these servos: the left and right correct speeds are not at symmetric distances from the stop speed.
#define TURNOUT_1_SERVO_SPEED_LEFT 80
#define TURNOUT_1_SERVO_SPEED_RIGHT 102
#define TURNOUT_2_SERVO_SPEED_LEFT 80
#define TURNOUT_2_SERVO_SPEED_RIGHT 102
#define TURNOUT_4_SERVO_SPEED_LEFT 78
#define TURNOUT_4_SERVO_SPEED_RIGHT 102
#define TURNOUT_5_SERVO_SPEED_LEFT 78
#define TURNOUT_5_SERVO_SPEED_RIGHT 102

// Constants and global variables related to reading commands from Bluetooth module
#define MAX_COMMAND_LENGTH 100
const char END_OF_COMMAND_CHAR = '\n';
char cmd[MAX_COMMAND_LENGTH];
unsigned int cmdIndex;

// I'm using the SoftwareSerial library here, to attach the Bluetooth module
// to some other pins than RX and TX. So i don't have to remove it every time
// i upload new program to my Arduino board.
SoftwareSerial Bluetooth(11, 13); // RX, TX

bool directionForwardTrain1 = true;
unsigned int trainSpeed_1 = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("====================================================");
  Serial.println("INFO - setup - Starting up Präänikuküla Controls...");

  delay(500); // wait for bluetooth module to start
  Bluetooth.begin(BLUETOOTH_MODULE_BAUD_RATE);

  cmdIndex = 0;

  // Initializing Motor-Driver
  pinMode(TRAIN1_MOTOR_ENA, OUTPUT);
  pinMode(TRAIN1_MOTOR_IN1, OUTPUT);
  pinMode(TRAIN1_MOTOR_IN2, OUTPUT);

  // Set PWM frequency for D9 & D10 of AtMega2560 (Arduino Mega R3) to 31372.55 Hz
  // because at lower frequencies the train motors make audible whizzing noise.
  TCCR2B = TCCR2B & B11111000 | B00000001;

  // Initialixing block switches
  pinMode(BLOCK2_ENA, OUTPUT);
  pinMode(BLOCK3_ENA, OUTPUT);
  pinMode(BLOCK4_ENA, OUTPUT);
  pinMode(BLOCK5_ENA, OUTPUT);

  // Initializing turnout switches and enable/disable pins
  pinMode(TURNOUT_1_ENA, OUTPUT);
  pinMode(TURNOUT_1_SWITCH, INPUT);
  pinMode(TURNOUT_2_ENA, OUTPUT);
  pinMode(TURNOUT_2_SWITCH, INPUT);
  pinMode(TURNOUT_4_ENA, OUTPUT);
  pinMode(TURNOUT_4_SWITCH, INPUT);
  pinMode(TURNOUT_5_ENA, OUTPUT);
  pinMode(TURNOUT_5_SWITCH, INPUT);

  // Disble all turnouts. Turnouts will be enabled only for the time they are turned.
  digitalWrite(TURNOUT_1_ENA, LOW);
  digitalWrite(TURNOUT_2_ENA, LOW);
  digitalWrite(TURNOUT_4_ENA, LOW);
  digitalWrite(TURNOUT_5_ENA, LOW);

  // Set default direction of train 1 to FORWARD
  setDirectionTrain1(true);

  // Block 2 is enabled by default
  Serial.println("DEBUG - setup - enabling block 2");
  digitalWrite(BLOCK2_ENA, HIGH);

  // Block 3 is disabled by default:
  Serial.println("DEBUG - setup - disabling block 3");
  digitalWrite(BLOCK3_ENA, LOW);

  // Block 4 is enabled by default:
  Serial.println("DEBUG - setup - enabling block 4");
  digitalWrite(BLOCK4_ENA, HIGH);

  // Block 5 is disabled by default:
  Serial.println("DEBUG - setup - disabling block 5");
  digitalWrite(BLOCK5_ENA, LOW);

}

void loop() {
  if (Bluetooth.available()) {
    char inChar = (char) Bluetooth.read();
    if(inChar == END_OF_COMMAND_CHAR) {
      cmd[cmdIndex] = 0;
      executeCommand();
      cmdIndex = 0;
    } else {      
      cmd[cmdIndex] = inChar;
      if (cmdIndex < (MAX_COMMAND_LENGTH - 1)) cmdIndex++;
    }
  }
}

boolean cmdStartsWith(const char *st) {
  for(int i=0; ; i++) {
    if(st[i]==0) return true;
    if(cmd[i]==0) return false;
    if(cmd[i]!=st[i]) return false;;
  }
  return false;
}


void executeCommand() {
  
  if( cmdStartsWith("speed_train1 ") ) {
    trainSpeed_1 = atoi(cmd + 13);
    setSpeedTrain1();
  }
  
  if( cmdStartsWith("block_2_on") ) {
    Serial.println("DEBUG - executeCommand - enabling block 2");
    digitalWrite(BLOCK2_ENA, HIGH);
  }
    
  if( cmdStartsWith("block_2_off") ) {
    Serial.println("DEBUG - executeCommand - disabling block 2");
    digitalWrite(BLOCK2_ENA, LOW);
  }
  
  if( cmdStartsWith("block_3_on") ) {
    Serial.println("DEBUG - executeCommand - enabling block 3");
    digitalWrite(BLOCK3_ENA, HIGH);
  }
    
  if( cmdStartsWith("block_3_off") ) {
    Serial.println("DEBUG - executeCommand - disabling block 3");
    digitalWrite(BLOCK3_ENA, LOW);
  }
  
  if( cmdStartsWith("block_4_on") ) {
    Serial.println("DEBUG - executeCommand - enabling block 4");
    digitalWrite(BLOCK4_ENA, HIGH);
  }
    
  if( cmdStartsWith("block_4_off") ) {
    Serial.println("DEBUG - executeCommand - disabling block 4");
    digitalWrite(BLOCK4_ENA, LOW);
  }
  
  if( cmdStartsWith("block_5_on") ) {
    Serial.println("DEBUG - executeCommand - enabling block 5");
    digitalWrite(BLOCK5_ENA, HIGH);
  }
    
  if( cmdStartsWith("block_5_off") ) {
    Serial.println("DEBUG - executeCommand - disabling block 5");
    digitalWrite(BLOCK5_ENA, LOW);
  }

  if( cmdStartsWith("turnout_1_calibrate") ) {
    Serial.println("DEBUG - executeCommand - calibrating turnout 1");
    calibrateTurnoutServo(TURNOUT_1_ENA, TURNOUT_1_SERVO, TURNOUT_1_SWITCH, TURNOUT_1_SERVO_SPEED_RIGHT, TURNOUT_1_SERVO_SPEED_LEFT, SERVO_SPEED_STOP, TURNOUT_1_SWITCH_STATE_AT_LEFT, TURNOUT_1_SWITCH_STATE_AT_RIGHT);
  }
 
  if( cmdStartsWith("turnout_1_straight") ) {
    Serial.println("DEBUG - executeCommand - turning turnout 1 straight");
    // Turnout 1 is a left turnout. So when the turnout mechanism has moved to left, train goes straight. 
    // when turnout mechanism has moved to right, train turns left.
    turnTurnoutServo(TURNOUT_1_ENA, TURNOUT_1_SERVO, TURNOUT_1_SWITCH, TURNOUT_1_SWITCH_STATE_AT_LEFT, TURNOUT_1_SERVO_SPEED_LEFT, MOVEMENT_TIME_FOR_TURNOUT_CHANGE, SERVO_SPEED_STOP);
  }
 
  if( cmdStartsWith("turnout_1_turn") ) {
    Serial.println("DEBUG - executeCommand - turning turnout 1 turn");
    // Turnout 1 is a left turnout. So when the turnout mechanism has moved to right, train turns left.
    turnTurnoutServo(TURNOUT_1_ENA, TURNOUT_1_SERVO, TURNOUT_1_SWITCH, TURNOUT_1_SWITCH_STATE_AT_RIGHT, TURNOUT_1_SERVO_SPEED_RIGHT, MOVEMENT_TIME_FOR_TURNOUT_CHANGE, SERVO_SPEED_STOP);
  }
 
  if( cmdStartsWith("turnout_2_calibrate") ) {
    Serial.println("DEBUG - executeCommand - calibrating turnout 2");
    calibrateTurnoutServo(TURNOUT_2_ENA, TURNOUT_2_SERVO, TURNOUT_2_SWITCH, TURNOUT_2_SERVO_SPEED_RIGHT, TURNOUT_2_SERVO_SPEED_LEFT, SERVO_SPEED_STOP, TURNOUT_2_SWITCH_STATE_AT_LEFT, TURNOUT_2_SWITCH_STATE_AT_RIGHT);
  }
 
  if( cmdStartsWith("turnout_2_straight") ) {
    Serial.println("DEBUG - executeCommand - turning turnout 2 straight");
    // Turnout 2 is a rigth turnout. So when the turnout mechanism has moved to right, train goes straight. 
    // when turnout mechanism has moved to left, train turns right.
    turnTurnoutServo(TURNOUT_2_ENA, TURNOUT_2_SERVO, TURNOUT_2_SWITCH, TURNOUT_2_SWITCH_STATE_AT_RIGHT, TURNOUT_2_SERVO_SPEED_RIGHT, MOVEMENT_TIME_FOR_TURNOUT_CHANGE, SERVO_SPEED_STOP);
  }
 
  if( cmdStartsWith("turnout_2_turn") ) {
    Serial.println("DEBUG - executeCommand - turning turnout 2 turn");
    // Turnout 2 is a rigth turnout. So when the turnout mechanism has moved to right, train goes straight. 
    // when turnout mechanism has moved to left, train turns right.
    turnTurnoutServo(TURNOUT_2_ENA, TURNOUT_2_SERVO, TURNOUT_2_SWITCH, TURNOUT_2_SWITCH_STATE_AT_LEFT, TURNOUT_2_SERVO_SPEED_LEFT, MOVEMENT_TIME_FOR_TURNOUT_CHANGE, SERVO_SPEED_STOP);
  }
 
  if( cmdStartsWith("turnout_4_calibrate") ) {
    Serial.println("DEBUG - executeCommand - calibrating turnout 4");
    calibrateTurnoutServo(TURNOUT_4_ENA, TURNOUT_4_SERVO, TURNOUT_4_SWITCH, TURNOUT_4_SERVO_SPEED_RIGHT, TURNOUT_4_SERVO_SPEED_LEFT, SERVO_SPEED_STOP, TURNOUT_4_SWITCH_STATE_AT_LEFT, TURNOUT_4_SWITCH_STATE_AT_RIGHT);
  }
 
  if( cmdStartsWith("turnout_4_straight") ) {
    Serial.println("DEBUG - executeCommand - turning turnout 4 straight");
    // Turnout 4 is a rigth turnout. So when the turnout mechanism has moved to right, train goes straight. 
    // when turnout mechanism has moved to left, train turns right.
    turnTurnoutServo(TURNOUT_4_ENA, TURNOUT_4_SERVO, TURNOUT_4_SWITCH, TURNOUT_4_SWITCH_STATE_AT_RIGHT, TURNOUT_4_SERVO_SPEED_RIGHT, MOVEMENT_TIME_FOR_TURNOUT_CHANGE, SERVO_SPEED_STOP);
    // As turnouts 4 and 5 are connected in a reverse N shaped set, it does not make sense to have one of them straight and the other turned. So, set also the other one to the same state:
    turnTurnoutServo(TURNOUT_5_ENA, TURNOUT_5_SERVO, TURNOUT_5_SWITCH, TURNOUT_5_SWITCH_STATE_AT_RIGHT, TURNOUT_5_SERVO_SPEED_RIGHT, MOVEMENT_TIME_FOR_TURNOUT_CHANGE, SERVO_SPEED_STOP);
  }
 
  if( cmdStartsWith("turnout_4_turn") ) {
    Serial.println("DEBUG - executeCommand - turning turnout 4 turn");
    // Turnout 4 is a rigth turnout. So when the turnout mechanism has moved to right, train goes straight. 
    // when turnout mechanism has moved to left, train turns right.
    turnTurnoutServo(TURNOUT_4_ENA, TURNOUT_4_SERVO, TURNOUT_4_SWITCH, TURNOUT_4_SWITCH_STATE_AT_LEFT, TURNOUT_4_SERVO_SPEED_LEFT, MOVEMENT_TIME_FOR_TURNOUT_CHANGE, SERVO_SPEED_STOP);
    // As turnouts 4 and 5 are connected in a reverse N shaped set, it does not make sense to have one of them straight and the other turned. So, set also the other one to the same state:
    turnTurnoutServo(TURNOUT_5_ENA, TURNOUT_5_SERVO, TURNOUT_5_SWITCH, TURNOUT_5_SWITCH_STATE_AT_LEFT, TURNOUT_5_SERVO_SPEED_LEFT, MOVEMENT_TIME_FOR_TURNOUT_CHANGE, SERVO_SPEED_STOP);
  }
 
  if( cmdStartsWith("turnout_5_calibrate") ) {
    Serial.println("DEBUG - executeCommand - calibrating turnout 5");
    calibrateTurnoutServo(TURNOUT_5_ENA, TURNOUT_5_SERVO, TURNOUT_5_SWITCH, TURNOUT_5_SERVO_SPEED_RIGHT, TURNOUT_5_SERVO_SPEED_LEFT, SERVO_SPEED_STOP, TURNOUT_5_SWITCH_STATE_AT_LEFT, TURNOUT_5_SWITCH_STATE_AT_RIGHT);
  }
 
  if( cmdStartsWith("turnout_5_straight") ) {
    Serial.println("DEBUG - executeCommand - turning turnout 5 straight");
    // Turnout 5 is a rigth turnout. So when the turnout mechanism has moved to right, train goes straight. 
    // when turnout mechanism has moved to left, train turns right.
    turnTurnoutServo(TURNOUT_5_ENA, TURNOUT_5_SERVO, TURNOUT_5_SWITCH, TURNOUT_5_SWITCH_STATE_AT_RIGHT, TURNOUT_5_SERVO_SPEED_RIGHT, MOVEMENT_TIME_FOR_TURNOUT_CHANGE, SERVO_SPEED_STOP);
    // As turnouts 4 and 5 are connected in a reverse N shaped set, it does not make sense to have one of them straight and the other turned. So, set also the other one to the same state:
    turnTurnoutServo(TURNOUT_4_ENA, TURNOUT_4_SERVO, TURNOUT_4_SWITCH, TURNOUT_4_SWITCH_STATE_AT_RIGHT, TURNOUT_4_SERVO_SPEED_RIGHT, MOVEMENT_TIME_FOR_TURNOUT_CHANGE, SERVO_SPEED_STOP);
  }
 
  if( cmdStartsWith("turnout_5_turn") ) {
    Serial.println("DEBUG - executeCommand - turning turnout 5 turn");
    // Turnout 5 is a rigth turnout. So when the turnout mechanism has moved to right, train goes straight. 
    // when turnout mechanism has moved to left, train turns right.
    turnTurnoutServo(TURNOUT_5_ENA, TURNOUT_5_SERVO, TURNOUT_5_SWITCH, TURNOUT_5_SWITCH_STATE_AT_LEFT, TURNOUT_5_SERVO_SPEED_LEFT, MOVEMENT_TIME_FOR_TURNOUT_CHANGE, SERVO_SPEED_STOP);
    // As turnouts 4 and 5 are connected in a reverse N shaped set, it does not make sense to have one of them straight and the other turned. So, set also the other one to the same state:
    turnTurnoutServo(TURNOUT_4_ENA, TURNOUT_4_SERVO, TURNOUT_4_SWITCH, TURNOUT_4_SWITCH_STATE_AT_LEFT, TURNOUT_4_SERVO_SPEED_LEFT, MOVEMENT_TIME_FOR_TURNOUT_CHANGE, SERVO_SPEED_STOP);
  }
 
  if( cmdStartsWith("direction_train1") ) {
    setDirectionTrain1(!directionForwardTrain1);
  }
}

void setSpeedTrain1() {
    if (trainSpeed_1 >= 0 && trainSpeed_1 <= 255) {
      Serial.print("DEBUG - setSpeedTrain1 - trainSpeed_1 = ");
      Serial.print(trainSpeed_1);
      Serial.println();
      analogWrite(TRAIN1_MOTOR_ENA, trainSpeed_1);
    }
}

void setDirectionTrain1(const bool directionForward) {
  directionForwardTrain1 = directionForward;
  if (directionForward) {
    Serial.println("DEBUG - setDirectionTrain1 - Setting train 1 to move forward...");
    digitalWrite(TRAIN1_MOTOR_IN1, HIGH);
    digitalWrite(TRAIN1_MOTOR_IN2, LOW);
  } else {
    Serial.println("DEBUG - setDirectionTrain1 - Setting train 1 to move backward...");
    digitalWrite(TRAIN1_MOTOR_IN1, LOW);
    digitalWrite(TRAIN1_MOTOR_IN2, HIGH);
  }
}

/* 
 * I have Weinert Modellbau 74310 turnout point mechanisms (because i wanted the rotating turnout lanterns - 72381 and the lke - in my layout), but some old SG90 continuous 
 * rotation microservos, which is probably not the best combination.  Standard servos would be better, i guess.  So i'm relying on precise millisecond timing to turn my 
 * turnouts with these servos. I'm using the "frog" polarity switch of the turnout mechanism to detect which state the turnout is in. Initially i need to calibrate the 
 * turnout mechanims and servo, to make sure the turnout movement happens around the center of the possible movement range of the turnout mechanism, and the switching 
 * happens somewhere in the middle of this movement. 
 */
void calibrateTurnoutServo(
  const int servoEnabledPin,
  const int servoControlPin, 
  const int turnoutSwitchInputPin, 
  const int servoSpeedRight, 
  const int servoSpeedLeft, 
  const int servoSpeedStop,
  const int switchStateAtLeft,
  const int switchStateAtRight
) {
  const unsigned int MOVEMENT_COUNT_FOR_CALIBRATION_LIMIT = 100;
  const unsigned int MOVEMENT_TIME_FROM_SWITCH_POSITION_TO_LEFT_END = 30;
  digitalWrite(servoEnabledPin, HIGH);
  int turnoutSwitchState = digitalRead(turnoutSwitchInputPin);
  debugSwitchState(turnoutSwitchInputPin, String("beginning of calibration"));
  Servo myservo;
  myservo.attach(servoControlPin);  // attaches the servo on pin 9 to the servo object
  myservo.write(servoSpeedStop);
  delay(100);
  unsigned int countForSafety = 0;
  // If the switch is in "left" side, then first move to right until the switch switches to "right" state.
  turnoutSwitchState = digitalRead(turnoutSwitchInputPin);
  while (turnoutSwitchState == switchStateAtLeft && countForSafety <= MOVEMENT_COUNT_FOR_CALIBRATION_LIMIT) {
    myservo.write(servoSpeedRight);
    delay(10);
    turnoutSwitchState = digitalRead(turnoutSwitchInputPin);
    countForSafety++;
  }
  // Move back left in small steps until the switch goes again to "left" state.
  countForSafety = 0;
  while (turnoutSwitchState == switchStateAtRight && countForSafety <= MOVEMENT_COUNT_FOR_CALIBRATION_LIMIT) {
    myservo.write(servoSpeedLeft);
    delay(10);
    turnoutSwitchState = digitalRead(turnoutSwitchInputPin);
    countForSafety++;
  }
  // Now we have found the point where the switch happens, so we know where we are.  
  // Moving left for the experimentally found anmount of time (depends on the screw position of the switch), 
  // to reach the left end of the switchable area:
  myservo.write(servoSpeedLeft);
  delay(MOVEMENT_TIME_FROM_SWITCH_POSITION_TO_LEFT_END);
  myservo.write(servoSpeedStop);
  // wait a bit:
  delay(100);
  debugSwitchState(turnoutSwitchInputPin, String("end of calibration"));
  myservo.detach();
  digitalWrite(servoEnabledPin, LOW);
}

void turnTurnoutServo(
  const int servoEnabledPin,
  const int servoControlPin, 
  const int turnoutSwitchInputPin, 
  const int turnoutSwitchStateWhenDone, 
  const int servoSpeed, 
  const int turnoutMovementTimeMillis, 
  const int servoSpeedStop
) {
  digitalWrite(servoEnabledPin, HIGH);
  int turnoutSwitchState = digitalRead(turnoutSwitchInputPin);
  if (turnoutSwitchState != turnoutSwitchStateWhenDone) {
    Servo myservo;
    myservo.attach(servoControlPin);
    debugSwitchState(turnoutSwitchInputPin, String("beginning of turn"));
    myservo.write(servoSpeed);
    delay(turnoutMovementTimeMillis);
    myservo.write(servoSpeedStop);
    delay(100);
    debugSwitchState(turnoutSwitchInputPin, String("end of turn"));
    myservo.detach();
  }
  turnoutSwitchState = digitalRead(turnoutSwitchInputPin);
  if (turnoutSwitchState != turnoutSwitchStateWhenDone) {
    Serial.print("#### ERROR! #### - turnout switch is not in expected state after turn!\n");
  }
  digitalWrite(servoEnabledPin, LOW);
}


void debugSwitchState(const int turnoutSwitchInputPin, String switchStateMoment) {
  int turnoutSwitchState = digitalRead(turnoutSwitchInputPin);
  Serial.print("Switch state (");
  Serial.print(switchStateMoment);
  Serial.print(") : ");
  if (turnoutSwitchState == HIGH) {
    Serial.print("<HIGH>");
  } else if (turnoutSwitchState == LOW) {
    Serial.print("<LOW>");
  } else {
    Serial.print("<UNDEFINED>");
  }
  Serial.print("\n");  
}
