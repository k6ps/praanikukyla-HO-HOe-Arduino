#include <SoftwareSerial.h>

#define BLUETOOTH_MODULE_BAUD_RATE 9600

// MOTOR-DRIVER L298
#define TRAIN1_MOTOR_ENA 9
#define TRAIN1_MOTOR_IN1 8
#define TRAIN1_MOTOR_IN2 7

#define BLOCK2_ENA 12
#define BLOCK3_ENA 4
#define BLOCK4_ENA 22

#define MAX_COMMAND_LENGTH 100

const char END_OF_COMMAND_CHAR = '\n';

SoftwareSerial Bluetooth(11, 10); // RX, TX

char cmd[MAX_COMMAND_LENGTH];
unsigned int cmdIndex;

// 24 speed
//byte speedArray [] = {20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,255};
bool directionForwardTrain1 = true;
unsigned int requestedTrainSpeed_1 = 0;
unsigned int selectedTrainSpeed_1 = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("INFO - setup - Starting up Präänikuküla Controls...");

  delay(500); // wait for bluetooth module to start
  Bluetooth.begin(9600);

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
    requestedTrainSpeed_1 = atoi(cmd + 13);
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
  
  if( cmdStartsWith("direction_train1") ) {
    setDirectionTrain1(!directionForwardTrain1);
  }
}

unsigned int getSelectedSpeedFromRequestedSpeed(unsigned int *requestedSpeed) {
  return *requestedSpeed;
/*  if (*requestedSpeed > 10 && *requestedSpeed <= 20) {
    return 20;
  } else if (*requestedSpeed > 20 && *requestedSpeed <= 30) {
    return 30;
  } else if (*requestedSpeed > 30 && *requestedSpeed <= 40) {
    return 40;
  } else if (*requestedSpeed > 40 && *requestedSpeed <= 50) {
    return 50;
  } else if (*requestedSpeed > 50 && *requestedSpeed <= 60) {
    return 60;
  } else if (*requestedSpeed > 60 && *requestedSpeed <= 70) {
    return 70;
  } else if (*requestedSpeed > 70 && *requestedSpeed <= 80) {
    return 80;
  } else if (*requestedSpeed > 80 && *requestedSpeed <= 90) {
    return 90;
  } else if (*requestedSpeed > 90 && *requestedSpeed <= 100) {
    return 100;
  } else if (*requestedSpeed > 100 && *requestedSpeed <= 110) {
    return 110;
  } else if (*requestedSpeed > 110 && *requestedSpeed <= 120) {
    return 120;
  } else if (*requestedSpeed > 120 && *requestedSpeed <= 130) {
    return 130;
  } else if (*requestedSpeed > 130 && *requestedSpeed <= 140) {
    return 140;
  } else if (*requestedSpeed > 140 && *requestedSpeed <= 150) {
    return 150;
  } else if (*requestedSpeed > 150 && *requestedSpeed <= 160) {
    return 160;
  } else if (*requestedSpeed > 160 && *requestedSpeed <= 170) {
    return 170;
  } else if (*requestedSpeed > 170 && *requestedSpeed <= 180) {
    return 180;
  } else if (*requestedSpeed > 180 && *requestedSpeed <= 190) {
    return 190;
  } else if (*requestedSpeed > 190 && *requestedSpeed <= 200) {
    return 200;
  } else if (*requestedSpeed > 200 && *requestedSpeed <= 210) {
    return 210;
  } else if (*requestedSpeed > 210 && *requestedSpeed <= 220) {
    return 220;
  } else if (*requestedSpeed > 220 && *requestedSpeed <= 230) {
    return 230;
  } else if (*requestedSpeed > 230 && *requestedSpeed <= 240) {
    return 240;
  } else if (*requestedSpeed > 240 && *requestedSpeed <= 255) {
    return 255;
  } else {
    return 0; 
  }*/
}

void setSpeedTrain1() {
    if (requestedTrainSpeed_1 >= 0 && requestedTrainSpeed_1 <= 255) {
      Serial.print("DEBUG - setSpeedTrain1 - requestedTrainSpeed_1 = ");
      Serial.print(requestedTrainSpeed_1);
      Serial.println();
      selectedTrainSpeed_1 = getSelectedSpeedFromRequestedSpeed(&requestedTrainSpeed_1);
      Serial.print("DEBUG - setSpeedTrain1 - selectedTrainSpeed_1 = ");
      Serial.print(selectedTrainSpeed_1);
      Serial.println();
      analogWrite(TRAIN1_MOTOR_ENA, selectedTrainSpeed_1);
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
