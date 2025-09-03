#include <Servo.h>

Servo panServo;
Servo tiltServo;

int panPosition = 90;
int tiltPosition = 90;

const int panPin = 9;
const int tiltPin = 10;

String inputString = "";
boolean stringComplete = false;

void setup() {
  panServo.attach(panPin);
  tiltServo.attach(tiltPin);
  
  panServo.write(panPosition);
  
  
  Serial.begin(9600);
  inputString.reserve(200);
  
  Serial.println("Arduino Turret Ready");
}

void loop() {
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  delay(15);
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void processCommand(String command) {
  int separatorIndex = command.indexOf(':');
  
  if (separatorIndex > 0) {
    String panStr = command.substring(0, separatorIndex);
    String tiltStr = command.substring(separatorIndex + 1);
    
    int newPan = panStr.toInt();
    int newTilt = tiltStr.toInt();
    
    newPan = constrain(newPan, 0, 180);
    newTilt = constrain(newTilt, 0, 180);
    
    if (newPan != panPosition) {
      panPosition = newPan;
      panServo.write(panPosition);
    }
    
    if (newTilt != tiltPosition) {
      tiltPosition = newTilt;
      tiltServo.write(tiltPosition);
    }
    
    Serial.print("Moved to:");
    Serial.print(panPosition);
    Serial.print(":");
    Serial.println(tiltPosition);
  }
}
