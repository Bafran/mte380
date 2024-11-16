#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define NUM_SERVOS 6    // Using servos 0-5
#define SERVOMIN 150    // Minimum pulse length
#define SERVOMAX 600    // Maximum pulse length

// Define home positions and directions for each motor
const int homePositions[NUM_SERVOS] = {150, 450, 150, 450, 150, 450};
const int directions[NUM_SERVOS] = {1, -1, 1, -1, 1, -1};  // -1 for -, 1 for +

String inputString = "";
boolean stringComplete = false;

// Map serve input angle to a ticks value
int mapServoValue(int input) {
    return map(input, 0, 180, 0, 450);
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);
  
  // Move all servos to home positions on startup
  for(int i = 0; i < NUM_SERVOS; i++) {
    pwm.setPWM(i, 0, homePositions[i]);
  }
  
  Serial.println("Enter 6 comma-separated values for motor adjustments:");
}

void loop() {
  // Read serial input
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  // Process input when complete
  if (stringComplete) {
    // Parse comma-separated values
    int values[NUM_SERVOS];
    int valueIndex = 0;
    String currentValue = "";
    
    // Parse the input string
    for(int i = 0; i <= inputString.length(); i++) {
      if(i == inputString.length() || inputString.charAt(i) == ',') {
        if(valueIndex < NUM_SERVOS) {
          values[valueIndex] = currentValue.toInt();
          valueIndex++;
          currentValue = "";
        }
      } else {
        currentValue += inputString.charAt(i);
      }
    }

    // Check if we got all 6 values
    if(valueIndex == NUM_SERVOS) {
      // Move each servo according to its home position and direction
      for(int i = 0; i < NUM_SERVOS; i++) {
        int targetPosition = homePositions[i] + (directions[i] * mapServoValue(values[i]));
        
        // Constrain the target position
        targetPosition = constrain(targetPosition, SERVOMIN, SERVOMAX);
        
        pwm.setPWM(i, 0, targetPosition);
      }
    } else {
      Serial.println("Error: Please enter exactly 6 comma-separated values");
    }
    
    // Reset for next input
    inputString = "";
    stringComplete = false;
    Serial.println("Enter 6 comma-separated values for motor adjustments:");
  }
}