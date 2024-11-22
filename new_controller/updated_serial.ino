#include <Adafruit_PWMServoDriver.h>

// Servo Configuration
#define NUM_SERVOS 6
#define SERVO_FREQ 50
#define SERVOMIN 150
#define SERVOMAX 600

// Global variables
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int buffer[8] = { 0 };
int bufferIndex = 0;
bool packetStarted = false;
byte sum = 0;

const int homePositions[NUM_SERVOS] = {225, 375, 225, 375, 225, 375};
const int corrections[NUM_SERVOS] = {0,0,5,22,14,0};
const int directions[NUM_SERVOS] = {1, -1, 1, -1, 1, -1};  // -1 for -, 1 for +

void setup() {
  Serial.begin(115200);  // Higher baud rate for faster communication

  // Initialize servo driver
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  // Initialize all servos to middle position
  for(int i = 0; i < NUM_SERVOS; i++) {
    pwm.setPWM(i, 0, homePositions[i] + (mapServoValue(corrections[i]) * directions[i]));
  }
  
  Serial.println("System initialized");
}

int mapServoValue(int input) {
  return map(input, 0, 180, 0, 450);
}

void processSerialData() {
  int i = 0;

  while (Serial.available()) {  // Check if we have at least 8 bytes
    byte data = Serial.read();

    if (data == 0xFF) {
      packetStarted = true;
      i = 0;
      continue;
    }

    data -= 100;

    if (packetStarted) {
      int targetPosition = homePositions[i] + (directions[i] * data) + (mapServoValue(corrections[i]) * directions[i]);
      pwm.setPWM(i, 0, targetPosition);

      if (i == 5) {
        packetStarted = false;
      }

      i++;
    }
  }
}

void loop() {
  processSerialData();
}