/* Include necessary libraries */
//Arduino & serial communication:
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <HardwareSerial.h>
// Sensors (Ultrasonic, Compass, BMI):
#include <NewPing.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <BMI160Gen.h>
//#include <CurieIMU.h>
// Motors & Servos:
#include <L298N.h>
#include <ESP32Servo.h>
// PS5 controller:
#include <ps5Controller.h>

// /* Library and variable initialization for degree calculations from BMI160 */
// #include <MadgwickAHRS.h>
// Madgwick filter;
// unsigned long microsPerReading, microsPrevious;


/* Assign pin names, based on GPIO number: */
// Servos
const int shaftPWM = 16;
const int shaftPWMInvert = 4;
const int scraperPWM = 17;
// Motor Drivers
const int trayMDriverIn2 = 15;
const int trayMDriverIn1 = 2;
const int trackMDriverIn4 = 25;
const int trackMDriverIn3 = 26;
//const int trackMDriverIn2 = 27;
const int trackMDriverIn2 = 5;
const int trackMDriverIn1 = 14;
const int trayEnable = 4;
const int track1Enable = 13;
const int track2Enable = 12;
// Ultrasonic Sensors
const int frontUltrasonicEcho = 34;
const int frontUltrasonicTrig = 32;
const int binUltrasonicEcho = 35;
const int binUltrasonicTrig = 33;
// UART Communication w/ RPi
const int espRX = 3;
const int espTX = 1;
//const int espRX = 16;
//const int espTX = 17;
// I2C Communication w/ Compass
const int I2CSerialClock = 22;
const int CompassData = 21;
// SPI Communication w/ BMI
//const int BMIChipSelect = 5;
const int BMISerialClock = 18;
const int BMIMISO = 19;
const int BMIMOSI = 23;

/* Initialize necessary structures (from libraries): */
// Servos
Servo shaftServo;
Servo handServo;
Servo shaftServoInvert;
ESP32PWM pwm;
// Motor Drivers
L298N rightMotor(track1Enable, trackMDriverIn1, trackMDriverIn2);
L298N leftMotor(track2Enable, trackMDriverIn3, trackMDriverIn4);
L298N trayMotor(trayEnable, trayMDriverIn1, trayMDriverIn2);
// Ultrasonic Sensors
NewPing sonar(frontUltrasonicTrig, frontUltrasonicEcho, 200);
NewPing binFill(binUltrasonicTrig, binUltrasonicEcho, 40);
//UARTserial Comm w/ RPi
 HardwareSerial PiComm(2);
// Compass
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup() {
  // put your setup code here, to run once:
  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  // Standard 50hz servos
  shaftServo.setPeriodHertz(50);
  shaftServoInvert.setPeriodHertz(50);
  handServo.setPeriodHertz(50);

  // Mask MAC address onto PS5 controller
  ps5.begin("F0:2F:4B:0F:78:59");

  // Initialize the I2C library
  Wire.begin();
  
  //Initialize serial communication with a 9600 baud rate, through microUSB
  Serial.begin(9600);
  while (!Serial);  // wait for the serial port to open
  Serial.println("USB Serial Initialized"); // output to console when opened

  //Initialize serial communication to RPi (9600 bps, 8 bits, No Parity, 1 Stop Bit)
  // If only connected to RPi, use next line and RX0/TX) pins
  //PiComm.begin(9600, SERIAL_8N1, espRX, espTX);
  // If connected through USB to ESP use RX2/TX2 pins, for flashing
  PiComm.begin(9600);
  //PiComm.setDebugOutput(true);
  while(!PiComm); // wait for serial port to open
  //Serial.println("Serial Initialized"); // output to console when opened
  
  // Initialize the SPI library
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);  //divide the clock by 4

  // Set pins mode for each output
  pinMode(shaftPWM, OUTPUT);
  pinMode(shaftPWMInvert, OUTPUT);
  pinMode(scraperPWM, OUTPUT);
  pinMode(trayMDriverIn2, OUTPUT);
  pinMode(trayMDriverIn1, OUTPUT);
  pinMode(trayEnable, OUTPUT);
  pinMode(trackMDriverIn4, OUTPUT);
  pinMode(trackMDriverIn3, OUTPUT);
  pinMode(trackMDriverIn2, OUTPUT);
  pinMode(trackMDriverIn1, OUTPUT);
  pinMode(track1Enable, OUTPUT);
  pinMode(track2Enable, OUTPUT);
  pinMode(frontUltrasonicTrig, OUTPUT);
  pinMode(binUltrasonicTrig, OUTPUT);
  //pinMode(BMIChipSelect, OUTPUT);
  // Set pins mode for each input
  pinMode(frontUltrasonicEcho, INPUT);
  pinMode(binUltrasonicEcho, INPUT);
  // Serial.println("Pins Defined"); // output to console that pins were defined

  // Print error if magnetometor not initialized
  if(!mag.begin()) {
    /* There was a problem detecting the HMC5883 ... check your connections */
    // Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

  // int smallMinUs = 500; // 500 for SG90 servo (manually calibrated)
  // int smallMaxUs = 2400; // 2400 for SG90 servo (manually calibrated)
  int bigMinUs = 365; // 365 for MG996R servo (manually calibrated)
  int bigMaxUs = 2470; // 2460-2480 for MG996R servo (manually calibrated)

  shaftServo.attach(shaftPWM, bigMinUs, bigMaxUs);
  shaftServoInvert.attach(shaftPWMInvert, bigMinUs, bigMaxUs);
  handServo.attach(scraperPWM, bigMinUs, bigMaxUs);
  pwm.attachPin(27, 10000); //10khz frequency

  //Give sensors & serial communication time to initialize (100 ms)
  delay(100);
}

float convertRawAcceleration(int aRaw) {  // For BMI
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {  // For BMI
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

int compassDegree() { // For Magnetometer
  // Get a new sensor event
  sensors_event_t event;
  mag.getEvent(&event);

  // Read heading based on the x & y axis
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Add 'Declination Angle', corrects for error in local magnetic field
  // MUST BE UPDATED MANUALLY BASED ON LOCATIION: magnetic-declination.com
  float declinationAngle = 0.0573;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
  
  // Convert radians to degrees
  float headingDegrees = heading * 180/PI;
  // Return degree currently facing
  return headingDegrees;
}

void turn(int degree) { // For L298Ns
  // Initialize motor driver variables & assign arbitrarily (will be reassigned)
  L298N forwardMotor = trayMotor;
  L298N reverseMotor = trayMotor;

  // Get degree robot is initially facing
  int initDegree = compassDegree();
  int targetDegree, halfwayDegree;
  // Print message to computer (troubleshooting)
  // Serial.print("Initial degree of robot: ");
  // Serial.println(initDegree);

  // Assign which motor will move forward/reverse based on turning
  if (degree < 0) {   // Negative degree = Turn right
    forwardMotor = leftMotor;
    reverseMotor = rightMotor;
  }
  else if (degree > 0) {  // Positive degree = Turn left
    forwardMotor = rightMotor;
    reverseMotor = leftMotor;
  }
  else {  // No turn necessary
    return;
  }

  // Correct for any turns made past the 0/360 degree point (North)
  if ((initDegree - degree) >= 360) {
    targetDegree = initDegree - degree - 360;
    halfwayDegree = initDegree - (degree / 2) - 360;
  }
  else if ((initDegree - degree) < 0) {
    targetDegree = initDegree - degree + 360;
    halfwayDegree = initDegree - (degree / 2) + 360;
  }
  else {
    targetDegree = initDegree - degree;
    halfwayDegree = initDegree - (degree / 2);
  }

  if (halfwayDegree >= 360) {
    halfwayDegree = halfwayDegree - 360;
  }
  else if (halfwayDegree < 0) {
    halfwayDegree = halfwayDegree + 360;
  }
  // Serial.println(halfwayDegree);

  unsigned short dutyCycle = 90;
  int currentDegree;
  // Turn robot at incremental speed (no sudden burst of movement)
  for (; dutyCycle < 200; dutyCycle = dutyCycle + 5) {
    forwardMotor.setSpeed(dutyCycle);
    reverseMotor.setSpeed(dutyCycle);
    // Tracks must be rotating in opposite directions for static turn
    forwardMotor.forward();
    reverseMotor.backward();
    // Print message to computer (troubleshooting)
    // Serial.print("Turn with duty cycle: ");
    // Serial.print(dutyCycle);
    // Serial.print(" with function reading of: ");
    // Serial.println(forwardMotor.getSpeed());
    // Constantly check current degree reading and stop accelerating when halfway through turning
    currentDegree = compassDegree();
    // Serial.println(currentDegree);
    //if (currentDegree == halfwayDegree || currentDegree == halfwayDegree + 1 || currentDegree == halfwayDegree - 1 || currentDegree == halfwayDegree + 2 || currentDegree == halfwayDegree - 2) {
    if (currentDegree >= halfwayDegree-3  && currentDegree <= halfwayDegree+3) {
      // Print message to computer (troubleshooting)
      // Serial.print("Slowing turn at degree: ");
      // Serial.println(halfwayDegree);
      break;
    }
    delay(50);
  }

  // Keep turning even when max speed is reached, until reaching halfway point of turn
  //while (currentDegree != halfwayDegree && currentDegree != halfwayDegree + 1 && currentDegree != halfwayDegree - 1 && currentDegree != halfwayDegree + 2 && currentDegree != halfwayDegree - 2) {
  while (!(currentDegree >= halfwayDegree-3  && currentDegree <= halfwayDegree+3)) {
    currentDegree = compassDegree();
    // Print message to computer (troubleshooting)
    // Serial.print("Current degree: ");
    // Serial.print(currentDegree);
    // Serial.print(" waiting for halfway angle: ");
    // Serial.println(halfwayDegree);
  }
  // Print message to computer (troubleshooting)
  // Serial.print("Slowing turn at degree: ");
  // Serial.println(currentDegree);

  // Decrease speed of turn after halfway (no sudden stop)
  for (; dutyCycle > 90; dutyCycle = dutyCycle - 5) {
    forwardMotor.setSpeed(dutyCycle);
    reverseMotor.setSpeed(dutyCycle);
    // Tracks must be rotating in opposite directions for static turn
    forwardMotor.forward();
    reverseMotor.backward();
    // Print message to computer (troubleshooting)
    // Serial.print("Turn with duty cycle: ");
    // Serial.print(dutyCycle);
    // Serial.print(" with function reading of: ");
    // Serial.println(forwardMotor.getSpeed());
    // Constantly check current degree reading and stop when desired degree change is reached
    currentDegree = compassDegree();
    if (currentDegree > targetDegree -3  && currentDegree < targetDegree + 3) {
      // Print message to computer (troubleshooting)
      // Serial.print("Slowing turn at degree change: ");
      // Serial.println(targetDegree);
      break;
    }
    delay(50);
  }

  // Keep turning even when low speed is reached, until reaching desired degree change
  while (!(currentDegree >= targetDegree -3  && currentDegree <= targetDegree + 3)) {
    currentDegree = compassDegree();
    // Print message to computer (troubleshooting)
    // Serial.print("Current degree: ");
    // Serial.println(currentDegree);
    // Serial.print("Target degree: ");
    // Serial.println(targetDegree);
  }
  // Print message to computer (troubleshooting)
  // Serial.print("Stopping turn at degree change: ");
  // Serial.println(targetDegree);

  // Fully stop turning, should be facing object now
  forwardMotor.stop();
  reverseMotor.stop();
}

void openTray() {
  unsigned short dutyCycle = 50;
  // Move DC motor forward with increasing speed
  for (; dutyCycle < 250; dutyCycle = dutyCycle + 5) {
    trayMotor.setSpeed(dutyCycle);
    trayMotor.backward();
    // Serial.print("Opening with duty cycle: ");
    // Serial.print(dutyCycle);
    // Serial.print(" with function reading of: ");
    // Serial.println(trayMotor.getSpeed());
    delay(50);
  }
  // Move DC motor forward with decreasing speed
  for (; dutyCycle > 80; dutyCycle = dutyCycle - 5) {
    trayMotor.setSpeed(dutyCycle);
    trayMotor.backward();
    // Serial.print("Opening with duty cycle: ");
    // Serial.print(dutyCycle);
    // Serial.print(" with function reading of: ");
    // Serial.println(trayMotor.getSpeed());
    delay(50);
  }
}

void closeTray() {
  unsigned short dutyCycle = 50;
  // Move DC motor forward with increasing speed
  for (; dutyCycle < 250; dutyCycle = dutyCycle + 5) {
    trayMotor.setSpeed(dutyCycle);
    trayMotor.forward();
    // Serial.print("Clowing with duty cycle: ");
    // Serial.print(dutyCycle);
    // Serial.print(" with function reading of: ");
    // Serial.println(trayMotor.getSpeed());
    delay(50);
  }
  // Move DC motor forward with decreasing speed
  for (; dutyCycle > 80; dutyCycle = dutyCycle - 5) {
    trayMotor.setSpeed(dutyCycle);
    trayMotor.forward();
    // Serial.print("Closing with duty cycle: ");
    // Serial.print(dutyCycle);
    // Serial.print(" with function reading of: ");
    // Serial.println(trayMotor.getSpeed());
    delay(50);
  }
}

// Make sure to leave shaft lowered and hands closed whenever turning off/rebooting!!
int lastPosition = 0;
bool closed = true;
bool raised = false;

void closeHands() {
  // No need to close hands if they are already closed
  if (closed) {
    return;
  }

  // Turn from 170 degrees to 10 degrees in steps of 1 degree
  for (int pos = 170; pos >= 0; pos -= 1) { 
		handServo.write(pos);
		delay(10);             // waits 10ms for the servo to reach the position
	}

  // Print message to computer (troubleshooting)
  closed = true;
  // Serial.print("Hand servo value: ");
  // Serial.println(handServo.read());
}

void openHands() {
  // Hands cannot be opened if they are already open or the shaft is raised
  if (!closed || lastPosition == 130) {
    return;
  }

  // Turn from 10 degrees to 170 degrees in steps of 1 degree
  for (int pos = 0; pos <= 170; pos += 1) { 
		handServo.write(pos);
		delay(10);             // waits 10ms for the servo to reach the position
	}

  // Print message to computer (troubleshooting)i
  closed = false;
  // Serial.print("Hand servo value: ");
  // Serial.println(handServo.read());
}

void lowerShaft() {
  // To ensure lowering isn't called twice in a row (causes crazy jitter)
  if (lastPosition < 130) {
    return;
  }

  // Turn from 180 degrees to 30 degrees in steps of 1 degree
  for (int pos = 130; pos >= 5; pos -= 1) { 
		shaftServo.write(pos);
    shaftServoInvert.write((130-pos)+5);
    if (pos == 70) {
      lastPosition = shaftServo.read();
      openHands();
    }
		delay(20);
	}

  // Print message to computer (troubleshooting)
  lastPosition = shaftServo.read();
  raised = false;
  // Serial.print("Shaft servo value: ");
  // Serial.println(lastPosition);
}

void raiseShaft() {
  // To ensure raising isn't called twice in a row (causes crazy jitter)
  // also can't raise shaft if hands are open (would cause mechanical failure)
  if (lastPosition > 5 || !closed) {
    return;
  }

  // Turn from 50 degrees to 160 degrees in steps of 1 degree
  for (int pos = 5; pos <= 130; pos += 1) { 
		shaftServo.write(pos);
    shaftServoInvert.write((130-pos)+5);
		delay(20);             // waits 20ms for the servo to reach the position
	}

  // Print message to computer (troubleshooting)
  lastPosition = shaftServo.read();
  raised = true;
  // Serial.print("Shaft servo value: ");
  // Serial.println(shaftServo.read());
}

char data;

void loop() {
  // put your main code here, to run repeatedly:
  // Re-initialize these necessary variable at every iteration of loop
  bool error = false, keepData = false;
  if (!keepData) {
    data = 0;
  }
  
  // Read instruction from RPi when available
  if (PiComm.available()) {
    data = PiComm.read();
  }

  // RPi communication case (not testing)
  if (!ps5.isConnected()) { 
    // Only check what action to take if there was an instruction given
    if (data != 0) {
      // Determine what actions to take based on received instruction
      switch(data) {
        case 'A': { // Turn to a specific angle
          // Read degree of turn as int
          int degrees = PiComm.parseInt();

          // Send confirmation code to RPi (stalls litter detection)
          PiComm.write('7');

          // Print message to computer (troubleshooting)
          // Serial.print("Turning to degree: ");
          // Serial.println(degrees);

          turn(degrees);

          // Tell RPi when done turning (resume litter detection)
          PiComm.write('1');
          break;
        }

        case 'C': { // Object centered, continue forward
          // Initialize necessary local variables for case
          unsigned short rightCycle = rightMotor.getSpeed();
          unsigned short leftCycle = leftMotor.getSpeed();
          unsigned short dutyCycle = 200;
          unsigned long ping = 0;
          unsigned long distance = 0;
          boolean newInstruction = false;

          // Check if tracks are moving forward at same speed (shouldn't be)
          if (rightCycle != leftCycle) {
            // Set both motors to the lowest of the two's speeds
            if (rightCycle > leftCycle) {
              dutyCycle = leftCycle;
            }
            else {
              dutyCycle = rightCycle;
            }
          }

          // Move DC motors forward with increasing speed
          for (; dutyCycle < 245; dutyCycle = dutyCycle + 10) {
            rightMotor.setSpeed(dutyCycle);
            leftMotor.setSpeed(dutyCycle);
            rightMotor.forward();
            leftMotor.forward();
            // Print message to computer (troubleshooting)
            // Serial.print("Forward with duty cycle: ");
            // Serial.print(dutyCycle);
            // Serial.print(" with function reading of: ");
            // Serial.println(rightMotor.getSpeed());
            // Constantly check if another instruction is sent, if so break out of case
            if (PiComm.available()) {
              data = PiComm.read();
              if (data == 'C') {
                delay(1);
              }
              else {
                newInstruction = true;
                break;
              }
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(20);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, break out of case
            if (distance > 0 && distance < 50) {
              PiComm.write('7');
              break;
            }
            delay(50);
          }
            

          boolean out1 = true;
          while (!newInstruction && out1) {
            // Constantly check if another instruction is sent, if so break out of case
            if (PiComm.available()) {
              data = PiComm.read();
              if (data == 'C') {
                delay(1);
              }
              else {
                newInstruction = true;
                break;
              }
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(25);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, break out of case
            if (distance > 0 && distance < 50) {
              PiComm.write('7');
              out1 = false;
              break;
            }
          }

          if (!newInstruction) {
          // Move DC motors forward with decreasing speed
          for (; dutyCycle > 100; dutyCycle = dutyCycle - 10) {
            rightMotor.setSpeed(dutyCycle);
            leftMotor.setSpeed(dutyCycle);
            rightMotor.forward();
            leftMotor.forward();
            // Print message to computer (troubleshooting)
            // Serial.print("Forward with duty cycle: ");
            // Serial.print(dutyCycle);
            // Serial.print(" with function reading of: ");
            // Serial.println(rightMotor.getSpeed());
            // Constantly check if another instruction is sent, if so break out of case
            if (PiComm.available()) {
              data = PiComm.read();
              if (data == 'C') {
                delay(1);
              }
              else {
                newInstruction = true;
                break;
              }
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(20);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, stop and break out of case
            if (distance > 0 && distance < 10) {
              data = 'P';
              PiComm.write('7');
              rightMotor.stop();
              leftMotor.stop();
              lowerShaft();
              // Close hands under object and tilt it into the bin
               raiseShaft();
               PiComm.write('1');
               newInstruction = true;
              break;
            }
            delay(50);
          }
          }

          while (!newInstruction) {
            // Constantly check if another instruction is sent, if so break out of case
            if (PiComm.available()) {
              data = PiComm.read();
              if (data == 'C') {
                delay(1);
              }
              else {
                newInstruction = true;
                break;
              }
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(25);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, stop and break out of case
            if (distance > 0 && distance < 10) {
              data = 'P';
              PiComm.write('7');
              rightMotor.stop();
              leftMotor.stop();
              lowerShaft();
              // Close hands under object and tilt it into the bin
              raiseShaft();
              PiComm.write('1');
              newInstruction = true;
              break;
            }
          }

          break;
        }

        case 'D': { // Decelerate forward
          // Initialize necessary local variables for case
          unsigned short rightCycle = rightMotor.getSpeed();
          unsigned short leftCycle = leftMotor.getSpeed();
          unsigned short dutyCycle = 200;
          unsigned long ping = 0;
          unsigned long distance = 0;
          boolean newInstruction = false;

          // Check if tracks are moving forward at same speed
          if (rightCycle != leftCycle) {
            // Set both motors to the lowest of the two's speeds
            if (rightCycle > leftCycle) {
              dutyCycle = leftCycle;
            }
            else {
              dutyCycle = rightCycle;
            }
          }

          // Move DC motors forward with decreasing speed
          for (; dutyCycle > 90; dutyCycle = dutyCycle - 10) {
            rightMotor.setSpeed(dutyCycle);
            leftMotor.setSpeed(dutyCycle);
            rightMotor.forward();
            leftMotor.forward();
            // Print message to computer (troubleshooting)
            // Serial.print("Forward with duty cycle: ");
            // Serial.print(dutyCycle);
            // Serial.print(" with function reading of: ");
            // Serial.println(rightMotor.getSpeed());
            // Constantly check if another instruction is sent, if so break out of case
            if (PiComm.available()) {
              data = PiComm.read();
              newInstruction = true;
              break;
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(20);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, stop and break out of case
            if (distance > 0 && distance < 15) {
              PiComm.write('7');
              rightMotor.stop();
              leftMotor.stop();
              newInstruction = true;
              break;
            }
            delay(50);
          }

          while (!newInstruction) {
            // Constantly check if another instruction is sent, if so break out of case
            if (PiComm.available()) {
              data = PiComm.read();
              newInstruction = true;
              break;
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(25);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, stop and break out of case
            if (distance > 0 && distance < 15) {
              PiComm.write('7');
              rightMotor.stop();
              leftMotor.stop();
              newInstruction = true;
              break;
            }
          }

          break;
        }

        case 'E': { // Emergency stop
          rightMotor.stop();
          leftMotor.stop();
          error = true;
          break;
        }

        case 'F': { // Accelerate forward
          // Initialize necessary local variables for case
          unsigned short dutyCycle = 90;
          unsigned long ping = 0;
          unsigned long distance = 0;
          boolean newInstruction = false;

          // Move DC motors forward with increasing speed
          for (; dutyCycle < 250; dutyCycle = dutyCycle + 10) {
            rightMotor.setSpeed(dutyCycle);
            leftMotor.setSpeed(dutyCycle);
            rightMotor.forward();
            leftMotor.forward();
            // Print message to computer (troubleshooting)
            // Serial.print("Forward with duty cycle: ");
            // Serial.print(dutyCycle);
            // Serial.print(" with function reading of: ");
            // Serial.println(rightMotor.getSpeed());
            // Constantly check if another instruction is sent, if so break out of case
            if (PiComm.available()) {
              data = PiComm.read();
              if (data == 'C') {
                delay(1);
              }
              else {
                newInstruction = true;
                break;
              }
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(20);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, break out of case
            if (distance > 0 && distance < 50) {
              data = 'C';
              PiComm.write('7');
              newInstruction = true;
              break;
            }
            delay(50);
          }
            
          while (!newInstruction) {
            // Constantly check if another instruction is sent, if so break out of case
            if (PiComm.available()) {
              data = PiComm.read();
              if (data == 'C') {
                delay(1);
              }
              else {
                newInstruction = true;
                break;
              }
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(25);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, break out of case
            if (distance > 0 && distance < 50) {
              data = 'C';
              PiComm.write('7');
              newInstruction = true;
              break;
            }
          }

          break;
        }

        case 'L': { // Turn slightly left while moving forward
          // Initialize necessary local variables for case
          unsigned short dutyCycle = leftMotor.getSpeed();
          unsigned long ping = 0;
          unsigned long distance = 0;
          boolean newInstruction = false;

          if (!rightMotor.isMoving()) {
            // If robot is not moving at all, then only accelerate right track
            for (dutyCycle = 90; dutyCycle < 225; dutyCycle = dutyCycle + 5) {
              rightMotor.setSpeed(dutyCycle);
              rightMotor.forward();
              // Print message to computer (troubleshooting)
              // Serial.print("Forward with duty cycle: ");
              // Serial.print(dutyCycle);
              // Serial.print(" with function reading of: ");
              // Serial.println(rightMotor.getSpeed());
              // Constantly check if another instruction is sent, if so break out of case
              if (PiComm.available()) {
                data = PiComm.read();
                if (data == 'L') {
                  delay(1);
                }
                else {
                  newInstruction = true;
                  break;
                }
              }
              delay(50);
            }
          }

          if (!newInstruction) {
            // Slowly decelerate left motor to turn slightly left when already moving
            for (; dutyCycle > 90; dutyCycle = dutyCycle - 5) {
              leftMotor.setSpeed(dutyCycle);
              leftMotor.forward();
              // Print message to computer (troubleshooting)
              // Serial.print("Left with duty cycle: ");
              // Serial.print(dutyCycle);
              // Serial.print(" While right motor reading of: ");
              // Serial.println(rightMotor.getSpeed());
              // Constantly check if another instruction is sent, if so break out of case
              if (PiComm.available()) {
                data = PiComm.read();
                if (data == 'L') {
                  delay(1);
                }
                else {
                  newInstruction = true;
                  break;
                }
              }
              // Constanly check front ultrasonic sensor
              ping = sonar.ping_median(20);
              distance = sonar.convert_cm(ping);
              // Print message to computer (troubleshooting)
              // Serial.print("Sonar reading: ");
              // Serial.println(distance);
              // If object is close, stop moving forward and do static turn
              if (distance > 0 && distance < 50) {
                rightMotor.setSpeed(100);
                rightMotor.forward();
                leftMotor.stop();
                break;
              }
              delay(50);
            }
          }

          // Wait for RPi to send a different instruction
          while (!newInstruction) {
            if (PiComm.available()) {
                data = PiComm.read();
                if (data == 'L') {
                  delay(1);
                }
                else {
                  newInstruction = true;
                  break;
                }
            }
          }

          // If finish the loop and still not centered there is an error
          if (!newInstruction) {
            error = true;
          }
          break;
        }

        case 'O': {// Everything ok bro?
          
          break;
        }

        case 'P': { // Engage pickup function
          // Serial.println("Pickup function here");
          // Send confirmation code to RPi (stalls litter detection)
          PiComm.write('7');
          // Open hands while still above bin then lower them (around the object)
          //openHands();
          lowerShaft();
          // Close hands under object and tilt it into the bin
          closeHands();
          raiseShaft();
          // Tell RPi when finished picking up
          PiComm.write('1');
          break;
        }

        case 'R': { // Turn slightly right while moving forward
          // Initialize necessary local variables for case
          unsigned short dutyCycle = rightMotor.getSpeed();
          unsigned long ping = 0;
          unsigned long distance = 0;
          boolean newInstruction = false;

          if (!leftMotor.isMoving()) {
            // If robot is not moving at all, then only accelerate left track
            for (dutyCycle = 90; dutyCycle < 225; dutyCycle = dutyCycle + 5) {
              leftMotor.setSpeed(dutyCycle);
              leftMotor.forward();
              // Print message to computer (troubleshooting)
              // Serial.print("Forward with duty cycle: ");
              // Serial.print(dutyCycle);
              // Serial.print(" with function reading of: ");
              // Serial.println(rightMotor.getSpeed());
              // Constantly check if another instruction is sent, if so break out of case
              if (PiComm.available()) {
                data = PiComm.read();
                if (data == 'R') {
                  delay(1);
                }
                else {
                  newInstruction = true;
                  break;
                }
              }
              delay(50);
            }
          }

          if (!newInstruction) {
            // Slowly decelerate right motor to turn slightly right when already moving
            for (; dutyCycle > 90; dutyCycle = dutyCycle - 5) {
              rightMotor.setSpeed(dutyCycle);
              rightMotor.forward();
              // Print message to computer (troubleshooting)
              // Serial.print("Right with duty cycle: ");
              // Serial.print(dutyCycle);
              // Serial.print(" While left motor reading of: ");
              // Serial.println(leftMotor.getSpeed());
              // Constantly check if another instruction is sent, if so break out of case
              if (PiComm.available()) {
                data = PiComm.read();
                if (data == 'R') {
                  delay(1);
                }
                else {
                  newInstruction = true;
                  break;
                }
              }
              // Constanly check front ultrasonic sensor
              ping = sonar.ping_median(20);
              distance = sonar.convert_cm(ping);
              // Print message to computer (troubleshooting)
              // Serial.print("Sonar reading: ");
              // Serial.println(distance);
              // If object is close, stop moving forward and do static turn
              if (distance > 0 && distance < 50) {
                leftMotor.setSpeed(100);
                leftMotor.forward();
                rightMotor.stop();
                break;
              }
              delay(50);
            }
          }

          // Wait for RPi to send a different instruction
          while (!newInstruction) {
            if (PiComm.available()) {
                data = PiComm.read();
                if (data == 'L') {
                  delay(1);
                }
                else {
                  newInstruction = true;
                  break;
                }
            }
          }

          // If finish the loop and still not centered there is an error
          if (!newInstruction) {
            error = true;
          }
          break;
        }

        case 'T': { // Do a 180
          // Send confirmation code to RPi (stalls litter detection)
          PiComm.write('7');

          // Serial.println("Doing a full turn");
          turn(180);

          // Flush the serial buffer in case an instruction was accidentally sent
          while (PiComm.available()) {
            data = PiComm.read();
          }

          // Tell RPi when done turning (resume litter detection)
          PiComm.write('1');
          break;
        }

        default: {
          // PiComm.println("Serial communication error");
          // // Serial.println("Serial communication error");
          error = true;
          break;
        }
      }
    } 

    else  {
      // Serial.println("Waiting for RPI or PS5 comms");
      delay(500);
    }
  } 
  //End of RPI code line

  else if (ps5.isConnected()) { // Manual testing through PS5 controller
    // D-Pad Buttons:
    if (ps5.Right()) closeHands();
    if (ps5.Down()) lowerShaft();
    if (ps5.Up()) raiseShaft();
    if (ps5.Left()) openHands();

    // Shape Buttons:
    if (ps5.Square()) {
      turn(90);
    }
    if (ps5.Cross()) {
      turn(-45);
    }
    if (ps5.Circle()) {
      turn(-90);
    }
    if (ps5.Triangle()) {
      turn(45);
    }

    // Emergency Stop
    if (ps5.PSButton()) {
      rightMotor.stop();
      leftMotor.stop();
    }

    if (ps5.Options()) {
      sensor_t sensor;
      mag.getSensor(&sensor);
      // Serial.println("------------------------------------");
      // // Serial.print  ("Sensor:       "); Serial.println(sensor.name);
      // // Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
      // // Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
      // // // Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
      // // // Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
      // // // Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
      // Serial.println("------------------------------------");
      // Serial.println("");
      delay(500);

      while(!ps5.Share()) {
        // Get a new sensor event 
        sensors_event_t event; 
        mag.getEvent(&event);
      
        // Display the results (magnetic vector values are in micro-Tesla (uT)) 
        // // // Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
        // // // Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
        // // // // Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

        // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
        // Calculate heading when the magnetometer is level, then correct for signs of axis.
        float heading = atan2(event.magnetic.y, event.magnetic.x);
        
        // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
        // Find yours here: http://www.magnetic-declination.com/
        // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
        // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
        float declinationAngle = 0.0573;
        heading += declinationAngle;
        
        // Correct for when signs are reversed.
        if(heading < 0)
          heading += 2*PI;
        // Check for wrap due to addition of declination.
        if(heading > 2*PI)
          heading -= 2*PI;
        
        // Convert radians to degrees for readability.
        float headingDegrees = heading * 180/PI; 
        // // Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
        
        delay(500);
      }
    }

    if (ps5.Touchpad()) {
      Serial.println(sonar.ping_cm());
      Serial.println(sonar.convert_cm(sonar.ping_median(10)));
    }

    // L1 - Turn left
    if (ps5.L1()) {
      leftMotor.setSpeed(150);
      rightMotor.setSpeed(150);
      leftMotor.backward();
      rightMotor.forward();

      while(ps5.L1()) delay(1);

      leftMotor.stop();
      rightMotor.stop();
    }
    // R1 - Turn right
    if (ps5.R1()) {
      leftMotor.setSpeed(150);
      leftMotor.forward();
      rightMotor.setSpeed(150);
      rightMotor.backward();

      while(ps5.R1()){
        delay(1);
      }

      leftMotor.stop();
      rightMotor.stop();
    }
    
    // L2 - Variable Reverse
    if(ps5.L2()){
      // Only accept analog values higher thant 60 for PWMs
      if(ps5.L2Value() > 60){
        leftMotor.setSpeed(ps5.L2Value());
        rightMotor.setSpeed(ps5.L2Value());
        rightMotor.backward();
        leftMotor.backward();
        // Serial.println("Reverse");
        // // Serial.print("Right speed: ");  Serial.print(rightMotor.getSpeed());
        // // Serial.print("Left speed: ");  Serial.println(leftMotor.getSpeed());
      } 
      else {
        leftMotor.stop();
        rightMotor.stop();
      }
    } 
    // R2 - Variable Forward
    else if(ps5.R2()){
      // Only accept analog values higher thant 60 for PWMs
      if(ps5.R2Value() > 60){
        leftMotor.setSpeed(ps5.R2Value());
        rightMotor.setSpeed(ps5.R2Value());
        rightMotor.forward();
        leftMotor.forward();
        // Serial.println("Forward");
        // // Serial.print("Right speed: ");  Serial.print(rightMotor.getSpeed());
        // // Serial.print(" Left speed: ");  Serial.println(leftMotor.getSpeed());
      } 
      else {
        leftMotor.stop();
        rightMotor.stop();
      }
    } 
    // Stop motors when R2/L2 are released
    else {
      if (leftMotor.getSpeed() > 0 || rightMotor.getSpeed() > 0){
        // leftMotor.setSpeed(0);
        // rightMotor.setSpeed(0);
        leftMotor.stop();
        rightMotor.stop();
      }
    }
  }

  // If there was an error detected then print msg to computer
  if (error) {
    // Serial.println("Something went wrong!");
  }

  // Delay for visual troubleshooting
  //delay(1000);
}
