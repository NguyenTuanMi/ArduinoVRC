#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <PS2X_lib.h>

// Motor pins
#define servo_intake_1 7 // servo pins for intake 7, 6
#define servo_intake_2 6
#define servo_spinner 5 // servo pin for spinner 5

#define dc_high_1 10 //pins for drivebase motors 10, 11, 12, 13
#define dc_low_1 11
#define dc_high_2 12
#define dc_low_2 13
#define dc_elevator_high 8 // pins for elevator 8, 9 
#define dc_elevator_low 9
#define dc_loader_high 14 // pins for loader motors 14, 15
#define dc_loader_low 15

#define PS2_DAT 12 // MISO
#define PS2_CMD 13 // MOSI
#define PS2_SEL 15 // SS
#define PS2_CLK 14 // SLK

// Max speed for each components
#define MAX_SPEED 1500 
#define MAX_ELEVATOR 4000 
#define MAX_INTAKE 1000
#define MAX_LOADER 4000
#define MAX_SPINNER 4000
#define AUTO_SPEED 2000

// Time for auto
#define AUTO_FORWARD_TIME 50000 // in milliseconds
#define AUTO_REVERSE_TIME 50000
#define AUTO_LOADING_TIME 4000

bool driving_mode = true;

#define pressures false
#define rumble false

PS2X ps2x; // generate class PS2X
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // The controller for motors

void setupPS2X() {
  int error = -1;
  for (int i = 0; i < 10; i++) // try to connect to motors for 10 times
  {
    delay(1000); // wait 1 seconds
    // assign the pin and mode: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print(".");
  }

  switch (error) // check for error
  {
  case 0:
    Serial.println(" Connect to PS2X successfully");
    break;
  case 1:
    Serial.println(" ERROR: Can not find the ps2x, check the wiring ");
    break;
  case 2:
    Serial.println(" ERROR: Can not load the command");
    break;
  case 3:
    Serial.println(" ERROR: Cannnot enter the pressure mode ");
    break;
  }
}

// Function to control the dc motors
void setDCMotors(int c3, int c4, int c5, int c6, int c7, int c8, int c9, int c10) { //driver high 1, driver low 1, driver high 2, driver low 2, elevator high, elevator low
  pwm.setPWM(dc_high_1, 0, c3);
  pwm.setPWM(dc_low_1, 0, c4);
  pwm.setPWM(dc_high_2, 0, c5);
  pwm.setPWM(dc_low_2, 0, c6);
  pwm.setPWM(dc_elevator_high, 0, c7);
  pwm.setPWM(dc_elevator_low, 0, c8);
  pwm.setPWM(dc_loader_high, 0, c9);
  pwm.setPWM(dc_loader_low, 0, c10);
}

// Function to control the servo motors
void setServoMotors(int c0, int c1, int c2) {
  pwm.setPWM(servo_intake_1, 0, c1);
  pwm.setPWM(servo_intake_2, 0, c2);
  pwm.setPWM(servo_spinner, 0, c0);
}

// Initiallize the motors
void initMotors() {
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(1600);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("Connect to PS2X: ");
  initMotors();
  Serial.println("Initialize motors done");
  setupPS2X();
  Serial.println("Set up PS2X done");
}

void loop() {
  ps2x.read_gamepad(false, false); // gọi hàm để đọc tay điều khiển
  if (ps2x.ButtonPressed(PSB_PAD_UP)) {
    driving_mode =! driving_mode;
  }
  int c0= 0, c1= 0, c2= 0, c3= 0, c4= 0, c5= 0, c6= 0, c7= 0, c8= 0, c9= 0, c10= 0;
  if (driving_mode) {
    int value_left = 128 - ps2x.Analog(PSS_LY); // 
    int value_right = 128 - ps2x.Analog(PSS_RY);
  
    if (ps2x.Button(PSB_R3)) { // for the spinner
      c0 = MAX_SPINNER;
    }
  
    if (ps2x.Button(PSB_CROSS)) { // for the intake
      c1 = MAX_INTAKE;
      c2 = MAX_INTAKE;
    }
  
    if (ps2x.Button(PSB_TRIANGLE)) { // for the elevator 
      c7 = MAX_ELEVATOR;
    }

    if (ps2x.Button(PSB_SQUARE)) { // for the loader 
      c9 = MAX_LOADER;
    }

    if (ps2x.Button(PSB_L2)) { // reverse button
      if (ps2x.Button(PSB_TRIANGLE)) {
        c7 = 0;
        c8 = MAX_ELEVATOR;
      }
      if (ps2x.Button(PSB_SQUARE)) {
        c9 = 0;
        c10 = MAX_LOADER;
      }
    }
  
    // mannual control
    if (value_right < 0)
    {
      c5 = abs(value_right);
      c5 = map(c5, 0, 128, 0, MAX_SPEED);
    }

    else if (value_right > 0)
    {
      c6 = map(value_right, 0, 128, 0, MAX_SPEED) + 1;
    }

    if (value_left < 0)
    {
      c3 = abs(value_left);
      c3 = map(c3, 0, 128, 0, MAX_SPEED);
    }
    else if (value_left >  0)
    {
      c4 = map(value_left, 0, 128, 0, MAX_SPEED);
    }
    //Serial.println(c3);

    if (ps2x.Button(PSB_L1)) {
      c3 = 0;
      c4 = 0;
    }

    if (ps2x.Button(PSB_R1)) {
      c5 = 0;
      c6 = 0;
    }
  
    setDCMotors(c3, c4, c5, c6, c7, c8, c9, c10);
    setServoMotors(c0, c1, c2);
    delay(50);
  }
  
  else {
    int TIME = millis();
    
    while(millis() - TIME < AUTO_FORWARD_TIME) {
      setDCMotors(AUTO_SPEED, 0, AUTO_SPEED, 0, 0, 0, 0, 0);
    }
    delay(1000);
    setDCMotors(0, 0, 0, 0, 0, 0, 0, 0);
    
    while(millis() - TIME < AUTO_FORWARD_TIME + AUTO_LOADING_TIME) {
      setDCMotors(0, 0, 0, 0, 0, 0, 0, MAX_LOADER); 
    }
    delay(1000);
    setDCMotors(0, 0, 0, 0, 0, 0, 0, 0);
    
    while(millis() - TIME < AUTO_FORWARD_TIME + AUTO_LOADING_TIME + AUTO_REVERSE_TIME) {
      setDCMotors(0, AUTO_SPEED, 0, AUTO_SPEED, 0, 0, 0, 0);
    }
    delay(2000);
    setDCMotors(0, 0, 0, 0, 0, 0, 0, 0);
    driving_mode = true;
  }
  // put your main code here, to run repeatedly:
}