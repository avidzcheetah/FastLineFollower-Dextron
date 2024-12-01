//define the motor pin
#define Rightmotorf 9 // Rightmotorforward
#define Rightmotorb 6 // Rightmotorbackward 
#define Leftmotorf 5  // Leftmotorforward
#define Leftmotorb 3 // Leftmotorbackward
// Define the maximum speed of motor
#define Maxium_speed 250
int Motorbasespeed = 150;
// Declaration of necessary variables for line following
int IR_value[5] = {0, 0, 0, 0, 0};
int IR_weights[5] = {-10, -5, 0, 5, 10}; // Assign values to each sensor
int LMotorspeed = 0;
int RMotorspeed = 0;
int speedAdjust = 0;
float P, I, D;
float error = 0;
float PreviousError = 0;
float Kp = 10; // Propartional gain
float kd =5.7; // Derivative gain
float ki = 0;  // Integral gain
// Push button and LED
int button1 = 4;
int button2 = 7;
int button3 = 10;
int button4 = 11;
int led12 = 2;
int led13 = 12;
bool button1_state, button2_state, button3_state, button4_state;
// Caliberations
int sensorValues[5];
int minValues[5];
int maxValues[5];
float trus[5];
// Define the function
void PID_control();
void reading();
void set_forward();
void stop();
void turn();
void set_speed();
void button_status();
void caliberate();

void setup() {
  // Setup the pins
  pinMode(Rightmotorf, OUTPUT);
  pinMode(Rightmotorb, OUTPUT);
  pinMode(Leftmotorf, OUTPUT);
  pinMode(Leftmotorb, OUTPUT);
  // Button pins as input
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
  // LED pins as output
  pinMode(led12, OUTPUT);
  pinMode(led13, OUTPUT);
  Serial.begin(9600);
  //Caliberations
  for (int i = 0; i < 5; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
  }
}

void loop() {
  digitalWrite(led13, HIGH);
  button_status(); // Reading the status of push buttons

  // When button 1 is pressed, the robot will start to follow lines.
  while (button1_state != 1) {
    digitalWrite(led13, LOW);
    // Blink led12
    for (int i = 0; i < 5; ++i) {
      digitalWrite(led12, HIGH);
      delay(100);
      digitalWrite(led12, LOW);
      delay(100);
    }
    digitalWrite(led13, HIGH);
    digitalWrite(led12, HIGH);
    PID_control();
  }

  // When button 2 is pressed, check if both motors are running at the same speed.
  while (button2_state != 1) {
    digitalWrite(led13, LOW);
    for (int i = 0; i < 3; ++i) {
      digitalWrite(led12, HIGH);
      delay(500);
      digitalWrite(led12, LOW);
      delay(500);
    }
    set_forward();
    delay(3000);
    stop();
    button_status();
  }

  // Reading IR values and caliberation process
  while (button4_state != 1) {
    digitalWrite(led13, LOW);
    // Calibrations
    caliberate();
    // Blink the LED
    digitalWrite(led12, HIGH);
    delay(50);
    digitalWrite(led12, LOW);
    delay(50);
    button_status(); 
  }

  // Stop mode and print the maximum and minimum values if button 3 is pressed
  while (button3_state != 1) {
    stop();
    digitalWrite(led13, HIGH);
    button_status();
  }
}  

void reading() {
  error = 0;
  for (byte i = 0; i < 5; i++) {
    IR_value[i] = analogRead(i);
    // Convert analog value to digital value
    if (IR_value[i] > trus[i]) {
      IR_value[i] = 1;
    } else { 
      IR_value[i] = 0;
    }
    error += IR_value[i] * IR_weights[i];
  }
}

void PID_control() {
  while (1) {
    reading();
    // Calculations for PID control
    P = error;
    I = I + error;
    D = error - PreviousError;
    PreviousError = error;
    // Main equations
    speedAdjust = (Kp * P + ki * I + kd * D);
    // Set the speed
    LMotorspeed = Motorbasespeed + speedAdjust;
    RMotorspeed = Motorbasespeed - speedAdjust;

    if (LMotorspeed < 0) {
      LMotorspeed = 0;
    }
    if (LMotorspeed > Maxium_speed) {
      LMotorspeed = Maxium_speed;
    }
    if (RMotorspeed < 0) {
      RMotorspeed = 0;

    }
    if (RMotorspeed > Maxium_speed) {
      RMotorspeed = Maxium_speed;
    }
    if (RMotorspeed==Motorbasespeed) {
      RMotorspeed = Maxium_speed;
    }
    if (LMotorspeed==Motorbasespeed) {
      LMotorspeed = Maxium_speed;
    }
    set_speed();
  }
}

void set_speed() {
  analogWrite(Leftmotorf, LMotorspeed);
  analogWrite(Rightmotorf, RMotorspeed);
}

void set_forward() {
  analogWrite(Leftmotorf, Maxium_speed);
  analogWrite(Rightmotorf, Maxium_speed);
}

void stop() {
  analogWrite(Leftmotorf, 0);
  analogWrite(Rightmotorf, 0);
  analogWrite(Rightmotorb, 0);
  analogWrite(Leftmotorb, 0);
}

void turn() {
  analogWrite(Leftmotorf, 150);
  analogWrite(Rightmotorb, 150);
}

// Reading the status of push buttons and saving the state.
void button_status() {
  button1_state = digitalRead(button1);
  button2_state = digitalRead(button2);
  button3_state = digitalRead(button3);
  button4_state = digitalRead(button4);
}

void caliberate() {
  for (int i = 0; i < 10000; i++) {
    turn();
    for (int i = 0; i < 5; i++) {
      int value = analogRead(i);
      if (value < minValues[i]) {
        minValues[i] = value;
      }
      if (value > maxValues[i]) {
        maxValues[i] = value;
      }
    }
  }
  for (int i = 0; i < 5; i++) {
    trus[i] = (minValues[i] + maxValues[i]) / 2.0;
    Serial.print(trus[i]);
    Serial.print(" ");
  }
  Serial.println();
  stop();
}
