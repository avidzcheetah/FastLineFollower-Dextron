#define Lmotorf 6
#define Lmotorb 5
#define LmotorS 10

#define Rmotorf 4
#define Rmotorb 3
#define RmotorS 9

const int sensPins[5] = {A0,A1,A2,A3,A4};
int sensor[5];
float error = 0, P = 0, I = 0, D = 0, PID_value = 0, previous_error = 0;
int initspeed = 210;

float Kp = 10, Ki = 2, Kd = 0.8;

void setup() {
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
}

void loop() {
  Serial.print("Reading Sensors: ");
  for (int i = 0; i < 5; i++) {
    sensor[i] = analogRead(i);
    Serial.print(sensor[i]);
    Serial.print(" ");
    if (sensor[i] < 100) {
      sensor[i] = 1; // Close to the sensor (dark object)
    } else if (sensor[i] > 900) {
      sensor[i] = 0; // Far from the sensor (light object)
    }
    Serial.println();    
    Serial.print(sensor[i]);
    Serial.print(" ");

  }
  Serial.println(); // Print a new line after sensor values


   if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
      stop();  
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) {
      error = 5;  // tURN 90 DEGREES
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)) {
      error = 3;   // Far-right detection
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)) {
      error = 2;
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0)) {
      error = 1;
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)) {
      error = 1;
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) {
      error = 0;
    }else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) {
      error = -1;
    } else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
      error = -1;
    } else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
      error = -2;
    } else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
      error = -3;
    } else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) {
      error = -5;
     }else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) {
      stop();
    } else {
      stop();  //Don't know WTF is going on so stop
    }

  P = error;
  I += error;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_error = error;

  // Debugging output
  Serial.print("P: ");
  Serial.print(P);
  Serial.print(" I: ");
  Serial.print(I);
  Serial.print(" D: ");
  Serial.print(D);
  Serial.print(" PID_value: ");
  Serial.println(PID_value);

  int left_motor_speed = initspeed + PID_value;
  int right_motor_speed = initspeed - PID_value;

  if (left_motor_speed > 255){
    left_motor_speed = 255;
  } else if (left_motor_speed < -255){
    left_motor_speed = -255;
  }

  if (right_motor_speed > 255){
    right_motor_speed = 255;
  } else if (right_motor_speed < -255){
    right_motor_speed = -255;
  }  

    // Control LEFT motor
  if (left_motor_speed >= 0) {
    analogWrite(LmotorS, left_motor_speed);
    digitalWrite(Lmotorb, LOW);  // Forward
    digitalWrite(Lmotorf, HIGH);
  } else {
    analogWrite(LmotorS, abs(left_motor_speed));
    digitalWrite(Lmotorb, HIGH); // Reverse
    digitalWrite(Lmotorf, LOW);
  }

  // Control RIGHT motor
  if (right_motor_speed >= 0) {
    analogWrite(RmotorS, right_motor_speed);
    digitalWrite(Rmotorb, LOW);  // Forward
    digitalWrite(Rmotorf, HIGH);
  } else {
    analogWrite(RmotorS, abs(right_motor_speed));
    digitalWrite(Rmotorb, HIGH); // Reverse
    digitalWrite(Rmotorf, LOW);
  }

  // Debugging outputs for motor speeds
  Serial.print(" | Left Motor Speed: ");
  Serial.print(left_motor_speed);
  Serial.print(" | Right Motor Speed: ");
  Serial.println(right_motor_speed);
}

void stop(){
  analogWrite(Lmotorf, 0);
  analogWrite(Rmotorf, 0);
  analogWrite(Rmotorb, 0);
  analogWrite(Lmotorb, 0);
}