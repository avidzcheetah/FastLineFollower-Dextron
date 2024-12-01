#include <Arduino.h>

const int clrButtonPin = A5; // CLR button connected to pin A5

float Kp = 15, Ki = 0.2, Kd = 3; // Adjusted for increased sensitivity
int IR_weights[5] = {-4, -2, 0, 2, 4};
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0;
int sensor[5] = {0, 0, 0, 0, 0};
int initial_motor_speed = 190; // Moderate forward speed
int min_motor_speed = 130;    // Minimum effective motor speed
int max_motor_speed = 255;
int surface_color = -1; // -1 = undetermined, 0 = black surface, 1 = white surface.
unsigned long previous_millis = 0;
unsigned long forward_interval = 100; // Time between each small forward movement in ms

void read_sensor_values(void);
void determine_surface_color(void);
void calculate_pid(void);
void motor_control(void);
void handle_turns_and_junctions(void);
void initalize_pins(void);
void stop_robot(void);
void move_forward_bit(void);
void turn_left(void);
void turn_right(void);

void setup() {
  Serial.begin(9600);
  Serial.println("Setup Complete.");
  initalize_pins();
  pinMode(clrButtonPin, INPUT_PULLUP); // Set CLR button pin as input with pull-up resistor
}

void loop() {
  unsigned long current_millis = millis();

  // Check CLR button state
  int clrButtonState = digitalRead(clrButtonPin);
  Serial.print("CLR Button State: ");
  Serial.println(clrButtonState == LOW ? "Pressed (LOW)" : "Released (HIGH)");

  // Override surface color if CLR button is pressed
  if (clrButtonState == LOW) {
    surface_color = 0; // Override to white surface
    Serial.println("Surface Color Overridden to BLACK due to CLR Button.");
  } else {
    determine_surface_color(); // Normal surface color determination
  }

  // Simulate small forward movements in steps
  if (current_millis - previous_millis >= forward_interval) {
    previous_millis = current_millis;

    read_sensor_values();
    calculate_pid();
    handle_turns_and_junctions(); // Handle T and Cross junctions
    motor_control();              // Adjust motor speeds incrementally based on PID
  }
}

void read_sensor_values() {
  Serial.print("Reading Sensors: ");
  for (int i = 0; i < 5; i++) {
    sensor[i] = analogRead(i);
    Serial.print(sensor[i]);
    Serial.print(" ");
    if (sensor[i] < 100) {
      sensor[i] = 1; // Close to the sensor (dark object)
    } else if (sensor[i] > 900) {
      sensor[i] = 0; // Far (light object)
    } else {
      sensor[i] = 0; // Treat intermediate values as light object
    }
  }
  Serial.println();
}

void determine_surface_color() {
  int black_count = 0; // Count how many sensors detect the line color (black)
  int white_count = 0; // Count how many sensors detect the surface color (white)

  // Count the number of sensors detecting black or white
  for (int i = 0; i < 5; i++) {
    if (sensor[i] == 1) {
      black_count++;
    } else {
      white_count++;
    }
  }

  // Determine surface color based on majority sensor readings
  if (black_count > white_count) {
    surface_color = 0; // Black surface detected
  } else {
    surface_color = 1; // White surface detected
  }

  // Output surface color for debugging
  Serial.print("Surface Color: ");
  if (surface_color == 0) {
    Serial.println("Black");
  } else {
    Serial.println("White");
  }
}

void calculate_pid() {
  error = 0;

  // Adjust sensor values based on surface color
  if (surface_color == 0) {
    for (int i = 0; i < 5; i++) {
      sensor[i] = 1 - sensor[i];
    }
  }

  // Calculate weighted error for PID
  for (int i = 0; i < 5; i++) {
    error += sensor[i] * IR_weights[i];
  }

  P = error;
  I += error;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_error = error;

  // Output PID components for debugging
  Serial.print("P: ");
  Serial.print(P);
  Serial.print(", I: ");
  Serial.print(I);
  Serial.print(", D: ");
  Serial.print(D);
  Serial.print(", PID: ");
  Serial.println(PID_value);
}

void handle_turns_and_junctions() {
  // Detect T-junction or Cross-junction
  if (sensor[0] == 1 && sensor[4] == 1 && sensor[2] == 1) {
    Serial.println("Potential T or Cross junction detected. Moving forward to confirm.");
    move_forward_bit(); // Move forward slightly to confirm junction type

    read_sensor_values();
    if (sensor[2] == 1) {
      Serial.println("Confirmed Cross-junction. Moving forward.");
      return; // Continue forward movement for a cross-junction
    } else {
      Serial.println("Confirmed T-junction. Choosing turn direction.");

      // Decide turn direction at T-junction
      if (sensor[0] == 1) {
        Serial.println("Turning LEFT at T-junction.");
        turn_left();
      } else if (sensor[4] == 1) {
        Serial.println("Turning RIGHT at T-junction.");
        turn_right();
      } else {
        Serial.println("Stopping at T-junction.");
        stop_robot();
      }
    }
  }

  // Handle sharp turns if PID is large
  if (abs(PID_value) > 50) {
    if (PID_value > 0) {
      Serial.println("Sharp RIGHT turn initiated.");
      turn_right();
    } else {
      Serial.println("Sharp LEFT turn initiated.");
      turn_left();
    }
  }
}

void move_forward_bit() {
  int forward_speed = constrain(initial_motor_speed, min_motor_speed, max_motor_speed);

  analogWrite(10, forward_speed);
  digitalWrite(5, LOW);  // LEFT motor forward
  digitalWrite(6, HIGH);

  analogWrite(9, forward_speed);
  digitalWrite(3, LOW);  // RIGHT motor forward
  digitalWrite(4, HIGH);

  delay(200); // Move forward slightly
  stop_robot();
}

void turn_left() {
  while (true) {
    int turn_speed = constrain(abs(PID_value), min_motor_speed, max_motor_speed);

    analogWrite(10, turn_speed);
    digitalWrite(5, HIGH);  // LEFT motor backward
    digitalWrite(6, LOW);

    analogWrite(9, turn_speed);
    digitalWrite(3, LOW);  // RIGHT motor forward
    digitalWrite(4, HIGH);

    read_sensor_values();
    if (sensor[2] == 1) { // Stop turning when the center sensor aligns with the line
      stop_robot();
      break;
    }
  }
}

void turn_right() {
  while (true) {
    int turn_speed = constrain(abs(PID_value), min_motor_speed, max_motor_speed);

    analogWrite(10, turn_speed);
    digitalWrite(5, LOW);  // LEFT motor forward
    digitalWrite(6, HIGH);

    analogWrite(9, turn_speed);
    digitalWrite(3, HIGH);  // RIGHT motor backward
    digitalWrite(4, LOW);

    read_sensor_values();
    if (sensor[2] == 1) { // Stop turning when the center sensor aligns with the line
      stop_robot();
      break;
    }
  }
}

void motor_control() {
  int left_motor_speed = initial_motor_speed + PID_value;
  int right_motor_speed = initial_motor_speed - PID_value;

  // Ensure motor speeds are within operational range
  left_motor_speed = constrain(left_motor_speed, min_motor_speed, max_motor_speed);
  right_motor_speed = constrain(right_motor_speed, min_motor_speed, max_motor_speed);

  // Control LEFT motor
  analogWrite(10, left_motor_speed);
  digitalWrite(5, LOW);  // Forward
  digitalWrite(6, HIGH);

  // Control RIGHT motor
  analogWrite(9, right_motor_speed);
  digitalWrite(3, LOW);  // Forward
  digitalWrite(4, HIGH);

  // Output motor speeds for debugging
  Serial.print("Motor Speeds - Left: ");
  Serial.print(left_motor_speed);
  Serial.print(", Right: ");
  Serial.println(right_motor_speed);
}

void stop_robot() {
  analogWrite(10, 0);
  analogWrite(9, 0);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
}

void initalize_pins() {
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  // Initialize sensors if needed
}