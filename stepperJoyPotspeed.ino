#define step_pin 9 // Pin 9 connected to Steps pin on EasyDriver
#define dir_pin 8 // Pin 8 connected to Direction pin
#define step_pinB 11 // Pin 9 connected to Steps pin on EasyDriver
#define dir_pinB 10 // Pin 8 connected to Direction pin
#define X_pin A0 // Pin A0 connected to joystick x axis
#define pot_pin A1 // Pin A1 connected to the potentiometer
#define Limit01 2 // Pin 2 connected to Limit switch out
#define Limit02 3 // Pin 3 connected to Limit switch out

const int minSpeed = 300; // Minimum speed (fast)
const int maxSpeed = 2000; // Maximum speed (slow)

int step_speed; // Speed of Stepper motor (higher = slower)

void setup() {
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pinB, OUTPUT);
  pinMode(step_pinB, OUTPUT);
  pinMode(Limit01, INPUT_PULLUP); // Enable internal pull-up resistors for limit switches
  pinMode(Limit02, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  // Read the potentiometer value
  int potValue = analogRead(pot_pin);

  // Map the potentiometer value to the desired speed range
  step_speed = map(potValue, 0, 1023, minSpeed, maxSpeed);

  if (analogRead(X_pin) > 712) {
    // Move stepper motor clockwise
    if (digitalRead(Limit01) == LOW) {
      // Limit switch 01 is activated, stop the motor
      digitalWrite(dir_pin, LOW);
      digitalWrite(step_pin, LOW);
    } else {
      // Limit switch 01 is not activated, move motor clockwise
      digitalWrite(dir_pin, LOW);
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(step_speed);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(step_speed);

      digitalWrite(dir_pinB, LOW);
      digitalWrite(step_pinB, HIGH);
      delayMicroseconds(step_speed);
      digitalWrite(step_pinB, LOW);
      delayMicroseconds(step_speed);
    }
  }

  if (analogRead(X_pin) < 312) {
    // Move stepper motor counter-clockwise
    if (digitalRead(Limit02) == LOW) {
      // Limit switch 02 is activated, stop the motor
      digitalWrite(dir_pin, HIGH);
      digitalWrite(step_pin, LOW);
    } else {
      // Limit switch 02 is not activated, move motor counter-clockwise
      digitalWrite(dir_pin, HIGH);
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(step_speed);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(step_speed);

      digitalWrite(dir_pinB, HIGH);
      digitalWrite(step_pinB, HIGH);
      delayMicroseconds(step_speed);
      digitalWrite(step_pinB, LOW);
      delayMicroseconds(step_speed);
    }
  }
}