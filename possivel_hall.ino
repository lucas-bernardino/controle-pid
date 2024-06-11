#include <AFMotor.h>

#define HALL_PIN 13
#define R 0.3429
#define PERIMETER 2 * PI * R

#define DESIRABLE_SPEED 25

signed long T1 = 0;
signed long T2 = 0;
signed long time_seconds = 0;
AF_Stepper motor(200, 1);

float speed_sum = 0;
int counter = 0;

double integral_term = 0;
double error_prev = 0;

float kp = 0.5;
float ki = 0.00;
float kd = 0.00;

String setp = "";
String k_p = "";
String k_i = "";
String k_d = "";
bool is_setup_completed = false; 

double pid_controller(double avg, int dt){
  double error = DESIRABLE_SPEED - avg;
  integral_term = integral_term + (error * dt);
  float derivative_term = (error - error_prev) / dt;
  error_prev = error;

  return (kp * error) + (ki * integral_term) + (kd * derivative_term);
}

void forwardstep() {  
  motor.onestep(FORWARD, MICROSTEP);
}
void backwardstep() {  
  motor.onestep(BACKWARD, MICROSTEP);
}

void setup () {
  pinMode(HALL_PIN, INPUT);
  Serial.begin(9600);
  T1 = millis();
  motor.setSpeed(90); 
  Serial.print("setpont:");
  Serial.println(DESIRABLE_SPEED);
}

void read_from_python() {
  Serial.println("Entrei na funcao");
  Serial.println("Ready");
  delay(50);
  Serial.println("Depois dos segundos");
  setp = Serial.readStringUntil('S');
  delay(50);
  k_p = Serial.readStringUntil('P');
  delay(50);
  k_i = Serial.readStringUntil('I');
  delay(50);
  k_d = Serial.readStringUntil('D');
  if (setp != "" && k_p != "" && k_i != "" && k_d != "") {
    Serial.println("setup_completed");
    is_setup_completed = true;
  }
}

void loop () {
  if (!is_setup_completed) {
    read_from_python();
  }
  else {
    Serial.println("Valores: ");
    Serial.println(setp);
    Serial.println(k_p);
    Serial.println(k_i);
    Serial.println(k_d);
    delay(3000);
  }
  if (digitalRead(HALL_PIN) == LOW) {
    T2 = millis();
    time_seconds = (T2 - T1);
    float speed = (PERIMETER / time_seconds) * 1000 * 3.6;
    counter+=1;
    if (speed < 50) {
      Serial.print("tempo:");
      Serial.println(time_seconds);
      Serial.print("velocidade_atual:");
      Serial.println(speed);
      double step = pid_controller(speed, time_seconds);
      Serial.print("pid_output:");
      Serial.println(step);
      if (speed_sum > DESIRABLE_SPEED) {
        motor.step(abs(step), FORWARD, MICROSTEP);
      }
      if (speed_sum < DESIRABLE_SPEED) {
        motor.step(abs(step), BACKWARD, MICROSTEP);
      }
    }
    T1 = T2;
  }
}

