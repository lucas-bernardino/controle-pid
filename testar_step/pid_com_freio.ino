#include <AFMotor.h>
#include <arduino-timer.h>

#define HALL_PIN 13
#define RELAY_PIN 2
#define R 0.3429
#define PERIMETER 2 * PI * R

signed long T1 = 0;
signed long T2 = 0;
signed long time_seconds = 0;
AF_Stepper motor(200, 1);

float speed = 0;
int cycles = 0;

double integral_term = 0;
double error_prev = 0;

float setpoint = 0.0;
float kp = 0.5;
float ki = 0.00;
float kd = 0.00;

String setp = "";
String k_p = "";
String k_i = "";
String k_d = "";

bool is_setup_completed = false; 
bool is_breaking = false;

bool flag_break = false;
bool speed_control_state = true;

bool is_at_setpoint = true;

auto timer = timer_create_default();

double pid_controller(double avg, int dt){
  double error = setpoint - avg;
  integral_term = integral_term + (error * dt);
  float derivative_term = (error - error_prev) / dt;
  error_prev = error;

  return (kp * error) + (ki * integral_term) + (kd * derivative_term);
}

float get_speed(long* t_delta) {
    T2 = millis();
    time_seconds = (T2 - T1);
    *t_delta = time_seconds;
    float s = (PERIMETER / time_seconds) * 1000 * 3.6;
    T1 = T2;

    return s;
}

float print_on_serial(int t, float sp, float st) {
    Serial.print("tempo:");
    Serial.println(t);
    Serial.print("velocidade_atual:");
    Serial.println(sp);
    Serial.print("pid_output:");
    Serial.println(st);
}

void handle_step(float vel, float st) {
    if (vel > setpoint) {
        motor.step(abs(st), FORWARD, MICROSTEP);
    }
    if (vel < setpoint) {
        motor.step(abs(st), BACKWARD, MICROSTEP);
    }
}

void valve_stop() {
  Serial.println("Handler is off");
  if (!is_at_setpoint) {
    Serial.println("Acionei LOW");
    digitalWrite(RELAY_PIN, LOW);
    cycles++;
    speed_control_state = false;
  }
}

void valve_handler() {
  Serial.println("Handler is on");
  if (!is_at_setpoint) {
    Serial.println("Acionei HIGH");
    digitalWrite(RELAY_PIN, HIGH);
    speed_control_state = true;
    timer.in(5000, valve_stop);
  }
}

void read_from_python() {
  Serial.println("Ready");
  delay(50);
  setp = Serial.readStringUntil('S');
  delay(50);
  k_p = Serial.readStringUntil('P');
  delay(50);
  k_i = Serial.readStringUntil('I');
  delay(50);
  k_d = Serial.readStringUntil('D');
  if (setp != "" && k_p != "" && k_i != "" && k_d != "") {
    setp.trim();
    k_p.trim();
    k_i.trim();
    k_d.trim();
    setpoint = setp.toFloat();
    kp = k_p.toFloat();
    ki = k_i.toFloat();
    kd = k_d.toFloat();
    Serial.println("setup_completed");
    is_setup_completed = true;
  }
}

void setup () {
  pinMode(HALL_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  Serial.begin(9600);
  motor.setSpeed(90); 
  timer.every(15000, valve_handler);
  T1 = millis();
}

void loop () {
  if (!is_setup_completed) {
    delay(1000); // Tempo para iniciar o script no python
    read_from_python();
  }
  if (digitalRead(HALL_PIN) == LOW && cycles < 10) {
    long time_delta = 0;
    speed = get_speed(&time_delta);
    if (speed < 50) {
        double step = pid_controller(speed, time_delta);
        print_on_serial(time_delta, speed, step);
        handle_step(speed, step);
        if (abs(speed - setpoint) < 1) {
          Serial.println("Mudei o is_at_setponit");
          is_at_setpoint = false;
          Serial.println("Is at setpoint is true");
        }
          
    }
  }
  timer.tick();
}
