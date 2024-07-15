#include <AccelStepper.h>
#include <arduino-timer.h>
#include <Adafruit_ADS1X15.h>

#define HALL_PIN 13
#define RELAY_PIN 11
#define R 0.3429
#define PERIMETER 2 * PI * R

#define motorInterfaceType 1
#define dirPin 8
#define stepPin 9
#define enablePin 4

#define NUM_OF_CYCLES 2
#define MAX_TEMP_SENSOR 35

signed long T1 = 0;
signed long T2 = 0;
signed long time_seconds = 0;

AccelStepper motor(motorInterfaceType, stepPin, dirPin);
Adafruit_ADS1115 ads;

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

float print_on_serial(int t, float sp, float st, float temp) {
    Serial.print("tempo:");
    Serial.println(t);
    Serial.print("velocidade_atual:");
    Serial.println(sp);
    Serial.print("pid_output:");
    Serial.println(st);
    Serial.print("Temperatura:");
    Serial.println(temp);
}

void handle_step(float st) {
    motor.move(st);
    motor.runToPosition();
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

float getTemperature() {
  double adc0 = ads.readADC_SingleEnded(0);
  double DCV = (adc0 * 0.1875)/1000.0;
  return 252.01*DCV - 337.8;
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
  ads.begin();
  motor.setMaxSpeed(1000);
  motor.setAcceleration(1000);
  timer.every(15000, valve_handler);
  T1 = millis();
}


void loop () {  
  if (!is_setup_completed) {
    delay(1000); // Tempo para iniciar o script no python
    read_from_python();
  }
  if (digitalRead(HALL_PIN) == LOW) {
    long time_delta = 0;
    speed = get_speed(&time_delta);
    if (speed < 55) {
        float tempIR = getTemperature();
        if (cycles == NUM_OF_CYCLES) {
          is_at_setpoint = true;
          if (tempIR > MAX_TEMP_SENSOR) {
            print_on_serial(time_delta, speed, 0.00, tempIR);
            motor.moveTo(0);
            motor.runToPosition();
            /*
            Se tiver completado um ciclo e a temperatura for maior que 100,
            ele espera e nao faz nada ate a temperatura baixar. Caso seja menor que 100,
            ele zera o numero de ciclos e comeca tudo de novo.
            */
            return;
          } 
          cycles = 0;
        }

        double step = pid_controller(speed, time_delta);
        
        print_on_serial(time_delta, speed, step, tempIR);
        handle_step(step);
        
        if (abs(speed - setpoint) < 1) {
          is_at_setpoint = false;
        }
    }
  }
  timer.tick();
}
