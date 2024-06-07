#include <AFMotor.h>

#define HALL_PIN 13
#define R 0.3
#define PERIMETER 2 * PI * R

#define DESIRABLE_SPEED 17

/*
I put 500 on this, but I still have no idea what's the exactly number
I created the counter variable because I think we need to have a sort of set of data before
trying to move the motor. If we change the motor every time we get a reading, it'll probably get lost.
*/
#define NUMBER_OF_RECORDINGS 500 

signed long T1 = 0;
signed long T2 = 0;
signed long time_seconds = 0;
AF_Stepper motor(200, 1);

float speed_sum = 0;
int counter = 0;

double integral_term = 0;
double error_prev = 0;

float kp = 1;
float ki = 0.00;
float kd = 0.00;

double pid_controller(double avg, int dt){
  Serial.print("Avg value: ");
  Serial.println(avg);
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
  Serial.print("Iniciando com setpoint de ");
  Serial.println(DESIRABLE_SPEED);
}

void loop () {
  if (digitalRead(HALL_PIN) == LOW) {
    T2 = millis();
    time_seconds = (T2 - T1);
    float speed = (PERIMETER / time_seconds) * 1000 * 3.6;
    counter+=1;
    if (speed < 50) {
      Serial.println(speed);
    }
    double step = pid_controller(speed, time_seconds);
    Serial.print("PID_CONTROLLER OUTPUT: ");
    Serial.println(step);
    Serial.print("\n");
    if (speed_sum > DESIRABLE_SPEED) {
      Serial.println("Starting backwards"); 
      motor.step(abs(step), FORWARD, MICROSTEP);
      Serial.println("Stoping backwards");
    }
    if (speed_sum < DESIRABLE_SPEED) {
      Serial.println("Starting forward");
      motor.step(abs(step), BACKWARD, MICROSTEP);
      Serial.println("Stoping forward");
    }
    T1 = T2;
  }
}

