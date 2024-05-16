#include <AFMotor.h>

#define HALL_PIN 2
#define R 0.32
#define PERIMETER 2 * PI * R

#define DESIRABLE_SPEED 20

/*
I put 500 on this, but I still have no idea what's the exactly number
I created the counter variable because I think we need to have a sort of set of data before
trying to move the motor. If we change the motor every time we get a reading, it'll probably get lost.
*/
#define NUMBER_OF_RECORDINGS 500 

signed long T1 = 0;
AF_Stepper motor(200, 1);

float speed_sum = 0;
int counter = 0;

int calculate_step(int avg){
  // TODO: Calculate steps needed by doing some math.
  // This function should take as parameters the average speed and then perfom math operations
  // to know what will be the next steps by the motor.
  // So suppose the average speed is 25 km/h and the desirable speed defined is 20 km/h, it should
  // get a factor X involving these numbers and then make the motor go backwards by a X number of steps.
  if (avg > DESIRABLE_SPEED) {
    return 1;
  }
  if (avg < DESIRABLE_SPEED) {
    return 2;
  }
  return 1;
}

void setup () {
  pinMode(HALL_PIN, INPUT);
  Serial.begin(9600);
  T1 = millis();
  motor.setSpeed(30); 
  Serial.println("COMECANDO");
}

void loop () {
  if (digitalRead(HALL_PIN) == LOW) {
    if (counter == 500) {
      float average = speed_sum / counter;
      int step = calculate_step(average);
      if (average > DESIRABLE_SPEED) {
        motor.step(step, BACKWARD, MICROSTEP);
      }
      if (average < DESIRABLE_SPEED) {
        motor.step(step, FORWARD, MICROSTEP);
      }
      counter = 0;
      speed_sum = 0;
    }
    signed long T2 = millis();
    signed long time_seconds = (T2 - T1);
    float speed = (PERIMETER / time_seconds) * 1000 * 3.6;
    speed_sum += speed;
    counter+=1;
    Serial.print("Velocidade atual:: ");
    Serial.println(speed);
    Serial.print("t1: ");
    Serial.println(T1);
    Serial.print("t2: ");
    Serial.println(T2);
    Serial.println("Subtracao");
    Serial.println(T1 - T2);
    Serial.println("Time_Seconds");
    Serial.println(time_seconds);
    T1 = T2;
  }
}

