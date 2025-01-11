#include <AccelStepper.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>

#define HALL_PIN 13
#define RELAY_PIN 11
#define R 0.3429
#define PERIMETER 2 * PI *R

#define motorInterfaceType 1
#define dirPin 8
#define stepPin 9
#define enablePin 4

#define NUM_OF_CYCLES 10
#define MAX_TEMP_SENSOR 35

AccelStepper motor(motorInterfaceType, stepPin, dirPin);

signed long T1 = 0;
signed long T2 = 0;
signed long time_seconds = 0;

float setpoint = 15.0;
float kp = 1.45;

QueueHandle_t speedQueue;

double pid_controller(double avg) {
  double error = setpoint - avg;
  return (kp * error);
}

float get_speed(long *t_delta) {
  T2 = millis();
  time_seconds = (T2 - T1);
  *t_delta = time_seconds;
  float s = (PERIMETER / time_seconds) * 1000 * 3.6;
  T1 = T2;

  return s;
}

void TaskSpeed(void *pvParameters) {
  pinMode(HALL_PIN, INPUT);

  float speedVal = 0.0;
  long time_delta = 0;
  T1 = millis();

  int i = 0;

  for (;;) {
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    //if (digitalRead(HALL_PIN) == HIGH) {
      time_delta = 0;
      speedVal = i + 1;
      if (xQueueSend(speedQueue, &speedVal, portMAX_DELAY) != pdTRUE) {
        Serial.println("[ERROR] Failed to send speed to PID task.");
      }
      i++;

    //}
  }
}

void TaskPid(void *pvParameters) {

  float speedReceived = 0.0;
  double stepPID = 0.0;

  for (;;) {

    vTaskDelay(20 / portTICK_PERIOD_MS);
    if (xQueueReceive(speedQueue, &speedReceived, portMAX_DELAY) == pdPASS) {
      stepPID = pid_controller(speedReceived);
    }
  }
}

void TaskValve(void *pvParameters) {
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(6000 / portTICK_PERIOD_MS);
  }
}

void TaskPrintOnSerial(void *pvParameters) {

  float speedReceived = 0.0;

  for (;;) {
    if (xQueuePeek(speedQueue, &speedReceived, portMAX_DELAY)) {
      Serial.println(speedReceived);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    vTaskDelay(1);
  }


  Serial.println("Starting to create tasks...");

  speedQueue = xQueueCreate(2, sizeof(float));

  if (speedQueue != NULL) {
    xTaskCreate(TaskSpeed, "TaskSpeed", 128, NULL, 3, NULL);
    xTaskCreate(TaskPid, "TaskPid", 128, NULL, 3, NULL);
    xTaskCreate(TaskValve, "TaskValve", 128, NULL, 1, NULL);
    xTaskCreate(TaskPrintOnSerial, "TaskPrintOnSerial", 128, NULL, 1, NULL);
    Serial.println("After creating tasks...");
  }

  Serial.println("Hello there");
}

void loop() {
}
