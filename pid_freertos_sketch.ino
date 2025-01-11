#include <AccelStepper.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>

#define HALL_PIN 13
#define RELAY_PIN 11
#define R 0.3429
#define PERIMETER 2 * PI * R

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

double pid_controller(double avg){
  double error = setpoint - avg;
  return (kp * error);
}

float get_speed(long* t_delta) {
    T2 = millis();
    time_seconds = (T2 - T1);
    *t_delta = time_seconds;
    float s = (PERIMETER / time_seconds) * 1000 * 3.6;
    T1 = T2;

    return s;
}

QueueHandle_t speedQueue;
QueueHandle_t pidQueue;

void TaskSpeed(void *pvParameters) {
  pinMode(HALL_PIN, INPUT);

  float speedVal = 0.0;
  long time_delta = 0;
  T1 = millis();
  
  for (;;) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (digitalRead(HALL_PIN) == HIGH) {
      time_delta = 0;
      speedVal = get_speed(&time_delta);
      if (xQueueSend(speedQueue, &speedVal, portMAX_DELAY) != pdTRUE) {
        Serial.println("[ERROR] Failed to send speed to PID task.");    
      }
    }
    
  }
}

void TaskPid(void *pvParameters) {

  float speedReceived = 0.0;
  double stepPID = 0.0;
  
  for (;;) {
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (xQueueReceive(speedQueue, &speedReceived, portMAX_DELAY) == pdPASS) {
      Serial.print("[INFO] Speed received on PID task: ");
      Serial.println(speedReceived);

      stepPID = pid_controller(speedReceived);
      if (xQueueSend(pidQueue, &stepPID, portMAX_DELAY) != pdTRUE) {
        Serial.println("[ERROR] Failed to send stepPID to STEP task");
      }
    }
    
  }
}

void TaskStep(void *pvParameters) {
  double stepPID = 0.0;

  for (;;) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("Im at TaskStep");
    if (xQueueReceive(pidQueue, &stepPID, portMAX_DELAY) == pdPASS) {
      Serial.print("[INFO] stepPID received on TASK task: ");
      Serial.println(stepPID);
    }
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    vTaskDelay(1);
  }


  Serial.println("Starting to create tasks...");

  speedQueue = xQueueCreate(1, sizeof(float));
  pidQueue = xQueueCreate(1, sizeof(double));
  
  if (speedQueue != NULL) {
    xTaskCreate(TaskSpeed,"TaskSpeed",128,NULL, 3,NULL);
    xTaskCreate(TaskPid,"TaskPid",128,NULL, 3,NULL);
    xTaskCreate(TaskStep,"TaskStep",128,NULL, 3,NULL);
    Serial.println("After creating tasks...");
  }

}

void loop() {}
