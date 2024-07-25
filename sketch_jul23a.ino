#include <LittleFS.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// WiFi credentials
const char* ssid = "david2";
const char* password = "1234567890";

// WebSocket and Web Server
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Settings
static const TickType_t rpsInterval = 1000 / portTICK_PERIOD_MS;  // 1000ms = 1s (timer for getting pulses) - rotation per second
static const TickType_t controlSampleTime = 100 / portTICK_PERIOD_MS;
static SemaphoreHandle_t mutex;

int rpmInput;
long static rpmSetpoint;
long static rpmSetpointInit;

volatile int pulse; // Variable for counting rotating pulses

double VCC = 3.3; // For ESP32 3.3
int yPin = 25; // GPIO25 PWM
int sensorPin = 27;
int in1Pin = 12;
int in2Pin = 14;

double u, kp;
long rpmMax = 4080; // It requires validation - apply 255 to ENABLE A and measure speed RPM

// ISR fan rotations
void IRAM_ATTR countPulses() {
  pulse++; // Increments each RISING edge.
}

// Globals
static TimerHandle_t getRPMTimer = NULL;
static TimerHandle_t generatePIDControlTimer = NULL;

// WebSocket functions
void notifyClients() {
  ws.textAll(String(rpmSetpoint));
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo* info = (AwsFrameInfo*)arg;

  if (info && info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    if (len < sizeof(data)) {
      data[len] = 0;
    }

    if (strncmp((char*)data, "setRPM:", 7) == 0) {
      String message = String((char*)data);
      message.remove(0, 7); // Remove "setRPM:" from the string
      rpmInput = message.toInt();
      notifyClients();
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

// Timer Callbacks
void getRPMCallback(TimerHandle_t xTimer) {
  rpmInput = (pulse / 2) * 60; // Two pulses each rotation - hall effect sensor
  pulse = 0;
}

void generatePIDControlCallback(TimerHandle_t xTimer) {
  kp = (255.0 / rpmMax);
  u = kp * rpmSetpoint;
}

// Task Functions
void taskPrintGraph(void *parameters) {
  while (true) {
    Serial.print(rpmMax);
    Serial.print(" ");
    Serial.print(0);
    Serial.print(" ");
    Serial.print(rpmInput);
    Serial.println(); 
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void taskGetSetPoint(void *parameters) {
  while (true) {
    if (Serial.available() > 0) {
      xSemaphoreTake(mutex, portMAX_DELAY);
      rpmSetpoint = Serial.parseInt();
      xSemaphoreGive(mutex);
      if (rpmSetpoint != 0) {
        rpmSetpointInit = rpmSetpoint;
      } else if (rpmSetpoint == 0) {
        rpmSetpoint = rpmSetpointInit;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);

  // Conexão WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println(WiFi.localIP());

  // Inicializar WebSocket
  initWebSocket();
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });
  server.begin();

  // Timers
  getRPMTimer = xTimerCreate(
    "getRPMTimer",    // Name of timer
    rpsInterval,           // Period of timer (in ticks)
    pdTRUE,                     // Auto-reload TRUE, one_shot FALSE
    (void *)0,                  // Timer ID
    getRPMCallback);  // Callback function

  generatePIDControlTimer = xTimerCreate(
    "generatePIDControlTimer",    // Name of timer
    controlSampleTime,           // Period of timer (in ticks)
    pdTRUE,                     // Auto-reload TRUE, one_shot FALSE
    (void *)1,                  // Timer ID
    generatePIDControlCallback);  // Callback function

  mutex = xSemaphoreCreateMutex();
      
  // Configurações
  attachInterrupt(sensorPin, countPulses, RISING);
  analogWriteResolution(yPin, 8);
  analogWriteFrequency(yPin, 30000);
  pinMode(sensorPin, INPUT);
  pinMode(yPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  
  rpmSetpoint = 3000; 
  
  // Iniciar tarefas
  xTaskCreatePinnedToCore(taskPrintGraph, "taskPrintGraph", 1024, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskGetSetPoint, "taskGetSetPoint", 1024, NULL, 1, NULL, 0);

  xTimerStart(getRPMTimer, 0);
  xTimerStart(generatePIDControlTimer, 0);
}  
                            
void loop() { 
  ws.cleanupClients();
  analogWrite(yPin, (int)u); // ESP32 PWM - ENABLE A L298D
}
