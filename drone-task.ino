#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <esp_wifi.h>  // For WiFi power save functions
#include <esp_task_wdt.h>  // For watchdog timer
#include <esp_system.h>   // For reset reason
#include <time.h>         // For time functions
#include <sys/time.h>     // For struct timeval
#include <WiFiUdp.h>      // For NTPClient
#include <NTPClient.h>    // For NTP time synchronization


// Command types for flight control
enum CommandType {
    CMD_TAKEOFF,
    CMD_HOVER,
    CMD_LAND,
    CMD_EMERGENCY_STOP
};

// PID output structure
struct PIDOutput {
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    float altitude = 0;
} pidOutput;

// Global variables for mutexes and queues
SemaphoreHandle_t mutexAttitude = nullptr;
SemaphoreHandle_t mutexSensorData = nullptr;
SemaphoreHandle_t mutexPIDOutput = nullptr;
SemaphoreHandle_t mutexMotors = nullptr;
QueueHandle_t commandQueue = nullptr;
EventGroupHandle_t flightEvents = nullptr;

// Function declarations
void MainSensorManagerTask(void *pv);
void MainStabilizerTask(void *pv);
void MainCommandTask(void *pv);
void MainFailsafeTask(void *pv);
void handleFlightCommand(CommandType cmd);
void connectToWiFi();

// Forward declarations
struct SensorData;
struct Attitude;
struct MotorOutput;

// Function declarations
void monitorTasks();
void printStackTrace();

// 서버 전송 주기 (ms)
const unsigned long SERVER_UPDATE_INTERVAL = 1000;
unsigned long lastServerUpdate = 0;

// ========== WiFi 및 NTP 설정 ==========
const char* ssid = "U5555";
const char* password = "5555";
const char* serverUrl = "http://192.168.123.111:5003/api/data";  // Updated to match server IP

// NTP 서버 목록 (여러 서버를 사용하여 안정성 확보)
const char* ntpServers[] = {
  "time.google.com",
  "time.cloudflare.com",
  "kr.pool.ntp.org",
  "time.windows.com",
  "pool.ntp.org"
};
const int ntpServerCount = 5;
int currentNtpServer = 0;
unsigned long lastNtpUpdate = 0;
const unsigned long NTP_UPDATE_INTERVAL = 3600000; // 1시간마다 NTP 업데이트 시도
const unsigned long NTP_TIMEOUT = 5000; // NTP 요청 타임아웃 (5초)

// NTP 클라이언트 설정
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP); // 초기화만 해두고 나중에 서버 설정
bool timeSynced = false;


// ========== 공통 구조체 정의 ==========
struct Attitude {
  float pitch, roll, yaw;
  float altitude;  // 고도 추가
};

struct MotorOutput {
  float m1, m2, m3, m4;
};

// PIDOutput struct is defined at the top of the file

enum FlightPhase { 
    IDLE,
    TAKEOFF,
    HOVER,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    LAND,
    EMERGENCY_STOP
 };

// 비행 시나리오 관리 구조체
struct FlightScenario {
    FlightPhase currentPhase;
    unsigned long phaseStartTime;
    float targetAltitude;  // 목표 고도 (m)
    float currentAltitude; // 현재 고도 (m)
    float climbRate;       // 상승/하강률 (m/s)
};

FlightScenario flightState = {IDLE, 0, 0, 0, 0};

// ========== 상수 정의 ==========
const int SAMPLE_COUNT_FOR_OUTPUT = 350;  // N번에 한 번씩 출력

// ========== 공유 변수/핸들 ==========
Attitude attitude;
MotorOutput motors;

#define EVT_GPS_READY (1 << 0)
#define EVT_RC_READY  (1 << 1)


// ========== Task Monitoring ==========
void printTaskInfo(const char* taskName) {
  TaskHandle_t handle = xTaskGetHandle(taskName);
  if (handle != NULL) {
    Serial.print("[Task Monitor] ");
    Serial.print(taskName);
    Serial.print(" - Stack High Water Mark: ");
    Serial.print(uxTaskGetStackHighWaterMark(handle));
    Serial.print(", Core: ");
    Serial.println(xTaskGetAffinity(handle) & 0x01 ? "Core 0" : "Core 1");
  } else {
    Serial.print("[Task Monitor] Task not found: ");
    Serial.println(taskName);
  }
}

void monitorTasks() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 10000) {  // Print every 10 seconds
    lastPrint = millis();
    
    Serial.println("\n===== Task Status =====");
    
    // Monitor only the main tasks that we create in setup()
    printTaskInfo("MainSensMgr");
    printTaskInfo("MainStab");
    printTaskInfo("MainMotor");
    printTaskInfo("MainCmd");
    printTaskInfo("MainTlm");
    printTaskInfo("MainFail");
    // Sub-tasks are not monitored as they are created by main tasks
    
    // Check free heap
    Serial.print("\nFree Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.print(" bytes");
    
    // Check minimum free heap
    Serial.print("Minimum Free Heap: ");
    Serial.print(ESP.getMinFreeHeap());
    Serial.println(" bytes");
    
    Serial.println("======================\n");
  }
}

// ========== Crash Handler ==========
void printStackTrace() {
  Serial.println("\n\n===== CRASH OCCURRED =====");
  Serial.println("Stack trace:");
  
  // Print reset reason
  Serial.print("Reset reason: ");
  switch(esp_reset_reason()) {
    case ESP_RST_POWERON: Serial.println("POWERON_RESET"); break;
    case ESP_RST_SW: Serial.println("SW_RESET"); break;
    case ESP_RST_INT_WDT: 
    case ESP_RST_TASK_WDT: 
    case ESP_RST_WDT: Serial.println("WDT_RESET"); break;
    case ESP_RST_DEEPSLEEP: Serial.println("DEEPSLEEP_RESET"); break;
    case ESP_RST_BROWNOUT: Serial.println("BROWNOUT_RESET"); break;
    default: Serial.println(esp_reset_reason());
  }
  
  // Print task info
  Serial.println("\nActive tasks at time of crash:");
  char taskListBuffer[1024];  // Increased buffer size
  vTaskList(taskListBuffer);
  Serial.println(taskListBuffer);
  
  // Print detailed stack info
  Serial.println("\nStack usage:");
  for (int i = 0; i < uxTaskGetNumberOfTasks(); i++) {
    TaskStatus_t taskStatus;
    vTaskGetInfo(NULL, &taskStatus, pdTRUE, eInvalid);
    Serial.printf("Task %s: Stack High Water Mark: %u\n", 
                  taskStatus.pcTaskName, 
                  uxTaskGetStackHighWaterMark(NULL));
  }
  
  // Print heap info
  Serial.print("Free Heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  
  Serial.println("==========================\n");
}

// ========== NTP 시간 동기화 ==========
void syncTime() {
  static bool initialized = false;
  static unsigned long lastAttemptTime = 0;
  static int retryCount = 0;
  const unsigned long RETRY_INTERVAL = 10000; // 10초마다 재시도
  
  // 메모리 상태 확인
  Serial.printf("[NTP] Free heap: %d bytes\n", ESP.getFreeHeap());
  
  // WiFi 연결 확인
  if (!isWiFiConnected()) {
    Serial.println("[NTP] WiFi not connected, trying to reconnect...");
    connectToWiFi();
    if (!isWiFiConnected()) {
      timeSynced = false;
      return;
    }
  }
  
  // NTP 클라이언트 초기화
  if (!initialized) {
    Serial.println("[NTP] Initializing NTP client");
    timeClient.begin();
    timeClient.setUpdateInterval(0); // 자동 업데이트 비활성화 (수동으로 제어)
    timeClient.setTimeOffset(9 * 3600); // KST (UTC+9)
    initialized = true;
  }
  
  // 현재 시간 가져오기
  unsigned long currentTime = millis();
  
  // 주기적으로 NTP 업데이트 시도
  if (currentTime - lastAttemptTime >= RETRY_INTERVAL) {
    lastAttemptTime = currentTime;
    
    // 현재 NTP 서버 설정
    timeClient.setPoolServerName(ntpServers[currentNtpServer]);
    Serial.printf("[NTP] Trying server %d/%d: %s\n", 
                  currentNtpServer + 1, ntpServerCount, ntpServers[currentNtpServer]);
    
    // NTP 서버 응답 대기
    bool success = false;
    unsigned long startTime = millis();
    
    // 최대 5초 동안 시도
    while (millis() - startTime < NTP_TIMEOUT) {
      if (timeClient.forceUpdate()) {
        success = true;
        break;
      }
      delay(100); // 짧은 대기
    }
    
    if (success) {
      timeSynced = true;
      retryCount = 0;
      
      // 성공한 서버 정보 출력
      Serial.print("[NTP] Time updated: ");
      Serial.print(timeClient.getFormattedTime());
      Serial.print(" from ");
      Serial.println(ntpServers[currentNtpServer]);
      
      // 다음에 같은 서버를 먼저 시도하도록 유지
      lastNtpUpdate = millis();
    } else {
      // 실패 시 다음 서버로 전환
      currentNtpServer = (currentNtpServer + 1) % ntpServerCount;
      retryCount++;
      
      Serial.printf("[NTP] Failed to sync (Attempt %d)\n", retryCount);
      
      // 여러 번 실패하면 WiFi 재연결 시도
      if (retryCount >= ntpServerCount * 2) {
        Serial.println("[NTP] Too many retries, reconnecting WiFi...");
        WiFi.disconnect();
        connectToWiFi();
        retryCount = 0;
      }
    }
  }
  
  // 1시간마다 시간 출력 (디버깅용)
  static unsigned long lastPrintTime = 0;
  if (timeSynced && currentTime - lastPrintTime >= 3600000) {
    lastPrintTime = currentTime;
    Serial.print("[NTP] Current time: ");
    Serial.println(timeClient.getFormattedTime());
  }
}

// ========== 현재 타임스탬프 가져오기 ==========
unsigned long getCurrentTimestamp() {
  static unsigned long timeOffset = 0;
  static unsigned long lastMillis = 0;
  static bool firstRun = true;
  
  if (firstRun) {
    lastMillis = millis();
    firstRun = false;
  }
  
  // NTP 시간이 동기화되어 있으면 NTP 시간 사용
  if (timeSynced) {
    unsigned long currentMillis = millis();
    // 오버플로우 처리
    if (currentMillis < lastMillis) {
      timeOffset += 0xFFFFFFFF - lastMillis + currentMillis;
    } else {
      timeOffset += currentMillis - lastMillis;
    }
    lastMillis = currentMillis;
    return (timeClient.getEpochTime() * 1000) + (millis() % 1000);
  } 
  // NTP 동기화 실패 시 시스템 업타임 반환 (시작 후 경과 시간)
  else {
    return millis();
  }
}

// ========== WiFi Connection ==========
void connectToWiFi() {
  // 이미 연결되어 있으면 리턴
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  
  Serial.println("Connecting to WiFi...");
  WiFi.disconnect(true); // 이전 연결 정리
  delay(100);
  
  // WiFi 모드 설정
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  // 연결 시도 (최대 10초)
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startTime < 10000)) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // WiFi 연결 성공 시 시간 동기화 시도
    if (!timeSynced) {
      syncTime();
    }
  } else {
    Serial.println("\nFailed to connect to WiFi");
    timeSynced = false;
    WiFi.disconnect();
  }
}

// WiFi 연결 상태 확인 함수
bool isWiFiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

// ========== 1. Main Task Implementations ==========
void MainSensorManagerTask(void *pv) {
    // Initialize sensor manager
    Serial.println("Sensor Manager Task started");
    
    // Create sensor subtasks
    xTaskCreatePinnedToCore(SubSensorManager_IMUTask, "IMUTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(SubSensorManager_BarometerTask, "BaroTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(SubSensorManager_GPSTask, "GPSTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(SubSensorManager_MagnetometerTask, "MagTask", 4096, NULL, 1, NULL, 1);
    
    // Main sensor manager loop
    while (1) {
        // Update flight scenario state
        updateFlightScenario();
        
        // Monitor tasks and system health
        static unsigned long lastMonitorTime = 0;
        if (millis() - lastMonitorTime > 1000) {
            lastMonitorTime = millis();
            monitorTasks();
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// PID controller structure
struct PIDController {
    float kp, ki, kd;
    float integral = 0;
    float prev_error = 0;
    unsigned long last_time = 0;
    
    float compute(float setpoint, float input) {
        unsigned long now = millis();
        float dt = (now - last_time) / 1000.0f; // Convert to seconds
        if (dt <= 0) dt = 0.01f; // Prevent division by zero
        
        float error = setpoint - input;
        integral += error * dt;
        integral = constrain(integral, -100, 100); // Anti-windup
        
        float derivative = (error - prev_error) / dt;
        
        float output = kp * error + ki * integral + kd * derivative;
        
        prev_error = error;
        last_time = now;
        
        return output;
    }
};

// PID controllers for each axis
PIDController pitchPID = {2.0f, 0.5f, 1.0f};
PIDController rollPID = {2.0f, 0.5f, 1.0f};
PIDController yawPID = {1.0f, 0.1f, 0.5f};
PIDController altPID = {10.0f, 0.1f, 5.0f};

void MainStabilizerTask(void *pv) {
    // Initialize stabilizer
    Serial.println("Stabilizer Task started");
    
    // Main stabilizer loop
    while (1) {
        // Get current sensor data with mutex protection
        Attitude currentAttitude;
        xSemaphoreTake(mutexAttitude, portMAX_DELAY);
        currentAttitude = attitude;
        xSemaphoreGive(mutexAttitude);
        
        // Get current flight state
        FlightPhase currentPhase = flightState.currentPhase;
        float targetAltitude = flightState.targetAltitude;
        
        // Calculate PID outputs
        PIDOutput localPID;
        
        // Only apply stabilization if not in IDLE mode
        if (currentPhase != IDLE) {
            // Target angles (in degrees, converted to radians for calculations)
            const float targetPitch = 0.0f * (PI/180.0f);
            const float targetRoll = 0.0f * (PI/180.0f);
            const float targetYaw = 0.0f * (PI/180.0f);
            
            // Calculate PID outputs
            localPID.pitch = pitchPID.compute(targetPitch, currentAttitude.pitch);
            localPID.roll = rollPID.compute(targetRoll, currentAttitude.roll);
            localPID.yaw = yawPID.compute(targetYaw, currentAttitude.yaw);
            
            // Altitude control (only active in HOVER mode)
            if (currentPhase == HOVER) {
                localPID.altitude = altPID.compute(targetAltitude, currentAttitude.altitude);
            } else {
                localPID.altitude = 0;
            }
        } else {
            // Reset all PIDs to zero in IDLE mode
            localPID.pitch = 0;
            localPID.roll = 0;
            localPID.yaw = 0;
            localPID.altitude = 0;
        }
        
        // Update PID output with mutex protection
        xSemaphoreTake(mutexPIDOutput, portMAX_DELAY);
        pidOutput = localPID;
        xSemaphoreGive(mutexPIDOutput);
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void MainCommandTask(void *pv) {
    // Initialize command processor
    Serial.println("Command Task started");
    
    // Main command processing loop
    while (1) {
        // Process incoming commands
        CommandType cmd;
        if (xQueueReceive(commandQueue, &cmd, 100 / portTICK_PERIOD_MS) == pdPASS) {
            handleFlightCommand(cmd);
        }
    }
}

void MainFailsafeTask(void *pv) {
    // Initialize failsafe system
    Serial.println("Failsafe Task started");
    
    // Main failsafe loop
    while (1) {
        // Check system health and trigger failsafe if needed
        if (flightState.currentPhase != IDLE && millis() - xTaskGetTickCount() > 60000) {
            // If flight has been ongoing for more than 60 seconds, force land
            handleFlightCommand(CMD_LAND);
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// ========== 2. Sensor Simulation Tasks ==========
// Flight scenario state management
void updateFlightScenario() {
    static unsigned long lastUpdate = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastUpdate < 100) return; // Update every 100ms
    lastUpdate = currentTime;
    
    // Process each flight phase
    switch(flightState.currentPhase) {
        case TAKEOFF: {
            // Ascend to target altitude
            if (flightState.currentAltitude < flightState.targetAltitude) {
                flightState.currentAltitude += flightState.climbRate * 0.1; // Distance per 100ms
                if (flightState.currentAltitude > flightState.targetAltitude) {
                    flightState.currentAltitude = flightState.targetAltitude;
                    flightState.currentPhase = HOVER;
                    flightState.phaseStartTime = currentTime;
                    Serial.println("TAKEOFF complete, starting HOVER");
                }
            }
            break;
        }
            
        case HOVER: {
            // Hover for 10 seconds then start movement sequence
            if (currentTime - flightState.phaseStartTime > 10000) { // 10 seconds hover
                flightState.currentPhase = MOVE_FORWARD;
                flightState.phaseStartTime = currentTime;
                Serial.println("HOVER complete, starting MOVEMENT SEQUENCE");
            }
            break;
        }
        
        case MOVE_FORWARD: {
            // Move forward for 10 seconds
            if (currentTime - flightState.phaseStartTime > 10000) {
                flightState.currentPhase = MOVE_BACKWARD;
                flightState.phaseStartTime = currentTime;
                Serial.println("FORWARD complete, moving BACKWARD");
            } else {
                // Simulate forward movement (pitch forward)
                xSemaphoreTake(mutexAttitude, portMAX_DELAY);
                attitude.pitch = 10.0f; // 10 degrees forward
                xSemaphoreGive(mutexAttitude);
            }
            break;
        }
        
        case MOVE_BACKWARD: {
            // Move backward for 10 seconds
            if (currentTime - flightState.phaseStartTime > 10000) {
                flightState.currentPhase = MOVE_LEFT;
                flightState.phaseStartTime = currentTime;
                Serial.println("BACKWARD complete, moving LEFT");
            } else {
                // Simulate backward movement (pitch backward)
                xSemaphoreTake(mutexAttitude, portMAX_DELAY);
                attitude.pitch = -10.0f; // 10 degrees backward
                xSemaphoreGive(mutexAttitude);
            }
            break;
        }
        
        case MOVE_LEFT: {
            // Move left for 10 seconds
            if (currentTime - flightState.phaseStartTime > 10000) {
                flightState.currentPhase = MOVE_RIGHT;
                flightState.phaseStartTime = currentTime;
                Serial.println("LEFT complete, moving RIGHT");
            } else {
                // Simulate left movement (roll left)
                xSemaphoreTake(mutexAttitude, portMAX_DELAY);
                attitude.roll = -10.0f; // 10 degrees left
                xSemaphoreGive(mutexAttitude);
            }
            break;
        }
        
        case MOVE_RIGHT: {
            // Move right for 10 seconds
            if (currentTime - flightState.phaseStartTime > 10000) {
                flightState.currentPhase = LAND;
                flightState.phaseStartTime = currentTime;
                Serial.println("RIGHT complete, starting LAND");
            } else {
                // Simulate right movement (roll right)
                xSemaphoreTake(mutexAttitude, portMAX_DELAY);
                attitude.roll = 10.0f; // 10 degrees right
                xSemaphoreGive(mutexAttitude);
            }
            break;
        }
            
        case LAND: {
            // Descend to ground
            if (flightState.currentAltitude > 0.2) {
                flightState.currentAltitude -= flightState.climbRate * 0.08; // Slightly slower descent
                if (flightState.currentAltitude < 0.2) {
                    flightState.currentAltitude = 0;
                    flightState.currentPhase = IDLE;
                    flightState.phaseStartTime = currentTime;
                    Serial.println("LAND complete, back to IDLE");
                }
            }
            break;
        }
            
        case IDLE:
        default: {
            // Ready for next command
            break;
        }
    }
}

// Handle flight commands from queue
void handleFlightCommand(CommandType cmd) {
    switch(cmd) {
        case CMD_TAKEOFF: {
            if (flightState.currentPhase == IDLE) {
                flightState.currentPhase = TAKEOFF;
                flightState.targetAltitude = 5.0; // Target 5 meters
                flightState.climbRate = 1.0;      // 1 m/s climb rate
                flightState.phaseStartTime = millis();
                Serial.println("Starting TAKEOFF");
            }
            break;
        }
            
        case CMD_HOVER: {
            if (flightState.currentPhase == TAKEOFF) {
                flightState.currentPhase = HOVER;
                flightState.phaseStartTime = millis();
                Serial.println("Starting HOVER");
            }
            break;
        }
            
        case CMD_LAND: {
            if (flightState.currentPhase == HOVER || flightState.currentPhase == TAKEOFF) {
                flightState.currentPhase = LAND;
                flightState.targetAltitude = 0;
                flightState.phaseStartTime = millis();
                Serial.println("Starting LAND");
            }
            break;
        }
            
        case CMD_EMERGENCY_STOP: {
            flightState.currentPhase = IDLE;
            flightState.currentAltitude = 0;
            flightState.targetAltitude = 0;
            Serial.println("EMERGENCY STOP");
            break;
        }
    }
}

// 센서 데이터 구조체
struct IMUData {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
};
struct BarometerData {
  float altitude;
};
struct GPSData {
  float lat, lon, alt, speed;
};
struct MagnetometerData {
  float mag_x, mag_y, mag_z;
};
struct SensorData {
  IMUData imu;
  BarometerData baro;
  GPSData gps;
  MagnetometerData mag;
};

SensorData sensorData;

void SubSensorManager_IMUTask(void *pv) {
  float t = 0;
  while (1) {
    IMUData imu;
    
    // Base values with some noise
    float baseZ = 9.8;  // Gravity
    float noise = random(-50, 50) / 1000.0;  // Small noise
    
    // Adjust values based on flight phase
    switch(flightState.currentPhase) {
        case TAKEOFF:
            // Slight upward acceleration during takeoff
            imu.accel_z = baseZ + 0.5 + noise * 2.0;
            imu.gyro_x = 0.02 * sin(t) + random(-5,5)/1000.0;
            imu.gyro_y = 0.02 * cos(t) + random(-5,5)/1000.0;
            break;
            
        case HOVER:
            // Stable hover with minimal movement
            imu.accel_z = baseZ + noise;
            imu.gyro_x = 0.01 * sin(t*0.5) + random(-3,3)/1000.0;
            imu.gyro_y = 0.01 * cos(t*0.5) + random(-3,3)/1000.0;
            break;
            
        case MOVE_FORWARD:
            // Forward movement - slight negative pitch rate (gyro_y negative when moving forward)
            imu.accel_z = baseZ + 0.1 + noise * 1.5;
            imu.gyro_y = -0.5 + 0.05 * sin(t*2.0) + random(-10,10)/1000.0;
            imu.gyro_x = 0.01 * sin(t*0.5) + random(-3,3)/1000.0;
            break;
            
        case MOVE_BACKWARD:
            // Backward movement - slight positive pitch rate (gyro_y positive when moving backward)
            imu.accel_z = baseZ + 0.1 + noise * 1.5;
            imu.gyro_y = 0.5 + 0.05 * cos(t*2.0) + random(-10,10)/1000.0;
            imu.gyro_x = 0.01 * cos(t*0.5) + random(-3,3)/1000.0;
            break;
            
        case MOVE_LEFT:
            // Left movement - positive roll rate (gyro_x positive when moving left)
            imu.accel_z = baseZ + 0.1 + noise * 1.5;
            imu.gyro_x = 0.5 + 0.05 * sin(t*2.0) + random(-10,10)/1000.0;
            imu.gyro_y = 0.01 * sin(t*0.5) + random(-3,3)/1000.0;
            break;
            
        case MOVE_RIGHT:
            // Right movement - negative roll rate (gyro_x negative when moving right)
            imu.accel_z = baseZ + 0.1 + noise * 1.5;
            imu.gyro_x = -0.5 + 0.05 * cos(t*2.0) + random(-10,10)/1000.0;
            imu.gyro_y = 0.01 * cos(t*0.5) + random(-3,3)/1000.0;
            break;
            
        case LAND:
            // Slight downward acceleration during landing
            imu.accel_z = baseZ - 0.3 + noise * 1.5;
            imu.gyro_x = 0.015 * sin(t*0.7) + random(-4,4)/1000.0;
            imu.gyro_y = 0.015 * cos(t*0.7) + random(-4,4)/1000.0;
            break;
            
        case IDLE:
        default:
            // Minimal sensor noise when idle
            imu.accel_z = baseZ + noise * 0.5;
            imu.gyro_x = random(-2,2)/1000.0;
            imu.gyro_y = random(-2,2)/1000.0;
            break;
    }
    
    // Common sensor values
    imu.accel_x = 0.05 * sin(t*1.3) + random(-50,50)/1000.0;
    imu.accel_y = 0.05 * cos(t*1.1) + random(-50,50)/1000.0;
    imu.gyro_z = 0.01 * sin(t*0.5) + random(-5,5)/1000.0;
    
    xSemaphoreTake(mutexSensorData, portMAX_DELAY);
    sensorData.imu = imu;
    xSemaphoreGive(mutexSensorData);
    
    t += 0.05;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void SubSensorManager_BarometerTask(void *pv) {
  float t = 0;
  while (1) {
    BarometerData baro;
    
    // Use the current altitude from flight state with some noise
    float altNoise = random(-10, 10) / 100.0;  // ±10cm noise
    baro.altitude = flightState.currentAltitude + altNoise;
    
    // Add some small oscillations
    float osc = 0.05 * sin(t * 0.5);  // Slow oscillation
    baro.altitude += osc;
    
    // Clamp to minimum 0
    if (baro.altitude < 0) baro.altitude = 0;
    
    xSemaphoreTake(mutexSensorData, portMAX_DELAY);
    sensorData.baro = baro;
    xSemaphoreGive(mutexSensorData);
    
    t += 0.03;
    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

void SubSensorManager_GPSTask(void *pv) {
  float lat0 = 37.5665, lon0 = 126.9780;
  float t = 0;
  while (1) {
    GPSData gps;
    gps.lat = lat0 + 0.0001 * sin(t);
    gps.lon = lon0 + 0.0001 * cos(t);
    gps.alt = 100.0 + 2.0 * sin(t/3.0);
    gps.speed = 1.5 + 0.5 * cos(t/2.0);
    xSemaphoreTake(mutexSensorData, portMAX_DELAY);
    sensorData.gps = gps;
    xSemaphoreGive(mutexSensorData);
    t += 0.07;
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void SubSensorManager_MagnetometerTask(void *pv) {
  float t = 0;
  while (1) {
    MagnetometerData mag;
    mag.mag_x = 0.3 * cos(t);
    mag.mag_y = 0.3 * sin(t);
    mag.mag_z = 0.1 * cos(2*t);
    xSemaphoreTake(mutexSensorData, portMAX_DELAY);
    sensorData.mag = mag;
    xSemaphoreGive(mutexSensorData);
    
    // Process any pending commands
    CommandType cmd;
    if (xQueueReceive(commandQueue, &cmd, 0) == pdPASS) {
        handleFlightCommand(cmd);
    }
    
    t += 0.01;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
    
    CommandType cmd;
    while (1) {
        if (xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdPASS) {
            // Process command
            handleFlightCommand(cmd);
            
            // 현재 비행 상태 출력
            const char* phaseStr = "";
            switch(flightState.currentPhase) {
                case TAKEOFF: phaseStr = "TAKEOFF"; break;
                case HOVER: phaseStr = "HOVER"; break;
                case LAND: phaseStr = "LAND"; break;
                case IDLE: phaseStr = "IDLE"; break;
            }
            Serial.printf("Flight Phase: %s, Altitude: %.2fm\n", phaseStr, flightState.currentAltitude);
        }
    }
}

// ========== 3. Motor Control Task ==========
void MainMotorOutputTask(void *pv) {
    mutexMotors = xSemaphoreCreateMutex();
    if (mutexMotors == NULL) {
        Serial.println("Failed to create motors mutex");
        while(1);
    }
    
    float baseThrottle = 1000; // Base throttle (idle)
    unsigned long lastPrint = 0;
    
    while (1) {
        // Get current PID values
        PIDOutput localPID;
        xSemaphoreTake(mutexPIDOutput, portMAX_DELAY);
        localPID = pidOutput;
        xSemaphoreGive(mutexPIDOutput);
        
        // Set base throttle based on flight phase
        switch(flightState.currentPhase) {
            case TAKEOFF: {
                // Ramp up throttle during takeoff
                float progress = flightState.currentAltitude / flightState.targetAltitude;
                baseThrottle = 1150 + (progress * 300); // 1150 to 1450
                if (baseThrottle > 1450) baseThrottle = 1450;
                break;
            }
                
            case HOVER: {
                // Maintain hover with small adjustments
                baseThrottle = 1420 + (sin(millis() * 0.002) * 10); // Small oscillations
                break;
            }
                
            case LAND: {
                // Gradually reduce throttle during landing
                float progress = 1.0 - (flightState.currentAltitude / 5.0);
                baseThrottle = 1400 - (progress * 400); // 1400 to 1000
                if (baseThrottle < 1000) baseThrottle = 1000;
                break;
            }
                
            case IDLE:
            default: {
                baseThrottle = 1000; // Motors off
                break;
            }
        }

        // Motor mixing (X configuration) with altitude control
        float alt_control = localPID.altitude * 10; // Altitude control gain
        
        // Calculate motor outputs with stabilization
        float m1 = baseThrottle + localPID.pitch + localPID.roll + localPID.yaw + alt_control;
        float m2 = baseThrottle + localPID.pitch - localPID.roll - localPID.yaw + alt_control;
        float m3 = baseThrottle - localPID.pitch + localPID.roll - localPID.yaw + alt_control;
        float m4 = baseThrottle - localPID.pitch - localPID.roll + localPID.yaw + alt_control;

        // Constrain outputs to valid PWM range (1000-2000μs)
        m1 = constrain(m1, 1000, 2000);
        m2 = constrain(m2, 1000, 2000);
        m3 = constrain(m3, 1000, 2000);
        m4 = constrain(m4, 1000, 2000);
        
        // Update motor outputs with mutex protection
        xSemaphoreTake(mutexMotors, portMAX_DELAY);
        motors.m1 = m1;
        motors.m2 = m2;
        motors.m3 = m3;
        motors.m4 = m4;
        xSemaphoreGive(mutexMotors);
        
        // Debug output every 2 seconds
        if (millis() - lastPrint > 2000) {
            lastPrint = millis();
            Serial.printf("Motors: %.0f, %.0f, %.0f, %.0f | Phase: %d | Alt: %.2fm\n", 
                         m1, m2, m3, m4, flightState.currentPhase, flightState.currentAltitude);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ========== 5. Telemetry Task ==========
void sendTelemetryData(const String& jsonData) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(serverUrl);
        http.addHeader("Content-Type", "application/json");
        
        int httpResponseCode = http.POST(jsonData);
        
        if (httpResponseCode > 0) {
            String response = http.getString();
            Serial.printf("[HTTP] POST... code: %d\n", httpResponseCode);
        } else {
            Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
        }
        
        http.end();
    } else {
        Serial.println("WiFi Disconnected. Cannot send telemetry data.");
    }
}

void MainTelemetryTask(void *pv) {
    unsigned long lastTelemetryTime = 0;
    
    // WiFi 연결
    connectToWiFi();
    
    // WiFi 연결 실패 시에도 계속 진행 (나중에 재시도)
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWarning: Starting telemetry without WiFi");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to WiFi. Telemetry data will not be sent to server.");
    }
    
    while (1) {
        unsigned long currentTime = millis();
        
        // Send telemetry every second
        if (currentTime - lastTelemetryTime >= 1000) {
            lastTelemetryTime = currentTime;
            
            // 센서 데이터 가져오기
            SensorData localData;
            xSemaphoreTake(mutexSensorData, portMAX_DELAY);
            localData = sensorData;
            xSemaphoreGive(mutexSensorData);
            
            // Get motor outputs with mutex protection
            float m1 = 0, m2 = 0, m3 = 0, m4 = 0;
            xSemaphoreTake(mutexMotors, portMAX_DELAY);
            m1 = motors.m1;
            m2 = motors.m2;
            m3 = motors.m3;
            m4 = motors.m4;
            xSemaphoreGive(mutexMotors);
            
            // Convert flight phase to string
            const char* phaseStr = "UNKNOWN";
            switch(flightState.currentPhase) {
                case IDLE: phaseStr = "IDLE"; break;
                case TAKEOFF: phaseStr = "TAKEOFF"; break;
                case HOVER: phaseStr = "HOVER"; break;
                case MOVE_FORWARD: phaseStr = "MOVE_FORWARD"; break;
                case MOVE_BACKWARD: phaseStr = "MOVE_BACKWARD"; break;
                case MOVE_LEFT: phaseStr = "MOVE_LEFT"; break;
                case MOVE_RIGHT: phaseStr = "MOVE_RIGHT"; break;
                case LAND: phaseStr = "LAND"; break;
                case EMERGENCY_STOP: phaseStr = "EMERGENCY_STOP"; break;
            }
            
            // JSON 형식으로 데이터 생성
            DynamicJsonDocument doc(2048);  // Create JSON document
            if (timeSynced) {
                doc["timestamp"] = getCurrentTimestamp();
            } else {
                doc["timestamp"] = millis();
                Serial.println("Warning: Using system uptime as timestamp (NTP not synced)");
            }
            doc["flight_phase"] = phaseStr;
            doc["altitude"] = flightState.currentAltitude;
            
            // IMU 데이터
            JsonObject imu = doc.createNestedObject("imu");
            imu["accel_x"] = localData.imu.accel_x;
            imu["accel_y"] = localData.imu.accel_y;
            imu["accel_z"] = localData.imu.accel_z;
            imu["gyro_x"] = localData.imu.gyro_x;
            imu["gyro_y"] = localData.imu.gyro_y;
            imu["gyro_z"] = localData.imu.gyro_z;
            
            // 모터 출력값 추가
            JsonObject motors = doc.createNestedObject("motors");
            motors["m1"] = m1;
            motors["m2"] = m2;
            motors["m3"] = m3;
            motors["m4"] = m4;
            
            // JSON을 문자열로 변환
            String output;
            serializeJson(doc, output);
            
            // 시리얼로 출력
            Serial.println("Sending telemetry data:");
            Serial.println(output);
            
            // 서버로 전송
            sendTelemetryData(output);
        }
        
        // Check pitch for failsafe
        xSemaphoreTake(mutexAttitude, portMAX_DELAY);
        float pitch = attitude.pitch;
        xSemaphoreGive(mutexAttitude);

        if (abs(pitch) > 45.0) {
            Serial.println("[Failsafe] Pitch out of range!");
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup() {
  // Initialize serial
  Serial.begin(115200);
  delay(1000);
  
  // Initialize mutexes and queues
  mutexAttitude = xSemaphoreCreateMutex();
  mutexSensorData = xSemaphoreCreateMutex();
  mutexPIDOutput = xSemaphoreCreateMutex();
  mutexMotors = xSemaphoreCreateMutex();
  commandQueue = xQueueCreate(5, sizeof(CommandType));
  flightEvents = xEventGroupCreate();
  
  // Initialize PID output
  memset(&pidOutput, 0, sizeof(pidOutput));
  
  // Memory check
  if (ESP.getFreeHeap() < 2048) {
    Serial.print("Warning: Low heap memory: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
  }
  
  // WiFi 연결
  connectToWiFi();

  mutexAttitude = xSemaphoreCreateMutex();
  mutexSensorData = xSemaphoreCreateMutex();
  mutexPIDOutput = xSemaphoreCreateMutex();
  mutexMotors = xSemaphoreCreateMutex();
  commandQueue = xQueueCreate(5, sizeof(CommandType));
  flightEvents = xEventGroupCreate();

  // 메인 태스크 생성 (스택 크기 증가)
  #define MAIN_SENSOR_STACK   16384  // 16KB
  #define MAIN_STABILIZER_STACK 16384 // 16KB
  #define MAIN_MOTOR_STACK    12288  // 12KB
  #define MAIN_COMMAND_STACK   8192   // 8KB
  #define MAIN_TELEMETRY_STACK 8192   // 8KB
  #define MAIN_FAILSAFE_STACK  8192   // 8KB

  xTaskCreatePinnedToCore([](void*){ 
    Serial.println("MainSensMgr starting..."); 
    MainSensorManagerTask(nullptr); 
    vTaskDelete(NULL); 
  }, "MainSensMgr", MAIN_SENSOR_STACK, NULL, 1, NULL, 1);
  
  xTaskCreatePinnedToCore([](void*){ 
    Serial.println("MainStab starting..."); 
    MainStabilizerTask(nullptr); 
    vTaskDelete(NULL); 
  }, "MainStab", MAIN_STABILIZER_STACK, NULL, 1, NULL, 1);
  
  xTaskCreatePinnedToCore([](void*){ 
    Serial.println("MainMotor starting..."); 
    MainMotorOutputTask(nullptr); 
    vTaskDelete(NULL); 
  }, "MainMotor", MAIN_MOTOR_STACK, NULL, 1, NULL, 1);
  
  xTaskCreatePinnedToCore([](void*){ 
    Serial.println("MainCmd starting..."); 
    MainCommandTask(nullptr); 
    vTaskDelete(NULL); 
  }, "MainCmd", MAIN_COMMAND_STACK, NULL, 1, NULL, 0);
  
  xTaskCreatePinnedToCore([](void*){ 
    Serial.println("MainTlm starting..."); 
    MainTelemetryTask(nullptr); 
    vTaskDelete(NULL); 
  }, "MainTlm", MAIN_TELEMETRY_STACK, NULL, 1, NULL, 0);
  
  xTaskCreatePinnedToCore([](void*){ 
    Serial.println("MainFail starting..."); 
    MainFailsafeTask(nullptr); 
    vTaskDelete(NULL); 
  }, "MainFail", MAIN_FAILSAFE_STACK, NULL, 1, NULL, 0);
  
  // Initialize Task Watchdog Timer (TWDT)
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 5000,  // 5 second timeout
    .idle_core_mask = (1 << 0) | (1 << 1),  // Monitor both cores
    .trigger_panic = true
  };
  
  // Disable watchdog for now to prevent resets during debugging
  // We'll enable it after all tasks are running
  // esp_err_t ret = esp_task_wdt_init(&twdt_config);
  // if (ret != ESP_OK) {
  //   Serial.print("Failed to initialize TWDT: ");
  //   Serial.println(esp_err_to_name(ret));
  // } else {
  //   // Register main task with watchdog
  //   ret = esp_task_wdt_add(NULL);
  //   if (ret != ESP_OK) {
  //     Serial.print("Failed to add main task to TWDT: ");
  //     Serial.println(esp_err_to_name(ret));
  //   } else {
  //     Serial.println("Watchdog timer initialized successfully");
  //   }
  // }
  
  // 시리얼 모니터 시작
  Serial.println("\nSystem Initialized!");
  Serial.print("Sensor data will be printed every ");
  Serial.print(SAMPLE_COUNT_FOR_OUTPUT);
  Serial.println(" samples");
  
  // Print initial task info
  Serial.println("\nInitial Task Information:");
  monitorTasks();
}

void loop() {
    // Handle serial commands
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        if (command == "TAKEOFF") {
            CommandType cmd = CMD_TAKEOFF;
            if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) == pdPASS) {
                Serial.println("TAKEOFF command queued");
            } else {
                Serial.println("Command queue full");
            }
        }
        else if (command == "LAND") {
            CommandType cmd = CMD_LAND;
            if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) == pdPASS) {
                Serial.println("LAND command queued");
            } else {
                Serial.println("Command queue full");
            }
        }
        else if (command == "HOVER") {
            CommandType cmd = CMD_HOVER;
            if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) == pdPASS) {
                Serial.println("HOVER command queued");
            } else {
                Serial.println("Command queue full");
            }
        }
        else if (command == "STOP") {
            CommandType cmd = CMD_EMERGENCY_STOP;
            if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) == pdPASS) {
                Serial.println("EMERGENCY STOP command queued");
            } else {
                Serial.println("Command queue full");
            }
        }
        else if (command == "STATUS") {
            Serial.println("=== DRONE STATUS ===");
            Serial.print("State: ");
            switch(flightState.currentPhase) {
                case IDLE: Serial.println("IDLE"); break;
                case TAKEOFF: Serial.println("TAKEOFF"); break;
                case HOVER: Serial.println("HOVER"); break;
                case LAND: Serial.println("LAND"); break;
            }
            Serial.printf("Altitude: %.2fm\n", flightState.currentAltitude);
            Serial.printf("Target: %.2fm\n", flightState.targetAltitude);
            Serial.println("===================\n");
        }
        else {
            Serial.println("Unknown command. Available: TAKEOFF, HOVER, LAND, STOP, STATUS");
        }
    }
    
    // Monitor tasks every 10 seconds
    static unsigned long lastMonitor = 0;
    if (millis() - lastMonitor > 10000) {
        lastMonitor = millis();
        monitorTasks();
        
        // Print status every 10 seconds
        const char* phaseStr = "";
        switch(flightState.currentPhase) {
            case IDLE: phaseStr = "IDLE"; break;
            case TAKEOFF: phaseStr = "TAKEOFF"; break;
            case HOVER: phaseStr = "HOVER"; break;
            case LAND: phaseStr = "LAND"; break;
        }
        Serial.printf("[Status] %s | Alt: %.2fm | Target: %.2fm\n", 
                     phaseStr, 
                     flightState.currentAltitude,
                     flightState.targetAltitude);
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
}
