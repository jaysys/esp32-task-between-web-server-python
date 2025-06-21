#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_wifi.h>  // For WiFi power save functions
#include <esp_task_wdt.h>  // For watchdog timer
#include <esp_system.h>   // For reset reason

// Forward declarations
struct SensorData;
struct Attitude;
struct PIDOutput;
struct MotorOutput;

// Function declarations
void monitorTasks();
void printStackTrace();

// WiFi 설정
const char* ssid = "U+1234345";
const char* password = "1234$H1A5";

// HTTP 서버 URL
const char* serverURL = "http://192.168.123.111:5003/api/data";

// 서버 전송 주기 (ms)
const unsigned long SERVER_UPDATE_INTERVAL = 1000;
unsigned long lastServerUpdate = 0;

// ========== 공통 구조체 정의 ==========
struct Attitude {
  float pitch, roll, yaw;
};

struct MotorOutput {
  float m1, m2, m3, m4;
};

struct PIDOutput {
    float roll, pitch, yaw;
};

enum CommandType { TAKEOFF, HOVER, LAND };

// ========== 상수 정의 ==========
const int SAMPLE_COUNT_FOR_OUTPUT = 350;  // N번에 한 번씩 출력

// ========== 공유 변수/핸들 ==========
Attitude attitude;
MotorOutput motors;
SemaphoreHandle_t mutexAttitude;
QueueHandle_t commandQueue;
EventGroupHandle_t flightEvents;

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
    Serial.println(" bytes");
    
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




// ========== 1. Sensor Simulation Tasks ==========
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
SemaphoreHandle_t mutexSensorData;

void SubSensorManager_IMUTask(void *pv) {
  float t = 0;
  while (1) {
    IMUData imu;
    imu.accel_x = 0.1 * sin(t) + random(-100,100)/1000.0;
    imu.accel_y = 0.1 * cos(t) + random(-100,100)/1000.0;
    imu.accel_z = 9.8 + random(-50,50)/1000.0;
    imu.gyro_x = 0.01 * sin(t) + random(-10,10)/1000.0;
    imu.gyro_y = 0.01 * cos(t) + random(-10,10)/1000.0;
    imu.gyro_z = 0.01 * sin(2*t) + random(-10,10)/1000.0;
    xSemaphoreTake(mutexSensorData, portMAX_DELAY);
    sensorData.imu = imu;
    xSemaphoreGive(mutexSensorData);
    t += 0.05;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void SubSensorManager_BarometerTask(void *pv) {
  float baseAlt = 100.0;
  float t = 0;
  while (1) {
    BarometerData baro;
    baro.altitude = baseAlt + 2.0*sin(t) + random(-20,20)/100.0;
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
    t += 0.09;
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// 센서 데이터 처리 및 모니터링 태스크
void SubSensorManager_MonitorTask(void *pv) {
    static int sampleCount = 0;
    while (1) {
        SensorData localData;
        xSemaphoreTake(mutexSensorData, portMAX_DELAY);
        localData = sensorData;
        xSemaphoreGive(mutexSensorData);

        // SAMPLE_COUNT_FOR_OUTPUT에 지정된 횟수마다 센서 데이터를 JSON 형식으로 출력
        if (++sampleCount >= SAMPLE_COUNT_FOR_OUTPUT) {
            sampleCount = 0;
            
            // JSON 형식으로 센서 데이터 출력
            Serial.print("{\"sensor_data\":{");
            // IMU 데이터
            Serial.print("\"imu\":{");
            Serial.print("\"accel\":{\"x\":"); Serial.print(localData.imu.accel_x, 3); Serial.print(",");
            Serial.print("\"y\":"); Serial.print(localData.imu.accel_y, 3); Serial.print(",");
            Serial.print("\"z\":"); Serial.print(localData.imu.accel_z, 3);
            Serial.print("},\"gyro\":{\"x\":"); Serial.print(localData.imu.gyro_x, 3); Serial.print(",");
            Serial.print("\"y\":"); Serial.print(localData.imu.gyro_y, 3); Serial.print(",");
            Serial.print("\"z\":"); Serial.print(localData.imu.gyro_z, 3);
            // Baro 데이터
            Serial.print("}},\"baro\":{\"altitude\":"); Serial.print(localData.baro.altitude, 2);
            // GPS 데이터
            Serial.print("},\"gps\":{");
            Serial.print("\"lat\":"); Serial.print(localData.gps.lat, 5); Serial.print(",");
            Serial.print("\"lon\":"); Serial.print(localData.gps.lon, 5); Serial.print(",");
            Serial.print("\"alt\":"); Serial.print(localData.gps.alt, 1); Serial.print(",");
            Serial.print("\"speed\":"); Serial.print(localData.gps.speed, 1);
            // Magnetometer 데이터
            Serial.print("},\"magnetometer\":{");
            Serial.print("\"x\":"); Serial.print(localData.mag.mag_x, 3); Serial.print(",");
            Serial.print("\"y\":"); Serial.print(localData.mag.mag_y, 3); Serial.print(",");
            Serial.print("\"z\":"); Serial.print(localData.mag.mag_z, 3);
            // JSON 종료
            Serial.println("}}}");
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// 메인 센서 매니저 태스크 - 모든 센서 서브태스크 관리
void MainSensorManagerTask(void *pv) {
    // 센서 서브태스크 생성 (스택 크기 4096으로 증가)
    xTaskCreatePinnedToCore(SubSensorManager_IMUTask,          "SubSensorMgrIMU",         4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(SubSensorManager_BarometerTask,    "SubSensorMgrBarometer",   4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(SubSensorManager_GPSTask,          "SubSensorMgrGPS",         4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(SubSensorManager_MagnetometerTask, "SubSensorMgrMagnetometer",4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(SubSensorManager_MonitorTask,      "SubSensorMgrMonitor",     4096, NULL, 1, NULL, 1);

    // 메인 태스크는 이벤트 루프만 실행
    while (1) {
        // monitorTasks();
        // esp_task_wdt_reset();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// ========== 2. Stabilizer (PID) Task ==========
// PID 출력 변수 및 뮤텍스
PIDOutput pidOutput;
SemaphoreHandle_t mutexPIDOutput;
SemaphoreHandle_t mutexMotors;

// PID 제어 파라미터
const float kp = 1.2, ki = 0.01, kd = 0.3;

// 자세 추정 서브태스크
void SubStabilizer_AttitudeEstimatorTask(void *pv) {
    while (1) {
        SensorData localData;
        xSemaphoreTake(mutexSensorData, portMAX_DELAY);
        localData = sensorData;
        xSemaphoreGive(mutexSensorData);

        // 가속도계와 자이로스코프 데이터를 사용한 자세 추정
        float roll = atan2(localData.imu.accel_y, localData.imu.accel_z) * 180.0 / PI;
        float pitch = atan2(-localData.imu.accel_x, 
                          sqrt(localData.imu.accel_y * localData.imu.accel_y + 
                               localData.imu.accel_z * localData.imu.accel_z)) * 180.0 / PI;
        float yaw = atan2(localData.mag.mag_y, localData.mag.mag_x) * 180.0 / PI;

        xSemaphoreTake(mutexAttitude, portMAX_DELAY);
        attitude.roll = roll;
        attitude.pitch = pitch;
        attitude.yaw = yaw;
        xSemaphoreGive(mutexAttitude);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Roll 축 PID 제어 서브태스크
void SubStabilizer_PIDRollTask(void *pv) {
    float target = 0.0, integral = 0.0, prevError = 0.0;
    
    while (1) {
        xSemaphoreTake(mutexAttitude, portMAX_DELAY);
        float current = attitude.roll;
        xSemaphoreGive(mutexAttitude);

        float error = target - current;
        integral += error;
        float derivative = error - prevError;
        prevError = error;

        float output = kp * error + ki * integral + kd * derivative;

        xSemaphoreTake(mutexPIDOutput, portMAX_DELAY);
        pidOutput.roll = output;
        xSemaphoreGive(mutexPIDOutput);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// Pitch 축 PID 제어 서브태스크
void SubStabilizer_PIDPitchTask(void *pv) {
    float target = 0.0, integral = 0.0, prevError = 0.0;
    
    while (1) {
        xSemaphoreTake(mutexAttitude, portMAX_DELAY);
        float current = attitude.pitch;
        xSemaphoreGive(mutexAttitude);

        float error = target - current;
        integral += error;
        float derivative = error - prevError;
        prevError = error;

        float output = kp * error + ki * integral + kd * derivative;

        xSemaphoreTake(mutexPIDOutput, portMAX_DELAY);
        pidOutput.pitch = output;
        xSemaphoreGive(mutexPIDOutput);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// Yaw 축 PID 제어 서브태스크
void SubStabilizer_PIDYawTask(void *pv) {
    float target = 0.0, integral = 0.0, prevError = 0.0;
    
    while (1) {
        xSemaphoreTake(mutexAttitude, portMAX_DELAY);
        float current = attitude.yaw;
        xSemaphoreGive(mutexAttitude);

        float error = target - current;
        integral += error;
        float derivative = error - prevError;
        prevError = error;

        float output = kp * error + ki * integral + kd * derivative;

        xSemaphoreTake(mutexPIDOutput, portMAX_DELAY);
        pidOutput.yaw = output;
        xSemaphoreGive(mutexPIDOutput);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// PID 출력을 모터 신호로 변환하는 서브태스크
void SubStabilizer_PIDControllerTask(void *pv) {
    while (1) {
        PIDOutput localPID;
        xSemaphoreTake(mutexPIDOutput, portMAX_DELAY);
        localPID = pidOutput;
        xSemaphoreGive(mutexPIDOutput);

        // 모터 믹싱 (X자형 쿼드콥터)
        float m1 = 1300 + localPID.pitch + localPID.roll + localPID.yaw;
        float m2 = 1300 + localPID.pitch - localPID.roll - localPID.yaw;
        float m3 = 1300 - localPID.pitch + localPID.roll - localPID.yaw;
        float m4 = 1300 - localPID.pitch - localPID.roll + localPID.yaw;

        // 출력 제한 (1000~2000us PWM 범위)
        m1 = constrain(m1, 1000, 2000);
        m2 = constrain(m2, 1000, 2000);
        m3 = constrain(m3, 1000, 2000);
        m4 = constrain(m4, 1000, 2000);

        xSemaphoreTake(mutexMotors, portMAX_DELAY);
        motors.m1 = m1;
        motors.m2 = m2;
        motors.m3 = m3;
        motors.m4 = m4;
        xSemaphoreGive(mutexMotors);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// 메인 Stabilizer Task - 서브태스크들을 관리
void MainStabilizerTask(void *pv) {
    // 서브태스크 생성 (스택 크기 4096으로 증가)
    xTaskCreatePinnedToCore(SubStabilizer_AttitudeEstimatorTask, "StabAttEst", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(SubStabilizer_PIDRollTask, "StabPIDRoll", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(SubStabilizer_PIDPitchTask, "StabPIDPitch", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(SubStabilizer_PIDYawTask, "StabPIDYaw", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(SubStabilizer_PIDControllerTask, "StabPIDCtrl", 4096, NULL, 1, NULL, 1);

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 메인 태스크는 관리만
    }
}

// ========== WiFi 및 서버 통신 함수 ==========
void connectToWiFi() {
  // Disable WiFi sleep mode for better performance
  WiFi.setSleep(false);
  
  // Configure WiFi mode to STA (Station)
  WiFi.mode(WIFI_STA);
  
  // Configure WiFi settings
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  
  // Disable WiFi power save for better performance
  WiFi.setSleep(WIFI_PS_NONE);
  
  while (true) {
    Serial.println("\nWiFi 연결 시작...");
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
      
      // Print WiFi status for debugging
      if (attempts % 10 == 0) {
        Serial.println();
        Serial.print("WiFi 상태: ");
        Serial.println(WiFi.status());
        Serial.print("시도 횟수: ");
        Serial.println(attempts);
      }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi 연결 성공!");
      Serial.print("SSID: ");
      Serial.println(WiFi.SSID());
      Serial.print("IP 주소: ");
      Serial.println(WiFi.localIP());
      Serial.print("신호 강도: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
      break; // 연결 성공 시 반복문 탈출
    } else {
      Serial.println("\nWiFi 연결 실패! 5초 후 재시도...");
      WiFi.disconnect(true);
      delay(100);
      WiFi.mode(WIFI_OFF);
      delay(100);
      WiFi.mode(WIFI_STA);
      delay(5000);
    }
  }
}

void sendDataToServer(const SensorData& sensorData, const Attitude& currentAttitude, 
                     const PIDOutput& currentPID, const MotorOutput& currentMotors) {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
    return;
  }

  HTTPClient http;
  http.begin(serverURL);
  http.addHeader("Content-Type", "application/json");

  // JSON 문서 생성
  DynamicJsonDocument doc(1024);
  
  // 센서 데이터 추가
  JsonObject sensor = doc.createNestedObject("sensor");
  JsonObject imu = sensor.createNestedObject("imu");
  imu["accel"] = JsonObject();
  imu["accel"]["x"] = sensorData.imu.accel_x;
  imu["accel"]["y"] = sensorData.imu.accel_y;
  imu["accel"]["z"] = sensorData.imu.accel_z;
  imu["gyro"] = JsonObject();
  imu["gyro"]["x"] = sensorData.imu.gyro_x;
  imu["gyro"]["y"] = sensorData.imu.gyro_y;
  imu["gyro"]["z"] = sensorData.imu.gyro_z;
  
  JsonObject baro = sensor.createNestedObject("baro");
  baro["altitude"] = sensorData.baro.altitude;
  
  JsonObject gps = sensor.createNestedObject("gps");
  gps["lat"] = sensorData.gps.lat;
  gps["lon"] = sensorData.gps.lon;
  gps["alt"] = sensorData.gps.alt;
  gps["speed"] = sensorData.gps.speed;
  
  JsonObject mag = sensor.createNestedObject("magnetometer");
  mag["x"] = sensorData.mag.mag_x;
  mag["y"] = sensorData.mag.mag_y;
  mag["z"] = sensorData.mag.mag_z;

  // 안정화 데이터 추가
  JsonObject stabilizer = doc.createNestedObject("stabilizer");
  
  JsonObject attitude = stabilizer.createNestedObject("attitude");
  attitude["pitch"] = currentAttitude.pitch;
  attitude["roll"] = currentAttitude.roll;
  attitude["yaw"] = currentAttitude.yaw;
  
  JsonObject pid = stabilizer.createNestedObject("pid");
  pid["pitch"] = currentPID.pitch;
  pid["roll"] = currentPID.roll;
  pid["yaw"] = currentPID.yaw;
  
  JsonObject motors = stabilizer.createNestedObject("motors");
  motors["m1"] = currentMotors.m1;
  motors["m2"] = currentMotors.m2;
  motors["m3"] = currentMotors.m3;
  motors["m4"] = currentMotors.m4;

  // JSON 직렬화
  String jsonString;
  serializeJson(doc, jsonString);
  
  // 서버로 전송
  int httpResponseCode = http.POST(jsonString);
  
  if (httpResponseCode > 0) {
    Serial.printf("Data sent to server. Response code: %d\n", httpResponseCode);
  } else {
    Serial.printf("Error sending data to server. Error: %s\n", http.errorToString(httpResponseCode).c_str());
  }
  
  http.end();
}

// ========== 3. Motor Control Task ==========
void MainMotorOutputTask(void *pv) {
  static int printCount = 0;
  while (1) {
    // SAMPLE_COUNT_FOR_OUTPUT에 지정된 횟수마다 모터 출력 및 안정화 상태 출력
    if (++printCount >= SAMPLE_COUNT_FOR_OUTPUT) {
      printCount = 0;
      
      // 현재 자세 정보 가져오기
      Attitude currentAttitude;
      xSemaphoreTake(mutexAttitude, portMAX_DELAY);
      currentAttitude = attitude;
      xSemaphoreGive(mutexAttitude);
      
      // PID 출력 가져오기
      PIDOutput currentPID;
      xSemaphoreTake(mutexPIDOutput, portMAX_DELAY);
      currentPID = pidOutput;
      xSemaphoreGive(mutexPIDOutput);
      
      // 모터 출력 가져오기
      MotorOutput currentMotors;
      xSemaphoreTake(mutexMotors, portMAX_DELAY);
      currentMotors = motors;
      xSemaphoreGive(mutexMotors);
      
      // 안정화 상태를 JSON 형식으로 출력
      Serial.print("{\"stabilizer_status\":{");
      // Attitude 출력
      Serial.print("\"attitude\":{");
      Serial.print("\"pitch\":"); Serial.print(currentAttitude.pitch, 2); Serial.print(",");
      Serial.print("\"roll\":");  Serial.print(currentAttitude.roll, 2);  Serial.print(",");
      Serial.print("\"yaw\":");   Serial.print(currentAttitude.yaw, 2);
      // PID 출력
      Serial.print("},\"pid_output\":{");
      Serial.print("\"pitch\":"); Serial.print(currentPID.pitch, 2); Serial.print(",");
      Serial.print("\"roll\":");  Serial.print(currentPID.roll, 2);  Serial.print(",");
      Serial.print("\"yaw\":");   Serial.print(currentPID.yaw, 2);
      // 모터 출력
      Serial.print("},\"motors\":{");
      Serial.print("\"m1\":"); Serial.print((int)currentMotors.m1); Serial.print(",");
      Serial.print("\"m2\":"); Serial.print((int)currentMotors.m2); Serial.print(",");
      Serial.print("\"m3\":"); Serial.print((int)currentMotors.m3); Serial.print(",");
      Serial.print("\"m4\":"); Serial.print((int)currentMotors.m4);
      // JSON 종료
      Serial.println("}}}");
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ========== 4. Command Task ==========
void MainCommandTask(void *pv) {
  Serial.println("MainCommandTask started");
  static unsigned long lastUpdateTime = 0;
  
  while (1) {  // Add infinite loop to keep task running
    unsigned long currentTime = millis();
    
    // 주기적으로 서버에 데이터 전송
    if (currentTime - lastUpdateTime >= SERVER_UPDATE_INTERVAL) {
      lastUpdateTime = currentTime;
      
      // 공유 변수에서 데이터 가져오기
      SensorData localSensorData;
      Attitude currentAttitude;
      PIDOutput currentPID;
      MotorOutput currentMotors;
      
      xSemaphoreTake(mutexSensorData, portMAX_DELAY);
      localSensorData = sensorData;
      xSemaphoreGive(mutexSensorData);
      
      xSemaphoreTake(mutexAttitude, portMAX_DELAY);
      currentAttitude = attitude;
      xSemaphoreGive(mutexAttitude);
      
      xSemaphoreTake(mutexPIDOutput, portMAX_DELAY);
      currentPID = pidOutput;
      xSemaphoreGive(mutexPIDOutput);
      
      xSemaphoreTake(mutexMotors, portMAX_DELAY);
      currentMotors = motors;
      xSemaphoreGive(mutexMotors);
      
      // 서버에 데이터 전송
      sendDataToServer(localSensorData, currentAttitude, currentPID, currentMotors);
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  
  // This code will never be reached due to the infinite loop
  vTaskDelete(NULL);
}


// ========== 5. Telemetry Task ==========
void MainTelemetryTask(void *pv) {
  while (1) {
    xSemaphoreTake(mutexAttitude, portMAX_DELAY);
    xSemaphoreGive(mutexAttitude);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// ========== 6. Failsafe Task ==========
void MainFailsafeTask(void *pv) {
  while (1) {
    // 데모: Attitude 값이 너무 크면 비정상
    xSemaphoreTake(mutexAttitude, portMAX_DELAY);
    float pitch = attitude.pitch;
    xSemaphoreGive(mutexAttitude);

    if (abs(pitch) > 45.0) {
      Serial.println("[Failsafe] Pitch out of range!");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // Initialize serial
  Serial.begin(115200);
  delay(1000);
  
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
  // Monitor tasks periodically
  static unsigned long lastMonitor = 0;
  if (millis() - lastMonitor > 10000) {  // Every 10 seconds
    lastMonitor = millis();
    monitorTasks();
  }
  
  // Small delay to prevent watchdog trigger
  vTaskDelay(pdMS_TO_TICKS(1000));
}
