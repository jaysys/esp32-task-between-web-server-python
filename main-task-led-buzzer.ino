
// ESP32 FreeRTOS 태스크 간 메시지 통신 + HTTP 전송 예제
// TaskYa와 TaskYe가 큐를 통해 메시지를 주고받고,
// TaskYo가 WiFi를 통해 JSON 데이터를 HTTP POST로 전송합니다

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define LED_YA 4
#define LED_YO 23
#define PIN_BUZZER 22  // 피에조 부저 연결 핀 (필요시 변경)

// WiFi 설정 (본인의 WiFi 정보로 변경하세요)
const char* ssid = "U5555";
const char* password = "5555";

// HTTP 서버 URL
const char* serverURL = "http://192.168.123.111:5003/api/data";

// 큐 핸들 선언
QueueHandle_t queueYaToYe;
QueueHandle_t queueYeToYa;

// TaskYo에서 생성한 마지막 랜덤값 (TaskMa에서 사용)
volatile int lastRandomNumber = 0;

// 메시지 구조체 정의
struct Message {
  int value;
  char text[50];
};

// TaskYa: 메시지를 보내고 받는 태스크 (코어 1에서 실행)
void TaskYa(void *parameter) {
  Message sendMsg, receiveMsg;
  int counter = 0;
  
  Serial.printf("TaskYa 시작! (코어 %d에서 실행)\n", xPortGetCoreID());
  
  while (true) {
    // 메시지 준비
    sendMsg.value = counter++;
    sprintf(sendMsg.text, "TaskYa에서 보낸 메시지 #%d", sendMsg.value);
    
    // TaskYe에게 메시지 전송
    if (xQueueSend(queueYaToYe, &sendMsg, portMAX_DELAY) == pdTRUE) {
      Serial.printf("[TaskYa-Core%d] 전송: %s\n", xPortGetCoreID(), sendMsg.text);
    }
    
    // TaskYe로부터 메시지 수신 대기
    if (xQueueReceive(queueYeToYa, &receiveMsg, pdMS_TO_TICKS(5000)) == pdTRUE) {
      Serial.printf("[TaskYa-Core%d] 수신: %s (값: %d)\n", 
                    xPortGetCoreID(), receiveMsg.text, receiveMsg.value);
      
      // TaskYa 데이터 송수신 시 4번 핀 LED ON/OFF
      digitalWrite(LED_YA, HIGH);
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(LED_YA, LOW);
    } else {
      Serial.printf("[TaskYa-Core%d] 메시지 수신 타임아웃\n", xPortGetCoreID());
    }
    
    // 2초 대기
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// TaskYe: 메시지를 받고 응답하는 태스크 (코어 0에서 실행)
void TaskYe(void *parameter) {
  Message receiveMsg, replyMsg;
  int replyCounter = 100;
  
  Serial.printf("TaskYe 시작! (코어 %d에서 실행)\n", xPortGetCoreID());
  
  while (true) {
    // TaskYa로부터 메시지 수신 대기
    if (xQueueReceive(queueYaToYe, &receiveMsg, portMAX_DELAY) == pdTRUE) {
      Serial.printf("[TaskYe-Core%d] 수신: %s (값: %d)\n", 
                    xPortGetCoreID(), receiveMsg.text, receiveMsg.value);
      
      // 응답 메시지 준비
      replyMsg.value = replyCounter++;
      sprintf(replyMsg.text, "TaskYe 응답: 받은값=%d, 응답값=%d", 
              receiveMsg.value, replyMsg.value);
      
      // 1초 후 응답 (처리 시간 시뮬레이션)
      vTaskDelay(pdMS_TO_TICKS(1000));
      
      // TaskYa에게 응답 전송
      if (xQueueSend(queueYeToYa, &replyMsg, portMAX_DELAY) == pdTRUE) {
        Serial.printf("[TaskYe-Core%d] 응답 전송: %s\n", 
                      xPortGetCoreID(), replyMsg.text);
      }
    }
  }
}

// TaskYo: HTTP JSON 데이터 전송 태스크 (코어 0에서 실행)
void TaskYo(void *parameter) {
  Serial.printf("TaskYo 시작! (코어 %d에서 실행)\n", xPortGetCoreID());
  
  // WiFi 연결 대기
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("[TaskYo] WiFi 연결 대기 중...");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
  Serial.printf("[TaskYo-Core%d] WiFi 연결됨. HTTP 전송 시작\n", xPortGetCoreID());
  
  HTTPClient http;
  
  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      // JSON 데이터 생성
      StaticJsonDocument<200> jsonDoc;
      jsonDoc["timestamp"] = millis();
      int randNum = random(1, 100);
      jsonDoc["random_number"] = randNum;
      lastRandomNumber = randNum;
      Serial.printf("[TaskYo] 생성 random_number: %d (lastRandomNumber=%d)\n", randNum, lastRandomNumber);
      jsonDoc["device_id"] = "ESP32_TaskYo";
      jsonDoc["core_id"] = xPortGetCoreID();
      
      String jsonString;
      serializeJson(jsonDoc, jsonString);
      
      // TaskYo 데이터 전송 시 23번 핀 LED ON
      digitalWrite(LED_YO, HIGH);

      // HTTP POST 요청
      http.begin(serverURL);
      http.addHeader("Content-Type", "application/json");
      
      int httpResponseCode = http.POST(jsonString);
      
      if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.printf("[TaskYo-Core%d] HTTP 전송 성공! JSON: %s\n", 
                      xPortGetCoreID(), jsonString.c_str());
        if (response.length() > 0) {
          Serial.printf("[TaskYo-Core%d] 서버 응답: %s\n", 
                        xPortGetCoreID(), response.c_str());
        }
      } else {
        Serial.printf("[TaskYo-Core%d] HTTP 전송 실패 (에러: %d)\n", 
                      xPortGetCoreID(), httpResponseCode);
      }
      
      http.end();

      // 데이터 전송 후 23번 핀 LED OFF (0.1초 유지)
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(LED_YO, LOW);
    } else {
      Serial.printf("[TaskYo-Core%d] WiFi 연결 끊어짐. 재연결 시도...\n", xPortGetCoreID());
    }
    
    // 3초 대기
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

// WiFi 연결 함수
void connectWiFi() {
  while (true) {
    Serial.println("WiFi 연결 시작...");
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.println("WiFi 연결 성공!");
      Serial.print("IP 주소: ");
      Serial.println(WiFi.localIP());
      break; // 연결 성공 시 반복문 탈출
    } else {
      Serial.println();
      Serial.println("WiFi 연결 실패! 5초 후 재시도...");
      delay(5000);
    }
  }
}

// 시스템 모니터링 태스크 (선택사항)
void TaskMonitor(void *parameter) {
  while (true) {
    // 10초마다 시스템 정보 출력
    vTaskDelay(pdMS_TO_TICKS(10000));
    
    Serial.println("========== 시스템 상태 ==========");
    Serial.printf("자유 힙 메모리: %d bytes\n", esp_get_free_heap_size());
    Serial.printf("최소 자유 힙: %d bytes\n", esp_get_minimum_free_heap_size());
    Serial.printf("WiFi 상태: %s\n", WiFi.status() == WL_CONNECTED ? "연결됨" : "연결 안됨");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("WiFi IP: %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("WiFi RSSI: %d dBm\n", WiFi.RSSI());
    }
    Serial.printf("TaskYa->TaskYe 큐 대기: %d\n", uxQueueMessagesWaiting(queueYaToYe));
    Serial.printf("TaskYe->TaskYa 큐 대기: %d\n", uxQueueMessagesWaiting(queueYeToYa));
    Serial.printf("TaskYo 마지막 random_number: %d\n", lastRandomNumber);
    Serial.printf("TaskMa 도레미 연주 대기 상태: %s\n", lastRandomNumber > 90 ? "연주 대기중" : "정상");
    Serial.println("==============================");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_YA, OUTPUT);
  pinMode(LED_YO, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  
  Serial.println("ESP32 듀얼 코어 FreeRTOS + HTTP 통신 예제");
  Serial.println("TaskYa -> 코어 1, TaskYe -> 코어 0, TaskYo -> 코어 0");
  Serial.println("===============================================");
  
  // WiFi 연결
  connectWiFi();
  
  // 큐 생성 (각각 5개의 메시지를 저장할 수 있음)
  queueYaToYe = xQueueCreate(5, sizeof(Message));
  queueYeToYa = xQueueCreate(5, sizeof(Message));
  
  if (queueYaToYe == NULL || queueYeToYa == NULL) {
    Serial.println("큐 생성 실패!");
    return;
  }
  
  Serial.println("큐 생성 완료");
  
  // TaskYa를 코어 1에 생성
  xTaskCreatePinnedToCore(
    TaskYa,           // 태스크 함수
    "TaskYa",         // 태스크 이름
    3072,             // 스택 크기 증가 (WiFi 사용)
    NULL,             // 매개변수
    2,                // 우선순위
    NULL,             // 태스크 핸들
    1                 // 코어 1에 할당
  );
  
  // TaskYe를 코어 0에 생성
  xTaskCreatePinnedToCore(
    TaskYe,           // 태스크 함수
    "TaskYe",         // 태스크 이름
    3072,             // 스택 크기 증가
    NULL,             // 매개변수
    2,                // 우선순위
    NULL,             // 태스크 핸들
    0                 // 코어 0에 할당
  );
  
  // TaskYo를 코어 0에 생성 (HTTP 전송)
  xTaskCreatePinnedToCore(
    TaskYo,           // 태스크 함수
    "TaskYo",         // 태스크 이름
    4096,             // HTTP 클라이언트용 큰 스택
    NULL,             // 매개변수
    2,                // 우선순위
    NULL,             // 태스크 핸들
    0                 // 코어 0에 할당
  );
  
  // 모니터링 태스크를 코어 0에 생성 (선택사항)
  xTaskCreatePinnedToCore(
    TaskMonitor,      // 태스크 함수
    "Monitor",        // 태스크 이름
    2048,             // 스택 크기
    NULL,             // 매개변수
    1,                // 낮은 우선순위
    NULL,             // 태스크 핸들
    0                 // 코어 0에 할당
  );
  
  Serial.println("태스크 생성 완료:");
  Serial.println("- TaskYa: 코어 1에서 실행 (태스크 간 통신)");
  Serial.println("- TaskYe: 코어 0에서 실행 (태스크 간 통신)");
  Serial.println("- TaskYo: 코어 0에서 실행 (HTTP JSON 전송)");
  Serial.println("- Monitor: 코어 0에서 실행 (시스템 모니터링)");
  Serial.println("모든 태스크 시작...");

  // TaskMa 태스크 생성 (코어 0)
  xTaskCreatePinnedToCore(
    TaskMa,           // 태스크 함수
    "TaskMa",         // 태스크 이름
    2048,             // 스택 크기
    NULL,             // 매개변수
    1,                // 우선순위
    NULL,             // 태스크 핸들
    0                 // 코어 0에 할당
  );
}


// TaskMa: TaskYo의 랜덤값이 60 초과 시 도레미 연주
void TaskMa(void *parameter) {
  while (true) {
    if (lastRandomNumber > 90) {
      Serial.printf("[TaskMa] random_number %d > 90, 띵동 연주!\n", lastRandomNumber);
      // 띵: 784Hz(G5), 동: 523Hz(C5)
      tone(PIN_BUZZER, 784, 200); // 띵
      vTaskDelay(pdMS_TO_TICKS(220));
      tone(PIN_BUZZER, 523, 400); // 동
      vTaskDelay(pdMS_TO_TICKS(420));
      noTone(PIN_BUZZER);
      lastRandomNumber = 0; // 연주 후 중복 방지
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // 0.1초마다 체크
  }
}

void loop() {
  // FreeRTOS에서는 loop() 함수가 비어있어도 됩니다
  // 모든 작업이 태스크에서 처리됩니다
  vTaskDelay(pdMS_TO_TICKS(1000));
}