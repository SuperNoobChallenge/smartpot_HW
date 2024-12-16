#include <WiFi.h>
#include "BluetoothSerial.h"
#include <Preferences.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include <esp_adc_cal.h>
#include <time.h>         // 시간 함수 사용
#include "DHT.h"          // DHT-22 센서 라이브러리

// wifi 리셋 버튼 핀 = 13
// 온습도 센서 파워 핀 = 33
// 온습도 센서 데이터 핀 = 32
// 토양 센서 데이터 핀 = 35
// 배터리 센서 데이터 핀 = 34
// 부저 핀 2
int buzzerPin = 2;

// 블루투스 및 Wi-Fi 설정
BluetoothSerial SerialBT;
Preferences preferences;
bool isConnected = false;
bool configMode = false;
bool bluetoothStarted = false;
int lastButtonState = HIGH; // 버튼이 눌리지 않은 상태
const int buttonPin = 13;   // GPIO12

// Firestore REST API 정보 
// 파이어베이스 정보 입력 필요
const String projectID = "";
const String apiKey = "";

// 간격 설정
const int intervalMinutes = 1; // 원하는 간격으로 설정 (예: 10분)

// 센서 전원 제어 핀 설정
const int sensorPowerPin = 33; // GPIO25 (사용 가능한 핀으로 설정)
// [0] 토양 센서, [1] 온습도 센서

// Firestore URL 설정
String firestoreBaseURL = "";
bool timeSynced = false; // 시간 동기화 상태

// DHT22 센서 설정
#define DHTPIN 32     // DHT22의 데이터 핀 (GPIO13)
#define DHTTYPE DHT22 // DHT 센서 종류
DHT dht(DHTPIN, DHTTYPE);

// 토양 습도 센서 설정
const int analogPin = 35;         // 아날로그 입력 핀 (GPIO 34)
const float voltageReference = 3.3; // ESP32의 참조 전압
const int moistureThreshold = 2000; // 촉촉한 토양의 임계값 (필요에 따라 조정)

// 추가 변수
const int AirValue = 520;   // 건조한 토양의 ADC 값 (필요에 따라 변경)
const int WaterValue = 260; // 촉촉한 토양의 ADC 값 (필요에 따라 변경)
int intervals = (AirValue - WaterValue) / 3;
int soilMoistureValue = 0;

// 평균 저장을 위한 추가 변수
float t; // Temperature
float h; // Humidity
float hic; // Heat index
int adcValue; // Soil moisture ADC value
float voltage; // Soil voltage

// 배터리 전압 측정 관련 변수
const int batteryAnalogPin = 34;    // 배터리 전압 측정을 위한 ADC 핀 (GPIO 35)
const float R1 = 10000.0;           // 저항 분압기 R1 (10kΩ)
const float R2 = 10000.0;           // 저항 분압기 R2 (10kΩ)
const float adcMaxValue = 4095.0;   // ESP32 ADC 최대 값 (12비트 해상도)

float previousVoltage = 0;          // 이전 전압값 저장
float batteryPercentage = 0.0;      // 배터리 잔량 저장
float hysteresisThreshold = 0.01;   // 히스테리시스 한계 (0.01V)
bool isFirstRead = true;            // 처음 측정 여부를 위한 플래그

int preVoltRead = 40;                // 미리 읽어서 안정화하는 횟수

// ADC 보정 구조체
esp_adc_cal_characteristics_t adc_chars;

// 이동 평균 계산을 위한 변수
const int numReadings = 10;
float readings[numReadings];
int readIndex = 0;
float total = 0;
float averageVoltage = 0;

// 그 날의 평균을 저장하기 위한 변수
float sumTemperature = 0.0;
float sumHumidity = 0.0;
int sumMoistureADC = 0;
int countReadings = 0;
String storedDate = "";

// configMode 실행 시간 저장하는 변수
unsigned long configModeStartTime = 0;

// 함수 선언 추가
void sendData();
uint64_t calculateSleepDuration();
void startWiFiScan();
void handleWiFiCredentials(String incoming);
void connectToWiFi(const char *ssid, const char *password);
void setupTime();
String getSanitizedMacAddress();
String getCurrentTimeAsString();
String getCurrentDateAsString(); // 현재 날짜를 반환하는 함수 선언
float readBatteryVoltage(); // 배터리 전압 측정 함수 선언

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started.");

  // 버튼 핀 초기화
  pinMode(buttonPin, INPUT_PULLUP); // 버튼이 GND에 연결된 것으로 가정
  int Wifibutton = digitalRead(buttonPin);
  if(Wifibutton == LOW){
    Serial.println("wifi 설정 버튼이 눌림");
    configMode = true;
  }
  // 센서 전원 제어 핀 초기화
  pinMode(sensorPowerPin, OUTPUT);
  digitalWrite(sensorPowerPin, LOW); // 초기에는 센서 전원 OFF


  // DHT22 센서 초기화
  dht.begin();

  // 배터리 전압 측정을 위한 ADC 설정
  analogSetPinAttenuation(batteryAnalogPin, ADC_11db);  // 최대 3.3V까지 측정 가능하도록 설정
  analogSetWidth(12); // ADC 해상도 설정
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars); // ADC 보정 설정
  delay(1000); // ADC 및 회로 안정화를 위해 1초 지연

  // 이동 평균 배열 초기화
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }


  // 깨어난 원인 확인
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("버튼 눌림으로 인해 깨어남 (EXT0)");
    configMode = true;
  }



  // Preferences 초기화
  preferences.begin("wifi", false);

  // 저장된 누적값과 카운트 불러오기
  sumTemperature = preferences.getFloat("sumTemp", 0.0);
  sumHumidity = preferences.getFloat("sumHum", 0.0);
  sumMoistureADC = preferences.getInt("sumMoisture", 0);
  countReadings = preferences.getInt("countReadings", 0);
  storedDate = preferences.getString("storedDate", "");

  // Wi-Fi 초기화
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(1000);

  // 저장된 Wi-Fi 자격 증명으로 연결 시도
  String storedSSID = preferences.getString("ssid", "");
  String storedPassword = preferences.getString("password", "");


  delay(1000);
  if (storedSSID != "") {
    Serial.println("저장된 Wi-Fi 자격 증명이 발견되었습니다. 연결을 시도합니다...");
    connectToWiFi(storedSSID.c_str(), storedPassword.c_str());
  } else {
    Serial.println("저장된 Wi-Fi 자격 증명이 없습니다.");
    configMode = true;
  }

  // Firestore 기본 URL 설정
  String sanitizedMacAddress = getSanitizedMacAddress();
  firestoreBaseURL = "https://firestore.googleapis.com/v1/projects/" + projectID +
                     "/databases/(default)/documents/testCollection/data/" + sanitizedMacAddress;
  Serial.println("정리된 MAC 주소: " + sanitizedMacAddress);
  Serial.println("Firestore 기본 URL: " + firestoreBaseURL);

  // Wi-Fi에 연결되어 있고 구성 모드가 아닌 경우 시간 설정
  if (WiFi.status() == WL_CONNECTED && !configMode) {
    setupTime();
  }

  // configMode가 true인 경우 블루투스 시작
  if (configMode) {
    // 블루투스가 아직 시작되지 않은 경우 시작
    if (!bluetoothStarted) {
      // Wi-Fi 초기화
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();
      
      // "삐------" 소리: 긴 소리
      tone(buzzerPin, 400); // 400Hz 톤
      delay(1000);          // 1초 동안 소리
      noTone(buzzerPin);
      delay(500);           // 0.5초 정지
      configModeStartTime = millis();

      SerialBT.begin("ESP32_BT"); // 블루투스 시작
      bluetoothStarted = true;
      Serial.println("블루투스 시작됨! 페어링 준비 완료.");

      delay(100);
    }
  }
}

void loop() {
  // 버튼 상태 읽기
  int buttonState = digitalRead(buttonPin);
  if (!configMode && lastButtonState == HIGH && buttonState == LOW) {
          // "삐------" 소리: 긴 소리
      // "삐------" 소리: 긴 소리
      tone(buzzerPin, 400); // 400Hz 톤
      delay(1000);          // 1초 동안 소리
      noTone(buzzerPin);
      configModeStartTime = millis();

    // 버튼이 방금 눌림
    Serial.println("버튼이 눌렸습니다.");
    if (!configMode) {
      Serial.println("구성 모드로 진입합니다.");
      configMode = true;

      // Wi-Fi 스택 재설정
      WiFi.disconnect(true); // Wi-Fi 연결 해제 및 스택 재설정
      WiFi.mode(WIFI_STA);   // STA 모드로 설정
      delay(100);

      // 블루투스가 아직 시작되지 않은 경우 시작
      if (!bluetoothStarted) {
        SerialBT.begin("ESP32_BT"); // 블루투스 시작
        bluetoothStarted = true;
        Serial.println("블루투스 시작됨! 페어링 준비 완료.");
      }
    }
  }
  lastButtonState = buttonState;

  if (configMode) {
    if (SerialBT.hasClient() && !isConnected) {
      Serial.println("클라이언트가 연결되었습니다!");
      isConnected = true;
    } else if (!SerialBT.hasClient() && isConnected) {
      Serial.println("클라이언트가 연결이 끊어졌습니다.");
      isConnected = false;
    }

      // 5분(300,000 밀리초)이 경과했는지 확인 // 배터리 빼려고 임시 비활성화
    if ((millis() - configModeStartTime) > 300000) { //300000
      Serial.println("구성 모드 시간 초과. 구성 모드를 종료합니다.");
      configMode = false;

      // 블루투스가 시작된 경우 종료
      if (bluetoothStarted) {
        SerialBT.end();
        bluetoothStarted = false;
        Serial.println("블루투스가 종료되었습니다.");
      }

      // 필요한 경우 Wi-Fi를 끄거나 다른 설정 초기화
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      Serial.println("Wi-Fi 연결이 해제되었습니다.");

      // 부저로 알림음 재생 (원하시는 경우)
      // "삐빅" 소리: 짧은 소리 두 번
      tone(buzzerPin, 800); // 첫 번째 짧은 소리
      delay(100);           // 0.2초 동안 소리
      noTone(buzzerPin);
      delay(50);           // 0.1초 정지

      tone(buzzerPin, 800); // 두 번째 짧은 소리
      delay(100);           // 0.1초 동안 소리
      noTone(buzzerPin);
      delay(50); 

      tone(buzzerPin, 800); // 두 번째 짧은 소리
      delay(100);           // 0.1초 동안 소리
      noTone(buzzerPin);
      delay(50); 
    }

    // 안드로이드로부터 데이터 수신
    if (SerialBT.available()) {
      Serial.print(".uiuiiu");
      String incoming = SerialBT.readStringUntil('\n');
      incoming.trim();
      Serial.print("수신됨: ");
      Serial.println(incoming);

      if (incoming.startsWith("START_WIFI_SCAN")) {
        // Wi-Fi 스캔 시작
        startWiFiScan();
      } else if (incoming.startsWith("WIFI_CREDENTIALS")) {
        // Wi-Fi 자격 증명 수신 및 연결 시도
        handleWiFiCredentials(incoming);
        // 자격 증명 처리 후 구성 모드 종료 가능
        configMode = false;
        // 블루투스 통신 종료
        SerialBT.end();
        bluetoothStarted = false;
        Serial.println("구성 모드를 종료합니다.");
        
          // "삐빅" 소리: 짧은 소리 두 번
        tone(buzzerPin, 800); // 첫 번째 짧은 소리
        delay(200);           // 0.2초 동안 소리
        noTone(buzzerPin);
        delay(100);           // 0.1초 정지

        tone(buzzerPin, 800); // 두 번째 짧은 소리
        delay(100);           // 0.2초 동안 소리
        noTone(buzzerPin);

        // Wi-Fi에 연결된 경우 시간 설정
        if (WiFi.status() == WL_CONNECTED) {
          setupTime();
        }
      }
      // 필요한 경우 추가 로직 추가
    }
  } else {
    // 구성 모드가 아니고 Wi-Fi에 연결된 경우 데이터 전송 및 Deep Sleep 모드로 진입
    if (WiFi.status() == WL_CONNECTED) {
      // 데이터 전송
      sendData();

      // 센서 전원 OFF (안전 차원에서)
      digitalWrite(sensorPowerPin, LOW);

      // Wi-Fi 연결 해제
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);

      // 다음 예정된 전송 시간까지의 시간 계산
      uint64_t sleepDuration = calculateSleepDuration();
      Serial.println("다음 " + String(sleepDuration / 1000000) +
                     "초 동안 슬립 모드에 들어갑니다. 버튼을 누르면 깨어납니다...");

      // 웨이크업 소스 설정
      // 타이머 웨이크업 활성화
      esp_sleep_enable_timer_wakeup(sleepDuration); // Deep Sleep 타이머
      // 버튼 핀(GPIO14)에서 ext0 웨이크업 활성화, 버튼이 눌리면(LOW) 깨어남
      esp_sleep_enable_ext0_wakeup((gpio_num_t)buttonPin, 0); // 0 = LOW

      esp_deep_sleep_start(); // Deep Sleep 모드 진입
    } else {
      // Wi-Fi에 연결되지 않음
      Serial.println("Wi-Fi에 연결되지 않았습니다. 재연결을 시도합니다...");
      // 저장된 자격 증명으로 연결 시도
      String storedSSID = preferences.getString("ssid", "");
      String storedPassword = preferences.getString("password", "");

      if (storedSSID != "") {
        connectToWiFi(storedSSID.c_str(), storedPassword.c_str());
        if (WiFi.status() == WL_CONNECTED) {
          // 연결된 경우 시간 설정
          setupTime();
        } else {
          Serial.println("Wi-Fi에 연결하지 못했습니다.");
          // 대기하고 버튼 눌림 확인
          delay(5000);
        }
      } else {
        Serial.println("저장된 Wi-Fi 자격 증명이 없습니다.");
        // 대기하고 버튼 눌림 확인
        delay(5000);
      }
    }
  }
  delay(500);
}

void sendData() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!timeSynced) {
      setupTime();
    }
    if (timeSynced) { // 시간 동기화된 경우에만 데이터 전송
      // **센서 전원 ON**
      digitalWrite(sensorPowerPin, HIGH);
      
      // **배터리 전압 측정 추가 시작**
      // 배터리 전압 측정 추가 시작
      float batteryVoltage;
      int batteryPercentageRound;
      for(int i = 0; i < preVoltRead; i++) {
        // 이전 측정값 제거
        total = total - readings[readIndex];
        // 새로운 값 읽기
        readings[readIndex] = readBatteryVoltage();
        // 총합에 새로운 값 추가
        total = total + readings[readIndex];
        // 배열 인덱스 이동
        readIndex = (readIndex + 1) % numReadings;
        // 평균 계산
        averageVoltage = total / numReadings;

        batteryVoltage = averageVoltage;

        // 처음 측정이거나 히스테리시스 조건을 충족할 때만 갱신
        if (isFirstRead || abs(batteryVoltage - previousVoltage) > hysteresisThreshold) {
          previousVoltage = batteryVoltage;  // 갱신된 전압 저장
          isFirstRead = false;  // 첫 측정 이후로는 히스테리시스 적용

          // 배터리 퍼센트 계산 수정 (비선형 매핑 사용)
          if (batteryVoltage >= 4.15) {
            batteryPercentage = 100.0;
          } else if (batteryVoltage >= 3.3) {
            // 3.3V ~ 4.15V을 0 ~ 1로 정규화
            float normalizedVoltage = (batteryVoltage - 3.3) / (4.15 - 3.3); // 3.3V ~ 4.15V을 0 ~1로 정규화
            batteryPercentage = 100.0 * (normalizedVoltage * normalizedVoltage); // 제곱 함수 적용
          } else {
            batteryPercentage = 0.0;
          }
        }

        batteryPercentageRound = (int)round(batteryPercentage);

        // 배터리 정보를 시리얼 모니터에 출력
        Serial.print("Battery Voltage: ");
        Serial.print(String(batteryVoltage, 3)); // 소수점 3자리까지 출력
        Serial.print(" V ");
        Serial.print(String(batteryPercentage, 3)); // 소수점 3자리까지 출력
        Serial.print(" % (Rounded: ");
        Serial.print(String(batteryPercentageRound)); // 반올림된 값 출력
        Serial.println(" %)");
        delay(30);
      }
      // **배터리 전압 측정 추가 끝**

      // 센서 데이터 읽기
      // DHT22 센서 데이터
      h = dht.readHumidity();        // 습도
      t = dht.readTemperature();     // 온도 (섭씨)
      hic = dht.computeHeatIndex(t, h, false); // 체감 온도

      // 토양 습도 센서 데이터
      adcValue = analogRead(analogPin); // 아날로그 값 읽기
      voltage = (adcValue / 4095.0) * voltageReference; // 전압으로 변환

      // 토양 습도 상태 판단
      String soilStatus;
      if (adcValue < moistureThreshold) {
        soilStatus = "Moist";
      } else {
        soilStatus = "Dry";
      }

      // **센서 전원 OFF**
      digitalWrite(sensorPowerPin, LOW);

      // **평균 계산을 위한 누적 및 카운트 업데이트**
      String currentDate = getCurrentDateAsString(); // 현재 날짜 가져오기
          
      // 날짜 변경 확인
      if (storedDate != currentDate) {
        // 날짜가 변경되었으므로 누적값과 카운트 초기화
        sumTemperature = 0.0;
        sumHumidity = 0.0;
        sumMoistureADC = 0;
        countReadings = 0;
        storedDate = currentDate;
      }

      // 현재 읽은 값을 누적합에 추가
      sumTemperature += t;
      sumHumidity += h;
      sumMoistureADC += adcValue;
      countReadings++;

      // 누적값과 카운트를 Preferences에 저장
      preferences.putFloat("sumTemp", sumTemperature);
      preferences.putFloat("sumHum", sumHumidity);
      preferences.putInt("sumMoisture", sumMoistureADC);
      preferences.putInt("countReadings", countReadings);
      preferences.putString("storedDate", storedDate);

      // 평균 계산
      float avgTemperature = sumTemperature / countReadings;
      float avgHumidity = sumHumidity / countReadings;
      float avgMoistureADC = (float)sumMoistureADC / countReadings;

      // 센서 데이터를 시리얼 모니터에 출력
      Serial.print("Humidity: ");
      Serial.print(h);
      Serial.print(" %\t");
      Serial.print("Temperature: ");
      Serial.print(t);
      Serial.print(" *C ");
      Serial.print("Heat index: ");
      Serial.print(hic);
      Serial.println(" *C ");

      Serial.print("ADC Value: ");
      Serial.print(adcValue);
      Serial.print(" | Voltage: ");
      Serial.println(voltage, 2);

      Serial.println("Soil Status: " + soilStatus);

      // 평균값 출력
      Serial.println("Average Temperature: " + String(avgTemperature));
      Serial.println("Average Humidity: " + String(avgHumidity));
      Serial.println("Average Moisture ADC: " + String(avgMoistureADC));

      HTTPClient http;

      // 현재 시간을 문서 ID로 사용 (분까지만 포함)
      String currentTime = getCurrentTimeAsString();
      String firestoreURL =
          firestoreBaseURL + "/nowdata" + "?key=" + apiKey;

      http.begin(firestoreURL); // Firestore URL 설정
      http.addHeader("Content-Type", "application/json");

      // 현재 시간을 타임스탬프로 가져오기
      time_t now;
      time(&now);

      // JSON 데이터 생성
      StaticJsonDocument<512> doc; // 더 많은 데이터를 수용하기 위해 크기 증가
      doc["fields"]["name"]["stringValue"] = "ESP32";
      doc["fields"]["temperature"]["doubleValue"] = t;
      doc["fields"]["humidity"]["doubleValue"] = h;
      doc["fields"]["heatIndex"]["doubleValue"] = hic;
      doc["fields"]["soilMoistureADC"]["integerValue"] = adcValue;
      doc["fields"]["soilVoltage"]["doubleValue"] = voltage;
      doc["fields"]["soilStatus"]["stringValue"] = soilStatus;
      doc["fields"]["timestamp"]["integerValue"] = now;
      doc["fields"]["currentTime"]["stringValue"] = currentTime;

      // 배터리 정보 추가
      doc["fields"]["batteryVoltage"]["doubleValue"] = batteryVoltage;
      doc["fields"]["batteryPercentage"]["doubleValue"] = batteryPercentage;
      doc["fields"]["batteryPercentageRound"]["integerValue"] = batteryPercentageRound;

      // JSON을 문자열로 직렬화
      String jsonData;
      serializeJson(doc, jsonData);

      // Firestore에 HTTP PATCH 요청 전송
      int httpResponseCode = http.PATCH(jsonData); // PATCH 사용하여 새 문서 생성

      if (httpResponseCode > 0) {
        Serial.print("POST 응답 코드: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println("응답: " + payload);
      } else {
        Serial.print("POST 전송 오류: ");
        Serial.println(httpResponseCode);
      }

      http.end(); // HTTP 연결 종료

      HTTPClient http2;
      // 평균 값을 저장하기 위한 변수
      String firestoreURL_average =
          firestoreBaseURL + "/"+ currentDate + "?key=" + apiKey;
      http2.begin(firestoreURL_average); // Firestore URL 설정
      http2.addHeader("Content-Type", "application/json");
      StaticJsonDocument<512> doc2; // 더 많은 데이터를 수용하기 위해 크기 증가
            // 평균값 추가
      doc2["fields"]["avgTemperature"]["doubleValue"] = avgTemperature;
      doc2["fields"]["avgHumidity"]["doubleValue"] = avgHumidity;
      doc2["fields"]["avgMoistureADC"]["doubleValue"] = avgMoistureADC;

            // JSON을 문자열로 직렬화
      String jsonData2;
      serializeJson(doc2, jsonData2);

      // Firestore에 HTTP PATCH 요청 전송
      int httpResponseCode2 = http2.PATCH(jsonData2); // PATCH 사용하여 새 문서 생성

      if (httpResponseCode2 > 0) {
        Serial.print("POST 응답 코드: ");
        Serial.println(httpResponseCode2);
        String payload2 = http2.getString();
        Serial.println("응답: " + payload2);
      } else {
        Serial.print("POST 전송 오류: ");
        Serial.println(httpResponseCode2);
      }
      http2.end(); // HTTP 연결 종료
    } else {
      Serial.println("시간이 동기화되지 않아 데이터 전송을 건너뜁니다.");
    }
  } else {
    Serial.println("Wi-Fi에 연결되지 않아 데이터를 전송할 수 없습니다.");
  }
}


String getCurrentDateAsString() {
    time_t now;
    struct tm timeinfo;
    time(&now);

    // 한국 시간(GMT+9)을 적용합니다.
    now += 9 * 3600;
    localtime_r(&now, &timeinfo);

    char dateString[9]; // YYYYMMDD + null terminator
    strftime(dateString, sizeof(dateString), "%Y%m%d", &timeinfo); // 형식: YYYYMMDD
    return String(dateString);
}

uint64_t calculateSleepDuration() {
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);

  int currentMinute = timeinfo.tm_min;
  int currentSecond = timeinfo.tm_sec;

  // 다음 간격까지 남은 분 계산
  int minutesToNextInterval = intervalMinutes - (currentMinute % intervalMinutes);
  if (minutesToNextInterval == intervalMinutes) {
    minutesToNextInterval = 0;
  }

  // 슬립 지속 시간 계산 (초 단위)
  int sleepSeconds = minutesToNextInterval * 60 - currentSecond;
  if (sleepSeconds <= 0) {
    sleepSeconds += intervalMinutes * 60;
  }

  // 최소 슬립 시간 설정 (인터벌의 절반)
  int minimumSleepTime = (intervalMinutes * 60) / 5;
  if (sleepSeconds < minimumSleepTime) {
    sleepSeconds += intervalMinutes * 60;
  }

  // 마이크로초로 변환
  uint64_t sleepDuration = (uint64_t)sleepSeconds * 1000000ULL;

  Serial.printf("현재 시간: %02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  Serial.printf("슬립 시간: %d초\n", sleepSeconds);

  return sleepDuration;
}

void startWiFiScan() {
  Serial.println("WiFi 스캔 시작...");

  // Wi-Fi 스택 재설정
  WiFi.disconnect(true); // Wi-Fi 연결 해제 및 스택 재설정
  WiFi.mode(WIFI_STA);   // STA 모드로 설정
  delay(100);            // 안정화를 위해 지연

  int n = WiFi.scanNetworks();
  Serial.println("스캔 완료");
  SerialBT.println("WIFI_SCAN_START");

  if (n == 0) {
    Serial.println("네트워크를 찾을 수 없습니다");
    SerialBT.println("No networks found");
  } else if (n > 0) {
    Serial.print(n);
    Serial.println("개의 네트워크를 찾았습니다");
    // 네트워크 목록 전송
    for (int i = 0; i < n; ++i) {
      String ssid = WiFi.SSID(i);
      Serial.printf("%d: %s\n", i + 1, ssid.c_str());
      SerialBT.println(ssid);
      delay(10);
    }
  } else {
    // 오류 처리
    Serial.print("Wi-Fi 스캔 중 오류 발생: ");
    Serial.println(n);
    SerialBT.println("Wi-Fi scan error");
  }
  SerialBT.println("WIFI_SCAN_END");
}
void handleWiFiCredentials(String incoming) {
  // 형식: WIFI_CREDENTIALS:ssid,password
  int separatorIndex = incoming.indexOf(':');
  if (separatorIndex > 0) {
    String credentials = incoming.substring(separatorIndex + 1);
    int commaIndex = credentials.indexOf(',');
    if (commaIndex > 0) {
      String ssid = credentials.substring(0, commaIndex);
      String password = credentials.substring(commaIndex + 1);
      ssid.trim();
      password.trim();
      Serial.printf("수신된 SSID: %s, 비밀번호: %s\n", ssid.c_str(), password.c_str());

      // Wi-Fi에 연결 시도
      connectToWiFi(ssid.c_str(), password.c_str());
    } else {
      Serial.println("잘못된 자격 증명 형식입니다.");
    }
  } else {
    Serial.println("잘못된 명령 형식입니다.");
  }
}

void connectToWiFi(const char *ssid, const char *password) {
  Serial.printf("WiFi SSID에 연결 중: %s\n", ssid);
  Serial.printf("WiFi 비밀번호로 연결 중: %s\n", password);

  // Wi-Fi 스택 완전히 재설정
  WiFi.disconnect(true); // Wi-Fi 연결 해제 및 스택 재설정WiFi에 연결되었습니다!
  WiFi.mode(WIFI_STA);   // STA 모드로 설정
  delay(100);            // 안정화를 위해 지연

  WiFi.begin(ssid, password);

  int maxAttempts = 20;
  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi에 연결되었습니다!");
    Serial.print("IP 주소: ");
    Serial.println(WiFi.localIP());
    if (configMode) {
      SerialBT.println("MACADDRESS "+getSanitizedMacAddress());
      delay(100);
      SerialBT.println("WIFI_CONNECT_SUCCESS");
    }

    // 자격 증명 저장
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    Serial.println("Wi-Fi 자격 증명이 저장되었습니다.");
  } else {
    Serial.println("\nWi-Fi에 연결하지 못했습니다.");
    SerialBT.println("WIFI_CONNECT_FAILED");
    if (configMode) {
      SerialBT.println("WIFI_CONNECT_FAILED");
    }
  }
}

void setupTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov"); // NTP 서버
  Serial.println("시간 동기화를 기다리는 중...");

  // 시간 동기화 대기
  struct timeval tv;
  struct tm timeinfo;
  int attempts = 0;
  int maxAttempts = 30; // 최대 시도 횟수

  while (attempts < maxAttempts) {
    gettimeofday(&tv, nullptr); // 현재 시간 가져오기
    localtime_r(&tv.tv_sec, &timeinfo);

    // 시간이 동기화되었는지 확인 (년도 >= 2020)
    if (timeinfo.tm_year >= (2020 - 1900)) {
      timeSynced = true;
      break;
    }

    Serial.print(".");
    delay(1000);
    attempts++;
  }

  if (timeSynced) {
    Serial.println();
    Serial.println("시간이 동기화되었습니다.");
  } else {
    Serial.println();
    Serial.println("시간 동기화에 실패했습니다.");
  }
}

String getSanitizedMacAddress() {
  String mac = WiFi.macAddress();
  mac.replace(":", ""); // 콜론 제거
  return mac;
}

String getCurrentTimeAsString() {
    time_t now;
    struct tm timeinfo;
    time(&now);

    // 한국 시간(GMT+9)을 적용합니다.
    now += 9 * 3600;
    localtime_r(&now, &timeinfo);

    // 초가 50초 이상이면 시간을 올림합니다.
    if (timeinfo.tm_sec >= 50) {
        timeinfo.tm_sec = 0;
        timeinfo.tm_min += 1;

        if (timeinfo.tm_min >= 60) {
            timeinfo.tm_min = 0;
            timeinfo.tm_hour += 1;

            if (timeinfo.tm_hour >= 24) {
                timeinfo.tm_hour = 0;
                timeinfo.tm_mday += 1;

                // 날짜를 정상화합니다.
                time_t adjustedTime = mktime(&timeinfo);
                localtime_r(&adjustedTime, &timeinfo);
            }
        }
    }

    char timeString[20];
    // 형식을 YYYYMMDDHHMM으로 변경하여 초를 제외합니다.
    strftime(timeString, sizeof(timeString), "%Y%m%d%H%M", &timeinfo); // 형식: YYYYMMDDHHMM
    return String(timeString);
}

// 보정된 ADC 값 읽기 (mV 단위)
uint32_t readCalibratedADC() {
  uint32_t adc_reading = 0;
  for (int i = 0; i < 64; i++) {
    adc_reading += analogRead(batteryAnalogPin);
  }
  adc_reading /= 64;
  uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
  return voltage; // mV 단위로 반환
}


// 배터리 전압을 측정하여 안정화된 값을 반환
float readBatteryVoltage() {
  int samples = 30;
  float voltageValues[samples];

  for (int i = 0; i < samples; i++) {
    uint32_t measuredVoltage = readCalibratedADC(); // mV 단위
    voltageValues[i] = (float)measuredVoltage * (R1 + R2) / R2 / 1000.0; // V 단위로 변환
    delay(10);
  }

  // 평균 계산
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += voltageValues[i];
  }
  float meanVoltage = sum / samples;

  // 표준편차 계산
  float variance = 0;
  for (int i = 0; i < samples; i++) {
    variance += pow(voltageValues[i] - meanVoltage, 2);
  }
  float standardDeviation = sqrt(variance / samples);

  // 유효한 값들로 평균 계산
  sum = 0;
  int validCount = 0;
  for (int i = 0; i < samples; i++) {
    if (abs(voltageValues[i] - meanVoltage) <= 2 * standardDeviation) {
      sum += voltageValues[i];
      validCount++;
    }
  }

  if (validCount > 0) {
    return sum / validCount;
  } else {
    return meanVoltage;
  }
}
