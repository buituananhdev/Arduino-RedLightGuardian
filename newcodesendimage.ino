#include <WiFi.h>
#include <WebServer.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <base64.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
//======================================== Insert your network credentials.
const char* ssid = "Iphone";
const char* password = "13131313";
//========================================

const int trigPin = 15;
const int echoPin = 13;
long duration;
int distance;
String response = "";

const int redPin = 14;
const int yellowPin = 2;
const int greenPin = 12;

TaskHandle_t TaskSendPhoto;
TaskHandle_t TaskSensor;
TaskHandle_t TaskTrafficLight;

void TaskSendPhotoCode(void* parameter);
void TaskSensorCode(void* parameter);
void TaskTrafficLightCode(void* parameter);

WebServer server(80);
// Server Address or Server IP.
String serverName = "20.115.48.152";  //--> Change with your server computer's IP address or your Domain name.
// The file path "upload_img.php" on the server folder.
String serverPath1 = "/detect";
String serverPath2 = "/api/storages/upload";
// Server Port.
int serverPort1 = 3012;
int serverPort2 = 3011;

WiFiClient client;

void sendPhotoToServer(int type) {
  String path;
  int port;
  if (type == 0) {
    path = serverPath1;
    port = serverPort1;
  } else {
    path = serverPath2;
    port = serverPort2;
  }
  String AllData;
  String DataBody;

  Serial.println();
  Serial.println("-----------");

  Serial.println("Taking a photo...");

  for (int i = 0; i <= 3; i++) {
    camera_fb_t* fb = NULL;
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      Serial.println("Restarting the ESP32 CAM.");
      delay(1000);
      ESP.restart();
      return;
    }
    esp_camera_fb_return(fb);
    delay(200);
  }

  camera_fb_t* fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    Serial.println("Restarting the ESP32 CAM.");
    delay(1000);
    ESP.restart();
    return;
  }

  Serial.println("Taking a photo was successful.");

  Serial.println("Connecting to server: " + serverName);

  if (client.connect(serverName.c_str(), port)) {
    Serial.println("Connection successful!");
    unsigned long currentMillis = millis();  // Hoặc sử dụng micros() để lấy thời gian chính xác hơn
    String fileName = "ESP32CAM_" + String(currentMillis) + ".jpg";
    String post_data = "--dataMarker\r\nContent-Disposition: form-data; name=\"file\"; filename=\"" + fileName + "\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String head = post_data;
    String boundary = "\r\n--dataMarker--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t dataLen = head.length() + boundary.length();
    uint32_t totalLen = imageLen + dataLen;

    client.println("POST " + path + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=dataMarker");
    client.println();
    client.print(head);

    uint8_t* fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n = n + 1024) {
      if (n + 1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      } else if (fbLen % 1024 > 0) {
        size_t remainder = fbLen % 1024;
        client.write(fbBuf, remainder);
      }
    }
    client.print(boundary);

    esp_camera_fb_return(fb);

    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;
    Serial.println("Response : ");
    while ((startTimer + timoutTimer) > millis()) {
      Serial.print(".");
      delay(200);

      // Skip HTTP headers
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (AllData.length() == 0) {
            state = true;
          }
          AllData = "";
        } else if (c != '\r') {
          AllData += String(c);
        }
        if (state == true) {
          DataBody += String(c);
        }
        startTimer = millis();
      }
      if (DataBody.length() > 0) {
        response = DataBody;  // Store the server response in the variable
        break;
      }
    }
    client.stop();
    Serial.println(response);
    Serial.println("-----------");
    Serial.println();

  } else {
    client.stop();
    DataBody = "Connection to " + serverName + " failed.";
    Serial.println(DataBody);
    Serial.println("-----------");
  }
}

void senPhotoToClient() {
  Serial.println("heheh");
  Serial.println("Đang vào hàm test");
  Serial.println("IP của Client: " + server.client().remoteIP().toString());
  Serial.println("Phương thức yêu cầu: " + server.method());
  Serial.println("URI của yêu cầu: " + server.uri());
  sendPhotoToServer(1);
  server.send(200, "application/json", response);
}

void setup() {

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);

  Serial.begin(115200);
  Serial.println();

  WiFi.mode(WIFI_STA);
  Serial.println();

  Serial.println();
  Serial.print("Connecting to : ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int connecting_process_timed_out = 20;  //--> 20 = 20 seconds.
  connecting_process_timed_out = connecting_process_timed_out * 2;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    if (connecting_process_timed_out > 0) connecting_process_timed_out--;
    if (connecting_process_timed_out == 0) {
      Serial.println();
      Serial.print("Failed to connect to ");
      Serial.println(ssid);
      Serial.println("Restarting the ESP32 CAM.");
      delay(1000);
      ESP.restart();
    }
  }
  server.enableCORS(true);
  server.on("/get-image", HTTP_GET, senPhotoToClient);
  server.begin();
  Serial.print("IP Address of network: ");  // will IP address on Serial Monitor
  Serial.println(WiFi.localIP());

  Serial.println();
  Serial.print("Successfully connected to ");
  Serial.println(ssid);

  Serial.println();
  Serial.print("Set the camera ESP32 CAM...");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //--> 0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 8;  //--> 0-63 lower number means higher quality
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    Serial.println();
    Serial.println("Restarting the ESP32 CAM.");
    delay(1000);
    ESP.restart();
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_SXGA);  //--> UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA

  Serial.println();
  Serial.println("Set camera ESP32 CAM successfully.");
  Serial.println();

  xTaskCreatePinnedToCore(
    TaskSendPhotoCode,
    "TaskSendPhoto",
    10000,
    NULL,
    1,
    &TaskSendPhoto,
    1);

  xTaskCreatePinnedToCore(
    TaskSensorCode,
    "TaskSensor",
    10000,
    NULL,
    1,
    &TaskSensor,
    0);

  xTaskCreatePinnedToCore(
    TaskTrafficLightCode,
    "TaskTrafficLight",
    10000,
    NULL,
    1,
    &TaskTrafficLight,
    0);
}

void loop() {
}

void TaskSendPhotoCode(void* parameter) {
  for (;;) {
    server.handleClient();
    bool isRedLightOn = digitalRead(redPin) == HIGH;

    // Nếu đèn đỏ sáng và khoảng cách nhỏ hơn 35mm (3.5cm), gửi ảnh lên server
    if (isRedLightOn && distance < 35 && distance > 0) {
      sendPhotoToServer(0);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay 10 seconds
  }
}

void TaskTrafficLightCode(void* parameter) {
  for (;;) {
    // Hiển thị đèn đỏ trong 10 giây
    digitalWrite(redPin, HIGH);
    delay(10000);
    digitalWrite(redPin, LOW);

    // Hiển thị đèn vàng trong 3 giây
    digitalWrite(yellowPin, HIGH);
    delay(1000);
    digitalWrite(yellowPin, LOW);

    // Hiển thị đèn xanh trong 10 giây
    digitalWrite(greenPin, HIGH);
    delay(4000);
    digitalWrite(greenPin, LOW);

    vTaskDelay(2000 / portTICK_PERIOD_MS);  // Delay 1 second
  }
}


void TaskSensorCode(void* parameter) {
  for (;;) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;  // Đổi thời gian thành khoảng cách (đơn vị mm)
    Serial.print(distance);
    Serial.println("cm");
    vTaskDelay(3000 / portTICK_PERIOD_MS);  // Delay 1 second
  }
}