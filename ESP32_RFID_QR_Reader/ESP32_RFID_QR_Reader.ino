#include "esp_camera.h"
#include "quirc.h"

#include <Wire.h>
#include <WiFi.h>


#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  21
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27

#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    19
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    5
#define Y2_GPIO_NUM    4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

#define DOOR_RELAY 15
#define LED 13

SET_LOOP_TASK_STACK_SIZE(16*1024)

camera_fb_t *fb = NULL;
/*
const char* ssid = "ASUS_NDOOOG";
const char* password = "FaFen542";

String serverName = "192.168.2.82";
*/
const char* ssid = "Hotspot";
const char* password = "";

String serverName = "10.42.0.1";

String serverPath = "/upload.php";
const int serverPort = 80;
String nama_akses = "";

struct quirc *q = NULL;
String QRCodeResult = "";

WiFiClient client;

#define NFC_INTERFACE_HSU

#include <PN532_HSU.h>
#include <PN532.h>
    
HardwareSerial mySerial(2);
PN532_HSU pn532hsu(mySerial);
PN532 nfc(pn532hsu);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();

  pinMode(DOOR_RELAY, OUTPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(DOOR_RELAY, HIGH);
  digitalWrite(LED,LOW);

  //Wire.begin();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(WIFI_PS_NONE);
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 8000000; //Set to 8 MHz to avoid interference with Wi-Fi or modify hardware
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 4;
  config.fb_count = 1;
  config.sccb_i2c_port = 0;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //s->set_vflip(s, 1); 
  //s->set_hmirror(s, 1); 
  s->set_framesize(s,FRAMESIZE_VGA);
  //s->set_special_effect(s,2);
  s->set_gainceiling(s,(gainceiling_t)4);
  delay(1000);

  Serial.println("Creating quirc object");
  q = quirc_new();
  if(q == NULL){
    Serial.println("Can't create quirc object");
  }
  quirc_resize(q, 640,480);
  
  mySerial.begin(115200, SERIAL_8N1,32,33);
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  nfc.setPassiveActivationRetries(0xFF);
  nfc.SAMConfig();
}

void loop() {
  // put your main code here, to run repeatedly:
  QRDetect();
  
  unsigned long uid = getID();
  Serial.println(QRCodeResult);
  Serial.println(uid);
  if(QRCodeResult == "" && uid == 0){
    Serial.println("No ID or QR");
    return;
  }
  long startTimer = millis();
  bool res = checkID(uid,QRCodeResult);
  if(res){
    Serial.println("ACCESS GRANTED");
    digitalWrite(DOOR_RELAY,LOW);
    digitalWrite(LED,HIGH);
    sendPhoto(uid,QRCodeResult);
    //delay(1000);
  }
  long endTimer = millis();
  Serial.print("Server response time : ");
  Serial.println(endTimer-startTimer);

  digitalWrite(DOOR_RELAY,HIGH);
  digitalWrite(LED,LOW);
  delay(100);
  QRCodeResult = "";
}

void QRDetect() {
  Serial.println("Detecting QR Code");
  camera_fb_t *fb = NULL;
  delay(100);
  
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s,FRAMESIZE_VGA);
  s->set_quality(s, 4);
  s->set_special_effect(s,2);
  delay(100);

  //Serial.println("Capturing frame");
  fb = esp_camera_fb_get();
  esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if(!fb){
    Serial.println("Camera capture failed");
    return;
  }

  //Serial.println("Allocating memory for RGB888");
  uint32_t image_len = (fb->height * fb->width) * 3; 
  uint8_t *imageRGB = (uint8_t *) heap_caps_malloc(image_len+1024, MALLOC_CAP_SPIRAM);
  if(!imageRGB){
    Serial.println("Couldn't allocate memory");
    esp_camera_fb_return(fb);
    return;
  }
 
 
  //Serial.println("Converting JPEG to RGB888");
  bool jpegConvert = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, imageRGB);
  if(!jpegConvert){
    Serial.println("Couldn't convert JPEG to RGB888");
    esp_camera_fb_return(fb);
    heap_caps_free(imageRGB);
    return;
  }
  esp_camera_fb_return(fb);
  fb = NULL;

  uint8_t * qrBuf = quirc_begin(q, NULL, NULL);

  //Serial.println("Convert grayscale");
  convertGrayscale(imageRGB, image_len, qrBuf);
  heap_caps_free(imageRGB);
  imageRGB = NULL;

  quirc_end(q);
 
  //Serial.println("Decoding QR");
  int num_codes;
  num_codes = quirc_count(q);
  for(int i =0; i < num_codes; i++){
    struct quirc_code code;
    struct quirc_data data;
    quirc_decode_error_t err;

    quirc_extract(q, i, &code);
    
    err = quirc_decode(&code, &data);
    
    if (err){
        Serial.printf("DECODE FAILED: %s\n", quirc_strerror(err));
    }
    else{
        Serial.printf("Version: %d\n", data.version);
        Serial.printf("ECC level: %c\n", "MLHQ"[data.ecc_level]);
        Serial.printf("Mask: %d\n", data.mask);
        Serial.printf("Data: %s\n", data.payload);
        QRCodeResult = (const char *) data.payload;
    }

  }

   //quirc_destroy(q);
}

void convertGrayscale(uint8_t *buf, size_t len, uint8_t * ptrOut)
{
  uint8_t *ptrFb = buf;
  
  for (uint32_t i = 0; i < len ; i+=3) {

    uint8_t b = *(ptrFb + i);
    uint8_t g = *(ptrFb + i + 1);
    uint8_t r = *(ptrFb + i + 2);

    ptrOut[i/3] = (r + b + g)/3;
  }
}

bool checkID(unsigned long tag_id, String qrResult){
  long time = millis();
  Serial.println("Transmiting QR and RFID data");
  int timeout = 0;
  while (!client.connect(serverName.c_str(), serverPort)){
    delay(200);
    timeout++;
    if (timeout > 5){
      return false;
    }
  }
  Serial.println("Client connected");
  String getAll;
  String getBody;
  
  String headImage = "--112020043Fadlyka\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
  String headId = "\r\n\r\n--112020043Fadlyka\r\nContent-Disposition: form-data; name=\"id\"\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n";
  String headQr = "\r\n\r\n--112020043Fadlyka\r\nContent-Disposition: form-data; name=\"qr\"\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n";
  String tail = "\r\n--112020043Fadlyka--\r\n";


  String payload_id = String(tag_id);
  String payload_qr = qrResult;
  uint32_t extraLen = tail.length() + headId.length() + headQr.length();
  uint32_t totalLen = extraLen + payload_id.length() + payload_qr.length();

  client.println("POST /script/checkid.php HTTP/1.1");
  client.println("Host: " + serverName);
  //client.println("Connection: keep-alive");
  client.println("Content-Length: " + String(totalLen));
  client.println("Content-Type: multipart/form-data; boundary=112020043Fadlyka");
  client.println();

  client.print(headId);
  client.print(payload_id);
  client.print(headQr);
  client.print(payload_qr);
  client.print(tail);
  client.println();


  int timoutTimer = 10000;
  long startTimer = millis();
  boolean state = false;

  while ((startTimer + timoutTimer) > millis()) {
    //Serial.print(".");
    delay(100);      
    while (client.available()) {
      char c = client.read();
      if (c == '\n') {
        if (getAll.length()==0) { state=true; }
        getAll = "";
      }
      else if (c != '\r') { getAll += String(c); }
      if (state==true) { getBody += String(c); }
      startTimer = millis();
    }
    if (getBody.length()>0) { break; }
  }

  client.stop();


  //buf = NULL;
  Serial.print("Response body : ");
  Serial.println(getBody);
  Serial.println(getBody.substring(3));
  Serial.print("first char body : ");
  Serial.println(getBody.charAt(1));
  if(!getBody){
    return false;
  }
  if(getBody.charAt(1) == '1'){
    nama_akses = getBody.substring(3);
    return true;
  }
  return false;
}


unsigned long getID(){
  bool success;
  uint8_t tempUID[] = {0,0,0,0};
  uint8_t uidLength;

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &tempUID[0], &uidLength);

  if(!success){
    return 0;
  }

  volatile unsigned long UID = 0;
  UID |= (tempUID[0] << 24);
  UID |= (tempUID[1] << 16);
  UID |= tempUID[2] << 8;
  UID |= tempUID[3];
  
  return UID;
}

void sendPhoto(unsigned long tag_id, String otp){
  Serial.println("Sending photo");
  Serial.print("Wi-Fi Strength : ");
  Serial.println(WiFi.RSSI());
  while (!client.connect(serverName.c_str(), serverPort)){
    delay(1000);
  }
  //Serial.println("Server connected");
  String getAll;
  String getBody;
  Serial.print("Free heap : ");
  Serial.println(ESP.getFreeHeap());

  
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s,FRAMESIZE_HD);
  s->set_quality(s, 5);
  s->set_special_effect(s,0);
  delay(200);
  

  for(int i = 0; i < 5; i++){
    fb = esp_camera_fb_get();
    delay(10);
    esp_camera_fb_return(fb);
  }

  fb = esp_camera_fb_get();
  if(!fb){
    Serial.println("Camera capture failed");
    return;
  }

  s->set_framesize(s,FRAMESIZE_VGA);
  s->set_quality(s, 4);
  s->set_special_effect(s,2);
  delay(200);
    Serial.print("Free heap : ");
  Serial.println(ESP.getFreeHeap());
  Serial.println("Allocating memory for RGB888");
  uint32_t image_len = fb->len; 
  uint8_t *image = (uint8_t *) heap_caps_malloc(image_len+1024, MALLOC_CAP_SPIRAM);
  if(!image){
    Serial.println("Couldn't allocate memory");
    esp_camera_fb_return(fb);
    return;
  }
  Serial.print("Free heap : ");
  Serial.println(ESP.getFreeHeap());
  memcpy(image,fb->buf,fb->len);
  //esp_fill_random(image,fb->len);
  esp_camera_fb_return(fb);
  
  String headImage = "--112020043Fadlyka\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
  String headId = "\r\n\r\n--112020043Fadlyka\r\nContent-Disposition: form-data; name=\"id\"\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n";
  String headQr = "\r\n\r\n--112020043Fadlyka\r\nContent-Disposition: form-data; name=\"qr\"\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n";
  String tail = "\r\n--112020043Fadlyka--\r\n";


  String payload_id = String(tag_id);
  Serial.println(payload_id);
  Serial.println(payload_id);
  String payload_qr = otp;

  uint8_t *buf = image;
  uint32_t imageLen = image_len;
  Serial.println(imageLen);
  uint32_t extraLen = headImage.length() + tail.length() + headId.length() + headQr.length();
  uint32_t totalLen = imageLen + extraLen + payload_id.length() + payload_qr.length();

  Serial.print("Transmitting photo with size of : ");
  Serial.println(totalLen);
  Serial.print("Free heap : ");
  Serial.println(ESP.getFreeHeap());
  client.println("POST /upload.php HTTP/1.1");
  client.println("Host: " + serverName);
  client.println("Connection: keep-alive");
  client.println("Content-Length: " + String(totalLen));
  client.println("Content-Type: multipart/form-data; boundary=112020043Fadlyka");
  client.println();

  client.print(headImage);
  client.write(buf,imageLen);
  Serial.println("A");
  client.print(headId);
  client.print(payload_id);
  Serial.println("B");
  client.print(headQr);
  client.print(payload_qr);
  client.print(tail);
  client.println();

  Serial.println("Photo transmitted");



  int timoutTimer = 10000;
  long startTimer = millis();
  boolean state = false;

  Serial.println("Waiting for response");
  while ((startTimer + timoutTimer) > millis()) {
    //Serial.print(".");
    delay(100);      
    while (client.available()) {
      char c = client.read();
      if (c == '\n') {
        if (getAll.length()==0) { state=true; }
        getAll = "";
      }
      else if (c != '\r') { getAll += String(c); }
      if (state==true) { getBody += String(c); }
      startTimer = millis();
    }
    if (getBody.length()>0) { break; }
  }

  Serial.println(getBody);
  client.stop();
  heap_caps_free(image);
  image = NULL;
  buf = NULL;
  
  }
