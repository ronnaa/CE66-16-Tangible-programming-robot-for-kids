#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"
#include <Wire.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_300MS, TCS34725_GAIN_1X); //กำหนดกาารใช้ Library
//LDR
const int ldrPin = 26; // กำหนดขาที่ใช้สำหรับเซ็นเซอร์ LDR
const int threshold = 4000; // กำหนดค่าเกณฑ์สำหรับการตรวจจับแสง
const int LEDPin = 4;     // กำหนดขาที่ใช้สำหรับ LED
//RGB
const int RedledPin = 13; // กำหนดขา LED
const int GreenledPin = 12; // กำหนดขา LED
const int BlueledPin = 14; // กำหนดขา LED

#define BUTTON_PLAY_PIN 36
bool buttonplay = false;
#define BUTTON_STOP_PIN 39
bool buttonstop = false;

HardwareSerial readSerial(1); // ใช้ UART1 สำหรับ ESPino32
const int rxPin = 16; // 
const int txPin = 17; // 

//Ultrasonic
int Pingpin = 2;
int Inpin = 15;
long duration , cm;

//Servo 
#define servoPin 27
int angle = 0;     // ค่ามุมเริ่มต้น
Servo myServo;  // สร้างอ็อบเจ็กต์ Servo

#define servo2Pin 21
int angle2 = 180;     // ค่ามุมเริ่มต้น
Servo myServo2;  // สร้างอ็อบเจ็กต์ Servo2

String prev_data = "";
String prev_data2 = "";

bool checkfinish = false;
bool checkultra = false;
bool checkred = false;
bool checkgreen = false;
bool checkblue = false;
bool checkcount = false;
bool checkldr = false;
bool checkbrushoff = false;
bool checkturntable = false;
bool checkrgbsensor = false;

int RGBSensor_value = 0;
bool checkredblue = false;
bool checkbluered = false;
bool checkredgreen = false;
bool checkgreenred = false;
bool checkgreenblue = false;
bool checkbluegreen = false;

bool checkultrabrushoff = false;
bool checkstop = false;

bool checksensorred = false;
bool checksensorblue = false;
bool checksensorgreen = false;

void BrushoffTask();


void setup() {
  Serial.begin(9600);
  readSerial.begin(9600, SERIAL_8N1, rxPin, txPin);

  pinMode(BUTTON_PLAY_PIN, INPUT_PULLUP);
  pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);

  pinMode(ldrPin, INPUT);
  pinMode(LEDPin, OUTPUT);
//Ultrasonic
    //Input Ultrasonic
  pinMode(Pingpin, OUTPUT);
  pinMode(Inpin, INPUT);

  myServo.attach(servoPin); // กำหนดพินที่ต่อกับมอเตอร์เซอร์โว
  myServo.write(angle);     // กำหนดมุมเริ่มต้น

  myServo2.attach(servo2Pin); // กำหนดพินที่ต่อกับมอเตอร์เซอร์โว
  myServo2.write(angle2);     // กำหนดมุมเริ่มต้น000

  xTaskCreatePinnedToCore(ReadDataTask, "Read QR Task", 4096 * 2, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(RGBledTask, "RGB led Taskk", 4096 * 2, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(UltrasonicTask, "Read QR Task", 4096 * 2, NULL, 1, NULL, 0);
  //xTaskCreatePinnedToCore(RGBSensorTask, "RGB Sensor Task", 4096 * 2, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ButtonTask, "But ton Task", 4096 * 2, NULL, 1, NULL, 1);

  led_set();
}

void loop() {
//////////////////////////////////////////////

}


void ButtonTask(void *parameter) {
  pinMode(BUTTON_PLAY_PIN, INPUT_PULLUP);
  pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);
for (;;) {
  if (digitalRead(BUTTON_PLAY_PIN) == LOW && !buttonplay) {
    Serial.println("StartPlay");
    readSerial.println("StartPlay");
    buttonplay = true;
    // ตรวจสอบว่าปุ่มถูกปล่อยออกหรือไม่
  }
  if (digitalRead(BUTTON_PLAY_PIN) == HIGH && buttonplay) {
    buttonplay = false;
  }
  if (digitalRead(BUTTON_STOP_PIN) == LOW && !buttonstop) {
    Serial.println("StartStop");
    readSerial.println("StartStop");
    led_set();
    digitalWrite(LEDPin, LOW);
    prev_data = "";
    prev_data2 = "";
    buttonstop = true;
    checkredblue = false;
    checkbluered = false;
    checkredgreen = false;
    checkgreenred = false;
    checkgreenblue = false;
    checkbluegreen = false;
    checkultra = false;
    checkred = false;
    checkgreen = false;
    checkblue = false;
    checkcount = false;
    checkldr = false;
    checkbrushoff = false;
    checkturntable = false;
    checkrgbsensor = false;
    checkultrabrushoff = false;
    checkredblue = false;
    checkbluered = false;
    checkredgreen = false;
    checkgreenred = false;
    checkgreenblue = false;
    checkbluegreen = false;
  }
  if (digitalRead(BUTTON_STOP_PIN) == HIGH && buttonstop) {
    buttonstop = false;
  }
  delay(5);
}}

void ReadDataTask(void *parameter) {
  String data = "";
  for (;;) {
    while (readSerial.available()) {
      String data = readSerial.readStringUntil('\n');
      data.trim();
      Serial.println(data);
    // Check if the message is not empty and contains valid data
    //if (data.length() > 0) {
      //Serial.print("Message from ESP32 1: ");
      //Serial.println(data);
     if (data == "finish"){
        checkfinish == true ;
        prev_data = "";
    prev_data2 = "";
    buttonstop = true;
    checkredblue = false;
    checkbluered = false;
    checkredgreen = false;
    checkgreenred = false;
    checkgreenblue = false;
    checkbluegreen = false;
    checkultra = false;
    checkred = false;
    checkgreen = false;
    checkblue = false;
    checkcount = false;
    checkldr = false;
    checkbrushoff = false;
    checkturntable = false;
    checkrgbsensor = false;
    checkultrabrushoff = false;
    checkredblue = false;
    checkbluered = false;
    checkredgreen = false;
    checkgreenred = false;
    checkgreenblue = false;
    checkbluegreen = false;
     }
     else if (data == "Start305" && prev_data == "Start304") {
        Serial.print(prev_data + data);
        prev_data = "";
        data = "";
        checkredblue = true;
        checkred = false;
        checkblue = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (data == "Start304" && prev_data == "Start305") {
        Serial.print(prev_data + data);
        prev_data = "";
        data = "";
        checkbluered = true;
        checkred = false;
        checkblue = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (data == "Start306" && prev_data == "Start304") {
        Serial.print(prev_data + data);
        prev_data = "";
        data = "";
        checkredgreen = true;
        checkred = false;
        checkgreen = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (data == "Start304" && prev_data == "Start306") {
        Serial.print(prev_data + data);
        prev_data = "";
        data = "";
        checkgreenred = true;
        checkred = false;
        checkgreen = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (data == "Start305" && prev_data == "Start306") {
        Serial.print(prev_data + data);
        prev_data = "";
        data = "";
        checkgreenblue = true;
        checkblue = false;
        checkgreen = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (data == "Start306" && prev_data == "Start305") {
        Serial.print(prev_data + data);
        prev_data = "";
        data = "";
        checkbluegreen = true;
        checkblue = false;
        checkgreen = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
     else if (data == "Start304") {
        Serial.print("Red");
        prev_data = data;
        checkred = true;
        RGBSensor_value = 1;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
      else if (data == "Start305") {
        prev_data = data;
        Serial.print("Blue");
        checkblue = true;
        RGBSensor_value = 3;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
      else if (data == "Start306") {
        prev_data = data;
        Serial.print("Green");
        checkgreen = true;
        RGBSensor_value = 2;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
      else if (data == "Start301") {
        Serial.print("Sensor");
        checkultra = true;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
      else if (data == "Start302") {
        Serial.print("brushoff");
        checkbrushoff = true;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
      else if (data == "Start303") {
        Serial.print("RGBSensor");
        checkrgbsensor = true;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
      else if (data == "Start307") {
        Serial.print("Count");
        checkcount = true;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
      else if (data == "Start308") {
        Serial.print("Turntable");
        checkturntable = true ;
        TurntableTask();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
      else if (data == "Start309") {
        Serial.print("LDRSensor");
        checkldr = true;
        LDRSensorTask();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
      
      else if (data == "if401301302") {
        Serial.print("if Sensor brushoff");
        checkultra = true;
        //checkbrushoff = true;
        checkultrabrushoff  = true;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
      else if (data == "if401301304") {
        Serial.print("if Sensor RED");
        checkultra = true;
        checksensorred = true;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (data == "if401301305") {
        Serial.print("if Sensor BLUE");
        checkultra = true;
        checksensorblue = true;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (data == "if401301306") {
        Serial.print("if Sensor GREEN");
        checkultra = true;
        checksensorgreen = true;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
      else if (data == "if401309") {
        Serial.print("if LDRSensor");
        checkldr = true;
        LDRSensorTask();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
      else if (data == "if401301303304") {
        Serial.print("if Sensor RGBSensor Red"); //ให้การเคลื่อนที่ต่อจากนี้ เป็นคิวการทำงาน
        checkultra = true;
        checkrgbsensor = true;
        readSerial.print("ifSensorRGBSensorRed");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    }
  }
}
void RGBledTask(void *parameter) {
  unsigned long Time = 0;
  for (;;) {    
     led_set();
     while (checkredblue == true){
         red_led(); 
         delay(300);
         blue_led();
         delay(300);
         vTaskDelay(pdMS_TO_TICKS(100));
     }
     while (checkbluered == true){
         blue_led(); 
         delay(300);
         red_led();
         delay(300);
         vTaskDelay(pdMS_TO_TICKS(100));
     }
     while (checkredgreen == true){
         red_led(); 
         delay(300);
         green_led();
         delay(300);
         vTaskDelay(pdMS_TO_TICKS(100));
     }
     while (checkgreenred == true){
         green_led(); 
         delay(300);
         red_led();
         delay(300);
         vTaskDelay(pdMS_TO_TICKS(100));
     }
     while (checkgreenblue  == true){
         green_led(); 
         delay(300);
         blue_led();
         delay(300);
         vTaskDelay(pdMS_TO_TICKS(100));
     }
     while (checkbluegreen  == true){
         blue_led(); 
         delay(300);
         green_led();
         delay(300);
         vTaskDelay(pdMS_TO_TICKS(100));
     }
      
    
    while (checkred == true){
        //Serial.print("Red");
        red_led(); 
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while (checkgreen == true){
        //Serial.print("Green");
        green_led(); 
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while (checkblue == true){
        //Serial.println("Blue");
        blue_led(); 
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelay(pdMS_TO_TICKS(500));
        }
  
  }

void UltrasonicTask(void *parameter) {
  //pinMode(scannerPin, OUTPUT);
  for (;;) {
    //  Ultrasonic duration
    digitalWrite(Pingpin, LOW);
    delayMicroseconds(2);
    digitalWrite(Pingpin, HIGH);
    delayMicroseconds(5);
    digitalWrite(Pingpin, LOW);
    duration = pulseIn(Inpin, HIGH);

    cm = microTime(duration);

    //Serial.print("Ultrasonic: ");
    //Serial.print(cm);
    //Serial.print("cm");
    //Serial.println();

    if (checkultra == true) {
      checkred = false;
      checkblue = false;
      checkgreen = false;
      if ( cm <= 25 ) {
        Serial.println("Stopnow");
        delay(10);
          if ( cm <= 25 && checkultrabrushoff == true) {
         Serial.print("Stopnow");

         checkbrushoff = true;
         delay(3000);
         BrushoffTask();
         vTaskDelay(pdMS_TO_TICKS(100));
       }
        if (checksensorred == true){
          checkred = true;
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (checksensorblue == true){
          checkblue = true;
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (checksensorgreen == true){
          checkgreen = true;
          vTaskDelay(pdMS_TO_TICKS(100));
        }
      }
      if ( cm == 0 ) {
        Serial.print("");
        //readSerial.print("Stopnow");
        checkbrushoff == false;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

long microTime(int microseconds) {
  return microseconds / 29 / 2;
}
void print_time(unsigned long time_millis);

void print_time(unsigned long time_millis){
  Serial.print("Time: ");
  Serial.print(time_millis/1000);
  Serial.print("s - ");
}

void RGBSensorTask(void *parameter) {
  int RGBSensor_value = 0;
  for (;;) {
      tcs.begin();
      uint16_t r, g, b, c, colorTemp, lux;
      tcs.getRawData(&r, &g, &b, &c);
      colorTemp = tcs.calculateColorTemperature(r, g, b);
      lux = tcs.calculateLux(r, g, b);

    // Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
    // Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
    // Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    // Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    // Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    // Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
    // Serial.println(" ");

    vTaskDelay(pdMS_TO_TICKS(100));
if (checkrgbsensor == true ){
      if (r > g && r > b) {
    // สีแดง
    Serial.println(" RED COLOR ");
} else if (g > r && g > b &&  c > 400 ) {
    // สีเขียว
    Serial.println(" GREEN COLOR ");
} else if (b > r && b > g) {
    // สีน้ำเงิน
    Serial.println(" BLUE COLOR ");
}if (c <= 400) {
    // ไม่พบสี
    Serial.println(" NO CoLOR ");
} else {
    // สีอื่น ๆ
    // ทำอะไรสำหรับสีอื่น ๆ
}
}
  }
}

void LDRSensorTask() {
    while(checkldr == true ){
      int ldrValue = analogRead(ldrPin); // อ่านค่าแอนะล็อกจากเซ็นเซอร์ LDR
      //Serial.print("LDR value: ");
      Serial.println(ldrValue);
      if (buttonstop == true ){
        checkldr = false ;
      }
      if (ldrValue > threshold) {
        digitalWrite(LEDPin, HIGH); // เปิด LED หากมีแสง
        //Serial.println("LED ON");
    // ทำอะไรบางอย่างเมื่อตรวจจับแสง
      } else {
        //Serial.println("No light detected.");
        // ทำอะไรบางอย่างเมื่อไม่ตรวจจับแสง
        digitalWrite(LEDPin, LOW); // ปิด LED หากไม่มีแสง
        //Serial.println("LED OFF");
  }
  vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void BrushoffTask() {
      for (int angle2 = 180; angle2 >= 0; angle2 -= 180) {
      myServo2.write(angle2); // สั่งให้ Servo Motor เลื่อนไปที่องศาที่กำหนด
      delay(1000); // หน่วงเวลาเพื่อให้ Servo Motor เลื่อนไปที่ตำแหน่ง
      Serial.println("ปัดออก");
  }
      delay(100); 
      for (int angle2 = 0; angle2 <= 180; angle2 +=180 ) {
      myServo2.write(angle2);
      Serial.println("ปัดเข้า");
      delay(1000); 
  }
    delay(100);
    Serial.print("UltrasonicisOK");
    readSerial.print("UltrasonicisOK");
    vTaskDelay(pdMS_TO_TICKS(100));
}

void TurntableTask() {
  if (checkturntable == true ){
      for (int angle = 0; angle <= 180; angle += 1) {
      myServo.write(angle); // สั่งให้ Servo Motor เลื่อนไปที่องศาที่กำหนด
      delay(100); // หน่วงเวลาเพื่อให้ Servo Motor เลื่อนไปที่ตำแหน่ง
  }
      delay(1000); 
      for (int angle = 180; angle >= 0; angle -= 1) {
      myServo.write(angle);
      delay(100); 
  }
    delay(1000);
  }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

void led_set(){
  pinMode(RedledPin, OUTPUT);
  pinMode(GreenledPin, OUTPUT);
  pinMode(BlueledPin, OUTPUT);
  digitalWrite(RedledPin, HIGH); 
  digitalWrite(GreenledPin, HIGH);
  digitalWrite(BlueledPin, HIGH); 
}
void red_led(){
    pinMode(GreenledPin, OUTPUT); 
    digitalWrite(GreenledPin, HIGH); 
    pinMode(BlueledPin, OUTPUT); 
    digitalWrite(BlueledPin, HIGH); 
    pinMode(RedledPin, OUTPUT); 
    digitalWrite(RedledPin, LOW); 
}
void green_led(){
    pinMode(GreenledPin, OUTPUT); 
    digitalWrite(GreenledPin, LOW); 
    pinMode(BlueledPin, OUTPUT); 
    digitalWrite(BlueledPin, HIGH); 
    pinMode(RedledPin, OUTPUT); 
    digitalWrite(RedledPin, HIGH); 
}
void blue_led(){
    pinMode(GreenledPin, OUTPUT); 
    digitalWrite(GreenledPin, HIGH); 
    pinMode(BlueledPin, OUTPUT); 
    digitalWrite(BlueledPin, LOW); 
    pinMode(RedledPin, OUTPUT); 
    digitalWrite(RedledPin, HIGH); 
}
/////////////////////////////////////////


