#include <Arduino.h>
#include <QTRSensors.h>
#include "ST7735_TEE.h"
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

QTRSensors qtr;
TEE_ST7735 lcd(18,21,0,25,5); //ESP8266 csk,sda,A0,rst,cs

HardwareSerial scannerSerial(1); // ใช้ UART1 สำหรับ ESPino32
const int rxPin = 16; // แก้ไขตามพินที่เชื่อมต่อกับ Rx ของ QR scanner
const int txPin = 17; // แก้ไขตามพินที่เชื่อมต่อกับ Tx ของ QR scanner

 HardwareSerial readSerial(0); // ใช้ UART1 สำหรับ ESPino32
 const int rx1Pin = 3; // แก้ไขตามพินที่เชื่อมต่อกับ Rx ของ QR scanner
 const int tx1Pin = 1; // แก้ไขตามพินที่เชื่อมต่อกับ Tx ของ QR scanner

const int leftMotorPin1 = 13;
const int leftMotorPin2 = 15;
const int rightMotorPin1 = 12;
const int rightMotorPin2 = 14;

const int PWM1 = 4;    // PWM left motor
const int PWM2 = 26;    // PWM right motor

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
float Kp=13.5,Ki=4,Kd=10;
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error=0,previous_I=0;
int initial_motor_speed=110;

TaskHandle_t taskHandle; // ประกาศตัวแปร Task Handle

QueueHandle_t commandQueue; // คิวสำหรับการส่งคำสั่ง
QueueHandle_t displayQueue; // คิวสำหรับการแสดงผล
QueueHandle_t reverseQueue; // คิวสำหรับการแสดงผล
QueueHandle_t displayreverseQueue;


//const int button1Pin = 6; 
//const int button2Pin = 7;

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void); 
void moveForward(void);
void read_QR_task(void *parameter);
void display_data(void *parameter); 
void motor_control_task(void *parameter);
void clearScreen(void);
void read_Serial_task(void *parameter);

int scanactive = 0; 
int checkplay = 1; 
int checkplayreverse = 1; 
bool checkreset = false; 
bool checkreset2 = false;
bool checkmotor = false;
bool onetimescan = false;
bool status_active = false;

String prev_data = "";
String prev_data2 = "";
//จิ๊กซอว์คำสั่ง
bool check_Forward = false;
bool check_TurnRight = false;
bool check_TurnLeft = false;
bool check_Backward = false;
bool check_TurnAround = false;
bool check_Start = false;
bool check_Stop = false;
bool check_Reset = false;
bool check_Delay = false;
bool check_Loop = false;
bool check_If = false;
bool check_Else = false;
//อุปกรณ์เสริม
bool checkultra = false;
bool checkred = false;
bool checkgreen = false;
bool checkblue = false;
bool checkcount = false;
bool checkldr = false;
bool checkbrushoff = false;
bool checkturntable = false;
bool checkrgbsensor = false;
//if
bool ultrabrushoff = false;
bool is_crossing = false;
bool check_reverse = false;
bool reverse = false;
bool left_check = false;
bool right_check = false;

bool ultra_stop = false;
bool brushoff_Ok = false;


void setup() {
  lcd.init(lcd.HORIZONTAL);
  lcd.fillScreen(BLACK);
  lcd.Printstr(10,55,(char*) "  . . . . . . .  ",WHITE);

  Serial.begin(9600);

  scannerSerial.begin(9600, SERIAL_8N1, rxPin, txPin);
  readSerial.begin(9600, SERIAL_8N1, rx1Pin, tx1Pin);


  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){36, 39, 32, 33, 34, 35}, SensorCount);
  qtr.setEmitterPin(2);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  commandQueue = xQueueCreate(50, sizeof(String));// สร้างคิวสำหรับคำสั่ง 
  displayQueue = xQueueCreate(1, sizeof(int)); // สร้างคิวสำหรับการแสดงผล
  reverseQueue = xQueueCreate(50, sizeof(String));// สร้างคิวสำหรับคำสั่ง 
  displayreverseQueue = xQueueCreate(1, sizeof(int)); // สร้างคิวสำหรับการแสดงผล


   xTaskCreatePinnedToCore(read_QR_task, "Read QR Task", 4096 * 2, NULL, 1, NULL, 0);
   xTaskCreatePinnedToCore(motor_control_task, "Motor Control Task", 4096 * 2, NULL, 2, NULL, 0);
   xTaskCreatePinnedToCore(display_data, "Display Data Task", 4096 * 2, NULL, 2, &taskHandle, 1); // เพิ่ม Task สำหรับการแสดงผล
   xTaskCreatePinnedToCore(read_Serial_task, "read_Serial_task", 4096, NULL, 1, NULL, 1);
   //xTaskCreatePinnedToCore(display_Reverse, "Display Reverse Task", 4096 * 2 , NULL, 2, NULL, 1); // เพิ่ม Task สำหรับการแสดงผล

  Create_gui();
}

void loop() {
  ///////////////////////////////////
}

void Create_gui() {
  //Screen Start 
  lcd.fillScreen(BLACK);
  lcd.Printstr(10,20,(char*) "                 ",WHITE);
  lcd.Printstr(10,40,(char*) "                 ",WHITE);
  lcd.Printstr(10,55,(char*) "  H E L L O !!!  ",WHITE);
  lcd.Printstr(10,80,(char*) "                 ",BLUE);
  lcd.Printstr(10,100,(char*)"                 ",BLUE);
  delay(1000);
  lcd.fillScreen(BLACK);
  lcd.Printstr(10,20,(char*)"                 ",WHITE);
  lcd.Printstr(10,40,(char*)"     I  A M      ",WHITE);
  lcd.Printstr(10,60,(char*)"  T A N G I R O  ",WHITE);
  lcd.Printstr(10,80,(char*)"                 ",BLUE);
  lcd.Printstr(10,80,(char*)"     ......      ",BLUE);

}

void read_Serial_task(void *parameter) {
  String data = "";
  for (;;) {
    while (readSerial.available()) {
    String data = readSerial.readStringUntil('\n');
    data.trim();
    Serial.println("Serial MCU2 : " + data);
    if (data == "Stopnow") {
        Serial.println("Stop now!!!");
        stopMoving();
        delay(2000  );
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      if (data == "StartPlay") {
        Serial.println(" Robot Play !!!");
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,50,(char*)"ROBOT START !!!",WHITE);
        xQueueSend(displayQueue, &checkplay, portMAX_DELAY);
        check_Start = true;
        status_active = true;
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      if (data == "StartStop") {
        Serial.println(" Robot Stop !!!");
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,50,(char*)"ROBOT STOP !!!",WHITE);
        check_Reset = true ;          
          int numQ = 0 ;
          int numI = 0 ;
          int numX = 0 ;
          int numR = 0;
          int numL = 0; 
          int scanactive = 0; 
int checkplay = 1; 
int checkplayreverse = 1; 
bool checkreset = false; 
bool checkreset2 = false;
bool checkmotor = false;
bool onetimescan = false;
bool status_active = false;

String prev_data = "";
String prev_data2 = "";
//จิ๊กซอว์คำสั่ง
bool check_Forward = false;
bool check_TurnRight = false;
bool check_TurnLeft = false;
bool check_Backward = false;
bool check_TurnAround = false;
bool check_Start = false;
bool check_Stop = false;
bool check_Reset = false;
bool check_Delay = false;
bool check_Loop = false;
bool check_If = false;
bool check_Else = false;
//อุปกรณ์เสริม
bool checkultra = false;
bool checkred = false;
bool checkgreen = false;
bool checkblue = false;
bool checkcount = false;
bool checkldr = false;
bool checkbrushoff = false;
bool checkturntable = false;
bool checkrgbsensor = false;
//if
bool ultrabrushoff = false;
bool is_crossing = false;
bool check_reverse = false;
bool reverse = false;
bool left_check = false;
bool right_check = false;

bool ultra_stop = false;
bool brushoff_ok = false;
        if (checkreset == true && checkreset2 == true) {
          stopMoving();
          prev_data = "";
          prev_data2 = "";
          checkplay = 1; 
          checkreset = false;
          checkreset2 = false;
          onetimescan = false;
          checkultra = false;
          checkmotor = false;
          check_Reset = true ;

          delay(100);
          int numMessages = uxQueueMessagesWaiting(commandQueue);
            for (int i = 0; i < numMessages; i++) {
              xQueueReceive(commandQueue, &data, 0);
              data.trim();
            }
            data = "";
      } else if (checkreset == true) {
      
        stopMoving();
        prev_data = "";
        prev_data2 = "";
        checkplay = 1;
        checkreset = false;
        checkreset2 = false;
        scanactive = 0;
        onetimescan = false;
        checkultra = false;
        checkmotor = false;
        check_Reset = true ;

      } else if (checkreset2 == true) {
        stopMoving();
        prev_data = "";
        prev_data2 = "";
        checkplay = 1;
        checkreset = false;
        checkreset2 = false;
        scanactive = 0;
        onetimescan = false;
        checkultra = false;
        checkmotor = false;
        check_Reset = true ;
      }

      else if (data == "ifSensorRGBSensorRed") {
        Serial.println("ifSensorRGBSensorRed Start");
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,50,(char*)"IF RED ",WHITE);
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      else {
        Serial.println("No image");
      }
    } else {
    }
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
  }

void read_QR_task(void *parameter) {
  String data = "";
  for (;;) {
  while (scannerSerial.available()) {
    String data = scannerSerial.readStringUntil('\n');
    data.trim();
    Serial.println("QR Code: " + data);
    clearScreen();

    if (prev_data == "401" && data == "301") {
        prev_data += data;
        Serial.println("Data =" + prev_data);
        Serial.println("401301");
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,50,(char*)"IF Sensor ",WHITE);
        checkreset = true;
        data = "";
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    if (prev_data == "401" && data == "309") {
        prev_data += data;
        Serial.println("Data =" + prev_data);
        Serial.println("if401309");
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,50,(char*)"IF Sensor ",WHITE);
        checkreset = true;
        data = "";
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    if (prev_data == "401" && data == "309") {
        prev_data += data;
        Serial.println("Data =" + prev_data);
        Serial.println("Start309");
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,50,(char*)"IF LDRSensor",WHITE);
        checkreset = true;
        data = "";
        prev_data = "";
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    if (prev_data == "401301" && data == "302") {
        prev_data += data;
        Serial.println("Data =" + prev_data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,50,(char*)"IF Sensor Brushoff",WHITE);
        Serial.println("if401301302");
        checkreset = true;
        prev_data = "";
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    if (prev_data == "401301" && data == "304") {
        prev_data += data;
        prev_data2 = prev_data;
        Serial.println("Data =" + prev_data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,50,(char*)"IF Sensor RED",WHITE);
        Serial.println("if401301304");
        checkreset = true;
        prev_data = "";
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    if (prev_data == "401301" && data == "305") {
        prev_data += data;
        Serial.println("Data =" + prev_data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,50,(char*)"IF Sensor Blue",WHITE);
        Serial.println("if401301305");
        checkreset = true;
        prev_data = "";
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    if (prev_data == "401301" && data == "306") {
        prev_data += data;
        Serial.println("Data =" + prev_data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,50,(char*)"IF Sensor GREEN",WHITE);
        Serial.println("if401301306");
        checkreset = true;
        prev_data = "";
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    if (prev_data2 == "401301304" && data == "305") {
        prev_data += data;
        Serial.println("Data =" + prev_data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,50,(char*)"IF Sensor RED ",WHITE);
        Serial.println("if401301304");
        checkreset = true;
        prev_data = "";
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    else  if (data == "101") {
        Serial.println("Send Data To Queue  = " + data);
        scan101();
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "102") {
        Serial.println("Send Data To Queue  = " + data);
        scan102();
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "103") {
        Serial.println("Send Data To Queue  = " + data);
        scan103();
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "104") {
        Serial.println("Send Data To Queue  = " + data);
        scan104();
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      else if (data == "105") {
        Serial.println("Send Data To Queue  = " + data);
        scan105();
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "201" ) {
        Serial.println(" Robot Play !!!");
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,50,(char*)"ROBOT START !!!",WHITE);
        xQueueSend(displayQueue, &checkplay, portMAX_DELAY);
        //check_Start = true;
        status_active = true;
        check_Loop = false;
        check_reverse == false;
        int numR = 0;
        int numL = 0;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      
      else if (data == "202") {
        Serial.println("Send Data To Queue  = " + data);
        Serial.println("Send Data To Queue  = " + data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,20,(char*)"202",WHITE);
        lcd.Printstr(10,50,(char*)"Robot Stop !!!",WHITE);
        xQueueSend(commandQueue, &data, portMAX_DELAY);
       check_Loop = false;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      
      else if (data == "203") {
        Serial.println("Send Data To Queue  = " + data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,20,(char*)"203",WHITE);
        lcd.Printstr(10,50,(char*)"Reset",WHITE);
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "204") {
        Serial.println("Send Data To Queue  = " + data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,20,(char*)"204",WHITE);
        lcd.Printstr(10,50,(char*)"Delay",WHITE);
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkreset = true;
        check_Start = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "205") {
        Serial.println("Send Data To Queue  = " + data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,20,(char*)"205",WHITE);
        lcd.Printstr(10,50,(char*)"Loop",WHITE);
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "210") {
        Serial.println("Send Data To Queue  = " + data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,20,(char*)"210",WHITE);
        lcd.Printstr(10,50,(char*)"Reverse",WHITE);
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkreset = true;
        reverse = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
       else if (data == "301") {
        Serial.println("Send Data To Queue  = " + data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,20,(char*)"301",WHITE);
        lcd.Printstr(10,50,(char*)"Sensor",WHITE);
        Serial.println("Start301");
        checkultra = true;
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "302") {
        Serial.println("Send Data To Queue  = " + data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,20,(char*)"302",WHITE);
        lcd.Printstr(10,50,(char*)"Brushoff",WHITE);
        checkbrushoff == true;
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "303") {
        Serial.println("Send Data To Queue  = " + data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,20,(char*)"303",WHITE);
        lcd.Printstr(10,50,(char*)"RGBSensor",WHITE);
        Serial.println("Start303");
        checkrgbsensor = true;
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "304") {
        Serial.println("Send Data To Queue  = " + data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,20,(char*)"304",WHITE);
        lcd.Printstr(10,50,(char*)"Red",WHITE);
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkred = true;
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "305") {
        Serial.println("Send Data To Queue  = " + data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,20,(char*)"305",WHITE);
        lcd.Printstr(10,50,(char*)"Blue",WHITE);
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkblue = true;
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "306") {
        Serial.println("Send Data To Queue  = " + data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,20,(char*)"306",WHITE);
        lcd.Printstr(10,50,(char*)"Green",WHITE);
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkgreen = true;
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "307") {
        Serial.println("Send Data To Queue  = " + data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,20,(char*)"307",WHITE);
        lcd.Printstr(10,50,(char*)"Cout",WHITE);
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "308") {
        Serial.println("Send Data To Queue  = " + data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,20,(char*)"308",WHITE);
        lcd.Printstr(10,50,(char*)"Turntable",WHITE);
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "309") {
        Serial.println("Send Data To Queue  = " + data);
        lcd.fillScreen(BLACK);
        lcd.Printstr(10,20,(char*)"309",WHITE);
        lcd.Printstr(10,50,(char*)"LDRSensor",WHITE);
        Serial.println("Start309");
        checkldr = true;
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "401") {
          lcd.fillScreen(BLACK);
          lcd.Printstr(10,20,(char*)"203",WHITE);
          lcd.Printstr(10,50,(char*)"If",WHITE);
          prev_data = data;
          Serial.println("Data =" + prev_data);
          checkreset = true;
          vTaskDelay(pdMS_TO_TICKS(100));
        }
      else if (data == "401") {
        Serial.println("Data =" + data);
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }  
      else if (data == "402") {
        prev_data = data;
        Serial.println("Data =" + prev_data);
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else if (data == "402") {
        Serial.println("Data =" + data);
        xQueueSend(commandQueue, &data, portMAX_DELAY);
        checkreset = true;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      else{
        Serial.println("No Data");
    }
      data = "";
  }
  vTaskDelay(pdMS_TO_TICKS(100));
}
}
int numQ = 0 ;
int numI = 0 ;
int numX = 0 ;
int numR = 0;
int numL = 0;
void display_data(void *parameter) {
  Create_gui();
  String data;
  int databutton = 2;
  for (;;) {
    if (check_reverse  == true){
      clearScreen();
    xQueueReceive(displayreverseQueue, &databutton, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (databutton == 1) {
      int numMessages1 = uxQueueMessagesWaiting(reverseQueue);
      numL = numMessages1;
      for (int i = 0; i < numMessages1; i++) {
        xQueueReceive(reverseQueue, &data, 0);
        // delete whitespace
        data.trim();
        Serial.println(data);
        clearScreen();
        if (data == "101") {
          clearScreen();
          play101();
          checkreset = true;
          Serial.println("Data =" + data);
          check_Forward = true;
          vTaskDelay(pdMS_TO_TICKS(4000));
        }
        else if (data == "102") {
          clearScreen();
          play102();
          checkreset = true;
          Serial.println("Data =" + data);
          check_TurnRight = true;
          vTaskDelay(pdMS_TO_TICKS(4000));
        }
        else if (data == "103") {
          clearScreen();
          play103();
          checkreset = true;
          Serial.println("Data =" + data);
          check_TurnLeft = true;
          vTaskDelay(pdMS_TO_TICKS(4000));
        }
        else if (data == "104") {
          play104();
          checkreset = true;
          Serial.println("Data =" + data);
          check_Backward = true;
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (data == "105") {
          play105();
          checkreset = true;
          Serial.println("Data =" + data);
          check_TurnAround = true;
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
        data = "";
        databutton = 2;
        vTaskDelay(pdMS_TO_TICKS(100));
    }}
    }

    else if (check_reverse  == false){
    clearScreen();
    xQueueReceive(displayQueue, &databutton, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (databutton == 1) {
      int numMessages = uxQueueMessagesWaiting(commandQueue);
      numQ = numMessages ;
      for (int i = 0; i < numMessages; i++) {
        numI = i+1 ;
        xQueueReceive(commandQueue, &data, 0);
        // delete whitespace
        data.trim();
        Serial.println(data);
        clearScreen();
         if (data == "401301") {
          prev_data2 = data;
          Serial.println("Data =" + prev_data2);
          checkreset = true;
          vTaskDelay(pdMS_TO_TICKS(2000));
        }
        else if (data == "302" && prev_data2 == "401301" && checkbrushoff == true) {
          prev_data2 = "";
          Serial.println("Data =" + prev_data2);
          Serial.println("if401301302");
          checkreset = true;
          checkreset2 = true;
          vTaskDelay(pdMS_TO_TICKS(2000));

        }
        else if (data == "101") {
          play101();
          checkreset = true;
          Serial.println("Data =" + data);
          check_Forward = true;
          if (reverse == true) {
          xQueueSendToFront(reverseQueue, &data, portMAX_DELAY);
          }
          else if (check_Loop == true) {
            xQueueSend(commandQueue, &data, portMAX_DELAY);
        }
          vTaskDelay(pdMS_TO_TICKS(5000));
        }
        else if (data == "102") {
          play102();
          checkreset = true;
          Serial.println("Data =" + data);
          check_TurnLeft = true;
          if (reverse == true) {
          xQueueSendToFront(reverseQueue, &data, portMAX_DELAY);
          }
          else if (check_Loop == true) {
            xQueueSend(commandQueue, &data, portMAX_DELAY);
            
        }
          vTaskDelay(pdMS_TO_TICKS(5000));
        }
        else if (data == "103") {
          play103();
          checkreset = true;
          Serial.println("Data =" + data);
          check_TurnRight = true;
          if (reverse == true) {
          xQueueSendToFront(reverseQueue, &data, portMAX_DELAY);
          }
          else if (check_Loop == true) {
            xQueueSend(commandQueue, &data, portMAX_DELAY);
        }
          vTaskDelay(pdMS_TO_TICKS(5000));
        }
        else if (data == "104") {
          play104();
          checkreset = true;
          Serial.println("Data =" + data);
          check_Backward = true;
          if (reverse == true) {
          xQueueSendToFront(reverseQueue, &data, portMAX_DELAY);
          }
          else if (check_Loop == true) {
            xQueueSend(commandQueue, &data, portMAX_DELAY);
        }
          vTaskDelay(pdMS_TO_TICKS(4000));
        }
        else if (data == "105") {
          play105();
          checkreset = true;
          Serial.println("Data =" + data);
          check_TurnAround = true;
          if (reverse == true) {
          xQueueSendToFront(reverseQueue, &data, portMAX_DELAY);
          }
          else if (check_Loop == true) {
            xQueueSend(commandQueue, &data, portMAX_DELAY);
          }
          vTaskDelay(pdMS_TO_TICKS(4000));
        }
        else if (data == "202") {
          lcd.fillScreen(BLACK);
          lcd.Printstr(10,20,(char*)"202",WHITE);
          lcd.Printstr(10,50,(char*)"Stop",WHITE);
          checkreset = true;
          Serial.println("Data =" + data);
          check_Stop = true;
          if (check_Loop == true) {
            xQueueSend(commandQueue, &data, portMAX_DELAY);
        }
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (data == "203") {
          lcd.fillScreen(BLACK);
          lcd.Printstr(10,20,(char*)"203",WHITE);
          lcd.Printstr(10,50,(char*)"Reset",WHITE);
          checkreset = true;
          Serial.println("Data =" + data);
          check_Reset = true;
          if (check_Loop == true) {
            xQueueSend(commandQueue, &data, portMAX_DELAY);
        }
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (data == "204") {
          lcd.fillScreen(BLACK);
          lcd.Printstr(10,20,(char*)"204",WHITE);
          lcd.Printstr(10,50,(char*)"Delay",WHITE);
          checkreset = true;
          Serial.println("Data =" + data);
          check_Delay = true;
          if (check_Loop == true) {
            xQueueSend(commandQueue, &data, portMAX_DELAY);
        }
          vTaskDelay(pdMS_TO_TICKS(5000));
        }
        else if (data == "205") {
          lcd.fillScreen(BLACK);
          lcd.Printstr(10,20,(char*)"205",WHITE);
          lcd.Printstr(10,50,(char*)"Loop",WHITE);
          checkreset = true;
          Serial.println("Data =" + data);
          check_Loop = true;
          if (check_Loop == true) {
            xQueueSend(commandQueue, &data, portMAX_DELAY);
            numI = i+1 ;
            numX = numI;
        }
          vTaskDelay(pdMS_TO_TICKS(3000));
        }
        else if (data == "210") {
          lcd.fillScreen(BLACK);
          lcd.Printstr(10,20,(char*)"205",WHITE);
          lcd.Printstr(10,50,(char*)"Reverse",WHITE);
          checkreset = true;
          Serial.println("Data =" + data);
          rotate180();
          numR = 0;
          numL = 0;
          check_reverse = true;
          vTaskDelay(pdMS_TO_TICKS(3000));
        }
        else if (data == "304") {
          lcd.fillScreen(BLACK);
          lcd.Printstr(10,20,(char*)"304",WHITE);
          lcd.Printstr(10,50,(char*)"Red Start",WHITE);
          checkreset = true;
          Serial.println("Data =" + data);
          Serial.println("Start304");
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (data == "305") {
          lcd.fillScreen(BLACK);
          lcd.Printstr(10,20,(char*)"305",WHITE);
          lcd.Printstr(10,50,(char*)"Blue Start",WHITE);
          checkreset = true;
          Serial.println("Data =" + data);
          Serial.println("Start305");
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (data == "306") {
          lcd.fillScreen(BLACK);
          lcd.Printstr(10,20,(char*)"306",WHITE);
          lcd.Printstr(10,50,(char*)"Green Start",WHITE);
          checkreset = true;
          Serial.println("Data =" + data);
          Serial.println("Start306");
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (data == "307") {
          lcd.fillScreen(BLACK);
          lcd.Printstr(10,20,(char*)"307",WHITE);
          lcd.Printstr(10,50,(char*)"Count Start",WHITE);
          checkreset = true;
          Serial.println("Data =" + data);
          Serial.println("Start307");
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (data == "308") {
          lcd.fillScreen(BLACK);
          lcd.Printstr(10,20,(char*)"308",WHITE);
          lcd.Printstr(10,50,(char*)"Turntable Start",WHITE);
          checkreset = true;
          Serial.println("Data =" + data);
          Serial.println("Start308");
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (data == "309") {
          lcd.fillScreen(BLACK);
          lcd.Printstr(10,20,(char*)"308",WHITE);
          lcd.Printstr(10,50,(char*)"LDRSensor Start",WHITE);
          checkreset = true;
          Serial.println("Data =" + data);
          Serial.println("Start309");
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (data == "401") {
          Serial.println("Data =" + data);
          checkreset = true;
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (data == "402") {
          Serial.println("Data =" + data);
          checkreset = true;
          vTaskDelay(pdMS_TO_TICKS(1000));
        }

        data = "";
        databutton = 2;
        

  }

}
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    }}

void read_sensor_values(){
  uint16_t position = qtr.readLineBlack(sensorValues);
   for (uint8_t i = 0; i < SensorCount; i++)
  {
    if (sensorValues[i] == 1000) {
     sensorValues[i] = 1 ;
    }
    else {
     sensorValues[i] = 0 ;
    }
    //Serial.print(sensorValues[i]);
    //Serial.print('\t');
  }
  //Serial.println(position);
  
 
  // ตรวจสอบสถานะของเซ็นเซอร์และปรับการเคลื่อนที่ของหุ่นยนต์
   if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) {
    // หยุด [1 1 1 1 1 1 ]
    //stopMoving();
     error= 13 ;
  }
  else if ( sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 0 && sensorValues[5] == 0) {
    // เดินตรงไปข้างหน้า [ 0 0 1 1 0 0 ]
    //forward();
    error= 0;
    //Serial.println("เดินหน้า");
  }
   
   else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0) {
    // หยุด [0 0 0 0 0 0 ]
    //stopMoving();
     error= 13 ;
  }
  
  //เคลื่อนที่ขวา
  else if (sensorValues[0] == 0 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 0 && sensorValues[5] == 0) {
    // ขวา1  [ 0 1 1 1 0 0 ]
    //Right(); 
    error= 1 ;
    
  }
   else if (sensorValues[0] == 0 && sensorValues[1] == 1 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0) {
    // ขวา2 [ 0 1 0 0 0 0 ]
    //Right();
    error= 2 ;
    
  }
   else if (sensorValues[0] == 0 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0) {
    // ขวา3 [ 0 1 1 0 0 0 ]
    //Right();
    error= 3 ;
    
  }
   else if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0) {
    // ขวา4 [ 1 1 1 0 0 0 ]
    //Right();
    error= 4 ;
    
  }
   else if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0) {
    // ขวา5 [ 1 1 0 0 0 0 ]
    //Right();
    error= 5 ;
    
   }
     else if (sensorValues[0] == 1 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0) {
    // ขวา6 [ 1 0 0 0 0 0 ]
    //Right();
    error= 6 ;
  
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) {
    // เลี้ยวขวา [ 0 0 1 1 1 1 ]
    //turnRight();
    error= 11 ;
  }

  // เคลื่อนที่ซ้าย
 
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 0) {
    // ซ้าย [ 0 0 1 1 1 0 ]
    //Left();
    error= -1 ;
  }
   else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 1 && sensorValues[5] == 0) {
    // ซ้าย1 [ 0 0 0 0 1 0 ]
    //Left();
    error= -2 ;
  }
   else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 0) {
    // ซ้าย2 [ 0 0 0 1 1 0 ]
    //Left();
    error= -3 ;
  }
   else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) {
    // ซ้าย3 [ 0 0 0 1 1 1 ]
    //Left();
    error= -4 ;
  }
   else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 1 && sensorValues[5] == 1) {
    // ซ้าย4 [ 0 0 0 0 1 1 ]
    //Left();
    error= -5 ;
  }
   else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 1) {
    // ซ้าย5 [ 0 0 0 0 0 1 ]
    //Left();
    error= -6 ;
  }
   else if ( sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 0 && sensorValues[5] == 0) {
    // เลี้ยวซ้าย [ 1 1 1 1 0 0 ]
    //turnLeft();
     error= 12 ;
  }
  vTaskDelay(pdMS_TO_TICKS(100));
}

void Forward() {
   analogWrite(PWM1, initial_motor_speed);
   digitalWrite(leftMotorPin1, HIGH);
   digitalWrite(leftMotorPin2, LOW);
   analogWrite(PWM2, initial_motor_speed);
   digitalWrite(rightMotorPin1, LOW);
   digitalWrite(rightMotorPin2, HIGH);
   delay(200);
}

void Left() {

  analogWrite(PWM1, initial_motor_speed);
  digitalWrite(leftMotorPin1,HIGH);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(PWM2, initial_motor_speed);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  while (sensorValues[0]==0){
    read_sensor_values();
  }
  while (sensorValues[3]==1){
    read_sensor_values();
  } 
  stopMoving();
}
void Right() {
  
  analogWrite(PWM1, initial_motor_speed);
  digitalWrite(leftMotorPin1,LOW);
  digitalWrite(leftMotorPin2, HIGH);
  analogWrite(PWM2, initial_motor_speed);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  
     while (sensorValues[5]==0){
    read_sensor_values();
    }
      while (sensorValues[2]==1){
    read_sensor_values();
    }
 stopMoving();
  }

void stopMoving() {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
}
void Back(){
   analogWrite(PWM1, initial_motor_speed);
   digitalWrite(leftMotorPin1, LOW); 
   digitalWrite(leftMotorPin2, HIGH);
   analogWrite(PWM2, initial_motor_speed);
   digitalWrite(rightMotorPin1, HIGH);  
   digitalWrite(rightMotorPin2, LOW);
   delay(500);
}

void Backward() {
      int left_motor_speed = initial_motor_speed - PID_value;
      int right_motor_speed = initial_motor_speed + PID_value;
      // จำกัดค่าความเร็ว
      left_motor_speed = constrain(left_motor_speed, 0, 255);
      right_motor_speed = constrain(right_motor_speed, 0, 255);
      // สั่งให้มอเตอร์เคลื่อนที่
      analogWrite(PWM2,initial_motor_speed+PID_value); //Left Motor Speed
      analogWrite(PWM1,initial_motor_speed-PID_value); //Right Motor Speed
      digitalWrite(leftMotorPin1,LOW);
      digitalWrite(leftMotorPin2,HIGH);
      digitalWrite(rightMotorPin1,HIGH);
      digitalWrite(rightMotorPin2,LOW);
}
void rotate180() {
  Serial.println(" rotate180 ");
  // ตั้งค่าความเร็วในการหมุน
  int rotateSpeed = 100;

  // หยุดการเคลื่อนที่ของมอเตอร์ทั้งสอง
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);

  // หมุนไปทางที่ต้องการ 180 องศา
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);

  // รอให้หุ่นยนต์หมุนถึงมุม 180 องศา
  read_sensor_values();
  Serial.println(" check white ");
  while (sensorValues[0]==0){
    read_sensor_values();
  }
  Serial.println(" check black ");
  read_sensor_values();
  while (sensorValues[4]==1){
    read_sensor_values();
  }
  stopMoving();
  
  // หยุดการหมุน
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
}

void calculate_pid()
{
  if (error == 13) {
     PID_value = 1313;
    }
  else if (error == 11) {
     //turnRight();
     PID_value = 0;
    }
  else if (error == 12) {
     //turnLeft();
     PID_value = 0;
    }
  else {
      P = error;
      I = I + previous_I;
      D = error-previous_error;
      PID_value = (Kp*P) + (Ki*I) + (Kd*D);  //float Kp=0,Ki=0,Kd=0;
      previous_I=I;
      previous_error=error;
    }
}
void motor_control() {
   // ตรวจสอบค่า PID_value เพื่อควบคุมมอเตอร์
   if (PID_value == 1313) {
      stopMoving();
   }
   else {
      // คำนวณความเร็วของมอเตอร์
      int left_motor_speed = initial_motor_speed + PID_value;
      int right_motor_speed = initial_motor_speed - PID_value;
      // จำกัดค่าความเร็ว
      left_motor_speed = constrain(left_motor_speed, 0, 255);
      right_motor_speed = constrain(right_motor_speed, 0, 255);
      // สั่งให้มอเตอร์เคลื่อนที่
      analogWrite(PWM1,initial_motor_speed-PID_value); //Left Motor Speed
      analogWrite(PWM2,initial_motor_speed+PID_value); //Right Motor Speed
      digitalWrite(leftMotorPin1,HIGH);
      digitalWrite(leftMotorPin2,LOW);
      digitalWrite(rightMotorPin1,LOW);
      digitalWrite(rightMotorPin2,HIGH);
   }
}
void clearScreen() {
  lcd.fillScreen(BLACK);
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

void motor_control_task(void *parameter) {
  unsigned long Time = 0;
  read_sensor_values();
  for (;;) {
    if (check_Forward == true){
      Serial.println(" moveRobotForward ");
      error = 0;
      Forward();
      delay(100);
      while (error != 13 ){
        read_sensor_values();
        calculate_pid();
        motor_control();
        if (error == 13){
          stopMoving();
          Serial.println(" Stop ");
          check_Forward = false;
          numX = numX+1 ;
          numR = numR+1 ;
          finish_check();
          vTaskDelay(pdMS_TO_TICKS(100));}
    }
    }
    else if (check_TurnRight == true){
      Serial.println(" Turn Right ");
      error = 0;
      Forward();
      delay(100);
      while (error != 13){
        read_sensor_values();
        calculate_pid();
        motor_control();
        vTaskDelay(pdMS_TO_TICKS(10));
        if (error == 13 && is_crossing == true){
          stopMoving();
          Serial.println(" Stop ");
          check_TurnRight = false;
          numX = numX+1 ;
          numR = numR+1 ;
          is_crossing = false;
          finish_check();
          vTaskDelay(pdMS_TO_TICKS(500));
          }
        else if (error == 11 && is_crossing == false){
          Serial.println(" Right ");
          Forward();
          delay(100);
          stopMoving();
          read_sensor_values();
          Right();
          is_crossing = true;
          vTaskDelay(pdMS_TO_TICKS(500));
          //delay(1500);
        }
        else if (error == 13 && is_crossing == false){
          Forward();
          delay(90);
          stopMoving();
          Serial.println(" Stop Right ");
          read_sensor_values();
          Right();
          stopMoving();
          is_crossing = true;
          vTaskDelay(pdMS_TO_TICKS(500));
          }
    }
  }
    else if (check_TurnLeft == true){
      Serial.println(" Turn Left ");
      error = 0;
      Forward();
      delay(100);
      while (error != 13){
        read_sensor_values();
        calculate_pid();
        motor_control();
        vTaskDelay(pdMS_TO_TICKS(10));
        if (error == 13 && is_crossing == true  ){
          stopMoving();
          Serial.println(" Stop ");
          check_TurnLeft = false;
          numX = numX+1 ;
          numR = numR+1 ;
          is_crossing = false;
          finish_check();
          vTaskDelay(pdMS_TO_TICKS(500));
          }
        else if (error == 12  && is_crossing == false ){
          stopMoving();
          Serial.println("  Left ");
          //left_check = true;
           Forward();
          delay(90);
          stopMoving();
          Serial.println(" Stop  Left ");
           read_sensor_values();
           Left();
          //Back();
          //delay(60);
          stopMoving();
          is_crossing = true;
          vTaskDelay(pdMS_TO_TICKS(500));
          //delay(1500);
        }
        else if (error == 13 && is_crossing == false){
          Forward();
          delay(90);
          stopMoving();
          Serial.println(" Stop  Left ");
           read_sensor_values();
           Left();
          //Back();
          //delay(60);
          stopMoving();
          is_crossing = true;
          vTaskDelay(pdMS_TO_TICKS(500));
          }
    }
    }
    else if (check_Backward == true) {
      Serial.println(" Backward ");
      error = 0;
      Back(); // เรียกใช้ฟังก์ชันถอยหลัง
      while (error != 13){
        read_sensor_values();
        calculate_pid();
        Backward();
        if (error == 13){
          stopMoving();
          Serial.println(" Stop ");
          check_Backward = false;
          numX = numX+1 ;
          numR = numR+1 ;
          finish_check();
          vTaskDelay(pdMS_TO_TICKS(100));}
    }
     /* while (error != 13){
        read_sensor_values();
        calculate_pid();
        Backward();
        vTaskDelay(pdMS_TO_TICKS(100)); 
        if (error == 13){
          stopMoving();
          Serial.println(" Stop ");
          check_Backward = false;
          vTaskDelay(pdMS_TO_TICKS(100));}
    }*/
    }
    else if (check_TurnAround == true){
      Serial.println(" Turn Around ");
      error = 0;
      Forward();
      delay(500);
      rotate180();
      check_TurnAround = false;
      numX = numX+1 ;
      numR = numR+1 ;
      finish_check();
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (check_Stop == true) {
      Serial.println(" Stop Queue ");
      stopMoving();
      vTaskSuspend(taskHandle); // หยุดการทำงานของ Task ที่ดึงข้อมูลจากคิว
      if (check_Start == true ){
        Serial.println(" Start Qeueu");
        vTaskResume(taskHandle); // เริ่มการทำงานของ Task ที่ดึงข้อมูลจากคิวอีกครั้ง
        check_Stop = false;
        numX = numX+1 ;
        check_Start == false;
        finish_check();
        vTaskDelay(pdMS_TO_TICKS(100));}
    }
    else if (check_Reset == true){
      Serial.println(" Robot Reset ");
      xQueueReset(commandQueue);
      xQueueReset(displayQueue);
      xQueueReset(reverseQueue);
      xQueueReset(displayreverseQueue);
      check_Loop = false;
      numX = numX+1 ;
      check_Reset = false;
      finish_check();
      vTaskDelay(pdMS_TO_TICKS(100));
      }
    else if (check_Delay == true){
      Serial.println(" Robot Delay ");
      delay(5000);
      check_Delay = false;
      numX = numX+1 ;
      finish_check();
      vTaskDelay(pdMS_TO_TICKS(100));
      }
    else if (ultra_stop == true ){
      stopMoving();
      Serial.println(" Stop Queue ");
      vTaskSuspend(taskHandle); // หยุดการทำงานของ Task ที่ดึงข้อมูลจากคิว
      stopMoving();
      Serial.println("brushoff_On");
      
      }
    else if (brushoff_Ok == true){
      vTaskResume(taskHandle); // เริ่มการทำงานของ Task ที่ดึงข้อมูลจากคิวอีกครั้ง
      ultra_stop = false ; 
      brushoff_Ok = false;
    }
     else if (check_Loop == true){
      //
       }
    else if (check_reverse  == true){
      xQueueSend(displayreverseQueue, &checkplayreverse, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(100));
      }      
        delay(500);
      } 
      }
 

void ItemTask(void * pvParameters) {
  for (;;) {
    if (checkultra == true){
      //////////////////////
      } 
  else if (checkultra == true && checkbrushoff == true){

      }

   delay(550);
}
}
void finish_check(){
  // ตรวจสอบคิว
  int queueLength = uxQueueMessagesWaiting(commandQueue);
    //Serial.println(numQ);
    //Serial.println(numX);
  // Serial.println(numQ);
  // Serial.println(numI);
  // ดำเนินการตามจำนวนข้อความที่รอในคิว
  if (queueLength == 0) {
    Serial.println("finish");
    Create_gui();

  }
  if (check_Loop == true && numX == numQ ){
     Serial.println("loop again");
     check_Loop == false;
    xQueueSend(displayQueue, &checkplay, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  if (numL == 0  && check_reverse == true){

     Serial.println("reverse finish");
    check_reverse = false;
    reverse = false;
    check_Reset = true ;
    int numR = 0;
    int numL = 0;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  if (numL == numR  && check_reverse == true){

     Serial.println("reverse finish");
    check_reverse = false;
    reverse = false;
    check_Reset = true ;
    int numR = 0;
    int numL = 0;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
      
void scan101() {
  //Screen Forward 
  lcd.fillScreen(BLACK);
  lcd.Printstr(10,20,(char*)"      SCAN       ",WHITE);
  lcd.Printstr(10,60,(char*)"  Move Forward  ",WHITE);
  lcd.Printstr(10,80,(char*)"     ......      ",BLUE);
}
void scan102() {
  //Screen Turn Left 
  lcd.fillScreen(BLACK);
  lcd.Printstr(10,20,(char*)"      SCAN       ",WHITE);
  lcd.Printstr(10,60,(char*)"    Turn Left    ",WHITE);
  lcd.Printstr(10,80,(char*)"     ......      ",BLUE);
}
void scan103() {
  //Screen Turn Right
  lcd.fillScreen(BLACK);
  lcd.Printstr(10,20,(char*)"      SCAN       ",WHITE);
  lcd.Printstr(10,60,(char*)"   Turn Right    ",WHITE);
  lcd.Printstr(10,80,(char*)"     ......      ",BLUE);
}
void scan104() {
  //Screen Bcakward
  lcd.fillScreen(BLACK);
  lcd.Printstr(10,20,(char*)"      SCAN       ",WHITE);
  lcd.Printstr(10,60,(char*)"    Bcakward     ",WHITE);
  lcd.Printstr(10,80,(char*)"     ......      ",BLUE);
}
void scan105() {
  //Screen Turn around 
  lcd.fillScreen(BLACK);
  lcd.Printstr(10,20,(char*)"      SCAN       ",WHITE);
  lcd.Printstr(10,60,(char*)"   Turn around   ",WHITE);
  lcd.Printstr(10,80,(char*)"     ......      ",BLUE);
}

////// PLAY //////
void play101() {
  // Start 
  lcd.fillScreen(BLACK);
  lcd.Printstr(10,20,(char*)"      PLAT       ",WHITE);
  lcd.Printstr(10,60,(char*)"  Move Forward  ",WHITE);
}
void play102() {
  //Screen Turn Left 
  lcd.fillScreen(BLACK);
  lcd.Printstr(10,20,(char*)"      PLAY       ",WHITE);
  lcd.Printstr(10,60,(char*)"    Turn Left    ",WHITE);
}
void play103() {
  // Turn Right
  lcd.fillScreen(BLACK);
  lcd.Printstr(10,20,(char*)"      PLAY       ",WHITE);
  lcd.Printstr(10,60,(char*)"   Turn Right    ",WHITE);
}
void play104() {
  // Bcakward
  lcd.fillScreen(BLACK);
  lcd.Printstr(10,20,(char*)"      PLAY       ",WHITE);
  lcd.Printstr(10,60,(char*)"    Bcakward     ",WHITE);
}
void play105() {
  // Turn around 
  lcd.fillScreen(BLACK);
  lcd.Printstr(10,20,(char*)"      SCAN       ",WHITE);
  lcd.Printstr(10,60,(char*)"   Turn around   ",WHITE);
}
