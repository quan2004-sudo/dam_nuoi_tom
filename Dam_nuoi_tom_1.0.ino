/*  gửi đi 9 thông số 
 mode  chế độ thủ công tự động
  state1,  trạng thái quạt
   state2, trạng thái valve
    h,  độ ẩm
    t, nhiệt độ
     ppm,  chất lượng kk
      pHValue,  độ pH
      canhbao,  cảnh báo gì đó
      v*/




#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MQ135.h"  //Thêm thư viện
#include "DHT.h"
#include "DFRobot_PH.h"
#include <EEPROM.h>


uint32_t last_check = 0;
uint32_t lastRead = 0;
uint32_t lastSendData = 0;

const int DHTPIN = 5;       //Đọc dữ liệu từ DHT11 ở chân 2 trên mạch Arduino
const int DHTTYPE = DHT11;  //Khai báo loại cảm biến, có 2 loại là DHT11 và DHT22

const int RL_EN = 10;  // băm xung
const int R_PWM = 11;  // điều khiển chiều quay
const int L_PWM = 12;  // điều khiển chiều quay
const int relay = 9; //
//const int relay = 8;
float data1[3];

float v, h, t, ppm, d = 30, r = 30, c = 30;  // dai:30cm ,rong:30 ,cao 30 // v luong nuoc con lai trong binh chua // h ,t : do am nhiet do
float pHhigh = 8.5, pHlow = 7.5;

#define PH_PIN A2
float voltage,pHValue,temperature = 25;
DFRobot_PH ph;

const int bientro = A1;
const int PIN_MQ135 = A3;  //Khai báo pin nối với chân AO
const int SensorPin = A2;  //pH meter Analog output to Arduino Analog Input 2
const int btn0 = 2;        // nut nhan dao trang thai che do hoat dong
const int btn2 = 3;        // nut nhan mo dong co tao oxy o che do thu cong // gan vao chan ngat
const int btn1 = 18;       // nut nhan mo van dien tu o che do thu cong // gan vao chan ngat
const int battery = A0;
const int trig = 6;  // chân trig của HC-SR04
const int echo = 7;  // chân echo của HC-SR04

 bool mode = false, check, state1 = false, state2 = false, canhbao = false;  // trang thai che do

#define Offset 0.00  //deviation compensate

 //times of collection
  //Store the average value of the sensor feedback
int tocdo;

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
MQ135 mq135_sensor = MQ135(PIN_MQ135);  //Khai báo đối tượng thư viện




byte degree[8] = {
  0B01110,
  0B01010,
  0B01110,
  0B00000,
  0B00000,
  0B00000,
  0B00000,
  0B00000
};

void setup(void) {
 
  Serial.begin(9600);
    ph.begin();
  Serial.println("pH meter ok");  //Test the serial monitor
  dht.begin();
  lcd.init();
  lcd.backlight();
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2,  INPUT_PULLUP);
  pinMode(btn0,  INPUT_PULLUP);
  attachInterrupt(1, chonchedo, RISING);   // chan 2 tren arduino mega
  attachInterrupt(0, trangthai1, RISING);  // chan 3 tren arduino mega
  attachInterrupt(5, trangthai2,  RISING);  // chan 18 tren arduino mega
  pinMode(RL_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);

  digitalWrite(R_PWM, HIGH);
  digitalWrite(L_PWM, HIGH);  //STOP

  pinMode(trig, OUTPUT);  // chân trig sẽ phát tín hiệu
  pinMode(echo, INPUT);   // chân echo sẽ nhận tín hiệu

  pinMode(bientro, INPUT);
  pinMode(relay, OUTPUT);
  lcd.createChar(1, degree);
}


void loop(void) {  /////////////////////////////////////////////////////////////////////////
                   //   DOC GIA TRI CAM BIEN pH
 temperature =  readTemperature() ;

   static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U){                  //time interval: 1s
        timepoint = millis();
        //temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
        voltage = analogRead(PH_PIN)/1024.0*5000;  // read the voltage
        pHValue = ph.readPH(voltage,temperature);  // convert voltage to pH with temperature compensation=
        Serial.print("^C  pH:");
        Serial.println(pHValue,2);
    }
    ph.calibration(voltage,temperature);           // calibration process by Serail CMD


  ////////////////////////////////////////////////////////////////////////////////////


  if (mode == false)  // che do tu dong
  {
    Serial.println("mode tu dong");
    if (pHValue > 0 && pHValue < 7.5) {
      digitalWrite(relay, HIGH);
      state2 = true;
    }
    if (pHValue >= 7.5 && pHValue <= 8.5) {
      digitalWrite(relay, LOW);
      state2 = false;
    }
    mode_auto();
  } else  // che do thu cong
  {
       Serial.println("mode thu cong");
    mode_manual();
 
  }


  if (millis() - lastSendData > 2000) {
    update_dulieu();
    Serial.println("upload done ");

    lastSendData = millis();
  }
}


double avergearray(int* arr, int number) {  // do va tinh trung binh cac gia tri do duoc
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5) {
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0];
      max = arr[1];
    } else {
      min = arr[1];
      max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;  //arr<min
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;  //arr>max
          max = arr[i];
        } else {
          amount += arr[i];  //min<=arr<=max
        }
      }  //if
    }    //for
    avg = (double)amount / (number - 2);
  }  //if
  return avg;
}



void readSensor() {
  ppm = mq135_sensor.getPPM();  //Đọc giá trị ppm
  Serial.print("MQ135: ");
  Serial.print(ppm);
  Serial.println("ppm");
  delay(500);
  h = dht.readHumidity();     //Đọc độ ẩm
  t = dht.readTemperature();  //Đọc nhiệt độ

  Serial.print("Nhiet do: ");
  Serial.println(t);  //Xuất nhiệt độ
  Serial.print("Do am: ");
  Serial.println(h);                                  //Xuất độ ẩm
  float V_battery = 0.0148356 * analogRead(battery);  //((R1 + R2) / (R2 * 1023)) * analogRead(battery)
  Serial.print("dien ap");
  Serial.println(V_battery);
}



void mucnuoc() {
  unsigned long duration;  // biến đo thời gian
  int distance;            // biến lưu khoảng cách

  /* Phát xung từ chân trig */
  digitalWrite(trig, 0);  // tắt chân trig
  delayMicroseconds(2);
  digitalWrite(trig, 1);  // phát xung từ chân trig
  delayMicroseconds(5);   // xung có độ dài 5 microSeconds
  digitalWrite(trig, 0);  // tắt chân trig

  /* Tính toán thời gian */
  // Đo độ rộng xung HIGH ở chân echo.
  duration = pulseIn(echo, HIGH);
  // Tính khoảng cách đến vật.
  distance = int(duration / 2 / 29.412);

  /* In kết quả ra Serial Monitor */
  Serial.print(distance);
  Serial.println("cm");
  delay(200);

  v = (c - distance) * d * r;

  Serial.print("the tich con lai");
  Serial.print(v / 1000);
  Serial.print(" lit");
  if (distance > 25) {
    Serial.print(" canh bao sap het dung dich ");
  }
}



void display() {

  lcd.clear();
  lcd.setCursor(0, 0);lcd.print("H:");lcd.print(round(h));lcd.print("%");
  lcd.setCursor(0, 1); lcd.print("T:");lcd.print(round(t));lcd.write(1);lcd.print("C");
  lcd.setCursor(6, 0); lcd.print("MQ:");lcd.print(ppm);lcd.print("ppm");
  lcd.setCursor(8, 1); lcd.print("pH:"); lcd.print(pHValue);
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Mode:");
  if (mode == false) {
    lcd.print("auto");
  } else {
    lcd.print("manual");
  }
  lcd.setCursor(0, 1); lcd.print("Fan:");
  if (state1 == true) {
    lcd.print("ON");
  } else {
    lcd.print("OFF");
  }
  lcd.setCursor(8, 1);lcd.print("val:");
  if (state2 == true) {
    lcd.print("ON");
  } else {
    lcd.print("OFF");
  }
  delay(1000);
}

void mode_manual() {
  readSensor();
  mucnuoc();
  display();
  check = true;
  canhbao = false;
  if (state1 == true)  // bat dong co
  {
    int val = analogRead(bientro);
     tocdo = map(val, 680, 840, 0, 255);
     if (tocdo <=0)   tocdo = 0;
     else if (tocdo>=255) tocdo =255;
    digitalWrite(R_PWM, HIGH);
    digitalWrite(L_PWM, LOW);  //clockwise
    analogWrite(RL_EN, tocdo);
    Serial.print("bat dong co"); Serial.print("\t tocdo :");  Serial.println(tocdo);


  } else  // tat dong co
  {
    digitalWrite(R_PWM, HIGH);
    digitalWrite(L_PWM, HIGH);  //clockwise
    analogWrite(RL_EN, 0);
    Serial.println("tat dong co");
  }
  if (state2 == true) {
    digitalWrite(relay, HIGH);
    Serial.print("xa dung dich");

  } else {
    digitalWrite(relay, LOW);
    Serial.println("ngung xa dung dich");
  }
  
}

void mode_auto() {
  readSensor();
  mucnuoc();
  display();


  check = false;
  if (h > 50 && t > 30) {
    Serial.println("bat dong co");
    digitalWrite(R_PWM, HIGH);
    digitalWrite(L_PWM, LOW);  //clockwise
    analogWrite(RL_EN, 200);
    state1 = true;

  } else {
    Serial.println("tat dong co");
    digitalWrite(R_PWM, LOW);
    digitalWrite(L_PWM, LOW);  //clockwise
    analogWrite(RL_EN, 200);
    state1 = false;
  }

  if (ppm > 50) { canhbao = true; }  // canhr bao len dien thoai
  else {
    canhbao = false;
  }
}

void chonchedo() {
  mode = !mode;
  state1 = false;
  state2 = false;
}

void trangthai1() {
  if (check) { state1 = !state1; }
}

void trangthai2() {
  if (check) { state2 = !state2; }
}


void update_dulieu() {


  float data[9] = { mode, state1, state2, h, t, ppm, pHValue, canhbao, v };  // Dữ liệu của 5  biến
  Serial3.write((byte*)data, sizeof(data));                                  // Gửi dữ liệu qua UART
  delay(100);


  ////////////////////////////////
  if (Serial3.available() >= sizeof(float) * 3) {  // Đọc dữ liệu khi có đủ dữ liệu cho 3 phần tử float ////

    Serial3.readBytes((byte*)data1, sizeof(data1));  // Đọc dữ liệu
    Serial.print("Received data: ");
    for (int i = 0; i < 3; i++) {
      Serial.print(data1[i]);
      Serial.print(" ");
    }
    Serial.println();
    mode = data1[0];
    state1 = data1[1];
    state2 = data1[2];
  }
}


float readTemperature()
{
  t = dht.readTemperature();
  return t ;
}