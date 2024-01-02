

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <EEPROM.h>

#include <SoftwareSerial.h>
uint32_t last_check = 0;
uint32_t lastRead = 0;
uint32_t lastSendData = 0;

ESP8266WiFiMulti wifiMulti;
SocketIOclient socketIO;

SoftwareSerial mySerial(D4, D3);  // Tạo giao diện UART ảo với chân RX là D5, chân TX là D6

float v, h, t, ppm, pHValue;
bool mode, state1, state2, mode_1, state1_1, state2_2;

void socketIOEvent(socketIOmessageType_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case sIOtype_DISCONNECT:
      Serial.printf("[IOc] Disconnected!\n");
      break;
    case sIOtype_CONNECT:
      Serial.printf("[IOc] Connected to url: %s\n", payload);

      // join default namespace (no auto join in Socket.IO V3)
      socketIO.send(sIOtype_CONNECT, "/");
      break;
    case sIOtype_EVENT:
      Serial.printf("[IOc] get event: %s\n", payload);
      break;
    case sIOtype_ACK:
      Serial.printf("[IOc] get ack: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_ERROR:
      Serial.printf("[IOc] get error: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_EVENT:
      Serial.printf("[IOc] get binary: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_ACK:
      Serial.printf("[IOc] get binary ack: %u\n", length);
      hexdump(payload, length);
      break;
  }
}

void setup() {
  Serial.begin(9600);     // Baud rate của UART với Serial Monitor
  mySerial.begin(57600);  // Baud rate của UART
  Serial.println();

  if (WiFi.getMode() & WIFI_AP) {
    WiFi.softAPdisconnect(true);
  }
  wifiMulti.addAP("American Study", "66668888");
  wifiMulti.addAP("thungracthongminh", "66668888");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  //ket noi server
  //  socketIO.begin("http://localhost:3002",3002, /socket.io/?EIO=4);  // /socket.io/?EIO=4
  socketIO.begin("smtrash.com", 80, "/socket.io/?EIO=4");  // /socket.io/?EIO=4
  socketIO.onEvent(socketIOEvent);
 //  socketIO.on("data_from_server", onDataReceived);
}



void loop() {
  socketIO.loop();
  float data1[3] = { mode_1, state1_1, state2_2 };  // Dữ liệu của 3
  mySerial.write((byte *)data1, sizeof(data1));     // Gửi dữ liệu qua UART
  delay(100);

  if (mySerial.available() >= sizeof(float) * 9) {  // Đọc dữ liệu khi có đủ dữ liệu cho 5 phần tử float ////
    float data[8];
    mySerial.readBytes((byte *)data, sizeof(data));  // Đọc dữ liệu
    Serial.print("Received data: ");
    for (int i = 0; i < 9; i++) {
      Serial.print(data[i]);
      Serial.print(" ");
      socketIO.loop();
    }
    Serial.println();
    mode = data[0];
    state1 = data[1];
    state2 = data[2];
    h = data[3];
    t = data[4];
    ppm = data[5];
    phValue = data[6];
    canhbao = data[7];
    v = data[8];
    socketIO.loop();
    Serial.print("mode:");
    Serial.print(mode);
    Serial.print("\t Fan:");
    Serial.print(state1);
    socketIO.loop();
    Serial.print("\t Valve:");
    Serial.print(state2);
    Serial.print("\t Hum:");
    Serial.print(h);
    socketIO.loop();
    Serial.print("\t Temp:");
    Serial.print(t);
    socketIO.loop();
    Serial.print("\t Air:");
    Serial.print(ppm);
    Serial.print("\t phValue:");
    Serial.print(phValue);
    Serial.print("\t canhbao:");
    Serial.print(canhbao);
    Serial.print("\t V_conlai:");
    Serial.println(v);

    if (millis() - lastSendData > 5000) {
      sendData();
      Serial.println("upload done ");
      socketIO.loop();
      lastSendData = millis();
    }
  }
}

void sendData() {
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  array.add("message");
  JsonObject param1 = array.createNestedObject();

  param1["mode"] = String(mode);
  param1["fan"] = String(state1);
  param1["valve"] = String(state2);
  param1["hum"] = String(h);
  param1["temp"] = String(t);
  param1["MQ135"] = String(ppm);
  param1["pH"] = String(pHValue);
  param1["canhbao"] = String(canhbao);
  param1["thetich"] = String(v);

  String output;
  serializeJson(doc, output);
  socketIO.sendEVENT(output);
  Serial.println(output);
  delay(20);
}


