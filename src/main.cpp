#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
#define MAX485_DE D0     // GPIO16
#define MAX485_RE_NEG D1 // GPIO5

#define MAX485_DE2 D6     // GPIO12
#define MAX485_RE_NEG2 D7 // GPIO13

#define RXD01 4  // D2
#define TXD01 0  // D3
#define RXD02 2  // D4
#define TXD02 14 // D5

/*ADD YOUR PASSWORD BELOW*/
//const char *ssid = "such a person";
//const char *password = "zidanedane";
const char *ssid = "iMats";
const char *password = "adminmavens";
//const char *ssid = "realme 2 Pro";
//const char *password = "sembarang123";

const char *mqtt_server = "118.98.64.212";
const char *userBroker = "admin";
const char *passBroker = "adminmavens";

const int mpptBaudRate = 115200;
const int usBaudRate = 9600;
int timerTask1, timerTask2, timerTask3;
int battChargeCurrent, battDischargeCurrent, battOverallCurrent, battChargePower;
int bvoltage, ctemp, btemp, bremaining, lpower, lcurrent, pvvoltage, pvcurrent, pvpower;
int stats_today_pv_volt_min, stats_today_pv_volt_max;
uint8_t result;
bool rs485DataReceived = true;
bool loadPoweredOn = true;

unsigned long startTime, currentTime, currentTime2, previousMillis;

String namaData[] = {" WL ", " BR ", "TEM ", "VPV ", "IPV ", "VBAT", "IBAT"};
String id = "iMts-WLL10001";
int data[50] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};

const long periodeKirimData = 20000;
const long periodeAkuisisi = 1000;

AsyncWebServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);
/*
 * Connect your controller to WiFi
 */
void connectToWiFi()
{
  // Connect to WiFi Network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to WiFi");
  Serial.println("...");
  WiFi.begin(ssid, password);
  int retries = 0;
  while ((WiFi.status() != WL_CONNECTED) && (retries < 15))
  {
    retries++;
    delay(500);
    Serial.print(".");
  }
  if (retries > 14)
  {
    Serial.println(F("WiFi connection FAILED"));
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println(F("WiFi connected!"));
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
  Serial.println(F("Setup ready"));
}

//============================================================
// fungsi connect MQTT ke Server Broker
void connectMQTT()
{
  // Loop sampai reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // membuat client ID random
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), userBroker, passBroker))
    {
      Serial.println("connected");
      // Jika connected, publish topic sekali...
      client.publish("logger/awlr", "Pembacaan Sensor");
      // ... dan resubscribe
      // client.subscribe("sensor/suara");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Delay 5 detik sampai tersambung lagi
      delay(5000);
    }
  }
}
//============================================================<

// fungsi kirim data format json
void sendJsonData(String id, int wl, int br, int tem, int vpv, int ipv, int vbat, int ibat)
{
  StaticJsonBuffer<300> JSONbuffer;
  JsonObject &JSONencoder = JSONbuffer.createObject();

  JSONencoder["ID"] = id;
  JSONencoder["WL"] = wl;
  JSONencoder["BR"] = br;
  JSONencoder["TEM"] = tem;
  JSONencoder["VPV"] = vpv;
  JSONencoder["IPV"] = ipv;
  JSONencoder["VBAT"] = vbat;
  JSONencoder["IBAT"] = ibat;

  char JSONmessageBuffer[100];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Kirim data analisa to MQTT Broker");
  Serial.println(JSONmessageBuffer);

  if (client.publish("imats/logger", JSONmessageBuffer) == true)
  {
    Serial.println("Success sending message");
  }
  else
  {
    Serial.println("Error sending message");
  }

  client.loop();
  Serial.println("-------------");
}
//============================================================>

ModbusMaster node;

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

ModbusMaster node2;

void preTransmission2()
{
  digitalWrite(MAX485_RE_NEG2, 1);
  digitalWrite(MAX485_DE2, 1);
}

void postTransmission2()
{
  digitalWrite(MAX485_RE_NEG2, 0);
  digitalWrite(MAX485_DE2, 0);
}

//=============================================================<

void AddressRegistry_0000()
{
  delay(10);
  result = node2.readHoldingRegisters(0x0000, 1);
  if (result == node2.ku8MBSuccess)
  {
    data[0] = node2.getResponseBuffer(0x00);
    // Serial.println(data[0]);
  }
  else
  {
    // update of status failed
    Serial.println("readHoldingRegisters(0x0000, 1) failed!");
  }
}

void AddressRegistry_3100()
{
  result = node.readInputRegisters(0x3100, 6);

  if (result == node.ku8MBSuccess)
  {
    data[3] = node.getResponseBuffer(0x00);
    data[4] = node.getResponseBuffer(0x01);
    data[5] = node.getResponseBuffer(0x04);
    data[6] = node.getResponseBuffer(0x05);
  }
  else
  {
    // update of status failed
    rs485DataReceived = false;
    Serial.println("readInputRegisters(0x3100, 6) failed!");
  }
}

void AddressRegistry_311A()
{
  result = node.readInputRegisters(0x311A, 2);

  if (result == node.ku8MBSuccess)
  {
    // bremaining = node.getResponseBuffer(0x00);
    data[1] = node.getResponseBuffer(0x00);
    // Serial.print("Battery Remaining %: ");
    // Serial.println(bremaining);

    // btemp = node.getResponseBuffer(0x01);
    data[2] = node.getResponseBuffer(0x01);
    // Serial.print("Battery Temperature: ");
    // Serial.println(btemp);
  }
  else
  {
    rs485DataReceived = false;
    Serial.println("Read register 0x311A failed!");
  }
}

// A list of the regisities to query in order
typedef void (*RegistryList[])();

RegistryList Registries = {
    AddressRegistry_0000,
    AddressRegistry_3100,
    AddressRegistry_311A,
};

// keep log of where we are
uint8_t currentRegistryNumber = 0;

// function to switch to next registry
void nextRegistryNumber()
{
  // better not use modulo, because after overlow it will start reading in incorrect order
  currentRegistryNumber++;
  if (currentRegistryNumber >= ARRAY_SIZE(Registries))
  {
    currentRegistryNumber = 0;

    // cetak array nilai register
    for (int j = 0; j < 7; j++)
    {
      Serial.print(namaData[j]);
      Serial.print(" | ");
      // dataHigh[j] = 0;
    }
    Serial.println();

    for (int k = 0; k < 7; k++)
    {
      Serial.print(data[k]);
      Serial.print(" | ");
    }
    Serial.println();
    Serial.println();
  }
}

// ****************************************************************************

// --------------------------------------------------------------------------------

// exec a function of registry read (cycles between different addresses)
void executeCurrentRegistryFunction()
{
  Registries[currentRegistryNumber]();
}

SoftwareSerial mySerial;
SoftwareSerial mySerial2;

/*
 * call connectToWifi() in setup()
 */
void setup()
{
  //...other setup code here…
  Serial.begin(9600);
  connectToWiFi();
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hi, Imats WLL10 Running..."); });

  AsyncElegantOTA.begin(&server); // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

  client.setServer(mqtt_server, 1883);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  pinMode(MAX485_RE_NEG2, OUTPUT);
  pinMode(MAX485_DE2, OUTPUT);

  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  digitalWrite(MAX485_RE_NEG2, 0);
  digitalWrite(MAX485_DE2, 0);

  mySerial.begin(mpptBaudRate, SWSERIAL_8N1, RXD01, TXD01, false);
  mySerial2.begin(usBaudRate, SWSERIAL_8N1, RXD02, TXD02, false);

  // Modbus slave ID 1
  node.begin(1, mySerial);
  // Modbus slave ID 1
  node2.begin(2, mySerial2);

  // callbacks to toggle DE + RE on MAX485
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  node2.preTransmission(preTransmission2);
  node2.postTransmission(postTransmission2);
}

void loop()
{
  // put your main code here, to run repeatedly:
  
  currentTime = millis();
  if (currentTime - previousMillis >= periodeAkuisisi)
  {

    // Serial.println("debug periode akuisisi");
    executeCurrentRegistryFunction();
    nextRegistryNumber();

    previousMillis = currentTime;
  }
  
  currentTime = millis();
  if (currentTime - startTime >= periodeKirimData)
  {
    Serial.println(WiFi.localIP());
    Serial.println("Kirim Data ....");

    /*
    String data1 = "Aman";
    int data2 = 1000;
    int data3 = 2000;
    int data4 = 3000;
    int data5 = 4000;
    //
    sendJsonData(data1, data2, data3, data4, data5);*/

    sendJsonData(id, data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
    startTime = currentTime;
  }

  if (!client.connected())
  {
    connectMQTT();
  }
  client.loop();
}
