#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32AnalogRead.h>
#include <math.h>

#include "soc/soc.h"          // Disable brownour problems
#include "soc/rtc_cntl_reg.h" // Disable brownour problems
#include <WiFi.h>             //Wifi http

#include <ESPAsyncWebServer.h> //html
#include <SPIFFS.h>
#include <FS.h>           // file read apprend
#include <PubSubClient.h> //mqtt

#include <ESPmDNS.h>    //OTA
#include <WiFiUdp.h>    //OTA
#include <ArduinoOTA.h> //OTA

#include <string.h>
#include <stdio.h>

void phCalibrate();

char t_char[8];
char h_char[8];
char dallas_t_char[8];
char ph_char[8];
int sensorfail = 0;
float ph, rawadc = 0.00;
float h, t, dallas_t;
int samplect = 10;
char charResetCount[200];
unsigned int timer1s;
unsigned int intResetCount;
String S_ResetCount;

//parameter ph
int Int_phSensor; // ph sensor active or no active
String S_phSensor;
char phVol_char[20];
char phVol1d_char[20];
char adc_char[20];
int adc_int;

const int phPin = 34; // adc on GPIO 34
int sensorValue = 0;
unsigned long int avgValue;
float b;
int buf[10], temp;
float f_offsetCal;
float phValue;
char C_offsetCal[8];
String S_offsetCal;
float pHVol;
float phVol1d;

#define DHTPIN 23     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // DHT 11
DHT dht(DHTPIN, DHTTYPE);
const int ledPin = 2; // ledPin refers to ESP32 GPIO 23

#ifdef __cplusplus //config ph meter
extern "C"
{
#endif
  uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();
#define ONE_WIRE_BUS 15
// Setup PT100 a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

//timer interrupt
volatile int interruptCounter;
int timerCount; //test statement for each step in second
char c_timerCount[8];
boolean flagEx = false; // flag to excute 1 time the statement

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//mqtt configuration
const char *mqtt_server = "192.168.0.50";
WiFiClient espClient;
PubSubClient client(espClient);

// wifi configuration
const char *ssid = "CLV";
const char *password = "Pi@Riya*1";

bool internet_connected = false;
struct tm timeinfo;
time_t now;
char strftime_buf[64];              //time for webserver
char C_ip_adress[14] = "IP adress"; //for Mqtt ID
char C_mac_adr[18];                 //for Mqtt ID
char C_idHostname[40];
char C_topic_t_Hostname[40] = "esp32/temp/";
char C_topic_h_Hostname[40] = "esp32/hum/";
char C_topic_dallas_t_Hostname[40] = "esp32/dallas_t/";
char C_topic_ph_Hostname[40] = "esp32/ph/";
char C_topic_ph_Vol[40] = "esp32/phVol/";
char C_topic_ph_Vol1d[40] = "esp32/phVol1d/";
char C_topic_adc[40] = "esp32/adc/";


int Ledboard = 2;
int mQtyFailCt = 5;
int i = 5;  // variable for loop
int y = 10; //variable for wifi reset

//my time
// int day, hours, minutes, seconds, year, month, date, minuteSave;
// WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP, "0.pool.ntp.org", 25200, 0);
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;
const int daylightOffset_sec = 0;

//variable for webserver
const char *PARAM_ipAdress = "ipAdress";
const char *PARAM_macAdress = "macAdress";
const char *PARAM_idHostname = "idHostname";
const char *PARAM_timeCycle = "timeCycle";
const char *PARAM_offsetCal = "offsetCal";

int Int_timeCycle;

String S_ipAdress;
String S_macAdress;
String S_idHostname;
String S_timeCycle;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

//SPIFFS read & write
String readFile(fs::FS &fs, const char *path)
{
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");
  if (!file || file.isDirectory())
  {
    Serial.println("- empty file or failed to open file");
    return String();
  }
  Serial.println("- read from file:");
  String fileContent;
  while (file.available())
  {
    fileContent += String((char)file.read());
  }
  Serial.println(fileContent);
  return fileContent;
}

bool init_wifi()
{
  int connAttempts = 0;
  Serial.println("\r\nConnecting to: " + String(ssid));
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);

  S_idHostname = readFile(SPIFFS, "/idHostname.txt");
  S_idHostname.toCharArray(C_idHostname, 40);
  WiFi.setHostname(C_idHostname);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(2000);
    Serial.print(".");
    if (connAttempts > 10)
      return false;
    connAttempts++;
  }
  return true;
}

void init_time()
{
  struct tm timeinfo;
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  // wait for time to be set
  time_t now = 0;
  timeinfo = {0};
  int retry = 0;
  const int retry_count = 10;
  while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count)
  {
    Serial.printf("Waiting for system time to be set... (%d/%d)\n", retry, retry_count);
    delay(2000);
    time(&now);
    localtime_r(&now, &timeinfo);
  }
}

//SPIFFS read & write
void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, "w");
  if (!file)
  {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("- file written");
  }
  else
  {
    Serial.println("- write failed");
  }
}
//Processor read back to value on website
String processor(const String &var) //display value on http
{
  if (var == "idHostname")
  {
    S_idHostname = readFile(SPIFFS, "/idHostname.txt");
    return readFile(SPIFFS, "/idHostname.txt");
  }
  else if (var == "timeNow")
  {
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%F_%H_%M_%S", &timeinfo);
    return String(strftime_buf);
  }
  else if (var == "ipAdress")
  {
    return String(WiFi.localIP().toString());
  }
  else if (var == "macAdress")
  {
    return String(WiFi.macAddress());
  }
  else if (var == "resetCount")
  {
    return readFile(SPIFFS, "/resetCount.txt");
  }
  else if (var == "timeCycle")
  {
    S_timeCycle = readFile(SPIFFS, "/timeCycle.txt");
    return readFile(SPIFFS, "/timeCycle.txt");
  }
  else if (var == "t")
  {

    return readFile(SPIFFS, "/t.txt");
  }
  else if (var == "h")
  {
    return readFile(SPIFFS, "/h.txt");
  }
  else if (var == "dallas_t")
  {
    return readFile(SPIFFS, "/dallas_t.txt");
  }
  else if (var == "ph")
  {
    return readFile(SPIFFS, "/ph.txt");
  }
  else if (var == "phVol")
  {
    return readFile(SPIFFS, "/phVol.txt");
  }
  else if (var == "offsetCal")
  {
    S_offsetCal = readFile(SPIFFS, "/offsetCal.txt");
    return readFile(SPIFFS, "/offsetCal.txt");
  }
  else if (var == "timerCount")
  {
    return readFile(SPIFFS, "/timerCount.txt");
  }
  else if (var == "phSensor")
  {
    S_phSensor = readFile(SPIFFS, "/phSensor.txt");
    return readFile(SPIFFS, "/phSensor.txt");
  }

  return String();
}

void init_server() //Server init
{
  File file = SPIFFS.open("/index.html", "r");
  if (!file)
  {
    Serial.println("file open failed");
  }
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  //Read hostname
  S_idHostname = readFile(SPIFFS, "/idHostname.txt");
  //Read timeCycle
  S_timeCycle = readFile(SPIFFS, "/timeCycle.txt");
  Int_timeCycle = S_timeCycle.toInt();

  S_offsetCal = readFile(SPIFFS, "/offsetCal.txt");
  f_offsetCal = S_offsetCal.toFloat();

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
    writeFile(SPIFFS, "/resetCount.txt", "0");
    writeFile(SPIFFS, "/offsetCal.txt", "0");

    delay(10);
    ESP.restart();
  });
  server.on("/phOn", HTTP_GET, [](AsyncWebServerRequest *request) {
    writeFile(SPIFFS, "/phSensor.txt", "On");
    Serial.println("Ph On");
    delay(1000);
    timerCount = 0;

    request->redirect("/");
  });
  server.on("/phOff", HTTP_GET, [](AsyncWebServerRequest *request) {
    writeFile(SPIFFS, "/phSensor.txt", "Off");
    writeFile(SPIFFS, "/ph.txt", " ");
    Serial.println("Ph off");
    timerCount = 0;

    request->redirect("/");
  });
  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
    phCalibrate();
    timerCount = 0;

    request->redirect("/");
  });

  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
    String inputMessage;
    // String inputParam; //no used
    // GET timeBetween value on <ESP_IP>/get?timeBetween=<inputMessage>

    if (request->hasParam(PARAM_idHostname))
    {
      inputMessage = request->getParam(PARAM_idHostname)->value();
      writeFile(SPIFFS, "/idHostname.txt", inputMessage.c_str());
    }
    else if (request->hasParam(PARAM_timeCycle))
    {
      inputMessage = request->getParam(PARAM_timeCycle)->value();
      writeFile(SPIFFS, "/timeCycle.txt", inputMessage.c_str());
    }
    else if (request->hasParam(PARAM_offsetCal))
    {
      inputMessage = request->getParam(PARAM_offsetCal)->value();
      writeFile(SPIFFS, "/offsetCal.txt", inputMessage.c_str());
    }
    else
    {
      inputMessage = "No message sent";
    }
    request->send(200, "text/text", inputMessage);
  });
  server.begin();
} //end Server init
// call back mqtt
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String messageTemp;
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  // Switch on the LED if an 1 was received as first character
  if (String(topic) == "esp32/output")
  {
    Serial.print("Changing output to ");
    if (messageTemp == "on")
    {
      Serial.println("on");
      digitalWrite(Ledboard, HIGH);
    }
    else if (messageTemp == "off")
    {
      Serial.println("off");
      digitalWrite(Ledboard, LOW);
    }
  }
}

void reconnect() //reconnect mqtt server
{
  // Loop until we're reconnected
  while (!client.connected() && (mQtyFailCt >= 0))
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = C_idHostname;
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("esp32/output");
      mQtyFailCt = 5;
    }
    else if (mQtyFailCt == 0)
    {
      Serial.println("Mqtt fail 5 time restart esp32");
      ESP.restart();
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      mQtyFailCt--;
    }
  }
}
//code OTA
void init_OTA()
{
  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });

  ArduinoOTA.begin();
}

void dallasRead()
{
  sensors.requestTemperatures();
  dallas_t = sensors.getTempCByIndex(0);

  if (dallas_t != DEVICE_DISCONNECTED_C)
  {
    dtostrf(dallas_t, 2, 2, dallas_t_char); // convertion float to char
    writeFile(SPIFFS, "/dallas_t.txt", dallas_t_char);
    client.publish(C_topic_dallas_t_Hostname, dallas_t_char);
    Serial.print("dallas_t: ");
    Serial.println(dallas_t_char);
  }
  else
  {
    // delay(100);
    // sensors.requestTemperatures();
    // dallas_t = sensors.getTempCByIndex(0);
    client.publish(C_topic_dallas_t_Hostname, "");
  }
}

void dhtRead()
{
  h = dht.readHumidity();
  dtostrf(h, 2, 2, h_char); // convertion float to string
  writeFile(SPIFFS, "/h.txt", h_char);
  t = dht.readTemperature();
  dtostrf(t, 2, 2, t_char); // convertion float to string
  writeFile(SPIFFS, "/t.txt", t_char);

  if (isnan(h) || isnan(t))
  {
    Serial.println("Failed to read from DHT sensor!");
    delay(2000);
    sensorfail = 1;
  }

  Serial.print("ESP32_t: ");
  Serial.println((temprature_sens_read() - 32) / 1.8);

  client.publish(C_topic_t_Hostname, t_char);
  Serial.println(C_topic_t_Hostname);
  Serial.print("dht22_t: ");
  Serial.println(t_char);
  client.publish(C_topic_h_Hostname, h_char);
  Serial.print("dht22_h: ");
  Serial.println(h_char);

  digitalWrite(ledPin, HIGH); // turn the LED off by making the voltage LOW
  delay(200);
  sensorfail = 0;
}

void InternetphRead()
{
  S_phSensor = readFile(SPIFFS, "/phSensor.txt");
  if (S_phSensor == "On")
  {
    for (int i = 0; i < 10; i++)
    {
      buf[i] = analogRead(phPin);
      Serial.print("AD = ");
      Serial.println(buf[i]);
      delay(10);
    }
    for (int i = 0; i < 9; i++)
    {
      for (int j = i + 1; j < 10; j++)
      {
        if (buf[i] > buf[j])
        {
          temp = buf[i];
          buf[i] = buf[j];
          buf[j] = temp;
        }
      }
    }
    avgValue = 0;
    for (int i = 2; i < 8; i++)
      avgValue += buf[i];
    //float pHVol=(float)avgValue*5.0/1024/6;
    float pHVol = (float)avgValue * 3.366 / 4096 / 6;
    Serial.print("v = ");
    Serial.println(pHVol);
    dtostrf(pHVol, 2, 3, phVol_char);
    writeFile(SPIFFS, "/phVol.txt", phVol_char);
    float phValue = (-1000 / 241) * pHVol + 14;
    //float phValue = 7 + ((2.5 - pHVol) / 0.18);
    dtostrf(phValue, 2, 2, ph_char); // convertion float to string
    writeFile(SPIFFS, "/ph.txt", ph_char);
    Serial.print("ph: ");
    Serial.println(phValue);
    client.publish(C_topic_ph_Hostname, ph_char);
  }
}

void adcSetting()
{
  //analogSetCycles(100);
  //analogSetSamples(samplect);
  analogReadResolution(10);
  adcAttachPin(phPin);
}
void phRead()
{
  S_phSensor = readFile(SPIFFS, "/phSensor.txt");
  if (S_phSensor == "On")
  {

    adc_int = analogRead(phPin);
    for(i=0, ,i<10, i++ )
    {
      
    }
    pHVol = float(adc_int) * 3.3 / 1023; //convertie in volt
    // pHVol = adc.readVoltage();
    //https://www.mathportal.org/calculators/analytic-geometry/two-point-form-calculator.php
    // BNC to GND = 3000 (2,5V) & 7ph, Amonnique  1900 (1,65V) & 11ph
    
    phVol1d = round(pHVol*10.0)/10.0;
    dtostrf(pHVol, 2, 2, phVol_char);
    dtostrf(phVol1d,2,1,phVol1d_char);
    itoa(adc_int,adc_char,4);

    client.publish(C_topic_ph_Vol,phVol_char);
    client.publish(C_topic_ph_Vol1d,phVol1d_char);
    client.publish(C_topic_adc,adc_char);
 

    writeFile(SPIFFS, "/phVol.txt", phVol_char);

    S_offsetCal = readFile(SPIFFS, "/offsetCal.txt");
    f_offsetCal = S_offsetCal.toFloat();
    phValue = -5.70 * phVol1d + (21.34 - f_offsetCal);
    //float phValue = -4.17 * (pHVol + f_offsetCal) + 14.03;
    //float phValue = 7 + ((2.5 - pHVol) / 0.18);
    dtostrf(phValue, 2, 2, ph_char); // convertion float to string
    writeFile(SPIFFS, "/ph.txt", ph_char);
    Serial.print("ph: ");
    Serial.println(phValue);
    client.publish(C_topic_ph_Hostname, ph_char);
  }
}

void phCalibrate()
{
  f_offsetCal = 6.86 - phValue; //devider resistor 330/680ohm 5V = 3.366V  / 2.5V = 1.687V
  dtostrf(f_offsetCal, 2, 3, C_offsetCal);
  writeFile(SPIFFS, "/offsetCal.txt", C_offsetCal);
  Serial.print("offset calibration: ");
  Serial.println(C_offsetCal);
}

void checkConnection()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    delay(10);
    Serial.println("Wifi Connected");
    y = 10;
  }
  else if ((WiFi.status() != WL_CONNECTED) && (y > 0))
  {
    WiFi.reconnect();
    delay(100);
    Serial.print("Wifi no connected : ");
    Serial.println(y);
    --y; //decrease in interrupt
  }
  else if (y == 0)
  {
    Serial.println("Wifi No Connected need to reboot");
    S_ResetCount = readFile(SPIFFS, "/resetCount.txt");
    intResetCount = S_ResetCount.toInt() + 1;
    writeFile(SPIFFS, "/resetCount.txt", itoa(intResetCount, charResetCount, 10));
    ESP.restart();
  }
}

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup()
{
  // put your setup code here, to run once:
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  sensors.begin(); //dallas start
  dht.begin();
  //EEPROM.begin(EEPROM_SIZE);
  // adcAttachPin(34);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  Serial.write("Hello world");
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  else
  {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }
  if (init_wifi())
  { // Connected to WiFi
    internet_connected = true;
    Serial.println("Internet connected");
    // Print ESP32 Local IP Address
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.macAddress());
    init_time();
    time(&now);
    // setenv("TZ", "GMT0BST,M3.5.0/01,M10.5.0/02", 1);
    // tzset();
    WiFi.localIP().toString().toCharArray(C_ip_adress, 14); // Convert IP adress to String then to Char Array
    WiFi.macAddress().toCharArray(C_mac_adr, 18);           // Convert Mac adr to Char array
    Serial.write("Wifi connected");
  }
  //check SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  else
  {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }
  init_server();                       //start server
  client.setServer(mqtt_server, 1883); //start mqtt
  client.setCallback(callback);
  strcat(C_topic_t_Hostname, C_idHostname);        //topic preparation
  strcat(C_topic_h_Hostname, C_idHostname);        //topic preparation
  strcat(C_topic_dallas_t_Hostname, C_idHostname); //topic preparation
  strcat(C_topic_ph_Hostname, C_idHostname);       //topic preparation
  strcat(C_topic_ph_Vol, C_idHostname);
  strcat(C_topic_ph_Vol1d, C_idHostname);
  strcat(C_topic_adc,C_idHostname);

  //Init pin mode
  pinMode(Ledboard, OUTPUT);
  //OTA init
  init_OTA();
  timerAlarmEnable(timer);
  adcSetting();
}

void loop()
{
  if (timer1s > 0)
  {
    timer1s = 0;
    checkConnection();
  }
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  ArduinoOTA.handle();

  if (interruptCounter > 0)
  {
    portENTER_CRITICAL(&timerMux);
    interruptCounter = 0;
    portEXIT_CRITICAL(&timerMux);
    timerCount++;
    writeFile(SPIFFS, "/timerCount.txt", itoa(timerCount, c_timerCount, 10));
    S_timeCycle = readFile(SPIFFS, "/timeCycle.txt");
    Int_timeCycle = S_timeCycle.toInt();
    flagEx = false;
    Serial.print("timerCount_b: ");
    Serial.println(timerCount);
    timer1s++;
  }

  if (timerCount == Int_timeCycle)
  {
    if (flagEx == false)
    {
      dhtRead();
      dallasRead();
      phRead();
      flagEx = true;
      timerCount = 0;
    }
  }
  else if (timerCount > Int_timeCycle)
  {
    if (flagEx == false)
    {
      flagEx = true;
      timerCount = 0;
    }
  }
}