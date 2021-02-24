/******************************************************************************************************
 * This program was written with the intent to measure temperature and humidity using an Espresif ESP32,
 * a DHT Temperature and Humidity sensor, and a power supply set to 3.3V. The program sends data to
 * the serial monitor and an MQTT server using specific topics. In this case the MQTT server is setup
 * on a raspberry pi or Linux PC. 
 * 
 * Parts needed for this program:
 * -Esp Dev Module
 * -DHT Temp and Hum Sensor
 * -7-12VDC to 3.3/5V power supply with both jumpers set to "3.3V"
 * -1k Ohm Resistor(Brown/Black/Red)
 * -Yellow LED
 * -Photoresistor
 * -7 male to male jumper wires
 * -1 mini(400 pin solderless) bread board
 * -7-12VDC Power supply
 * 
 * Wiring (Refers to the position on the breadboard(Rows(1-30) and Columns(a-j, +, -))):
 * ESP32 pressed into column a and i rows 16-30
 * DHT pressed into column j rows 10-12
 * Power supply pressed into (+) and (-) on both sides, rows 1-5
 * Long side of the LED pressed into b13
 * Short side of the LED pressed into b10
 * One side of the photoresistor pressed into a6 and the other pressed into (+)(Use a 1k Ohm resistor with the photoresistor when used in direct sunlight, a 4.7k Ohm can be used when inside)
 * One side of the resistor pressed into b6 and the other side pressed into (-)
 * Jumper wire from e6 to e19(Pin D34 on the ESP32)(Wire must be put into e19 and bent 90*, then the EWSP32 goes on top of it)
 * One side of the resistor pressed into a10 and the other side pressed into (-)
 * Jumper wire from (+) to j30(Pin 3V3 on the ESP32)
 * Jumper wire from (-) to j29(Pin GND on the ESP32)
 * Jumper wire from f10(DHT(-)) to (-)
 * Jumper wire from f11(DHT('out") to j21(Pin D19 on ESP32)
 * Jumper wire from f12(DHT(+)) to (+)
 * Jumper wire from e13 to j23(Pin D5 on the ESP32)
***************************************************************************************************/
#include <WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>//For sending data to MQTT Server
#include "time.h"

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  55        /* Time ESP32 will go to sleep (in seconds) */
#define DHTPIN 19// Pin which is connected to the DHT22 sensor.
#define DHTTYPE DHT22// DHT 22 (AM2302)


//*****+*+*+*+*+*+*+*+*+*+**************************************************+*+*+*+*+*+*+*+*+*+*****
//*****+*+*+*+*+*+*+*+*+*+*****************Things to modify*****************+*+*+*+*+*+*+*+*+*+*****
//*****+*+*+*+*+*+*+*+*+*+**************************************************+*+*+*+*+*+*+*+*+*+*****
const char* ssid = "{Your network SSID goes between the quotes}";//**********WiFi SSID**********
const char* password =  "{Your network password goes between the quotes}";//**********SSID Password**********
const char* mqttUser = "{Your MQTT username goes between the quotes}";
const char* mqttPassword = "{Your MQTT password goes between the quotes}";
const char* mqtt_server = "{Your MQTT server IP Address goes between the quotes}";
const int mqttPort = 1883;

//******************************Unique Setting for each board**************************************
const char* mqttClientID = "ESP32ClientPatio";
WiFiClient espClientPatio;//**********This is the name of the specific ESP32**********
PubSubClient client(espClientPatio);//**********This is the name of the specific ESP32**********
const char* topicPrefix = "apt/patio";//This is specifically for apt/room or area name/xxxxx
const char* topicRoom = "patio";
IPAddress local_IP({The IP Address of the ESP32 goes between the parenthesis});
IPAddress gateway({The IP Address of your gateway goes between the parenthesis(Comma seperated)});
IPAddress subnet({The IP Address of your subnet goes between the parenthesis(Comma seperated)});
//*****+*+*+*+*+*+*+*+*+*+**************************************************+*+*+*+*+*+*+*+*+*+*****
//*****+*+*+*+*+*+*+*+*+*+*************End of things to modify**************+*+*+*+*+*+*+*+*+*+*****
//*****+*+*+*+*+*+*+*+*+*+**************************************************+*+*+*+*+*+*+*+*+*+*****


//**********Declaring Variables**********
static int wifiDisconnected, wifiConnected, mqttDisconnected, mqttConnected, minute, m, hour;
static double second, s, ms;
int WiFiLed = 2;//**********Pin 2 is the on board blue LED**********
int mqttLed = 5;//**********This is the pin that the LED is connected to()**********
int photoresistorPin = 34;
int lightInIt = 0;
int lightVal = 0;
long lastMsg = 0;
byte mac[6];
RTC_DATA_ATTR int bootCounter = 0;
RTC_DATA_ATTR int avgL, maxL;
RTC_DATA_ATTR double avgT, maxT, avgTCalc, avgH, maxH, avgHCalc, avgLCalc;
RTC_DATA_ATTR double minT = 120;
RTC_DATA_ATTR double minH = 100;
const char* ntpServer = "pool.ntp.org";
char msgTopic[100], msgPayload[100];
char bootTopic[100], bootString[8];
char tempTopic[100], tempString[8];
char avgTempTopic[100], avgTempString[8];
char maxTempTopic[100], maxTempString[8];
char minTempTopic[100], minTempString[8];
char humTopic[100], humString[8];
char avgHumTopic[100], avgHumString[8];
char maxHumTopic[100], maxHumString[8];
char minHumTopic[100], minHumString[8];
char rssiTopic[100], rssiString[8];
char lightValTopic[100], lightValString[8];
char avgLightValTopic[100], avgLightValString[8];
char maxLightValTopic[100], maxLightValString[8];

//HTU21D htu;
DHT htu(DHTPIN, DHTTYPE);

//Method to print the reason by which ESP32 has been awaken from sleep
void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

//**************************************
//*********Checking WiFi status*********
//**************************************
void checkingWifi()
{
  pinMode(WiFiLed, OUTPUT);
  if (!WiFi.config(local_IP, gateway, subnet)) 
  {
    Serial.println("STA Failed to configure");
  }  
  
  while (WiFi.status() != WL_CONNECTED)
  {  
    wifiDisconnected++;
    digitalWrite(WiFiLed, LOW);
    Serial.print("\nConnecting to WiFi");
    WiFi.begin(ssid, password);
    for (int i = 0; i < 20; i++)
    {
      WiFi.begin(ssid, password);
      delay(1000);//*************************Delay*************************
      Serial.print(".");
      if(WiFi.status() == WL_CONNECTED)
      {
        break;
      }
    }
    if(WiFi.status() != WL_CONNECTED)
    {
      Serial.print("Having trouble connecting to WiFi, it's been 20 seconds. Trying to connect again");
    }
    if(WiFi.status() == WL_CONNECTED)
    {
      digitalWrite(WiFiLed, HIGH);
      Serial.print("\tConnected to ");
      Serial.print(ssid);
      Serial.print(" @ ");
      Serial.println(WiFi.gatewayIP());
      Serial.print("ESP32 IP Address is ");
      Serial.println(WiFi.localIP());
      wifiConnected++;
    }
  }
}

//**************************************
//*********Checking MQTT status*********
//**************************************
void checkingMqtt()
{  
  pinMode(mqttLed, OUTPUT);
  client.setServer(mqtt_server, mqttPort);//configure the MQTT server with IPaddress and port
  while(!client.connected())
  {
    mqttDisconnected++;
    client.connect(mqttClientID, mqttUser, mqttPassword, "apt/patio/lastWill", 0, 1, "I'm dying!!", true);
    Serial.print("Connecting to MQTT");
    for (int i = 0; i < 20; i++)
    {
      delay(1000);//*************************Delay*************************
      Serial.print(".");
      client.connect(mqttClientID, mqttUser, mqttPassword, "apt/patio/lastWill", 0, 1, "I'm dying!!", true);
      if(client.connected())
      {
        break;
      }
    }
    if(client.connected())
    {
      Serial.print("\tConnected to MQTT server @ ");
      Serial.println(mqtt_server);  
      digitalWrite(mqttLed, HIGH);
      mqttConnected++;      
    }
   }
}

//*****************************************
//**********Printing MAC Address***********
//*****************************************
void printEspInfo()
{
  WiFi.macAddress(mac);
  Serial.print("This ESP32's MAC Address is: ");
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);
}

//*********************************************************************************
//******************Sending to MQTT Topic Prefix (ex. apt/xxxxx)*******************
//*********************************************************************************
void printPrefix()
{   
  const char* specificMsgTopic = "/message";
  strcpy(msgTopic, topicPrefix);
  strcat(msgTopic, specificMsgTopic);
  const char* msgStuff1 = "\n~~apt/";
  const char* msgStuff2 = "~~";
  strcpy(msgPayload, msgStuff1);
  strcat(msgPayload, topicRoom);
  strcat(msgPayload, msgStuff2);
  Serial.println(msgPayload);
}

//***********************************************************************************************
//****************Printing to serial and sending to MQTT the number of data points***************
//***********************************************************************************************
void printBootCounter()
{
  const char* specificBootTopic = "/dataPts";
  strcpy(bootTopic, topicPrefix);
  strcat(bootTopic, specificBootTopic);
  Serial.print("Total Cycles: ");
  dtostrf(bootCounter, 1, 0, bootString);
  Serial.println(bootString);
}  

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

//********************************************************************************
//********Printing to serial and sending to MQTT Temperature in Farenheit*********
//********************************************************************************
double printTemp()
{
  const char* specificTempTopic = "/temp";
  strcpy(tempTopic, topicPrefix);
  strcat(tempTopic, specificTempTopic);
  double f = htu.readTemperature(true);
  if(isnan(f))
  {
    Serial.println("DHT Sensor not detected");
  }
  if (!isnan(f))
  {
    dtostrf(f, 1, 2, tempString);// Convert the value to a char array
    Serial.print("Temperature: ");
    Serial.print(tempString);//Print the value to the serial port
    Serial.print("*F\t\t");
    return f;
  }
  else
  {
    return avgT;
  }
}

//************************************************************************************
//******************Printing to serial Avg Temperature in Farenheit*******************
//************************************************************************************
void avgTemp(double f)
{
  if (!isnan(f) || tempString == "nan")
  {
    const char* specificTempTopic = "/avgTemp";
    strcpy(avgTempTopic, topicPrefix);
    strcat(avgTempTopic, specificTempTopic);
    avgTCalc = avgTCalc + f;
    if (bootCounter == 1)
    {
      avgT = f;
      avgTCalc = f;     
    }
    else
    {
      avgT = avgTCalc / bootCounter;
    }
    dtostrf(avgT, 1, 2, avgTempString);// Convert the value to a char array
    Serial.print("Avg Temp is: ");
    Serial.print(avgTempString);
    Serial.print("*F\t\t");
  }
}

//************************************************************************************
//******************Printing to serial Max Temperature in Farenheit*******************
//************************************************************************************
void calcMaxTemp(double f)
{
  const char* specificTempTopic = "/maxTemp";
  strcpy(maxTempTopic, topicPrefix);
  strcat(maxTempTopic, specificTempTopic);
  maxT = max(maxT, f);
  dtostrf(maxT, 1, 2, maxTempString);// Convert the value to a char array
  Serial.print("Max Temp is: ");
  Serial.print(maxTempString);
  Serial.print("*F\t\t");
}

//************************************************************************************
//******************Printing to serial Min Temperature in Farenheit*******************
//************************************************************************************
void calcMinTemp(double f)
{
  if(f != 0)
  {
    const char* specificTempTopic = "/minTemp";
    strcpy(minTempTopic, topicPrefix);
    strcat(minTempTopic, specificTempTopic);
    minT = min(minT, f);
    dtostrf(minT, 1, 2, minTempString);// Convert the value to a char array
    Serial.print("Min Temp is: ");
    Serial.print(minTempString);
    Serial.print("*F\t\t");
  }
}

//********************************************************************************
//********************Printing to serial Humidity in Percent**********************
//********************************************************************************
double printHum()
{
  const char* specificHumTopic = "/hum";
  strcpy(humTopic, topicPrefix);
  strcat(humTopic, specificHumTopic);
  double h = htu.readHumidity();
  if (!isnan(h))
  {
    dtostrf(h, 1, 2, humString);// Convert the value to a char array
    Serial.print("Humidity: ");
    Serial.print(humString);//Print the value to the serial port
    Serial.println("%");
    return h;
  }
  else
  {
    return avgH;
  }
}

//************************************************************************************
//********************Printing to serial Avg Humidity in Percent**********************
//************************************************************************************
void avgHum(double h)
{
  if (!isnan(h) || humString == "nan")
  {
    const char* specificHumTopic = "/avgHum";
    strcpy(avgHumTopic, topicPrefix);
    strcat(avgHumTopic, specificHumTopic);
    avgHCalc = avgHCalc + h;
    if (bootCounter == 1)
    {
      avgH = h;
      avgHCalc = h;     
    }
    else
    {
      avgH = avgHCalc / bootCounter;
    }
    dtostrf(avgH, 1, 2, avgHumString);// Convert the value to a char array
    Serial.print("Avg Hum is: ");
    Serial.print(avgH);
    Serial.println("%");
  }
}

//************************************************************************************
//********************Printing to serial Max Humidity in Percent**********************
//************************************************************************************
void calcMaxHum(double h)
{
  const char* specificHumTopic = "/maxHum";
  strcpy(maxHumTopic, topicPrefix);
  strcat(maxHumTopic, specificHumTopic);
  maxH = max(maxH, h);
  dtostrf(maxH, 1, 2, maxHumString);// Convert the value to a char array
  Serial.print("Max Hum is: ");
  Serial.print(maxH);
  Serial.println("%");
}

//************************************************************************************
//********************Printing to serial Min Humidity in Percent**********************
//************************************************************************************
void calcMinHum(double h)
{
  const char* specificHumTopic = "/minHum";
  strcpy(minHumTopic, topicPrefix);
  strcat(minHumTopic, specificHumTopic);
  minH = min(minH, h);
  dtostrf(minH, 1, 2, minHumString);// Convert the value to a char array
  Serial.print("Min Hum is: ");
  Serial.print(minH);
  Serial.println("%");
}

//********************************************************************************
//******************Printing to serial and sending to MQTT RSSI*******************
//********************************************************************************
void printRssi()
{
  const char* specificRssiTopic = "/rssi";
  strcpy(rssiTopic, topicPrefix);
  strcat(rssiTopic, specificRssiTopic);
  Serial.print("RSSI: ");
  int rssi = WiFi.RSSI();
  dtostrf(rssi, 1, 0, rssiString);// Convert the value to a char array
  Serial.println(rssiString);
}

//*******************************************************************************************
//****************Printing to serial and sending to MQTT Counting elapsed time***************
//*******************************************************************************************
/*void printElapsedTime()
{
//  s = timer;
  s = s / 1000;
  second = s;
  if(s > 59)
  {
    second = fmod(s, 60);//Use fmod on doubles instead of modulus(%) (modulus only works with int)
  }
  if(s > 59)
  {
    m = s / 60;
    minute = m;
  }
  if(minute > 59)
  {
    minute = minute % 60;
  }
  if(m > 59)
  {
    hour = m / 60;
  }
  String elapsedTime;
  String hh = "h:";
  String mm = "m:";
  String ss = "s";
  int len;
  const char* specificElapsedTimeTopic = "/elapsedTime";
  strcpy(elapsedTimeTopic, topicPrefix);
  strcat(elapsedTimeTopic, specificElapsedTimeTopic);
  elapsedTime = hour + hh + minute + mm + second + ss;
  Serial.print("Elapsed Time: ");
  len = elapsedTime.length() + 1;
  elapsedTime.toCharArray(elapsedTimeString, len);// Convert the value to a char array
  Serial.println(elapsedTimeString);
}*/

//*******************************************************************************
//**********************Printing to serial MQTT Light Value**********************
//*******************************************************************************
double printLightValue()
{
  const char* specificLightValTopic = "/lightVal";
  strcpy(lightValTopic, topicPrefix);
  strcat(lightValTopic, specificLightValTopic);
  lightVal = analogRead(photoresistorPin);
  lightVal = lightVal / 40.95;//Calibration factor
  Serial.print("The value of light is: ");
  dtostrf(lightVal, 1, 0, lightValString);
  Serial.print(lightValString);
  Serial.println("%");
  return lightVal;
}

//************************************************************************************
//************************Printing to serial Avg Light Value**************************
//************************************************************************************
void avgLightVal(int lightVal)
{
  if (!isnan(lightVal))
  {
    const char* specificAvgLightValTopic = "/avgLightVal";
    strcpy(avgLightValTopic, topicPrefix);
    strcat(avgLightValTopic, specificAvgLightValTopic);
    avgLCalc = avgLCalc + lightVal;
    if (bootCounter == 1)
    {
      avgL = lightVal;
      avgLCalc = lightVal;     
    }
    else
    {
      avgL = avgLCalc / bootCounter;
    }
    dtostrf(avgL, 1, 2, avgLightValString);// Convert the value to a char array
    Serial.print("The avg value of light is: ");
    Serial.print(avgLightValString);
    Serial.println("%");
  }
}

//************************************************************************************
//************************Printing to serial Max Light Value**************************
//************************************************************************************
void calcMaxLightVal(int lightVal)
{
  const char* specificMaxLightValTopic = "/maxLightVal";
  strcpy(maxLightValTopic, topicPrefix);
  strcat(maxLightValTopic, specificMaxLightValTopic);
  maxL = max(maxL, lightVal);
  dtostrf(maxL, 1, 0, maxLightValString);// Convert the value to a char array
  Serial.print("Max value of light is: ");
  Serial.print(maxLightValString);
  Serial.println("%");
}

//***************************************************
//********Publishing data to the MQTT Server*********
//***************************************************
void publishToMqtt()
{
  client.publish(msgTopic, msgPayload);//client.publish(const char[], const char[])
  client.publish(bootTopic, bootString);
  client.publish(tempTopic, tempString);//Publish the value to the MQTT Server
  client.publish(avgTempTopic, avgTempString);//Publish the value to the MQTT Server
  client.publish(maxTempTopic, maxTempString);//Publish the value to the MQTT Server
  client.publish(minTempTopic, minTempString);//Publish the value to the MQTT Server
  client.publish(humTopic, humString);//Publish the value to the MQTT Server 
  client.publish(avgHumTopic, avgHumString);//Publish the value to the MQTT Server 
  client.publish(maxHumTopic, maxHumString);//Publish the value to the MQTT Server 
  client.publish(minHumTopic, minHumString);//Publish the value to the MQTT Server 
  client.publish(rssiTopic, rssiString);
  client.publish(lightValTopic, lightValString);
  client.publish(avgLightValTopic, avgLightValString);
  client.publish(maxLightValTopic, maxLightValString);
  client.loop();  //Running the PubSub Loop
  client.disconnect();
  if (!client.connected())
  {
    client.disconnect();
    delay(500);
  }
}

//********************************************
//********Disconnecting MQTT and WiFi*********
//********************************************
void mqttAndWifiDisconnect()
{
  client.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(1000);
  if (!client.connected())
  {
    digitalWrite(mqttLed, LOW);
    Serial.println("MQTT has been disconnected");      
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi has been disconnected");
    digitalWrite(WiFiLed, LOW);
  }
}

//*******************************************
//********Printing MQTT Server State*********
//*******************************************
void printState()
{
  Serial.print("The current state of the MQTT Client is: ");
  Serial.print(client.state());
  switch (client.state())
  {
    case -4: 
      Serial.println(" MQTT_CONNECTION_TIMEOUT - the server didn't respond within the keepalive time");
      break;
    case -3:
      Serial.println(" MQTT_CONNECTION_LOST - the network connection was broken");
      break;
    case -2:
      Serial.println(" MQTT_CONNECT_FAILED - the network connection failed");
      break;
    case -1:
      Serial.println(" MQTT_DISCONNECTED - the client is disconnected cleanly");
      break;
    case 0:
      Serial.println(" MQTT_CONNECTED - the client is connected");
      break;
    case 1:
      Serial.println(" MQTT_CONNECT_BAD_PROTOCOL - the server doesn't support the requested version of MQTT");
      break;
    case 2:
      Serial.println(" MQTT_CONNECT_BAD_CLIENT_ID - the server rejected the client identifier");
      break;
    case 3:
      Serial.println(" MQTT_CONNECT_UNAVAILABLE - the server was unable to accept the connection");
      break;
    case 4:
      Serial.println(" MQTT_CONNECT_BAD_CREDENTIALS - the username/password were rejected");
      break;
    case 5:
      Serial.println(" MQTT_CONNECT_UNAUTHORIZED - the client was not authorized to connect");
      break;
  }
}

void setup(){
  Serial.begin(115200);
  pinMode(photoresistorPin, INPUT);
  lightInIt = analogRead(photoresistorPin);
  htu.begin();

  //Increment boot number and print it every reboot
  ++bootCounter;
  Serial.println("Boot number: " + String(bootCounter));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

  //init and get the time

  printEspInfo();
  checkingWifi();
  checkingMqtt();
  printPrefix();
  printBootCounter();
  configTime(61200, 0, ntpServer);
//  printLocalTime();
  double f = printTemp();
  double h = printHum();
  if (!isnan(f) || !isnan(h))
  {
    avgTemp(f);
    avgHum(h);
    calcMaxTemp(f);
    calcMaxHum(h);
    calcMinTemp(f);
    calcMinHum(h); 
  }  printRssi();
  int lightVal = printLightValue();
  avgLightVal(lightVal);
  calcMaxLightVal(lightVal);
  publishToMqtt();
  mqttAndWifiDisconnect();
  printState();

  Serial.println("Going to sleep now");
  Serial.flush(); 
  esp_deep_sleep_start();
}
void loop() {
  // put your main code here, to run repeatedly:
}
