/******************************************************************************************************
 * This program was written with the intent to measure temperature and humidity using an Espresif ESP32,
 * a HTU21D Temperature and Humidity sensor, and a power supply set to 3.3V. The program sends data to
 * the serial monitor and an MQTT server using specific topics. In this case the MQTT server is setup
 * on a raspberry pi. 
 * 
 * Parts needed for this program:
 * -Esp Dev Module
 * -HTU21D Temp and Hum Sensor
 * -7-12VDC to 3.3/5V power supply with both jumpers set to "3.3V"
 * -2 1k Ohm Resistor(Brown/Black/Red)
 * -Yellow LED
 * -Photoresistor
 * -8 male to male jumper wires
 * -1 mini(400 pin solderless) bread board
 * -7-12VDC Power supply
 * 
 * 
 * Wiring (Refers to the position on the breadboard(Rows(1-30) and Columns(a-j, +, -))):
 * ESP32 pressed into column a and i rows 16-30
 * HTU21D pressed into column j rows 9-12
 * Power supply pressed into (+) and (-) on both sides, rows 1-5
 * Long side of the LED pressed into b13
 * Short side of the LED pressed into b10
 * One side of the resistor pressed into a10 and the other side pressed into (-)
 * One side of the photoresistor pressed into a6 and the other pressed into (+)(Use a 1k Ohm resistor with the photoresistor when used in direct sunlight, a 4.7k Ohm can be used when inside)
 * One side of the resistor pressed into b6 and the other side pressed into (-)
 * Jumper wire from (+) to j30(Pin 3V3 on the ESP32)
 * Jumper wire from (-) to j29(Pin GND on the ESP32)
 * Jumper wire from f9(HTU21D(+)) to (+)
 * Jumper wire from f10(HTU21D(-)) to (-)
 * Jumper wire from f11(HTU21D(DA)) to j20(Pin D21 on ESP32)
 * Jumper wire from f12(HTU21D(CL)) to j17(Pin D22 on ESP32)
 * Jumper wire from e13 to j23(Pin D5 on the ESP32)
 * Jumper wire from e6 to e19(Pin D34 on the ESP32)(Wire must be put into e19 and bent 90*, then the EWSP32 goes on top of it)
 * 
***************************************************************************************************/
#include <WiFi.h>
#include <Wire.h>
#include "SparkFunHTU21D.h"
#include <PubSubClient.h>
#include "time.h"


//*****+*+*+*+*+*+*+*+*+*+**************************************************+*+*+*+*+*+*+*+*+*+*****
//*****+*+*+*+*+*+*+*+*+*+*****************Things to modify*****************+*+*+*+*+*+*+*+*+*+*****
//*****+*+*+*+*+*+*+*+*+*+**************************************************+*+*+*+*+*+*+*+*+*+*****
const char* ssid = "NETGEAR15";//**********WiFi SSID**********
const char* password =  "gentlewind328";//**********SSID Password**********
const char* mqttUser = "username";
const char* mqttPassword = "12345678";
const char* mqtt_server = "192.168.0.45";
const int mqttPort = 1883;

//******************************Unique Setting for each board**************************************
const char* mqttClientID = "ESP32ClientPatio";
WiFiClient espClientPatio;//**********This is the name of the specific ESP32**********
PubSubClient client(espClientPatio);//**********This is the name of the specific ESP32**********
const char* topicPrefix = "apt/patio";//This is specifically for apt/room or area name/xxxxx
const char* topicRoom = "patio";
//int delayTime = 4901;//This is adjusted so a cycle happens every 5 seconds(5000 ms)
int delayTime = 56156;//This is adjusted so a cycle happens every 60 seconds(60000 ms)
IPAddress local_IP(192,168,0,53);//The IP Address assigned to this ESP32
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);
//*****+*+*+*+*+*+*+*+*+*+**************************************************+*+*+*+*+*+*+*+*+*+*****
//*****+*+*+*+*+*+*+*+*+*+*************End of things to modify**************+*+*+*+*+*+*+*+*+*+*****
//*****+*+*+*+*+*+*+*+*+*+**************************************************+*+*+*+*+*+*+*+*+*+*****


//**********Declaring Global Variables**********
static int wifiDisconnected, wifiConnected, mqttDisconnected, mqttConnected, minute, m, hour;
static double second;
double s;
//double t;
//t = millis();
//t = t/1000;
//const double t = 60.00;//is the approximate time it takes for one 60 second loop to interate
//const double t = 5.00;//is the approximate time it takes for one 5 second loop to interate
int WiFiLed = 2;//**********Pin 2 is the on board blue LED**********
int mqttLed = 5;//**********This is the pin that the LED is connected to()**********
int photoresistorPin = 34;
int lightInIt = 0;
int lightVal = 0;
long lastMsg = 0;
int loopCounter = 0;
byte mac[6];
char msgTopic[100], msgPayload[100];
char loopTopic[100], loopString[8];
char tempTopic[100], tempString[8];
char humTopic[100], humString[8];
char rssiTopic[100], rssiString[8];
char elapsedTimeTopic[100], elapsedTimeString[100];
char wifiConTopic[100], wifiConnectedString[8];
char wifiDisTopic[100], wifiDisconnectedString[8];
char mqttConTopic[100], mqttConnectedString[8];
char mqttDisTopic[100], mqttDisconnectedString[8];
char lightValTopic[100], lightValString[8];
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -25200;
const int daylightOffset_sec = 0;//0 for daylight savings and 3600 for no daylight savings

HTU21D htu;

//**************************************
//**********Connecting to WiFi**********
//**************************************
void wifiConnect()
{
  pinMode(WiFiLed, OUTPUT);
  if (!WiFi.config(local_IP, gateway, subnet)) 
  {
    Serial.println("STA Failed to configure");
  }
  WiFi.begin(ssid, password);  
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED)
  {
    for (int i = 0; i < 20; i++)
    {
      delay(1000);//*************************Delay*************************
      Serial.print(".");
      WiFi.begin(ssid, password);
      if(WiFi.status() == WL_CONNECTED)
      {
        break;
      }
    }
    if(WiFi.status() != WL_CONNECTED)
    {
      Serial.print("Having trouble connecting to WiFi, it's been 20 seconds. Trying to connect again");
    }
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
      wifiConnected++;
    }
  }
}

//***************************************
//*******Connecting to MQTT Server*******
//***************************************
void mqttConnect()
{
  pinMode(mqttLed, OUTPUT);
  client.setServer(mqtt_server, mqttPort);//configure the MQTT server with IPaddress and port
  Serial.print("Connecting to MQTT");
  
  while(!client.connected())
  {
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
      digitalWrite(mqttLed, HIGH);
      Serial.print("\tConnected to MQTT server @ ");
      Serial.println(mqtt_server);
      mqttConnected++;      
    }
    else
    {
      digitalWrite(mqttLed, LOW);
      Serial.print("Trying to connect to MQTT");
      mqttDisconnected++;
      Serial.print("State: ");
      Serial.println(client.state());
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
  Serial.print("ESP32 IP Address is ");
  Serial.println(WiFi.localIP());
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
void printLoopCounter()
{
  const char* specificLoopTopic = "/dataPts";
  strcpy(loopTopic, topicPrefix);
  strcat(loopTopic, specificLoopTopic);
  loopCounter++;
  Serial.print("Total Cycles: ");
  dtostrf(loopCounter, 1, 0, loopString);
  Serial.println(loopString);
}  

//********************************************************************************
//********Printing to serial and sending to MQTT Temperature in Farenheit*********
//********************************************************************************
void printTemp()
{
  const char* specificTempTopic = "/temp";
  strcpy(tempTopic, topicPrefix);
  strcat(tempTopic, specificTempTopic);
  float f = htu.readTemperature();
  f = (f * 9/5) + 32;
  dtostrf(f, 1, 2, tempString);// Convert the value to a char array
  Serial.print("Temperature: ");
  Serial.print(tempString);//Print the value to the serial port
  Serial.print("*F\t\t");
}

//********************************************************************************
//**********Printing to serial and sending to MQTT Humidity in Percent************
//********************************************************************************
void printHum()
{
  const char* specificHumTopic = "/hum";
  strcpy(humTopic, topicPrefix);
  strcat(humTopic, specificHumTopic);
  float h = htu.readHumidity();
  dtostrf(h, 1, 2, humString);// Convert the value to a char array
  Serial.print("Humidity: ");
  Serial.print(humString);//Print the value to the serial port
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
  long rssi = WiFi.RSSI();
  dtostrf(rssi, 1, 2, rssiString);// Convert the value to a char array
  Serial.println(rssiString);
}

//*******************************************************************************************
//****************Printing to serial and sending to MQTT Counting elapsed time***************
//*******************************************************************************************
void printElapsedTime()
{
  s = millis();
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
}

//*********************************************************************************************
//**Printing to serial and sending to MQTT Counting how many times wifiConnected has happened**
//*********************************************************************************************
void printWifiConnected()
{
  const char* specificWifiConTopic = "/wifiCon";
  strcpy(wifiConTopic, topicPrefix);
  strcat(wifiConTopic, specificWifiConTopic);
  Serial.print("WiFi has Connected to the Gateway ");
  dtostrf(wifiConnected, 1, 0, wifiConnectedString);// Convert the value to a char array
  Serial.print(wifiConnectedString);
  if(wifiConnected == 1)
  {
    Serial.println(" time");
  }
  else
  {
    Serial.println(" times");
  }
}

//*************************************************************************************************
//***Printing to serial and sending to MQTT Counting how many times wifiDisconnected has happened**
//*************************************************************************************************
void printWifiDisconnected()
{
  const char* specificWifiDisTopic = "/wifiDiscon";
  strcpy(wifiDisTopic, topicPrefix);
  strcat(wifiDisTopic, specificWifiDisTopic);
  Serial.print("WiFi has disconnected from the Gateway ");
  dtostrf(wifiDisconnected, 1, 0, wifiDisconnectedString);// Convert the value to a char array
  Serial.print(wifiDisconnectedString);
  if(wifiDisconnected == 1)
  {
    Serial.println(" time");
  }
  else
  {
    Serial.println(" times");
  }
}
    
//*********************************************************************************************
//**Printing to serial and sending to MQTT Counting how many times mqttConnected has happened**
//*********************************************************************************************
void printMqttConnected()
{
  const char* specificMqttConTopic = "/mqttCon";
  strcpy(mqttConTopic, topicPrefix);
  strcat(mqttConTopic, specificMqttConTopic);
  Serial.print("MQTT has Connected to the server ");
  dtostrf(mqttConnected, 1, 0, mqttConnectedString);// Convert the value to a char array
  Serial.print(mqttConnectedString);
  if(mqttConnected == 1)
  {
    Serial.println(" time");
  }
  else
  {
    Serial.println(" times");
  }
}

//*************************************************************************************************
//***Printing to serial and sending to MQTT Counting how many times mqttDisconnected has happened**
//*************************************************************************************************
void printMqttDisconnected()
{
  const char* specificMqttDisTopic = "/mqttDiscon";
  strcpy(mqttDisTopic, topicPrefix);
  strcat(mqttDisTopic, specificMqttDisTopic);
  Serial.print("MQTT has disconnected from the server ");
  dtostrf(mqttDisconnected, 1, 0, mqttDisconnectedString);// Convert the value to a char array
  Serial.print(mqttDisconnectedString);
  if(mqttDisconnected == 1)
  {
    Serial.println(" time");
  }
  else
  {
    Serial.println(" times");
  }
}

//*******************************************************************************
//**************Printing to serial and sending to MQTT Light Value***************
//*******************************************************************************
void printLightValue()
{
  const char* specificLightValTopic = "/lightVal";
  strcpy(lightValTopic, topicPrefix);
  strcat(lightValTopic, specificLightValTopic);
  lightVal = analogRead(photoresistorPin);
  lightVal = lightVal / 40.95;//Calibration factor
  Serial.print("The value of light is: ");
  dtostrf(lightVal, 1, 0, lightValString);
  Serial.println(lightValString);
}

//***************************************************
//********Publishing data to the MQTT Server*********
//***************************************************
void publishToMqtt()
{
//  client.loop();  //Running the PubSub Loop
  client.publish(msgTopic, msgPayload);//client.publish(const char[], const char[])
  client.publish(loopTopic, loopString);
  client.publish(tempTopic, tempString);//Publish the value to the MQTT Server
  client.publish(humTopic, humString);//Publish the value to the MQTT Server 
  client.publish(rssiTopic, rssiString);
  client.publish(elapsedTimeTopic, elapsedTimeString);
  client.publish(wifiConTopic, wifiConnectedString);
  client.publish(wifiDisTopic, wifiDisconnectedString);
  client.publish(mqttConTopic, mqttConnectedString);
  client.publish(mqttDisTopic, mqttDisconnectedString);
  client.publish(lightValTopic, lightValString);
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
  if (loopCounter != 0)
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
      Serial.println("WiFi has been disconnected to save power");
      digitalWrite(WiFiLed, LOW);
    }
  }
}

//**************************************
//********Printing Current Time*********
//**************************************
void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
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

void setup()
{
  Serial.begin(115200);
  pinMode(photoresistorPin, INPUT);
  lightInIt = analogRead(photoresistorPin);
//  wifiConnect();
//  mqttConnect();
  printEspInfo();
  htu.begin();//Starting the HTU21D Sensor
//  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);//init and get the time
//  printLocalTime();
}

void loop()
{ 
  checkingWifi();
  checkingMqtt();
  htu.begin();
//  client.loop();  //Running the PubSub Loop

  long now = millis();
  if (now - lastMsg > 1500)
  {
    lastMsg = now;
    
    printPrefix();
    printLoopCounter();
    printTemp();
    printHum();
    printRssi();
    printElapsedTime();
    printWifiConnected();
    printWifiDisconnected();
    printMqttConnected();
    printMqttDisconnected();
    printLightValue();
    publishToMqtt();
    mqttAndWifiDisconnect();
    printState();

    delay(delayTime);
  }
}
