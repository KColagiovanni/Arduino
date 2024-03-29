/******************************************************************************************************
 * This program was written with the intent to measure temperature and humidity using a Sparkfun ESP32,
 * a DHT22 Temperature and Humidity sensor, and a power supply set to 3.3V. The program sends data to
 * the serial monitor and an MQTT server using specific topics. In this case the MQTT server is setup
 * on a raspberry pi or Linux PC. 
 * 
 * !!!The below parts and wiring is not for a Sparkfun ESP32 Thing and pin numbers need to be updated!!!
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
 * One side of the photoresistor pressed into a6 and the other pressed into (+)
 *      (Use a 1k Ohm resistor with the photoresistor when used in direct sunlight, a 4.7k Ohm can be used when inside)
 * One side of the resistor pressed into b6 and the other side pressed into (-)
 * Jumper wire from e6 to e19(Pin D34 on the ESP32)
 *      (Wire must be put into e19 and bent 90*, then the EWSP32 goes on top of it)
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

#define DHTPIN 19// Pin which is connected to the DHT22 sensor.
#define DHTTYPE DHT22// DHT 22 (AM2302)


//*****+*+*+*+*+*+*+*+*+*+**************************************************+*+*+*+*+*+*+*+*+*+*****
//*****+*+*+*+*+*+*+*+*+*+*****************Things to modify*****************+*+*+*+*+*+*+*+*+*+*****
//*****+*+*+*+*+*+*+*+*+*+**************************************************+*+*+*+*+*+*+*+*+*+*****
const char* ssid = "<SSID>";//**********WiFi SSID**********
const char* password =  "<SSID Password>";//**********SSID Password**********
const char* mqttUser = "<MQTT username>";
const char* mqttPassword = "<MQTT Password>";
const char* mqtt_server = "<MQTT Server IP>";
const int mqttPort = 1883;

//******************************Unique Setting for each board**************************************
const char* mqttClientID = "ESP32ClientPatio";
WiFiClient espClientPatio;//**********This is the name of the specific ESP32**********
PubSubClient client(espClientPatio);//**********This is the name of the specific ESP32**********
const char* topicPrefix = "apt/patio";//This is specifically for apt/room or area name/xxxxx
const char* topicRoom = "patio";
int delayTime = 59901;//This is adjusted so a cycle happens every 60 seconds(60000 ms)
IPAddress local_IP(192,168,0,70);//The IP Address assigned to this ESP32
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);
IPAddress dns1(75,75,75,75);//dns1 and dns2 are needed to get local time
IPAddress dns2(75,75,76,76);//dns1 and dns2 are needed to get local time
//*****+*+*+*+*+*+*+*+*+*+**************************************************+*+*+*+*+*+*+*+*+*+*****
//*****+*+*+*+*+*+*+*+*+*+*************End of things to modify**************+*+*+*+*+*+*+*+*+*+*****
//*****+*+*+*+*+*+*+*+*+*+**************************************************+*+*+*+*+*+*+*+*+*+*****


//**********Declaring Variables**********
int d, mo, y, wifiConnected, mqttConnected, resetHour, resetMin;
int WiFiLed = 5;//**********Pin 5 is the on board blue LED**********
//int mqttLed = 5;//**********This is the pin that the LED is connected to**********
int photoresistorPin = 36;
int wifiDisconnected = -1;
int mqttDisconnected = -1;
int vBattPin = 37;
int motionPin = 23;
int motionState = LOW;
int motion = 0;
int motionCount = 0;
int lightVal = 0;
int resetHTime = 23;
int resetMTime = 59;
int count = 0;
double vBatt = 0;
long lastMsg = 0;
byte mac[6];
int cycleCounter = 0;
int maxL, s, ssec, m, h;
double avgT, maxT, maxTToday, avgTCalc, avgH, maxH, maxHToday, avgHCalc, avgL, avgLCalc, avgP, maxP, avgPCalc, avgRssi, maxRssi, avgRssiCalc;
double minT = 120;
double minH = 100;
double minP = 1100;
double minRssi = -100;
double minTToday = 120;
double minHToday = 100;
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -25200;
const int   daylightOffset_sec = 0;
char msgTopic[100], msgPayload[100];
char cycleTopic[100], cycleString[8];
char wDayTopic[100], wDayString[12];
char dateTopic[100], dateStamp[10];
char timeTopic[100], timeStamp[10];
char tempTopic[100], tempString[8];
char avgTempTopic[100], avgTempString[8];
char maxTempTopic[100], maxTempString[8];
char maxTempTodayTopic[100], maxTempTodayString[8];
char minTempTopic[100], minTempString[8];
char minTempTodayTopic[100], minTempTodayString[8];
char humTopic[100], humString[8];
char avgHumTopic[100], avgHumString[8];
char maxHumTopic[100], maxHumString[8];
char maxHumTodayTopic[100], maxHumTodayString[8];
char minHumTopic[100], minHumString[8];
char minHumTodayTopic[100], minHumTodayString[8];
char rssiTopic[100], rssiString[8];
char avgRssiTopic[100], avgRssiString[8];
char maxRssiTopic[100], maxRssiString[8];
char minRssiTopic[100], minRssiString[8];
char elapsedTimeTopic[100], elapsedTimeString[18];
char lightValTopic[100], lightValString[8];
char avgLightValTopic[100], avgLightValString[8];
char maxLightValTopic[100], maxLightValString[8];
char wifiConTopic[100], wifiConnectedString[8];
char wifiDisTopic[100], wifiDisconnectedString[8];
char mqttConTopic[100], mqttConnectedString[8];
char mqttDisTopic[100], mqttDisconnectedString[8];
char vBattTopic[100], vBattString[8];
char motionTopic[100], motionString[50];
char motionCountTopic[100], motionCountString[8];
char minString[4];
char hourString[4];

DHT htu(DHTPIN, DHTTYPE);

//**************************************
//**************** WiFi ****************
//**************************************
void Wifi()
{
  pinMode(WiFiLed, OUTPUT);
  if(!WiFi.config(local_IP, gateway, subnet, dns1, dns2)) 
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
//**************** MQTT ****************
//**************************************
void Mqtt()
{  
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
void printCycleCounter()
{
  const char* specificCycleTopic = "/dataPts";
  strcpy(cycleTopic, topicPrefix);
  strcat(cycleTopic, specificCycleTopic);
  Serial.print("Total Cycles: ");
  dtostrf(cycleCounter, 1, 0, cycleString);
  Serial.println(cycleString);
}  

//********************************************************************************
//**************Printing to serial and sending to MQTT a Timestamp****************
//********************************************************************************
void printLocalTime()
{
  const char* specificWDayTopic = "/day";
  strcpy(wDayTopic, topicPrefix);
  strcat(wDayTopic, specificWDayTopic);
  const char* specificDateTopic = "/date";
  strcpy(dateTopic, topicPrefix);
  strcat(dateTopic, specificDateTopic);
  const char* specificTimeTopic = "/time";
  strcpy(timeTopic, topicPrefix);
  strcat(timeTopic, specificTimeTopic);
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  char secString[4];
  char dayString[4];
  char monthString[4];
  char yearString[6];
  String wDayS;
  const char colon[2] = ":";
  const char space[2] = " ";
  const char dash[2] = "-";
  char zero[2] = "0";
  s = timeinfo.tm_sec;
  itoa(s, secString, 10);
  m = timeinfo.tm_min;
  resetMin = m;
  itoa(m, minString, 10);
  h = timeinfo.tm_hour;
  resetHour = h;
  itoa(h, hourString, 10);
  d = timeinfo.tm_mday;
  itoa(d, dayString, 10);
  mo = timeinfo.tm_mon;
  mo += 1;
  itoa(mo, monthString, 10);
  y = timeinfo.tm_year;
  y  += 1900;
  itoa(y, yearString, 10);
  strcpy(dateStamp, yearString);  
//  strcat(dateStamp, dash);
  if(mo < 10)
  {
    strcat(dateStamp, "0");    
  }
  strcat(dateStamp, monthString);
//  strcat(dateStamp, dash);
  if(d < 10)
  {
    strcat(dateStamp, "0");
  }
  strcat(dateStamp, dayString);  
//  strcat(dateStamp, space);
  if(h < 10)
  {
    strcpy(timeStamp, zero);
    strcat(timeStamp, hourString);
  }
  else
  {
    strcpy(timeStamp, hourString);
  }
//  strcat(timeStamp, colon);
  if(m < 10)
  {
    strcat(timeStamp, zero);
    strcat(timeStamp, minString);
  }
  else
  {
    strcat(timeStamp, minString);
  }
//  strcat(timeStamp, colon);
  if(s < 10)
  {
    strcat(timeStamp, zero);
    strcat(timeStamp, secString);
  }
  else
  {
    strcat(timeStamp, secString);
  }
  int wDay, yDay, isdst;
  wDay = timeinfo.tm_wday;
  switch(wDay)
  {
    case 0:
      wDayS = "'Sunday'";
      break;
    case 1:
      wDayS = "'Monday'";
      break;
    case 2:
      wDayS = "'Tuesday'";
      break;
    case 3:
      wDayS = "'Wednesday'";
      break;
    case 4:
      wDayS = "'Thursday'";
      break;
    case 5:
      wDayS = "'Friday'";
      break;
    case 6:
      wDayS = "'Saturday'";
      break;
  }
  Serial.print("Day: ");
  int len = wDayS.length() + 1;
  wDayS.toCharArray(wDayString, len);// Convert the value to a char array
  Serial.println(wDayString);
  Serial.print("Date: ");
  Serial.println(dateStamp);
  Serial.print("Time: ");
  Serial.println(timeStamp);
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
    Serial.println("Temp Sensor not detected");
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
    if (cycleCounter == 1)
    {
      avgT = f;
      avgTCalc = f;     
    }
    else
    {
      avgT = avgTCalc / cycleCounter;
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

//******************************************************************************************
//******************Printing to serial Daily Max Temperature in Farenheit*******************
//******************************************************************************************
void calcMaxTempToday(double f)
{
  const char* specificTempTopic = "/maxTempToday";
  strcpy(maxTempTodayTopic, topicPrefix);
  strcat(maxTempTodayTopic, specificTempTopic);
  if(resetHour == resetHTime && resetMin == resetMTime)
  {
    maxTToday = 0;
  }
  maxTToday = max(maxTToday, f);
  dtostrf(maxTToday, 1, 2, maxTempTodayString);// Convert the value to a char array
  Serial.print("Max Temp Today is: ");
  Serial.print(maxTempTodayString);
  Serial.print("*F\t");
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

//******************************************************************************************
//******************Printing to serial Daily Min Temperature in Farenheit*******************
//******************************************************************************************
void calcMinTempToday(double f)
{
  const char* specificTempTopic = "/minTempToday";
  strcpy(minTempTodayTopic, topicPrefix);
  strcat(minTempTodayTopic, specificTempTopic);
  if(resetHour == resetHTime && resetMin == resetMTime)
  {
    minTToday = 120;
  }
  minTToday = min(minTToday, f);
  dtostrf(minTToday, 1, 2, minTempTodayString);// Convert the value to a char array
  Serial.print("Min Temp Today is: ");
  Serial.print(minTempTodayString);
  Serial.print("*F\t");
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
    Serial.println("%\t\t");
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
    if (cycleCounter == 1)
    {
      avgH = h;
      avgHCalc = h;     
    }
    else
    {
      avgH = avgHCalc / cycleCounter;
    }
    dtostrf(avgH, 1, 2, avgHumString);// Convert the value to a char array
    Serial.print("Avg Hum is: ");
    Serial.print(avgH);
    Serial.println("%\t\t");
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
  Serial.println("%\t\t");
}

//*************************************************************************************
//******************Printing to serial Daily Max Humidity in Percent*******************
//*************************************************************************************
void calcMaxHumToday(double h)
{
  const char* specificHumTopic = "/maxHumToday";
  strcpy(maxHumTodayTopic, topicPrefix);
  strcat(maxHumTodayTopic, specificHumTopic);
  if(resetHour == resetHTime && resetMin == resetMTime)
  {
    maxHToday = 0;
  }
  maxHToday = max(maxHToday, h);
  dtostrf(maxHToday, 1, 2, maxHumTodayString);// Convert the value to a char array
  Serial.print("Max Hum Today is: ");
  Serial.print(maxHumTodayString);
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
  Serial.println("%\t\t");
}

//*************************************************************************************
//******************Printing to serial Daily Min Humidity in Percent*******************
//*************************************************************************************
void calcMinHumToday(double h)
{
  const char* specificHumTopic = "/minHumToday";
  strcpy(minHumTodayTopic, topicPrefix);
  strcat(minHumTodayTopic, specificHumTopic);
  if(resetHour == resetHTime && resetMin == resetMTime)
  {
    minHToday = 100;
  }
  minHToday = min(minHToday, h);
  dtostrf(minHToday, 1, 2, minHumTodayString);// Convert the value to a char array
  Serial.print("Min Hum Today is: ");
  Serial.print(minHumTodayString);
  Serial.println("%");
}

//********************************************************************************
//******************Printing to serial and sending to MQTT RSSI*******************
//********************************************************************************
double printRssi()
{
  const char* specificRssiTopic = "/rssi";
  strcpy(rssiTopic, topicPrefix);
  strcat(rssiTopic, specificRssiTopic);
  double rssi = WiFi.RSSI();
  dtostrf(rssi, 1, 0, rssiString);// Convert the value to a char array
  Serial.print("RSSI: ");
  Serial.print(rssiString);
  Serial.println("dBm");
  return rssi;
}

//*****************************************************************************************************
//***********************Printing to serial and send to MQTT Avg Pressure in hPa***********************
//*****************************************************************************************************
void calcAvgRssi(int rssi)
{
  if (!isnan(rssi) || rssiString == "nan")
  {
    const char* specificAvgRssiTopic = "/avgRssi";
    strcpy(avgRssiTopic, topicPrefix);
    strcat(avgRssiTopic, specificAvgRssiTopic);
    avgRssiCalc = avgRssiCalc + rssi;
    if (cycleCounter == 1)
    {
      avgRssi = rssi;
      avgRssiCalc = rssi;     
    }
    else
    {
      avgRssi = avgRssiCalc / cycleCounter;
    }
    dtostrf(avgRssi, 1, 2, avgRssiString);// Convert the value to a char array
    Serial.print("Avg RSSI is: ");
    Serial.print(avgRssiString);
    Serial.println("dBm");
  }
}

//*******************************************************************************************
//******************Printing to serial and sending to MQTT Max RSSI in dBm*******************
//*******************************************************************************************
void calcMaxRssi(double rssi)
{
  const char* specificMaxRssiTopic = "/maxRssi";
  strcpy(maxRssiTopic, topicPrefix);
  strcat(maxRssiTopic, specificMaxRssiTopic);
  maxRssi = min(maxRssi, rssi);
  dtostrf(maxRssi, 1, 0, maxRssiString);// Convert the value to a char array
  Serial.print("Max RSSI is: ");
  Serial.print(maxRssiString);
  Serial.println("dBm");  
}

//*****************************************************************************************************
//***********************Printing to serial and send to MQTT Min Pressure in hPa***********************
//*****************************************************************************************************
void calcMinRssi(double rssi)
{
  if(rssi != 0)
  {
    const char* specificMinRssiTopic = "/minRssi";
    strcpy(minRssiTopic, topicPrefix);
    strcat(minRssiTopic, specificMinRssiTopic);
    minRssi = max(minRssi, rssi);
    dtostrf(minRssi, 1, 0, minRssiString);// Convert the value to a char array
    Serial.print("Min RSSI is: ");
    Serial.print(minRssiString);
    Serial.println("dBm");
  }
}

//*******************************************************************************************
//****************Printing to serial and sending to MQTT Counting elapsed time***************
//*******************************************************************************************
void motionSensor()
{
  const char* specificMotionTopic = "/motion";
  strcpy(motionTopic, topicPrefix);
  strcat(motionTopic, specificMotionTopic);
  const char* specificMotionCountTopic = "/motionCount";
  strcpy(motionCountTopic, topicPrefix);
  strcat(motionCountTopic, specificMotionCountTopic);
  pinMode(motionPin, INPUT);
}

//*******************************************************************************************
//****************Printing to serial and sending to MQTT Counting elapsed time***************
//*******************************************************************************************
void printElapsedTime()
{
  struct tm timeinfo;
  int second = 0;
  int minute = 0;
  int hour = 0;
  if(!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  if(cycleCounter == 1)
  {
    ssec = timeinfo.tm_sec;
    m = 0;
    h = 0;
  }
  else
  {
    s = (cycleCounter - 1) * 60 + (timeinfo.tm_sec - ssec);
  }
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
  String ss;
  String mm;
  String lz = "0";
  if(second < 10)
  {
    ss.concat(lz);
  }
  else
  {
    ss = "";
  }
  if(minute < 10)
  {
    mm.concat(lz);
  }
  else
  {
    mm = "";
  }
  int len;
  const char* specificElapsedTimeTopic = "/elapsedTime";
  strcpy(elapsedTimeTopic, topicPrefix);
  strcat(elapsedTimeTopic, specificElapsedTimeTopic);
  elapsedTime = hour + mm + minute + ss + second;
  Serial.print("Elapsed Time: ");
  len = elapsedTime.length() + 1;
  elapsedTime.toCharArray(elapsedTimeString, len);// Convert the value to a char array
  Serial.println(elapsedTimeString);
}


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
    if(lightVal == 0)
    {
      avgL = avgL;
    }
    else
    {
      if (cycleCounter == 1)
      {
        avgL = lightVal;
        avgLCalc = lightVal;     
      }
      else
      {
        avgL = avgLCalc / cycleCounter;
      }
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
  Serial.print(maxLightValString);  Serial.println("%;  ");
}

//*********************************************************************************************
//****************Printing to serial how many times wifiConnected has happened*****************
//*********************************************************************************************
void printWifiConnected()
{
  const char* specificWifiConTopic = "/wifiCon";
  strcpy(wifiConTopic, topicPrefix);
  strcat(wifiConTopic, specificWifiConTopic);
  Serial.print("WiFi has Conected to the server ");
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
//*****************Printing to serial how many times wifiDisconnected has happened*****************
//*************************************************************************************************
void printWifiDisconnected()
{
  const char* specificWifiDisTopic = "/wifiDiscon";
  strcpy(wifiDisTopic, topicPrefix);
  strcat(wifiDisTopic, specificWifiDisTopic);
  Serial.print("Wifi has disconnected from the server ");
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
//****************Printing to serial how many times mqttConnected has happened*****************
//*********************************************************************************************
void printMqttConnected()
{
  const char* specificMqttConTopic = "/mqttCon";
  strcpy(mqttConTopic, topicPrefix);
  strcat(mqttConTopic, specificMqttConTopic);
  Serial.print("MQTT has Conected to the server ");
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
//*****************Printing to serial how many times mqttDisconnected has happened*****************
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
//********************Printing to serial MQTT Battery Voltage********************
//*******************************************************************************
double printBattVoltage()
{
  const char* specificVBattTopic = "/vBatt";
  strcpy(vBattTopic, topicPrefix);
  strcat(vBattTopic, specificVBattTopic);
  vBatt = analogRead(vBattPin);
  vBatt = vBatt / 809;//Calibration factor
  Serial.print("The Battery Voltage is: ");
  dtostrf(vBatt, 1, 2, vBattString);
  Serial.print(vBattString);
  Serial.println("V");
  return vBatt;
}

//***************************************************
//********Publishing data to the MQTT Server*********
//***************************************************
void publishToMqtt()
{
  client.publish(msgTopic, msgPayload);//client.publish(const char[], const char[])
  client.publish(cycleTopic, cycleString);
  client.publish(tempTopic, tempString);//Publish the value to the MQTT Server
  client.publish(avgTempTopic, avgTempString);//Publish the value to the MQTT Server
  client.publish(maxTempTopic, maxTempString);//Publish the value to the MQTT Server
  client.publish(maxTempTodayTopic, maxTempTodayString);//Publish the value to the MQTT Server
  client.publish(minTempTopic, minTempString);//Publish the value to the MQTT Server
  client.publish(minTempTodayTopic, minTempTodayString);//Publish the value to the MQTT Server
  client.publish(humTopic, humString);//Publish the value to the MQTT Server 
  client.publish(avgHumTopic, avgHumString);//Publish the value to the MQTT Server 
  client.publish(maxHumTopic, maxHumString);//Publish the value to the MQTT Server 
  client.publish(maxHumTodayTopic, maxHumTodayString);//Publish the value to the MQTT Server 
  client.publish(minHumTopic, minHumString);//Publish the value to the MQTT Server 
  client.publish(minHumTodayTopic, minHumTodayString);//Publish the value to the MQTT Server 
  client.publish(rssiTopic, rssiString);
  client.publish(avgRssiTopic, avgRssiString);//Publish the value to the MQTT Server 
  client.publish(maxRssiTopic, maxRssiString);//Publish the value to the MQTT Server 
  client.publish(minRssiTopic, minRssiString);//Publish the value to the MQTT Server 
  client.publish(lightValTopic, lightValString);
  client.publish(avgLightValTopic, avgLightValString);
  client.publish(maxLightValTopic, maxLightValString);
  client.publish(wDayTopic, wDayString);
  client.publish(dateTopic, dateStamp);
  client.publish(timeTopic, timeStamp);
  client.publish(elapsedTimeTopic, elapsedTimeString);
  client.publish(wifiConTopic, wifiConnectedString);
  client.publish(wifiDisTopic, wifiDisconnectedString);
  client.publish(mqttConTopic, mqttConnectedString);
  client.publish(mqttDisTopic, mqttDisconnectedString);
//  client.publish(vBattTopic, vBattString);
  client.loop();  //Running the PubSub Loop
//  client.disconnect();
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

void delayLoop(){}

void setup(){
  Serial.begin(115200);
  while(!Serial) {} // Wait
  pinMode(photoresistorPin, INPUT);
  Wifi();
  Mqtt();
  printEspInfo();
  htu.begin();
}

void loop() 
{
  motion = digitalRead(motionPin);  // read input value
  if (motion == HIGH)
  {
    motionSensor();
    if (motionState == LOW)
    {
      motionCount++;
      dtostrf(motionCount, 1, 0, motionCountString);
      const char* motionMsg = "Motion Detected!";
      strcpy(motionString, motionMsg);
      Serial.println("Motion detected!");
      motionState = HIGH;
      Serial.print("Motion has been detected ");
      Serial.print(motionCount);
      if(motionCount < 2)
      {
        Serial.println(" time");
      }
      else
      {
        Serial.println(" times");
      }
      client.publish(motionTopic, motionString);
      client.publish(motionCountTopic, motionCountString);
    }    
  }
  else
  {
    if (motionState == HIGH){
//      motionSensor();
      const char* motionMsg = "No Motion Detected";
      strcpy(motionString, motionMsg);
      Serial.println("Motion ended!");
      motionState = LOW;
      client.publish(motionTopic, motionString);
    }
  }
//  else
//  {
  while(count > 59)
  {
    count = 0;
    ++cycleCounter;  //Increment cycle number and print it every reboot
    Wifi();
    Mqtt();
    printPrefix();
    printCycleCounter();
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();
    printElapsedTime();
    double f = printTemp();
    double h = printHum();
    if (!isnan(f) || !isnan(h))
    {
      avgTemp(f);
      avgHum(h);
      calcMaxTemp(f);
      calcMaxHum(h);
      calcMaxTempToday(f);
      calcMaxHumToday(h);
      calcMinTemp(f);
      calcMinHum(h); 
      calcMinTempToday(f);
      calcMinHumToday(h);
    }
    double rssi = printRssi();
    calcAvgRssi(rssi);
    calcMaxRssi(rssi);
    calcMinRssi(rssi);
    int lightVal = printLightValue();
    avgLightVal(lightVal);
    calcMaxLightVal(lightVal);
    printWifiConnected();
    printWifiDisconnected();
    printMqttConnected();
    printMqttDisconnected();
//    printBattVoltage();
    publishToMqtt();
    printState();
  }
  count++;
  delay(1000);
  Serial.println(count);

//  }
}
