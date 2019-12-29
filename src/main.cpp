#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <AutoConnect.h>
#include <PubSubClient.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define LED_PIN D6

#define LED_SKIP 2
#define NUM_LEDS 4

// Data wire is plugged into port D2 on the ESP8266
#define ONE_WIRE_BUS D2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dallasSensors(&oneWire);
int dallasDeviceCount = 0;

const short int BUILTIN_LED1 = 2;  //GPIO2
const short int BUILTIN_LED2 = 16; //GPIO16
const char *MQTTSserver = "10.20.0.40";

// Function prototypes
void showStrip();
void setPixel(int Pixel, byte red, byte green, byte blue);
void setAll(byte red, byte green, byte blue);
void subscribeReceive(char *topic, byte *payload, unsigned int length);

Adafruit_NeoPixel strip(NUM_LEDS *LED_SKIP, LED_PIN, NEO_GRB + NEO_KHZ800);

ESP8266WebServer Server(80);

AutoConnect Portal(Server);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

long _millis;

void rootPage()
{
  char content[] = "Hello, world";
  Server.send(200, "text/plain", content);
}

/*bool MakeRestCall(char *url, DynamicJsonDocument *jsonBuffer)
{
  Serial.print("Making rest call to ");
  Serial.print(url);

  HTTPClient http; //Object of class HTTPClient
  //http.setTimeout(1000);
  http.begin(url);
  int httpCode = http.GET();
  Serial.println("Return Code ");
  Serial.println(httpCode);

  if (httpCode > 0)
  {
    Serial.println("Succeeded.");
    const size_t bufferSize = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(5) + JSON_OBJECT_SIZE(8) + 370;
    Serial.print(http.getString());
    jsonBuffer = new DynamicJsonDocument(bufferSize);
    DeserializationError error = deserializeJson(*jsonBuffer, http.getString());
    if (error)
    {
      Serial.println("Unable to deserialize Json");
      return false;
    }
    //  JsonObject root = jsonBuffer.parseObject(http.getString());

    /*int id = root["id"]; 
      const char* name = root["name"]; 
      const char* username = root["username"]; 
      const char* email = root["email"]; 

      Serial.print("Name:");
      Serial.println(name);
      Serial.print("Username:");
      Serial.println(username);
      Serial.print("Email:");
      Serial.println(email);*
  }
  else
  {
    Serial.println("No response.");
  }

  http.end(); //Close connection

  return true;
}*/

bool OctoPrintGetVersion(char *oServer, char *api, char *server, char *description)
{
  bool retval = false;
  char url[100];
  strcpy(url, "http://");
  strcat(url, oServer);
  strcat(url, "/api/version");

  Serial.print("Making rest call to ");
  Serial.println(url);

  HTTPClient http; //Object of class HTTPClient
  http.setTimeout(1000);
  http.begin(url);
  int httpCode = http.GET();
  Serial.print("Return Code: ");
  Serial.print(httpCode);

  if (httpCode > 0)
  {
    Serial.println(" Succeeded.");
    const size_t bufferSize = JSON_OBJECT_SIZE(3) + 60;

    DynamicJsonDocument jsonBuffer(bufferSize);
    DeserializationError error = deserializeJson(jsonBuffer, http.getString());
    if (error)
    {
      Serial.println("Unable to deserialize Json");
      Serial.println(error.c_str());
    }

    strcpy(api, jsonBuffer["api"].as<char *>());
    strcpy(server, jsonBuffer["server"].as<char *>());
    strcpy(description, jsonBuffer["text"].as<char *>());
    retval = true;
  }
  else
  {
    Serial.println(" No response.");
  }

  http.end(); //Close connection

  return retval;
}

bool OctoPrintEnclosureGetTemperature(char *oServer, int id, float *temperature)
{
  bool retval = false;
  char url[100];
  char sid[10];
  itoa(id, sid, 10);
  strcpy(url, "http://");
  strcat(url, oServer);
  strcat(url, "/plugin/enclosure/inputs/");
  strcat(url, sid);

  Serial.print("Making rest call to ");
  Serial.println(url);

  HTTPClient http; //Object of class HTTPClient
  http.setTimeout(1000);
  http.begin(url);
  int httpCode = http.GET();
  Serial.print("Return Code: ");
  Serial.print(httpCode);

  if (httpCode > 0)
  {
    Serial.println(" Succeeded.");
    const size_t bufferSize = JSON_OBJECT_SIZE(19) + 396;

    DynamicJsonDocument jsonBuffer(bufferSize);
    DeserializationError error = deserializeJson(jsonBuffer, http.getString());
    if (error)
    {
      Serial.println("Unable to deserialize Json");
      Serial.println(error.c_str());
    }

    *temperature = jsonBuffer["temp_sensor_temp"];
    retval = true;
  }
  else
  {
    Serial.println(" No response.");
  }

  http.end(); //Close connection

  return retval;
}

void MQTTCconnect()
{
  Serial.print("Attempting MQTT connection...");
  // Create a random client ID
  String clientId = "ESP8266Client-";
  clientId += String(random(0xffff), HEX);
  // Attempt to connect
  if (mqttClient.connect(clientId.c_str()))
  {
    Serial.println("Connection has been established");
    // Once connected, publish an announcement...
    if (mqttClient.publish("3D-Enclosure", "hello world"))
    {
      Serial.println("Publish message success");
    }
    else
    {
      Serial.println("Could not send message :(");
    }
    // ... and resubscribe
    // Ensure that we are subscribed to the topic "MakerIOTopic"
    mqttClient.subscribe("3D-Enclosure");
  }
  else
  {
    Serial.print("Looks like the server connection failed. State ");
    Serial.println(mqttClient.state());
  }
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    if (deviceAddress[i] < 0x10)
      Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7)
      Serial.print(", ");
  }
  Serial.println("");
}

void pollTemperatures()
{
  Serial.println("Polling Dallas Temperature Sensors.");
  dallasSensors.requestTemperatures();
  for (int i = 0; i < dallasDeviceCount; i++)
  {
    float temperatureC = dallasSensors.getTempCByIndex(i);
    Serial.print(temperatureC);
    Serial.println("ºC");
  }

  Serial.println("Polling OctoPrint Enclosure Temperature Sensors.");
  float temperature;
  for (int i = 1; i <= 3; i++)
  {
    if (OctoPrintEnclosureGetTemperature("10.20.0.147", i, &temperature))
    {
      Serial.print("Temperature Sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(temperature);
    }
  }
}

void setup()
{
  pinMode(BUILTIN_LED1, OUTPUT); // Initialize the BUILTIN_LED1 pin as an output
  pinMode(BUILTIN_LED2, OUTPUT); // Initialize the BUILTIN_LED2 pin as an output

  digitalWrite(BUILTIN_LED1, HIGH); // Turn the LED ON by making the voltage LOW
  digitalWrite(BUILTIN_LED2, HIGH); // Turn the LED off by making the voltage HIGH

  setAll(100, 100, 100);
  setPixel(1, 100, 0, 0);
  showStrip();

  delay(1000);
  Serial.begin(9600);
  // Serial.begin(115200);
  Serial.println("Starting...");
  Serial.println();

  strip.begin();
  setAll(100, 100, 100);
  setPixel(1, 100, 0, 0);
  showStrip();

  digitalWrite(BUILTIN_LED1, HIGH); // Turn the LED off by making the voltage HIGH
  digitalWrite(BUILTIN_LED2, LOW);  // Turn the LED ON by making the voltage LOW

  // Start up the library
  DeviceAddress Thermometer;
  dallasSensors.begin();

  // locate devices on the bus
  Serial.println("Locating devices...");
  Serial.print("Found ");
  dallasDeviceCount = dallasSensors.getDeviceCount();
  Serial.print(dallasDeviceCount, DEC);
  Serial.println(" devices.");
  Serial.println("");

  Serial.println("Printing addresses...");
  for (int i = 0; i < dallasDeviceCount; i++)
  {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" : ");
    dallasSensors.getAddress(Thermometer, i);
    printAddress(Thermometer);
  }

  Server.on("/", rootPage);
  if (Portal.begin())
  {
    Serial.println("HTTP server:" + WiFi.localIP().toString());
  }

  digitalWrite(BUILTIN_LED1, LOW); // Turn the LED off by making the voltage HIGH
  digitalWrite(BUILTIN_LED2, LOW); // Turn the LED ON by making the voltage LOW
  delay(1000);
  mqttClient.setServer(MQTTSserver, 1883);
  mqttClient.setCallback(subscribeReceive);
  MQTTCconnect();

  char api[20];
  char server[20];
  char description[20];

  if (OctoPrintGetVersion("10.20.0.147", api, server, description))
  // ConnectOctoPrint();
  {
    Serial.print("API Version: ");
    Serial.println(api);
    Serial.print("Server Version: ");
    Serial.println(server);
    Serial.print("Description: ");
    Serial.println(description);
  }
  else
  {
    Serial.println("OctoPrint not connected!!");
  }



  _millis = millis();
}

void loop()
{
  Portal.handleClient();

  mqttClient.loop();

  if ( millis() > _millis+ 2000)
  {
    _millis = millis();

    // MakeRestCall("http://jsonplaceholder.typijjhcode.com/users/1");
      pollTemperatures();
  }


}

void subscribeReceive(char *topic, byte *payload, unsigned int length)
{
  // Print the topic
  Serial.print("Topic: ");
  Serial.println(topic);

  // Print the message
  Serial.print("Message: ");
  for (int i = 0; i < length; i++)
  {
    Serial.print(char(payload[i]));
  }

  // Print a newline
  Serial.println("");
}

// Apply LED color changes
void showStrip()
{
#ifdef ADAFRUIT_NEOPIXEL_H
  // NeoPixel
  strip.show();
#endif
#ifndef ADAFRUIT_NEOPIXEL_H
  // FastLED
  FastLED.show();
#endif
}

// Set a LED color (not yet visible)
void setPixel(int Pixel, byte red, byte green, byte blue)
{
#ifdef ADAFRUIT_NEOPIXEL_H
  // NeoPixel
  strip.setPixelColor(Pixel * LED_SKIP, strip.Color(red, green, blue));
#endif
#ifndef ADAFRUIT_NEOPIXEL_H
  // FastLED
  leds[Pixel].r = red;
  leds[Pixel].g = green;
  leds[Pixel].b = blue;
#endif
}

// Set all LEDs to a given color and apply it (visible)
void setAll(byte red, byte green, byte blue)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    setPixel(i, red, green, blue);
  }
  showStrip();
}
