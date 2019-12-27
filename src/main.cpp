#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <AutoConnect.h>
#include <PubSubClient.h>

#define LED_PIN D6

#define LED_SKIP 2
#define NUM_LEDS 4

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

void rootPage()
{
  char content[] = "Hello, world";
  Server.send(200, "text/plain", content);
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
  Serial.begin(115200);
  Serial.println("Starting...");
  Serial.println();

  strip.begin();
  setAll(100, 100, 100);
  setPixel(1, 100, 0, 0);
  showStrip();

  digitalWrite(BUILTIN_LED1, HIGH); // Turn the LED off by making the voltage HIGH
  digitalWrite(BUILTIN_LED2, LOW);  // Turn the LED ON by making the voltage LOW

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
}

void loop()
{
  Portal.handleClient();

  mqttClient.loop();


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
