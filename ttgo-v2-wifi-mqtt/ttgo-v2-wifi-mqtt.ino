#include <WiFi.h>
#include <PubSubClient.h>
#include <SSD1306.h>

const char* ssid = "XXXXXX";
const char* password =  "XXXXXX";
const char* mqttServer = "XXXXX";
const int mqttPort = 1883;
const char* mqttUser = "XXXXX";
const char* mqttPassword = "XXXXX";

WiFiClient espClient;
PubSubClient client(espClient);

// Screen configuration
#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 21
#define OLED_SCL 22

SSD1306 display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

void setup() {

  Serial.begin(115200);
  //Set up and reset the OLED
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(50);
  digitalWrite(OLED_RESET, HIGH);

  display.init ();
  display.flipScreenVertically ();
  display.setFont (ArialMT_Plain_10);

  display.setTextAlignment (TEXT_ALIGN_LEFT);

  display.drawString (0, 0, "Starting....");
  display.display ();

  WiFi.begin(ssid, password);

  display.clear();
  display.drawString (0, 0, "Connecting to WiFi..");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");
  display.clear();
  display.drawString (0, 0, "Connected..");
  display.display ();

  client.setServer(mqttServer, mqttPort);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {

      Serial.println("connected");

    } else {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }
  }

  client.publish("wifi01/test", "Hello from ESP32");

}

void loop() {
  client.loop();
}
