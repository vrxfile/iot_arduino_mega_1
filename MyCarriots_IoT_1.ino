#include <SPI.h>
#include "DHT.h"
#include <Wire.h>
#include <Encoder.h>
#include <UIPEthernet.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_SleepyDog.h>

const String APIKEY = "97f31f8321a8df31ed5efbb4f3e22072d5732d1b5d075f5d3ee85f74115d1716";
const String DEVICE = "defaultDevice@vrxfile.vrxfile";

#define SERVER_UPDATE_TIME 60000         // Update Carriots data server every 60000 ms (1 minute)

#define TIMEOUT 1000 // 1 second timout

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xAE, 0xCD };
// Local IP if DHCP fails
IPAddress ip(192, 168, 1, 250);
IPAddress dnsServerIP(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

IPAddress carriots_server(82, 223, 244, 60);

EthernetClient client;

unsigned long timer1 = 0;
unsigned long timer2 = 0;

int failedResponse = 0;

#define DHTPIN 5
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

#define GasSensorPIN A0
#define FlameSensorPIN A1
#define SoundSensorPIN A2
#define LightSensorPIN A3
#define VibroSensorPIN 7

Encoder myEnc(VibroSensorPIN, VibroSensorPIN);

float h1 = 0;
float t1 = 0;
float hic1 = 0;
float p1 = 0;
float t2 = 0;
float alt1 = 0;
float mx1 = 0;
float my1 = 0;
float mz1 = 0;
float gas1 = 0;
float flame1 = 0;
float sound1 = 0;
float light1 = 0;
long vibro1 = 0;

void setup()
{
  // Serial port
  Serial.begin(9600);
  Serial.println("-= Carriots data client =-\n");

  // Watchdog
  int countdownMS = Watchdog.enable(8000);
  Serial.print("Enabled the watchdog with max countdown of ");
  Serial.print(countdownMS, DEC);
  Serial.println(" milliseconds!");
  Serial.println();

  // Ethernet ENC28J60
  if (Ethernet.begin(mac) == 0)
  {
    Serial.println("Failed to configure Ethernet using DHCP");
    Ethernet.begin(mac, ip, dnsServerIP, gateway, subnet);
  }
  Serial.print("LocalIP:\t\t");
  Serial.println(Ethernet.localIP());
  Serial.print("SubnetMask:\t\t");
  Serial.println(Ethernet.subnetMask());
  Serial.print("GatewayIP:\t\t");
  Serial.println(Ethernet.gatewayIP());
  Serial.print("dnsServerIP:\t\t");
  Serial.println(Ethernet.dnsServerIP());
  Serial.println("");

  // Reset watchdog timer
  Watchdog.reset();

  // DHT11
  dht.begin();

  // BMP085
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor!");
  }

  // HMC5883L
  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883L sensor!");
  }

  // Reset watchdog timer
  Watchdog.reset();
}

void loop()
{
  // Test for timeout
  if (millis() > timer1 + SERVER_UPDATE_TIME)
  {
    // Read all sensors
    readAllSensors();

    // Print read data
    printAllSenors();

    // Send data to servser
    sendCarriotsStream();

    // Reset vibro sensor
    myEnc.write(0);

    // Reset timeout timer
    timer1 = millis();
  }

  // Vibro sensor
  vibro1 = myEnc.read();
   
  // Reset watchdog timer
  Watchdog.reset();

  delay(10);
}

void sendCarriotsStream()
{
  if (client.connect(carriots_server, 80))
  {
    // Reset watchdog timer
    Watchdog.reset();

    if (client.connected())
    {
      Serial.println("Sending data to Carriots server...\n");

      String json_data = "{\"protocol\":\"v2\",\"device\":\"";
      json_data = json_data + DEVICE;
      json_data = json_data + "\",\"at\":\"now\",\"data\":{";
      json_data = json_data + "\"temperature1\":";
      json_data = json_data + "\"" + String(t1, 2) + "\",";
      json_data = json_data + "\"temperature2\":";
      json_data = json_data + "\"" + String(t2, 2) + "\",";
      json_data = json_data + "\"humidity\":";
      json_data = json_data + "\"" + String(h1, 2) + "\",";
      json_data = json_data + "\"pressure\":";
      json_data = json_data + "\"" + String(p1, 2) + "\",";
      json_data = json_data + "\"magnetic_x\":";
      json_data = json_data + "\"" + String(mx1, 2) + "\",";
      json_data = json_data + "\"magnetic_y\":";
      json_data = json_data + "\"" + String(my1, 2) + "\",";
      json_data = json_data + "\"magnetic_z\":";
      json_data = json_data + "\"" + String(mz1, 2) + "\",";
      json_data = json_data + "\"light\":";
      json_data = json_data + "\"" + String(light1, 2) + "\",";
      json_data = json_data + "\"sound\":";
      json_data = json_data + "\"" + String(sound1, 2) + "\",";
      json_data = json_data + "\"flame\":";
      json_data = json_data + "\"" + String(flame1, 2) + "\",";
      json_data = json_data + "\"gas\":";
      json_data = json_data + "\"" + String(gas1, 2) + "\",";
      json_data = json_data + "\"vibration\":";
      json_data = json_data + "\"" + String(vibro1, DEC) + "\"}}";

      Serial.println("Data to be send:");
      Serial.println(json_data);

      client.println("POST /streams HTTP/1.1");
      client.println("Host: api.carriots.com");
      client.println("Accept: application/json");
      client.println("User-Agent: Arduino-Carriots");
      client.println("Content-Type: application/json");
      client.print("carriots.apikey: ");
      client.println(APIKEY);
      client.print("Content-Length: ");
      int thisLength = json_data.length();
      client.println(thisLength);
      client.println("Connection: close");
      client.println();
      client.println(json_data);

      // Reset watchdog timer
      Watchdog.reset();

      delay(1000);

      // Reset watchdog timer
      Watchdog.reset();

      timer2 = millis();
      while ((client.available() == 0) && (millis() < timer2 + TIMEOUT));

      // Reset watchdog timer
      Watchdog.reset();

      while (client.available() > 0)
      {
        char inData = client.read();
        Serial.print(inData);
      }
      Serial.println("\n");

      // Reset watchdog timer
      Watchdog.reset();

      client.stop();

      // Reset watchdog timer
      Watchdog.reset();
    }
  }
}

void readAllSensors()
{
  // Reset watchdog timer
  Watchdog.reset();

  // DHT11
  h1 = dht.readHumidity();
  t1 = dht.readTemperature();
  if (isnan(h1) || isnan(t1)) {
    Serial.println("Failed to read from DHT sensor!");
  }
  else
  {
    hic1 = dht.computeHeatIndex(t1, h1, false);
  }

  // Reset watchdog timer
  Watchdog.reset();

  // BMP085
  sensors_event_t p_event;
  bmp.getEvent(&p_event);
  if (p_event.pressure) {
    p1 = p_event.pressure;
    t2; bmp.getTemperature(&t2);
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    alt1 = bmp.pressureToAltitude(seaLevelPressure, p_event.pressure);
  }

  // Reset watchdog timer
  Watchdog.reset();

  // HMC5883L
  sensors_event_t m_event;
  mag.getEvent(&m_event);
  mx1 = m_event.magnetic.x;
  my1 = m_event.magnetic.y;
  mz1 = m_event.magnetic.z;

  // Reset watchdog timer
  Watchdog.reset();

  // Vibro sensor
  //vibro1 = myEnc.read();

  // Analog sensors
  float sens1 = analogRead(GasSensorPIN);
  float sens2 = analogRead(FlameSensorPIN);
  float sens3 = analogRead(SoundSensorPIN);
  float sens4 = analogRead(LightSensorPIN);
  gas1 = sens1 / 1023.00 * 100.00;
  flame1 = (1023.00 - sens2) / 1023.00 * 100.00;
  sound1 = sens3 / 1023.00 * 100.00;
  light1 = (1023.00 - sens4) / 1023.00 * 100.00;

  // Reset watchdog timer
  Watchdog.reset();
}

void printAllSenors()
{
  // Reset watchdog timer
  Watchdog.reset();

  Serial.print("Temperature1: ");
  Serial.print(t1);
  Serial.println(" *C");
  Serial.print("Temperature2: ");
  Serial.print(t2);
  Serial.println(" *C");
  Serial.print("Humidity1: ");
  Serial.print(h1);
  Serial.println(" %");
  Serial.print("Heat index1: ");
  Serial.print(hic1);
  Serial.println(" *C");
  Serial.print("Pressure1: ");
  Serial.print(p1);
  Serial.println(" hPa");
  Serial.print("Altitude1: ");
  Serial.print(alt1);
  Serial.println(" m");
  Serial.print("Magnetic vector X: ");
  Serial.print(mx1);
  Serial.println(" uT");
  Serial.print("Magnetic vector Y: ");
  Serial.print(my1);
  Serial.println(" uT");
  Serial.print("Magnetic vector Z: ");
  Serial.print(mz1);
  Serial.println(" uT");
  Serial.print("Gas concentration: ");
  Serial.print(gas1);
  Serial.println(" %");
  Serial.print("Flame detection: ");
  Serial.print(flame1);
  Serial.println(" %");
  Serial.print("Sound detection : ");
  Serial.print(sound1);
  Serial.println(" %");
  Serial.print("Light detection : ");
  Serial.print(light1);
  Serial.println(" %");
  Serial.print("Vibration detection : ");
  Serial.print(vibro1);
  Serial.println(" counts");
  Serial.println("");

  // Reset watchdog timer
  Watchdog.reset();
}

