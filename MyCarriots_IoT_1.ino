#include <SPI.h>
#include "DHT.h"
#include <Wire.h>
#include <Encoder.h>
#include <TimeLib.h>
#include <DS3232RTC.h>
#include <LiquidCrystal_I2C.h>
#include <UIPEthernet.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_ADXL345_U.h>
#include <EEPROM24LC256_512.h>

const String APIKEY = "97f31f8321a8df31ed5efbb4f3e22072d5732d1b5d075f5d3ee85f74115d1716";
const String DEVICE = "defaultDevice@vrxfile.vrxfile";

#define SERVER_UPDATE_TIME 60000  // Update Carriots data server every 60000 ms (1 minute)
#define DHT_UPDATE_TIME 3000      // Update time for DHT sensors
#define BMP_UPDATE_TIME 1000      // Update time for pressure sensors
#define HMC_UPDATE_TIME 1000      // Update time for magnetic sensors
#define ACC_UPDATE_TIME 1000      // Update time for acceleration sensors
#define ANALOG_UPDATE_TIME 5      // Update time for analog sensors
#define VIBRO_UPDATE_TIME 5       // Update time for vibro sensors
#define LCD_UPDATE_TIME 5000      // Update time for lcd display
#define HRST_UPDATE_TIME 7200000  // Update time for full reset
//#define HRST_UPDATE_TIME 180000  // Update time for full reset

#define TIMEOUT 1000 // 1 second timout

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xAE, 0xCD };
// Local IP if DHCP fails
IPAddress ip(192, 168, 1, 250);
IPAddress dnsServerIP(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

IPAddress carriots_server(82, 223, 244, 60);

IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 102); // time-b.timefreq.bldrdoc.gov
// IPAddress timeServer(132, 163, 4, 103); // time-c.timefreq.bldrdoc.gov
const int timeZone = 3;     // Moscow
EthernetUDP Udp;
unsigned int localPort = 8888;
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

EthernetClient client;

unsigned long timer_main = 0;
unsigned long timer_carriots = 0;
unsigned long timer_dht11 = 0;
unsigned long timer_hmc5883l = 0;
unsigned long timer_bmp085 = 0;
unsigned long timer_adxl345 = 0;
unsigned long timer_analog = 0;
unsigned long timer_vibro = 0;
unsigned long timer_lcd = 0;
unsigned long timer_hreset = 0;

unsigned long counter_main = 0;
unsigned long counter_carriots = 0;
unsigned long counter_dht11 = 0;
unsigned long counter_hmc5883l = 0;
unsigned long counter_bmp085 = 0;
unsigned long counter_adxl345 = 0;
unsigned long counter_analog = 0;
unsigned long counter_vibro = 0;
unsigned long counter_lcd = 0;

#define MAX_WDT 1000 // Software watchdog 10 seconds
unsigned long timer3_counter = 0;
unsigned long wdt_timer = 0;

int failedResponse = 0;

#define DHTPIN 10
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(15883);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(10345);

#define GasSensorPIN A0
#define FlameSensorPIN A1
#define SoundSensorPIN A2
#define LightSensorPIN A3
#define CurrentSensorPIN A4
#define VoltageSensorPIN A5
#define VibroSensorPIN 11

Encoder myEnc(VibroSensorPIN, VibroSensorPIN);

#define LEDPIN 13
#define RESETPIN 48
#define RSTETHPIN 49
#define RELAYPIN 10

float h1 = 0;
float t1 = 0;
float hic1 = 0;
float p1 = 0;
float t2 = 0;
float alt1 = 0;
float mx1 = 0;
float my1 = 0;
float mz1 = 0;
float ax1 = 0;
float ay1 = 0;
float az1 = 0;
float gas1 = 0;
float flame1 = 0;
float sound1 = 0;
float light1 = 0;
float current1 = 0;
float voltage1 = 0;
float vibro1 = 0;
float sum_h1 = 0;
float sum_t1 = 0;
float sum_hic1 = 0;
float sum_p1 = 0;
float sum_t2 = 0;
float sum_alt1 = 0;
float sum_mx1 = 0;
float sum_my1 = 0;
float sum_mz1 = 0;
float sum_ax1 = 0;
float sum_ay1 = 0;
float sum_az1 = 0;
float sum_gas1 = 0;
float sum_flame1 = 0;
float sum_sound1 = 0;
float sum_light1 = 0;
float sum_current1 = 0;
float sum_voltage1 = 0;
float sum_vibro1 = 0;
float avg_h1 = 0;
float avg_t1 = 0;
float avg_hic1 = 0;
float avg_p1 = 0;
float avg_t2 = 0;
float avg_alt1 = 0;
float avg_mx1 = 0;
float avg_my1 = 0;
float avg_mz1 = 0;
float avg_ax1 = 0;
float avg_ay1 = 0;
float avg_az1 = 0;
float avg_gas1 = 0;
float avg_flame1 = 0;
float avg_sound1 = 0;
float avg_light1 = 0;
float avg_current1 = 0;
float avg_voltage1 = 0;
float avg_vibro1 = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Main setup
void setup()
{
  // Init RESET pin
  pinMode(RESETPIN, INPUT);

  // Init LED pin
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);

  // Serial port
  Serial.begin(19200);
  Serial.println("/* Carriots data client by Rostislav Varzar */\n");

  // Timer 3 interrupt (for custom WatchDog)
  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;
  // Set timer1_counter to the correct value for our interrupt interval
  timer3_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer3_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer3_counter = 34286;   // preload timer 65536-16MHz/256/2Hz
  TCNT3 = timer3_counter;   // preload timer
  TCCR3B |= (1 << CS12);    // 256 prescaler
  TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts

  // Init analog PINS
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A12, INPUT);
  pinMode(A13, INPUT);
  pinMode(A14, INPUT);
  pinMode(A15, INPUT);

  // Reset software watchdog
  watchdog_reset();

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

  // Reset software watchdog
  watchdog_reset();

  // NTP init and get time
  Udp.begin(localPort);
  Serial.println("Waiting for sync NTP...");
  setSyncProvider(getNtpTime);
  if (timeStatus() == timeNotSet)
  {
    Serial.println("Failed to sync from NTP!");
    Serial.println("Waiting for sync RTC...");
    setSyncProvider(RTC.get);
  }
  if (timeStatus() == timeNotSet)
  {
    Serial.println("Failed to sync from NTP!");
  }
  else
  {
    Serial.println("Syncing to RTC...");
    RTC.set(now());
    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());
    Serial.print(":");
    Serial.print(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print("-");
    Serial.print(month());
    Serial.print("-");
    Serial.print(year());
    Serial.println();
  }

  // Reset software watchdog
  watchdog_reset();

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

  // ADXL345
  if (!accel.begin())
  {
    Serial.println("Could not find a valid ADXL345 sensor!");
  }
  else
  {
    accel.setRange(ADXL345_RANGE_2_G);
    accel.setDataRate(ADXL345_DATARATE_0_10_HZ);
  }

  // Reset software watchdog
  watchdog_reset();

  // LCD via I2C
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Reset software watchdog
  watchdog_reset();
}

// Main loop
void loop()
{
  // Main timeout
  if (millis() > timer_main + SERVER_UPDATE_TIME)
  {
    // Calculate average data for sensors
    calc_sensors();
    // Reset software watchdog
    watchdog_reset();
    // Print data from sensors
    printAllSenors();
    // Reset software watchdog
    watchdog_reset();
    // Send data to servser
    sendCarriotsStream();
    // Reset software watchdog
    watchdog_reset();
    // Reset variables and counters
    reset_cnt_var();
    // Reset software watchdog
    watchdog_reset();
    // Reset timeout timer
    timer_main = millis();
    counter_main ++;
  }

  // DHT sensors timeout
  if (millis() > timer_dht11 + DHT_UPDATE_TIME)
  {
    readDHT();
    sum_t1 += t1;
    sum_h1 += h1;
    sum_hic1 += hic1;
    counter_dht11 ++;
    timer_dht11 = millis();
  }

  // Pressure sensors timeout
  if (millis() > timer_bmp085 + BMP_UPDATE_TIME)
  {
    readBMP();
    sum_p1 += p1;
    sum_t2 += t2;
    sum_alt1 += alt1;
    counter_bmp085 ++;
    timer_bmp085 = millis();
  }

  // Magnetic sensors timeout
  if (millis() > timer_hmc5883l + HMC_UPDATE_TIME)
  {
    readHMC();
    sum_mx1 += mx1;
    sum_my1 += my1;
    sum_mz1 += mz1;
    counter_hmc5883l ++;
    timer_hmc5883l = millis();
  }

  // Acceleration sensors timeout
  if (millis() > timer_adxl345 + ACC_UPDATE_TIME)
  {
    readACC();
    sum_ax1 += ax1;
    sum_ay1 += ay1;
    sum_az1 += az1;
    counter_adxl345 ++;
    timer_adxl345 = millis();
  }

  // Analog sensors timeout
  if (millis() > timer_analog + ANALOG_UPDATE_TIME)
  {
    readANALOG();
    sum_flame1 += flame1;
    sum_light1 += light1;
    sum_gas1 += gas1;
    sum_sound1 += sound1;
    sum_current1 += current1;
    sum_voltage1 += voltage1;
    counter_analog ++;
    timer_analog = millis();
  }

  // Vibro sensors timeout
  if (millis() > timer_vibro + VIBRO_UPDATE_TIME)
  {
    readVIBRO();
    counter_vibro ++;
    timer_vibro = millis();
  }

  // LCD display timeout
  if (millis() > timer_lcd + LCD_UPDATE_TIME)
  {
    if (counter_lcd == 0)
    {
      lcd.clear();
      lcd.setCursor(0, 0);  lcd_printstr("T1 = " + String(t1) + " *C");
      lcd.setCursor(0, 1);  lcd_printstr("T2 = " + String(t2) + " *C");
    }
    if (counter_lcd == 1)
    {
      lcd.clear();
      lcd.setCursor(0, 0);  lcd_printstr("H1 = " + String(h1) + " %");
      lcd.setCursor(0, 1);  lcd_printstr("P1 = " + String(p1) + " hPa");
    }
    if (counter_lcd == 2)
    {
      lcd.clear();
      lcd.setCursor(0, 0);  lcd_printstr("U = " + String(voltage1) + " V");
      lcd.setCursor(0, 1);  lcd_printstr("I = " + String(current1) + " A");
    }
    if (counter_lcd == 3)
    {
      lcd.clear();
      lcd.setCursor(0, 0);  lcd_printstr("CNT = " + String(counter_main));
    }
    counter_lcd ++;
    if (counter_lcd > 3)
    {
      counter_lcd = 0;
    }
    timer_lcd = millis();
  }

  // Hard reset of device timeout
  if (millis() > timer_hreset + HRST_UPDATE_TIME)
  {
    Serial.println("Hard reset timeout!\n");
    while (1)
    {
    }
  }

  // Reset software watchdog
  watchdog_reset();
}

// Send IoT packet
void sendCarriotsStream()
{
  if (client.connect(carriots_server, 80))
  {
    if (client.connected())
    {
      Serial.println("Sending data to Carriots server...\n");

      String json_data = "{\"protocol\":\"v2\",\"device\":\"";
      json_data = json_data + DEVICE;
      json_data = json_data + "\",\"at\":\"now\",\"data\":{";
      json_data = json_data + "\"counter_main\":";
      json_data = json_data + "\"" + String(counter_main) + "\",";
      json_data = json_data + "\"temperature1\":";
      json_data = json_data + "\"" + String(avg_t1, 2) + "\",";
      json_data = json_data + "\"temperature2\":";
      json_data = json_data + "\"" + String(avg_t2, 2) + "\",";
      json_data = json_data + "\"humidity\":";
      json_data = json_data + "\"" + String(avg_h1, 2) + "\",";
      json_data = json_data + "\"pressure\":";
      json_data = json_data + "\"" + String(avg_p1, 2) + "\",";
      json_data = json_data + "\"magnetic_x\":";
      json_data = json_data + "\"" + String(avg_mx1, 2) + "\",";
      json_data = json_data + "\"magnetic_y\":";
      json_data = json_data + "\"" + String(avg_my1, 2) + "\",";
      json_data = json_data + "\"magnetic_z\":";
      json_data = json_data + "\"" + String(avg_mz1, 2) + "\",";
      json_data = json_data + "\"acceleration_x\":";
      json_data = json_data + "\"" + String(avg_ax1, 2) + "\",";
      json_data = json_data + "\"acceleration_y\":";
      json_data = json_data + "\"" + String(avg_ay1, 2) + "\",";
      json_data = json_data + "\"acceleration_z\":";
      json_data = json_data + "\"" + String(avg_az1, 2) + "\",";
      json_data = json_data + "\"voltage\":";
      json_data = json_data + "\"" + String(avg_voltage1, 2) + "\",";
      json_data = json_data + "\"current\":";
      json_data = json_data + "\"" + String(avg_current1, 2) + "\",";
      json_data = json_data + "\"light\":";
      json_data = json_data + "\"" + String(avg_light1, 2) + "\",";
      json_data = json_data + "\"sound\":";
      json_data = json_data + "\"" + String(avg_sound1, 2) + "\",";
      json_data = json_data + "\"flame\":";
      json_data = json_data + "\"" + String(avg_flame1, 2) + "\",";
      json_data = json_data + "\"gas\":";
      json_data = json_data + "\"" + String(avg_gas1, 2) + "\",";
      json_data = json_data + "\"sound\":";
      json_data = json_data + "\"" + String(avg_sound1, 2) + "\",";
      json_data = json_data + "\"vibration\":";
      json_data = json_data + "\"" + String(avg_vibro1, 2) + "\"}}";

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

      delay(1000);

      timer_carriots = millis();
      while ((client.available() == 0) && (millis() < timer_carriots + TIMEOUT));

      while (client.available() > 0)
      {
        char inData = client.read();
        Serial.print(inData);
      }
      Serial.println("\n");

      client.stop();
    }
  }
}

// Read DHT sensors
void readDHT()
{
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
}

// Read magnetic sensors
void readHMC()
{
  // HMC5883L
  sensors_event_t m_event;
  mag.getEvent(&m_event);
  mx1 = m_event.magnetic.x;
  my1 = m_event.magnetic.y;
  mz1 = m_event.magnetic.z;
}

// Read pressure sensors
void readBMP()
{
  // BMP085
  sensors_event_t p_event;
  bmp.getEvent(&p_event);
  if (p_event.pressure) {
    p1 = p_event.pressure;
    bmp.getTemperature(&t2);
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    alt1 = bmp.pressureToAltitude(seaLevelPressure, p_event.pressure);
  }
}

// Read acceleration sensors
void readACC()
{
  // ADXL345
  sensors_event_t a_event;
  accel.getEvent(&a_event);
  ax1 = a_event.acceleration.x;
  ay1 = a_event.acceleration.y;
  az1 = a_event.acceleration.z;
}

// Read vibro sensors
void readVIBRO()
{
  // Vibro sensor
  vibro1 = myEnc.read();
}

// Read analog sensors
void readANALOG()
{
  // Analog sensors
  float sens1 = analogRead(GasSensorPIN);
  float sens2 = analogRead(FlameSensorPIN);
  float sens3 = analogRead(SoundSensorPIN);
  float sens4 = analogRead(LightSensorPIN);
  float sens5 = analogRead(CurrentSensorPIN);
  float sens6 = analogRead(VoltageSensorPIN);
  gas1 = sens1 / 1023.00 * 100.00;
  flame1 = (1023.00 - sens2) / 1023.00 * 100.00;
  sound1 = sens3 / 1023.00 * 100.00;
  light1 = (1023.00 - sens4) / 1023.00 * 100.00;
  current1 = (sens5 / 1023.00 * 5.00 - 2.50) / 0.185;
  voltage1 = sens6 / 1023.00 * 25.00;
}

// Print sensors data to terminal
void printAllSenors()
{
  Serial.print("Temperature1: ");
  Serial.print(avg_t1);
  Serial.println(" *C");
  Serial.print("Temperature2: ");
  Serial.print(avg_t2);
  Serial.println(" *C");
  Serial.print("Humidity1: ");
  Serial.print(avg_h1);
  Serial.println(" %");
  Serial.print("Heat index1: ");
  Serial.print(avg_hic1);
  Serial.println(" *C");
  Serial.print("Pressure1: ");
  Serial.print(avg_p1);
  Serial.println(" hPa");
  Serial.print("Altitude1: ");
  Serial.print(avg_alt1);
  Serial.println(" m");
  Serial.print("Magnetic vector X: ");
  Serial.print(avg_mx1);
  Serial.println(" uT");
  Serial.print("Magnetic vector Y: ");
  Serial.print(avg_my1);
  Serial.println(" uT");
  Serial.print("Magnetic vector Z: ");
  Serial.print(avg_mz1);
  Serial.println(" uT");
  Serial.print("Acceleration vector X: ");
  Serial.print(avg_ax1);
  Serial.println(" m/s^2");
  Serial.print("Acceleration vector Y: ");
  Serial.print(avg_ay1);
  Serial.println(" m/s^2");
  Serial.print("Acceleration vector Z: ");
  Serial.print(avg_az1);
  Serial.println(" m/s^2");
  Serial.print("Gas concentration: ");
  Serial.print(avg_gas1);
  Serial.println(" %");
  Serial.print("Flame detection: ");
  Serial.print(avg_flame1);
  Serial.println(" %");
  Serial.print("Sound detection : ");
  Serial.print(avg_sound1);
  Serial.println(" %");
  Serial.print("Light detection : ");
  Serial.print(avg_light1);
  Serial.println(" %");
  Serial.print("Input voltage : ");
  Serial.print(avg_voltage1);
  Serial.println(" V");
  Serial.print("Power consumption : ");
  Serial.print(avg_current1);
  Serial.println(" A");
  Serial.print("Vibration detection : ");
  Serial.print(avg_vibro1);
  Serial.println(" counts");
  Serial.print("Main counter : ");
  Serial.print(counter_main);
  Serial.println(" counts");
  Serial.print("DHT counter : ");
  Serial.print(counter_dht11);
  Serial.println(" counts");
  Serial.print("Pressure counter : ");
  Serial.print(counter_bmp085);
  Serial.println(" counts");
  Serial.print("Magnetic counter : ");
  Serial.print(counter_hmc5883l);
  Serial.println(" counts");
  Serial.print("Acceleration counter : ");
  Serial.print(counter_adxl345);
  Serial.println(" counts");
  Serial.print("Analog counter : ");
  Serial.print(counter_analog);
  Serial.println(" counts");
  Serial.print("Vibro counter : ");
  Serial.print(counter_vibro);
  Serial.println(" counts");
  Serial.println("");
}

// Read time data from NTP server
time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response!");
  return 0; // return 0 if unable to get the time
}

// Send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

// Reset software watchdog timer
void watchdog_reset()
{
  wdt_timer = 0;
}

// Interrupt service routine for timer1 overflow
ISR(TIMER3_OVF_vect)
{
  wdt_timer = wdt_timer + 1;
  if (wdt_timer > MAX_WDT)
  {
    wdt_timer = 0;
    TIMSK3 = 0;
    TCNT3 = 0;
    TCCR3A = 0;
    TCCR3B = 0;
    Serial.println("WDT! Resetting...\n");
    delay(1000);
    pinMode(RESETPIN, OUTPUT);
    digitalWrite(RESETPIN, LOW);
  }
  TCNT3 = timer3_counter;
}

// Reset all counters and vars
void reset_cnt_var()
{
  // Reset vibro sensor
  myEnc.write(0);
  // Reset counters
  counter_dht11 = 0;
  counter_hmc5883l = 0;
  counter_bmp085 = 0;
  counter_adxl345 = 0;
  counter_analog = 0;
  counter_vibro = 0;
  // Reset sums
  sum_h1 = 0;
  sum_t1 = 0;
  sum_hic1 = 0;
  sum_p1 = 0;
  sum_t2 = 0;
  sum_alt1 = 0;
  sum_mx1 = 0;
  sum_my1 = 0;
  sum_mz1 = 0;
  sum_ax1 = 0;
  sum_ay1 = 0;
  sum_az1 = 0;
  sum_gas1 = 0;
  sum_flame1 = 0;
  sum_sound1 = 0;
  sum_light1 = 0;
  sum_current1 = 0;
  sum_voltage1 = 0;
  sum_vibro1 = 0;
}

// Calculate average data for sensors
void calc_sensors()
{
  if (counter_dht11 != 0)
  {
    avg_t1 = sum_t1 / counter_dht11;
    avg_h1 = sum_h1 / counter_dht11;
    avg_hic1 = sum_hic1 / counter_dht11;
  }
  if (counter_bmp085 != 0)
  {
    avg_p1 = sum_p1 / counter_bmp085;
    avg_t2 = sum_t2 / counter_bmp085;
    avg_alt1 = sum_alt1 / counter_bmp085;
  }
  if (counter_hmc5883l != 0)
  {
    avg_mx1 = sum_mx1 / counter_hmc5883l;
    avg_my1 = sum_my1 / counter_hmc5883l;
    avg_mz1 = sum_mz1 / counter_hmc5883l;
  }
  if (counter_adxl345 != 0)
  {
    avg_ax1 = sum_ax1 / counter_adxl345;
    avg_ay1 = sum_ay1 / counter_adxl345;
    avg_az1 = sum_az1 / counter_adxl345;
  }
  if (counter_analog != 0)
  {
    avg_flame1 = sum_flame1 / counter_analog;
    avg_light1 = sum_light1 / counter_analog;
    avg_sound1 = sum_sound1 / counter_analog;
    avg_gas1 = sum_gas1 / counter_analog;
    avg_current1 = sum_current1 / counter_analog;
    avg_voltage1 = sum_voltage1 / counter_analog;
  }
  avg_vibro1 = vibro1;
}

// Print string on LCD via I2C
void lcd_printstr(String str1)
{
  for (int i = 0; i < str1.length(); i++)
  {
    lcd.print(str1.charAt(i));
  }
}

