#define BLYNK_TEMPLATE_ID "TMPL3OOTgUsYT"
#define BLYNK_TEMPLATE_NAME "FISHERMAN BORDER ALERT"
#define BLYNK_AUTH_TOKEN "c9KUIR5p6hZdnAzRdIDVPiNUTHPMccKS"
#define BLYNK_PRINT Serial // Comment this out to disable prints and save space

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

static const int RXPin = 4, TXPin = 5; // GPIO 4=D2(connect Tx of GPS) and GPIO 5=D1(Connect Rx of GPS)
static const uint32_t GPSBaud = 9600; // if Baud rate 9600 didn't work in your case then use 4800
TinyGPSPlus gps; // The TinyGPS++ object
WidgetMap myMap(V6); // V6 for virtual pin of Map Widget
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device
float spd; //Variable to store the speed
float sats; //Variable to store no. of satellites response
String bearing; //Variable to store orientation or direction of GPS
unsigned int move_index = 1; // fixed location for now

#include <DHT.h>
#define DHTPIN 2 // Digital pin 4
#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);

#define BLYNK_PRINT Serial // Comment this out to disable prints and save space
int buzzer = D8;
#define flame D0
#define trig D5
#define echo D6
#define relay1 D7
int irSensorPin = D3; // Replace D3 with the appropriate GPIO pin number


#define BORDER_CROSSED_ALERT_PIN V12 // Virtual pin for LED widget
#define BORDER_CROSSED_MSG_PIN V13 // Virtual pin for Terminal widget
#define FIRE_DETECTED_MSG_PIN V14 // Virtual pin for Terminal widget

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "c9KUIR5p6hZdnAzRdIDVPiNUTHPMccKS"; // Enter the Auth code which was sent by Blynk

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "temp"; // Enter your WIFI Name
char pass[] = "temp12345"; // Enter your WIFI Password

SimpleTimer timer;
bool isBorderCrossed = false; // Flag variable to track if border is crossed
bool isFlameDetected = false;

void setup()
{
  Serial.begin(9600); // See the connection status in Serial Monitor
  Blynk.begin(auth, ssid, pass);
  pinMode(buzzer, OUTPUT);
  pinMode(flame, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(relay1, OUTPUT);
  digitalWrite(relay1, HIGH);
  dht.begin();

  Serial.println();
  timer.setInterval(5000L, checkGPS); // every 5s check if GPS is connected, only really needs to be done once
  ss.begin(GPSBaud);
  timer.setInterval(100L, flameSensor);
  timer.setInterval(1000L, getSendData);
  timer.setInterval(100L, ultrasonic);
  timer.setInterval(1000L, sendSensor);
}

void checkGPS()
{
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    Blynk.virtualWrite(V4, "GPS ERROR"); // Value Display widget on V4 if GPS not detected
  }
}

void sendSensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit
  if (isnan(h) || isnan(t))
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more than 10 values per second.
  Blynk.virtualWrite(V1, h); //V1 is for Humidity
  Blynk.virtualWrite(V0, t); //V0 is for Temperature
}

void flameSensor()
{
  bool flameValue = digitalRead(flame);

  if (flameValue == HIGH && !isFlameDetected)
  {
    Blynk.logEvent("Warning", "Fire was detected");
    Blynk.virtualWrite(FIRE_DETECTED_MSG_PIN, "Fire Detected!"); // Display "Fire Detected!" message
    isFlameDetected = true;
    digitalWrite(buzzer, LOW); // Turn off the buzzer when flame is detected
  }
  else if (flameValue == LOW && isFlameDetected)
  {
    isFlameDetected = false;
    digitalWrite(buzzer, HIGH); // Turn on the buzzer when flame is not detected
  }
}


void getSendData()
{
  // Removed MQ2 sensor related code
}

void ultrasonic()
{
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long t = pulseIn(echo, HIGH);
  long cm = t / 29 / 2;
  Blynk.virtualWrite(V3, cm);

  if (cm < 15)
  {
    Blynk.virtualWrite(BORDER_CROSSED_MSG_PIN, "Reaching border Alert!"); // Display "Reaching border Alert!" message
  }
}



void displayInfo()
{
  if (gps.location.isValid())
  {
    float latitude = (gps.location.lat()); //Storing the Lat. and Lon.
    float longitude = (gps.location.lng());
    Serial.print("LAT: ");
    Serial.println(latitude, 6); // float to x decimal places
    Serial.print("LONG: ");
    Serial.println(longitude, 6);
    Blynk.virtualWrite(V7, String(latitude, 6));
    Blynk.virtualWrite(V8, String(longitude, 6));
    myMap.location(move_index, latitude, longitude, "GPS_Location");
    spd = gps.speed.kmph();            //get speed
    Blynk.virtualWrite(V9, spd);
    sats = gps.satellites.value();     //get number of satellites
    Blynk.virtualWrite(V10, sats);
    bearing = TinyGPSPlus::cardinal(gps.course.value());  // get the direction
    Blynk.virtualWrite(V11, bearing);
  }
  Serial.println();
}

void loop()
{
  // Check IR sensor state
  bool isIRSensorTriggered = digitalRead(irSensorPin) == HIGH;

  if (isIRSensorTriggered && !isBorderCrossed)
  {
    isBorderCrossed = true;
    Blynk.virtualWrite(V5, 0); // Turn on LED widget (replace V5 with the actual virtual pin number)
    Blynk.virtualWrite(BORDER_CROSSED_MSG_PIN, "Border Crossed!"); // Send alert message to Blynk Terminal
  }
  else if (!isIRSensorTriggered && isBorderCrossed)
  {
    isBorderCrossed = false;
    Blynk.virtualWrite(V5, 255); // Turn off LED widget (replace V5 with the actual virtual pin number)
  }

  // Check relay state and update LED widget accordingly
  bool isRelayOn = digitalRead(relay1) == HIGH;
  if (isRelayOn)
  {
    Blynk.virtualWrite(V15, 0); // Turn on LED widget (replace V15 with the actual virtual pin number)
  }
  else
  {
    Blynk.virtualWrite(V15, 255); // Turn off LED widget (replace V15 with the actual virtual pin number)
  }

  while (ss.available() > 0)
  {
    // sketch displays information every time a new sentence is correctly encoded.
    if (gps.encode(ss.read()))
      displayInfo();
  }
  Blynk.run();
  timer.run();
}
