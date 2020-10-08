
//----------------------------------------------------------------------------------------------------
//  Project Name : Solar Powered WiFi Weather Station
//  Authors: Keith Hungerford, Debasish Dutta, Marc Stahli, and Matt Keaveney
//  Website : www.opengreenenergy.com
//
//  This code is derived from the example code of farmerkeith_BMP280.h library and 3KUdelta
//  farmerkeith library: https://github.com/farmerkeith/BMP280-library
//  3KUdelta GitHub: https://github.com/3KUdelta/Solar_WiFi_Weather_Station
//
//  Main microcontroller (ESP8266) and BME280 both sleep between measurements
//  BME280 is used in single shot mode ("forced mode")
//  Measurement read command is delayed,
//  By repeatedly checking the "measuring" bit of status register (0xF3) until ready
//
//  Last updated on 2020-10-07

#include <Wire.h>
#include <farmerkeith_BMP280.h> // library, download from https://github.com/farmerkeith/BMP280-library
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include "Settings.h"

bme280 bme0(0, bme280Debug); // creates object bme0 of type bme280, base address

WiFiClient client;

double measuredTemp;
double measuredHumi;
double measuredPres;
float adjustedTemp;
float adjustedHumi;
float seaLevelPres;
int relativePres;
float heatIndex;
float volt;
double dewpointTemp;
float dewpointSpread;

void setup() {
  
  Serial.begin(115200); // use 9600 if you get errors with the faster rate
  Serial.println("\nStart of SolarWiFiWeatherStation v3.0");

  //******Battery Voltage Monitoring*********************************************
  
  // Voltage divider R1 = 220k+100k+220k =540k and R2=100k
  unsigned long raw = analogRead(A0);
  volt = raw * BATTERY_CALIBRATION_FACTOR / 1024;

  Serial.print( "Voltage = ");
  Serial.print(volt, 2); // print with 2 decimal places
  Serial.println (" V");

  //******Application going online*********************************************

  Serial.print("Connecting to WiFi ");
  
  WiFi.hostname("SolarWeatherStation");
  WiFi.begin(ssid, pass);

  int i=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    i++;

    if (i > WIFI_RETRIES) {
      Serial.println(" Could not connect to WiFi!");
      goToSleep(WIFI_RETRY_INTERVAL);
    }
    
    Serial.print(".");
  }
  
  Serial.println(" OK");

  //******Configure and start sensor reading cycle*********************************************
  
  byte temperatureSamples = pow(2, osrs_t - 1);
  byte pressureSamples = pow(2, osrs_p - 1);
  byte humiditySamples = pow(2, osrs_h - 1);
  
  Serial.print("Temperature samples=");
  Serial.print(temperatureSamples);
  Serial.print(" Pressure samples=");
  Serial.print(pressureSamples);
  Serial.print(" Humidity samples=");
  Serial.print(humiditySamples);
  Serial.println();

  // Wire.begin(); // initialise I2C protocol - not needed here since it is in bmp library
  bme0.begin(osrs_t, osrs_p, 1, 0, 0, 0, osrs_h);
  // parameters are (osrs_t, osrs_p, mode, t_sb, filter, spi3W_en, osrs_h)
  // see BME280 data sheet for definitions
  // this is single shot mode with no filtering

  measurementEvent();

  Serial.print("Sending data to server: ");
    
  int j=0;
  while (!sendDataToServer()) {
    delay(1000);
    j++;

    if (j > SERVER_RETRIES) {
      Serial.println("Could not send data to server!");
      break;
    }

    Serial.println("Resending data to server: ");
  }
  
  bme0.updateF4Control16xSleep();
  
  goToSleep(SLEEP_INTERVAL_MIN);
  
} // end of void setup()


void loop() {
} // end of void loop()


// ----------send data to server----------------------------
bool sendDataToServer() {

  bool success = false;

  if (client.connect(server, 80)) {
    String postStr = "";
    postStr+="GET /update?api_key=";
    postStr+=api_key;   
    postStr+="&field1=";
    postStr+=String(relativePres);
    postStr+="&field2=";
    postStr+=String(measuredTemp);
    postStr+="&field3=";
    postStr+=String(measuredHumi);
    postStr+="&field4=";
    postStr+=String(volt);
    postStr+="&field5=";
    postStr+=String(measuredPres);
    postStr+="&field6=";
    postStr+=String(dewpointTemp);
    postStr+="&field7=";
    postStr+=String(heatIndex);
    postStr+=" HTTP/1.1\r\nHost: a.c.d\r\nConnection: close\r\n\r\n";
    postStr+="";
    client.print(postStr);

    String line = "";
    while (client.connected()) {
      if (client.available()) {
        // lfirst line should be status
        line = client.readStringUntil('\n');
        line.trim();
//        Serial.println("[" + line + "]");
        break;
      }
    }

    client.stop();

    if (!line.length()) {
      Serial.println("ERROR - Empty response");
    } else {
      if (line == "HTTP/1.1 200 OK") {
        Serial.println("OK");
        success = true;
      } else {
        Serial.println("ERROR - Invalid response");
      }
    }
  } else {
    Serial.println("ERROR - Cannot connect!");
  }

  return success;
  
} // end of bool sendDataToServer()


// ----------measure events----------------------------
void measurementEvent() {
  
  while (bme0.readRegister(0xF3) >> 3); // loop until F3bit 3 ==0, measurement is ready

  // measure pressure, temperature and humidity
  measuredHumi = bme0.readHumidity(measuredTemp, measuredPres);

  // calculate relative pressure
  seaLevelPres = (((measuredPres * 100.0) / pow((1-((float)(ELEVATION))/44330), 5.255)) / 100.0);
  relativePres = (int)(seaLevelPres + 0.5);

  // calculate dewpoint using the Magnus formula (https://en.wikipedia.org/wiki/Dew_point#Calculating_the_dew_point)
  double tempCalc = log(measuredHumi / 100) + ((DEWPOINT_A * measuredTemp) / (DEWPOINT_B + measuredTemp));
  dewpointTemp = (DEWPOINT_B * tempCalc) / (DEWPOINT_A - tempCalc);

  // adjust temperature and humidity with dewpoint
  adjustedTemp = measuredTemp;
  if (adjustedTemp < dewpointTemp) {
    adjustedTemp = dewpointTemp;
  }

  // August-Roche-Magnus approximation (http://bmcnoldy.rsmas.miami.edu/Humidity.html)
  adjustedHumi = 100 * (exp((DEWPOINT_A * dewpointTemp) / (DEWPOINT_B + dewpointTemp)) / exp((DEWPOINT_A * adjustedTemp) / (DEWPOINT_B + adjustedTemp)));
  if (adjustedHumi > 100) {
    adjustedHumi = 100;
  }

  // calculate dewpoint spread (difference between actual temp and dewpoint, the smaller the number = rain or fog)
  dewpointSpread = measuredTemp - dewpointTemp;

  // Calculate heat index (only if temp > 26.7 deg C)
  if (measuredTemp > 26.7) {
    double c1 = -8.784, c2 = 1.611, c3 = 2.338, c4 = -0.146, c5 = -1.230e-2, c6 = -1.642e-2, c7 = 2.211e-3, c8 = 7.254e-4, c9 = -2.582e-6;
    double T = measuredTemp;
    double R = measuredHumi;
    double A = ((c5 * T) + c2) * T + c1;
    double B = ((c7 * T) + c4) * T + c3;
    double C = ((c9 * T) + c8) * T + c6;
    heatIndex = (C * R + B) * R + A;
  } else {
    Serial.println("Not warm enough (< 26.7 deg C) for heat index.");
    heatIndex = measuredTemp;
  }

//  Serial.print("Absolute pressure = ");
//  Serial.print(measuredPres, 2);
//  Serial.print(" hPa. Temperature = ");
//  Serial.print(measuredTemp, 2);
//  Serial.print( " deg C. Humidity = ");
//  Serial.print(measuredHumi, 2);
//  Serial.print( " %. Relative pressure = ");
//  Serial.print(relativePres);
//  Serial.print(" hPa. Dewpoint = ");
//  Serial.print(dewpointTemp, 2);
//  Serial.print(" deg C. Temperature adjusted = ");
//  Serial.print(adjustedTemp, 2);
//  Serial.print(" deg C. Humidity adjusted = ");
//  Serial.print(adjustedHumi, 2);
//  Serial.print(" %. Dewpoint spread = ");
//  Serial.print(dewpointSpread, 2);
//  Serial.print(" deg C. Heat index = ");
//  Serial.print(heatIndex, 2);
//  Serial.println(" deg C.");
  
} // end of void measurementEvent()


// ----------deep sleep cycle----------------------------
void goToSleep(unsigned int sleepMin) {

  Serial.println("Closing the WiFi connection");
  WiFi.disconnect();

  while (client.connected() || (WiFi.status() == WL_CONNECTED)) {
    Serial.println("Waiting for WiFi to disconnect before sleeping");
    delay(10);
  }

  delay(50);
  
  if (volt < BATTERY_LOW_LIMIT) {
    Serial.println("Battery low, increasing sleep cycle.");
    sleepMin = BATTERY_LOW_INTERVAL;
  }
  
  Serial.print("Going to sleep now for ");
  Serial.print(sleepMin);
  Serial.println(" minute(s).");
  
  ESP.deepSleep(sleepMin * 60 * 1000000); // convert to microseconds

} // end of void goToSleep()
