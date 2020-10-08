//----------------------------------------------------------------------------------------------------
//  Project Name : Solar Powered WiFi Weather Station
//  Authors: Keith Hungerford, Debasish Dutta, Marc Stahli, and Matt Keaveney
//  Website : www.opengreenenergy.com

// WiFi credentials
char ssid[] = "Matt"; // WiFi Router ssid
char pass[] = "######"; // WiFi Router password

// Thingspeak Write API
const char* server = "api.thingspeak.com";
const char* api_key = "######"; // API write key

/******* Thingspeak field assignments ----------------------
Field 1: Relative Pressure (hPa)
Field 2: Temperature (Celcius)
Field 3: Humidity (%)
Field 4: Battery (V)
Field 5: Measured/Absolute Pressure (hPa)
Field 6: Dewpoint (Celcius)
Field 7: Heat Index (Celcius)
***********************************************************/

// BME280 configuration settings
const bool bme280Debug = 0; // enable printing of BME280 or BMP280 transactions
const byte osrs_t = 2; // setting for temperature oversampling
const byte osrs_p = 5; // setting for pressure oversampling
const byte osrs_h = 5; // setting for humidity oversampling

#define SLEEP_INTERVAL_MIN (10)

#define SERVER_RETRIES (3)

#define WIFI_RETRIES (20)
#define WIFI_RETRY_INTERVAL (10)

#define ELEVATION (151) // elevation in m (Home)
//#define ELEVATION (182) // elevation in m (Cottage)

#define BATTERY_CALIBRATION_FACTOR (5.28) // change this value to calibrate the battery voltage
#define BATTERY_LOW_LIMIT (3.3)
#define BATTERY_LOW_INTERVAL (60)

#define DEWPOINT_A (17.67)
#define DEWPOINT_B (243.5)
