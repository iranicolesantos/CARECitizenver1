
#include <Arduino.h>
#include <SensirionI2CSen5x.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <WiFiManager.h> 
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include "MHZ19.h"                                        
#include <SoftwareSerial.h>   
#include <TinyGPS++.h> 
#include "time.h"
#include "sntp.h"


#define Access_Key "0de6PiyUGsT4HUwJ"


// The used commands use up to 48 bytes. On some Arduino's the default buffer
// space is not large enough
#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

#define RX_PIN 26                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 27                                          // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)

MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial

#define RXD2_gps 16
#define TXD2_gps 17
SoftwareSerial neo6m(TXD2_gps, RXD2_gps);
TinyGPSPlus gps;

unsigned long getDataTimer = 0;
unsigned long timeElapse = 0;
float SleepDuration; // Declare SleepDuration variable
int mod_of_op = 1;

void verifyRange(int range);
// SSID/Password combination
// const char* ssid = "PLDTHOMEFIBRBrEuK";
// const char* password = "PLDTWIFIDfPhp";

// const char* ssid = "SmartBro-FFC7";
// const char* password = "Anggandako21";

// MQTT Broker IP address:
#define mqtt_server "192.168.0.113"
#define mqtt_topic "everyJuan_Read"
#define mqtt_username "carecitizen"
#define mqtt_password "everyjuan"
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 8;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

int failedPayload =0;
unsigned long previousReconnectTime = 0;
const unsigned long RECONNECT_INTERVAL = 60000; // Reconnect interval in milliseconds (e.g., 1 minute)
const char* successfulFilePath = "/successfulData.txt";
const char* failedFilePath = "/failedData.txt";
const char* csvfilepath = "/csvfile.csv";

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 28800;
const int   daylightOffset_sec = 0;

static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

SensirionI2CSen5x sen5x;

void setup() {

    Serial.begin(9600);
    while (!Serial) {
        delay(100);
    }

    Wire.begin();
    neo6m.begin(9600);
    sen5x.begin(Wire);
    sntp_set_time_sync_notification_cb( timeavailable );
    sntp_servermode_dhcp(1);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
    uint16_t error;
    char errorMessage[256];
    error = sen5x.deviceReset();
    if (error) {
        Serial.print("Error trying to execute deviceReset(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
    
    // Start Measurement
    error = sen5x.startMeasurement();
    if (error) {
        Serial.print("Error trying to execute startMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
  
    delay(300);
    setup_wifi();

    // #ifdef ESP8266
    //   espClient.setInsecure();
    // #else
    //   espClient.setCACert(root_ca);      // enable this line and the the "certificate" code for secure connection
    // #endif

    mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start   
    myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin(). 

    //myMHZ19.autoCalibration(false);                              // Turn auto calibration ON (OFF autoCalibration(false))
    //Serial.print("ABC Status: "); myMHZ19.getABC() ? Serial.println("ON") :  Serial.println("OFF");  // now print it's status
    
    //Serial.println("Waiting 20 minutes to stabalise...");
   /* if you don't need to wait (it's already been this amount of time), remove the 2 lines */
    //timeElapse = 12e5;   //  20 minutes in milliseconds
    //delay(timeElapse);    //  wait this duration

    //Serial.println("Calibrating..");
    //myMHZ19.calibrate();    // Take a reading which be used as the zero point for 400 ppm 
    
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

    initSDCard();
    initFiles();

}

void initFiles(){
  if (!SD.exists(successfulFilePath)) {
        Serial.println("Successful payload file does not exist. Creating...");
        writeFile(SD, successfulFilePath, "Temperature, Humidity, PM 1.0, PM 2.5, PM 4, PM 10\n");
    } else {
        Serial.println("Successful payload file exists.");
    }

    // Check if failed payload file exists, if not create it
    if (!SD.exists(failedFilePath)) {
        Serial.println("Failed payload file does not exist. Creating...");
        writeFile(SD, failedFilePath, "Temperature, Humidity, PM 1.0, PM 2.5, PM 4, PM 10\n");
    } else {
        Serial.println("Failed payload file exists.");
    }

    if (!SD.exists(csvfilepath)) {
        Serial.println("CSV file does not exist. Creating...");
        writeFile(SD, csvfilepath, "Datetime,Humidity, Temp, PM2.5, PM10, C02, longitude, latitude\n");
    } else {
        Serial.println("CSV exists.");
    }
}

void SaveOnSD(String data, bool Upload) {
  Serial.print("Saving data: ");
  Serial.println(data);

  if (Upload){
    appendFile(SD, "/successfulData.txt", data.c_str());
  }
  else{// Append the data to the file
   appendFile(SD, "/failedData.txt", data.c_str());
  }
}

void setup_wifi() {
  delay(10);
  Serial.println();
  WiFi.mode(WIFI_STA);
  WiFi.begin("EEE192-429", "EEE192_Room429");
  // WiFiManager wm;
  // wm.resetSettings();
  // bool res;
  //   res = wm.autoConnect("CareCitizen", "CareForEVERYJUAN");
  //   if(!res){
  //     Serial.println("Failed to connect");
  //   }
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  static unsigned long lastReconnectAttempt = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousReconnectTime >= RECONNECT_INTERVAL) {
      Serial.print("Attempting MQTT connection...");
      String clientId = "ESPClient-";   // Random client ID
      clientId += Access_Key;
      // Attempt to connect
      if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
        Serial.println("connected");
        // Subscribe
        client.subscribe("esp32/output");
        File datafile = SD.open("/failedData.txt");
        if (datafile){
          while (datafile.available()) {
          String line = datafile.readStringUntil('\n');
          // Publish the failed payload
          Serial.println("Publishing failed payload: " + line);
          publishMessage(mqtt_topic, line, true);
          delay(1000);
          }
          datafile.close();
          // Delete the contents of the file
          SD.remove("/failedData.txt");
          Serial.println("Deleted contents of /failedData.txt");
        }
      } 
      else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again later");
        }
  }
}

void publishMessage(const char* topic, String payload , boolean retained){
  if (client.publish(topic, payload.c_str(), true)){
    Serial.println("Message published ["+String(topic)+"]: "+payload);
    SaveOnSD(payload,true);
  }
  else{
    Serial.println("Failed to publish message");
    failedPayload++;
    Serial.print("Number of failed data uploads: ");
    Serial.println(failedPayload);
    // Save failed payload to the failed file
    SaveOnSD(payload, false);
  }    
}

void writeDataToCSV(String data) {
  appendFile(SD, "/csvfile.csv", data.c_str());
}
// Initialize SD card
void initSDCard(){
   if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }
  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

// Write to the SD card
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file) {
    file.print(message);
    file.println(); 
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

static void smartdelay_gps(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (neo6m.available())
      gps.encode(neo6m.read());
  } while (millis() - start < ms);
}
float findMax(float a, float b, float c, float d, float e, float f) {
    float max_val = a;
    if (b > max_val) {
        max_val = b;
    }
    if (c > max_val) {
        max_val = c;
    }
    if (d > max_val) {
        max_val = d;
    }
    if (e > max_val) {
        max_val = e;
    }
    if (f > max_val) {
        max_val = f;
    }
    return max_val;
}

void KnowSleepDuration(float humidity_var, float temperature_var,float PM1p0_var,float PM2p5_var,float PM4p0_var,float PM10p0_var,float CO2_var){
  float k = 2;
  float maxvar = findMax(humidity_var, temperature_var, PM1p0_var, PM2p5_var, PM4p0_var, PM10p0_var);
  float threshold;
  float vardiff;
  float multiplier;
  
  if (humidity_var == maxvar) {
    threshold = k*sqrt(humidity_var);
    vardiff = threshold - humidity_var;
  }

  if (temperature_var == maxvar) {
    threshold = k*sqrt(temperature_var);
    vardiff = threshold - temperature_var;
  }

  if (PM1p0_var == maxvar) {
    threshold = k*sqrt(PM1p0_var);
    vardiff = threshold - PM1p0_var;
  }

  if (PM2p5_var == maxvar) {
    threshold = k*sqrt(PM2p5_var);
    vardiff = threshold - PM2p5_var;
  }

  if (PM4p0_var == maxvar) {
    threshold = k*sqrt(PM4p0_var);
    vardiff = threshold - PM4p0_var;
  }

  if (PM10p0_var == maxvar) {
    threshold = k*sqrt(PM10p0_var);
    vardiff = threshold - PM10p0_var;
  }

  if (maxvar > threshold){
    multiplier = vardiff/0.5;
    Serial.println(threshold);
    Serial.println(vardiff);
    Serial.println(multiplier);
    Serial.println(multiplier*60);
    SleepDuration = 180+60+multiplier*60;
  }
  else{
    SleepDuration = 60; // 1 min
  }
}
void timeavailable(struct timeval *t)
{
  Serial.println("Got time adjustment from NTP!");
}
unsigned long lastMeasurementTime = 0;
const unsigned long measureinterval = 30000; // 30 sec minutes in milliseconds
int measureIndex = 0;

void loop() {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    
    smartdelay_gps(1000);
    // Print all received characters from GPS module
    while (neo6m.available() > 0) {
      Serial.write(neo6m.read());
    }
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
    }
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    char time24hr[6];
    strftime(time24hr, 6, "%R", &timeinfo);
    char shortdate[9];
    strftime(shortdate, 9, "%D", &timeinfo);
    String datetimeNTP = String(shortdate) + " " + String(time24hr);
    Serial.println(datetimeNTP);

    // Read Measurement
    String latitude = "N/A";
    String longitude = "N/A";
    String date = "N/A";
    String time = "N/A";
    String datetime = "N/A";
    float massConcentrationPm1p0;
    float massConcentrationPm2p5;
    float massConcentrationPm4p0;
    float massConcentrationPm10p0;
    float ambientHumidity;
    float ambientTemperature;
    float vocIndex;
    float noxIndex;
    int CO2;
    int readingCount = 0;
    if (gps.location.isValid()) 
    {
    //Storing the Latitude. and Longitude
    latitude = String(gps.location.lat(), 6);
    longitude = String(gps.location.lng(), 6); 
    
    // Extracting date components
    int year = gps.date.year();
    int month = gps.date.month();
    int day = gps.date.day();
    date = String(year) +  "-" + String(month) +  "-" + String(day);

    // Extracting time components
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();
    
    // Adjusting time for UTC+8 (Philippines Standard Time)
    int hourLocal = (hour + 8) % 24;
    time = String(hourLocal) + ":" + String(minute) + ":" + String(second);

    datetime = date + " " + time;
    //Send to Serial Monitor for Debugging
    Serial.print("LAT:  ");
    Serial.print(latitude);  // float to x decimal places
    Serial.print("\t");
    Serial.print("LONG: ");
    Serial.print(longitude);
    Serial.println("Date:");
    Serial.print(date);
    Serial.print("\t");
    Serial.print("Time: ");
    Serial.print(time);
    }
    else{
      Serial.println("GPS No data");
    }

    uint16_t error;
    char errorMessage[256];

    delay(1000);


    //Data sequence arrays
    float humidity_arr[10];
    float temperature_arr[10];
    float PM1p0_arr[10];
    float PM2p5_arr[10];
    float PM4p0_arr[10];
    float PM10p0_arr[10];
    float CO2_arr[10];   

    unsigned long currentTime = millis();
    Serial.print("Current Time: ");
    Serial.println(currentTime);

    if (currentTime - lastMeasurementTime >= measureinterval) {
      Serial.println("30 secs has passed. Updating measurement...");
      error = sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);
      CO2 = myMHZ19.getCO2();
      humidity_arr[measureIndex] = ambientHumidity;
      temperature_arr[measureIndex] = ambientTemperature;
      PM1p0_arr[measureIndex] = massConcentrationPm1p0;
      PM2p5_arr[measureIndex] = massConcentrationPm2p5;
      PM4p0_arr[measureIndex] = massConcentrationPm4p0;
      PM10p0_arr[measureIndex] = massConcentrationPm10p0;
      CO2_arr[measureIndex] = CO2; 
      //delay(6000);
      
      Serial.print("Measurement ");
      Serial.println(measureIndex);
      // Update the index for the next measurement
      measureIndex++;
      Serial.print("time diff");
      Serial.println(currentTime - lastMeasurementTime);
      // Update the last measurement time
      lastMeasurementTime = currentTime;
      Serial.print("Last Measurement Time: ");
      Serial.println(lastMeasurementTime);
    }
      // Reset the index if it exceeds the array length
    if (measureIndex >= 10) {
      Serial.println("Resetting measureIndex...");
      measureIndex = 0;
        float humidity_mean = 0;
  float temperature_mean = 0;
  float PM1p0_mean = 0;
  float PM2p5_mean = 0;
  float PM4p0_mean = 0;
  float PM10p0_mean = 0;
  float CO2_mean = 0;

  // Calculate the mean
  for (int i = 0; i < 10; i++) {
    humidity_mean += humidity_arr[i];
    temperature_mean += temperature_arr[i];
    PM1p0_mean += PM1p0_arr[i];
    PM2p5_mean += PM2p5_arr[i];
    PM4p0_mean += PM4p0_arr[i];
    PM10p0_mean += PM10p0_arr[i];
    CO2_mean += CO2_arr[i];
  }

  humidity_mean /= 10;
  temperature_mean /= 10;
  PM1p0_mean /= 10;
  PM2p5_mean /= 10;
  PM4p0_mean /= 10;
  PM10p0_mean /= 10;
  CO2_mean /= 10;

  // Calculate the variance of humidity and temperature
  float humidity_var = 0;
  float temperature_var = 0;
  float PM1p0_var = 0;
  float PM2p5_var = 0;
  float PM4p0_var = 0;
  float PM10p0_var = 0;
  float CO2_var = 0;

  for (int i = 0; i < 10; i++) {
    humidity_var += (humidity_arr[i] - humidity_mean) * (humidity_arr[i] - humidity_mean);
    temperature_var += (temperature_arr[i] - temperature_mean) * (temperature_arr[i] - temperature_mean);
    PM1p0_var += (PM1p0_arr[i] - PM1p0_mean) * (PM1p0_arr[i] - PM1p0_mean);
    PM2p5_var += (PM2p5_arr[i] - PM2p5_mean) * (PM2p5_arr[i] - PM2p5_mean);
    PM4p0_var += (PM4p0_arr[i] - PM4p0_mean) * (PM4p0_arr[i] - PM4p0_mean);
    PM10p0_var += (PM10p0_arr[i] - PM10p0_mean) * (PM10p0_arr[i] - PM10p0_mean);
    CO2_var += (CO2_arr[i] - CO2_mean) * (CO2_arr[i] - CO2_mean);

  }
  humidity_var /= 10;
  temperature_var /= 10;
  PM1p0_var /= 10;
  PM2p5_var /= 10;
  PM4p0_var /= 10;
  PM10p0_var /= 10;
  CO2_var /= 10;

  Serial.print("Humidity Variance: ");
  Serial.println(humidity_var);
  Serial.print("Temperature Variance: ");
  Serial.println(temperature_var);
  Serial.print("PM 1.0 Variance: ");
  Serial.println(PM1p0_var);
  Serial.print("PM2.5 Variance: ");
  Serial.println(PM2p5_var);
  Serial.print("PM4.0 Variance: ");
  Serial.println(PM4p0_var);
  Serial.print("PM10.0 Variance: ");
  Serial.println(PM10p0_var);
  Serial.print("CO2 Variance: ");
  Serial.println(CO2_var);

  DynamicJsonDocument doc(2048);
  doc["AccessKey"] = Access_Key;
  doc["DateTime"] = datetime;
  doc["DateTimeNTP"] = datetimeNTP;
  doc["humidity"] = round(humidity_mean * 100.0) / 100.0;;
  doc["temperature"] = round(temperature_mean * 100.0) / 100.0;
  doc["PM2p5"] = round(PM2p5_mean * 100.0) / 100.0;
  doc["PM10"] = round(PM10p0_mean * 100.0) / 100.0;
  doc["CO2"] = round(CO2_mean * 100.0) / 100.0;
  doc["Latitude"] = latitude;
  doc["Longitude"] = longitude;

  char mqtt_message[1024];
  serializeJson(doc, mqtt_message);

  // Check if either WiFi or MQTT connection is lost
  if (!client.connected() || WiFi.status() != WL_CONNECTED) {
    // If either connection is lost, save the payload to SD card
    Serial.println("WiFi or MQTT connection lost.Failed to publish message. Saving payload to SD card.");
    failedPayload++;
    Serial.print("Number of failed data uploads: ");
    Serial.println(failedPayload);
    // Save the payload to the SD card
    Serial.println("Saving payload to SD card.");
    SaveOnSD(mqtt_message, false);
    reconnect();
  }else {
    // If both WiFi and MQTT connections are active, attempt to publish the message
    Serial.println("Both WiFi and MQTT connections are active. Attempting to publish message.");
    publishMessage(mqtt_topic, mqtt_message, true);
    SaveOnSD(mqtt_message, true);
    failedPayload = 0;
    }


  delay(100); //change to 180000
  
  String data = String(datetimeNTP) + "," + String(humidity_mean) + "," + String(temperature_mean) + "," + String(PM2p5_mean) + "," + String(PM10p0_mean) + "," + String (CO2_mean) +  "," + String (longitude) +  "," + String (latitude);
  writeDataToCSV(data);
  delay(1000);
  Serial.print("Data has been added to csv");
  Serial.println(data);

  mod_of_op = 0; // Power-saving mode is on
  if (mod_of_op == 1){
    KnowSleepDuration(humidity_var, temperature_var, PM1p0_var, PM2p5_var, PM4p0_var, PM10p0_var, CO2_var);
    Serial.print("Going to sleep for ");
    Serial.print(SleepDuration);
    Serial.println(" seconds");
    delay(100);
    esp_sleep_enable_timer_wakeup(SleepDuration * 1000000); // Convert seconds to microseconds
    esp_light_sleep_start();
    Serial.println("Woke up!"); 
  }
}


  delay(5000);
}
