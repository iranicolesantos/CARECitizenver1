//Used Arduino examples of SEN55, MHZ19, NEO-6M, microSD card module

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

//Modify Access Key based on API key in your User Information
#define Access_Key "9m4XnH1mdIaqCh4d" //demo_eee

#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

//MHZ-19B Pin definitions
#define RX_PIN 26  // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 27  // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600  // Device to MH-Z19 Serial baudrate (should not be changed)

MHZ19 myMHZ19; 
SoftwareSerial mySerial(RX_PIN, TX_PIN);                 

//NEO-6M GPS module Pin definitions
#define RXD2_gps 16 // Rx pin which the NEO-6M Tx pin is attached to
#define TXD2_gps 17 // Rx pin which the NEO-6M Tx pin is attached to

TinyGPSPlus gps;
SoftwareSerial neo6m(TXD2_gps, RXD2_gps);


unsigned long timeElapse = 0;

// MQTT Broker IP address:
#define mqtt_server "0e6bc2fecfc5477dbc224a6538ed5c0c.s1.eu.hivemq.cloud" //HiveMQ version
// #define mqtt_server "10.158.9.32" //Local Mosquitto version
// #define mqtt_server "192.168.1.3" //Local Mosquitto version
#define mqtt_topic "everyJuan_Read"

// username and password for HiveMQ version
#define mqtt_username "NodeA"
#define mqtt_password "NodeDeployment1"

// #define mqtt_username "carecitizen"
// #define mqtt_password "everyjuan"

#define willTopic "everyJuan_Disconnect"

byte willQoS = 0;
boolean willRetain = true;

const int mqtt_port = 8883; //for HiveMQ version
// const int mqtt_port = 1883;

WiFiClientSecure espClient;
PubSubClient client(espClient);
long lastMsg = 8;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

int failedPayload = 0;
unsigned long previousReconnectTime = 0;
const unsigned long RECONNECT_INTERVAL = 60000; // Reconnect interval in milliseconds 

//Create files in SD card
const char* successfulFilePath = "/successfulData.txt";
const char* failedFilePath = "/failedData.txt";
const char* csvfilepath = "/csvfile.csv";

//NTP Time servers
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 28800;
const int   daylightOffset_sec = 0;

//Root CA Certificate for TLS MQTT Server
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
  //Setup all modules
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

    #ifdef ESP8266
      espClient.setInsecure();
    #else
      espClient.setCACert(root_ca);    
    #endif

    mySerial.begin(BAUDRATE);     
    myMHZ19.begin(mySerial);                             

    //These lines enable MHZ-19B software zero-point calibration for 20 minutes
    myMHZ19.autoCalibration(false); // Turn auto calibration ON (OFF autoCalibration(false))
    Serial.print("ABC Status: "); myMHZ19.getABC() ? Serial.println("ON") :  Serial.println("OFF");  // now print it's status
    
    Serial.println("Waiting 20 minutes to stabilize...");
   
    timeElapse = 12e5;   //  20 minutes in milliseconds
    delay(timeElapse);    //  wait this duration

    Serial.println("Calibrating..");
    myMHZ19.calibrate();    // Take a reading which be used as the zero point for 400 ppm 
    
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

      // Set keep-alive interval to 120 seconds
    client.setKeepAlive(120);
  
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
  // Start WiFi Manager
  WiFiManager wm;
  // wm.resetSettings();
  bool res;
  res = wm.autoConnect("CareCitizen");
     if(!res){
       Serial.println("Failed to connect");
     }
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
void deleteLineFromFile(File &file, String lineToDelete) {
  // Create a temporary file
  File tempFile = SD.open("/temp.txt", FILE_WRITE);
  
  if (tempFile) {
    file.seek(0); // Start at the beginning of the original file
    while (file.available()) {
      String line = file.readStringUntil('\n');
      line.trim(); // Remove any leading/trailing whitespace
      
      if (line != lineToDelete) {
        tempFile.println(line); // Write line to temp file if it's not the line to delete
      }
    }
    
    // Close both files
    file.close();
    tempFile.close();
    
    // Delete the original file
    SD.remove("/failedData.txt");
    
    // Rename the temp file to the original filename
    SD.rename("/temp.txt", "/failedData.txt");
    
    Serial.println("Deleted line from /failedData.txt: " + lineToDelete);
  } else {
    Serial.println("Failed to create temp file");
  }
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
      if (client.connect(clientId.c_str(), mqtt_username, mqtt_password, willTopic, willQoS, willRetain, Access_Key)) {
        Serial.println("connected");
        // Subscribe
        client.subscribe("esp32/output");
        File datafile = SD.open("/failedData.txt");
        if (datafile){
          while (datafile.available()) {
          String line = datafile.readStringUntil('\n');
          // Publish the failed payload
          Serial.println("Publishing failed payload: " + line);
          if (publishMessage(mqtt_topic, line, true)) {
          // If publish is successful, delete the line from the file
          deleteLineFromFile(datafile, line);
          delay(500);
         }
          delay(1000);
          }
          datafile.close();

        }
      } 
      else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again later");
        }
  }
}

bool publishMessage(const char* topic, String payload , boolean retained){
  if (client.publish(topic, payload.c_str(), true)){
    Serial.println("Message published ["+String(topic)+"]: "+payload);
    SaveOnSD(payload,true);
    Serial.print("MQTT state=");
    Serial.print(client.state());
    return true; // Return true if publish is successful
  }
  else{
    Serial.println("Failed to publish message");
    Serial.print("MQTT state=");
    Serial.print(client.state());
    failedPayload++;
    Serial.print("Number of failed data uploads: ");
    Serial.println(failedPayload);
    // Save failed payload to the failed file
    SaveOnSD(payload, false);
    return false; // Return false if publish fails
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

void timeavailable(struct timeval *t)
{
  Serial.println("Got time adjustment from NTP!");
}

//Set measurement interval
unsigned long lastMeasurementTime = 0;
const unsigned long measureinterval = 60000*5; // 5 minutes in milliseconds

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
    String latitude = "0";
    String longitude = "0";
    String date = "0";
    String time = "0";
    String datetime = "0";
    String finaldate = "0";
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
    Serial.print(latitude);  
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
      if (gps.location.isValid()){
         finaldate = datetime;
      }else{
         finaldate = datetimeNTP;
      }
        DynamicJsonDocument doc(2048);
        doc["AccessKey"] = Access_Key;
        doc["DateTime"] = finaldate;
        // doc["DateTimeNTP"] = datetimeNTP;
        doc["humidity"] = round(ambientHumidity*pow(10, 4))/pow(10, 4);
        doc["temperature"] = round(ambientTemperature*pow(10, 4))/pow(10, 4);
        doc["PM2p5"] = round(massConcentrationPm2p5*pow(10, 4))/pow(10, 4);
        doc["PM10"] = round(massConcentrationPm10p0*pow(10, 4))/pow(10, 4);
        doc["CO2"] = CO2;
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
          if (publishMessage(mqtt_topic, mqtt_message, true)) {
          failedPayload = 0;;
         }
          
          }


        delay(100); 
        
        //Save data to csv format too
        String data = String(Access_Key) + "," + String(datetimeNTP) + "," + String(ambientHumidity) + "," + String(ambientTemperature) + "," + String(massConcentrationPm2p5) + "," + String(massConcentrationPm10p0) + "," + String (CO2) +  "," + String (longitude) +  "," + String (latitude);
        writeDataToCSV(data);
        delay(1000);
        Serial.print("Data has been added to csv");
        Serial.println(data);

      Serial.print("time diff");
      Serial.println(currentTime - lastMeasurementTime);

      // Update the last measurement time
      lastMeasurementTime = currentTime;
      Serial.print("Last Measurement Time: ");
      Serial.println(lastMeasurementTime);
    }
delay(5000);
}
