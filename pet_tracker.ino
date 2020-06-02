#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <EasyBuzzer.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// Print Data Setting
#define INTERVAL_PRINT 200 // 200ms
unsigned long time_print = 0;

#define ONBOARD_LED  2

// GPS Define
#define RX2 16 // pin D16 RX serial2 for TX gps
#define TX2 17 // pin D17 TX serial2 for RX gps
#define INTERVAL_GPS 1000 // set 1 seconds update
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
float latitude, longitude;
unsigned long time_gps = 0;

// Buzzer define
#define BuzzerPIN 32 

// SIM800L Define
#include "SIM800L.h"
#define SIM800_RST_PIN 4
#define RX1 18 // pin D18 RX serial1 for TX sim
#define TX1 19 // pin D19 TX serial1 for RX sim
HardwareSerial SerialSIM(1);
SIM800L* sim800l;

const char APN[] = "internet";
char dataSend[90];
char format[90] = "http://pet.zeroinside.net/insert_data.php?lat=%f&lon=%f&rssi=%d&api_key=pet123"; // %f float, %d integer

// Access Point Mode
#include <WiFi.h>              //wifi library for ESp32 to access other functionalities
// Set these to your desired credentials.
const char *Apssid = "PetTracker";     //Give AccessPoint name whatever you like. (this will be Name of your esp32 HOTSPOT)
const char *Appassword = "123456789";         //Password of your Esp32's hotspot,(minimum length 8 required)

// WebServer
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
AsyncWebServer server(80);
const char* PARAM_MESSAGE = "rssi";
int rssi = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Start Pet Tracker");
  delay(100);
  
  // Start GPS
  setupGPS();
  
  // Start SIM800L
  setupSIM800L();
  
  // Start Buzzer  
  setupBuzzer();

  // On Board LED
  setupOnBoardLED();
  
  // Start ESP32 Access Point
  setupAP();

  // send SMS
  // trySendSMS();

  // Start Web Server
  setupWebServer();
  
  delay(100);
}

void setupAP(){
  //uint32_t brown_reg_temp = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG); //save WatchDog register
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  
  WiFi.mode(WIFI_AP);                    // Changing ESP32 wifi mode to AccessPoint
  delay(100);

  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, brown_reg_temp); //enable brownout detector
  
  // You can remove the Appassword parameter if you want the hotspot to be open.
  WiFi.softAP(Apssid,Appassword);      //Starting AccessPoint on given credential
  delay(100);
  IPAddress myIP = WiFi.softAPIP();     //IP Address of our Esp8266 accesspoint(where we can host webpages, and see data)
  Serial.print("AP IP address: ");
  Serial.println(myIP);                //Default IP is 192.168.4.1
}

void setupGPS(){
  Serial.println("GPS Enabled");
  SerialGPS.begin(9600, SERIAL_8N1, RX2, TX2);
  Serial.println("Serial GPS Txd is on pin : "+ String (TX2));
  Serial.println("Serial GPS Rxd is on pin : "+ String (RX2));
  delay(100);
}

void setupSIM800L(){
  Serial.println("SIM800L Enabled");
  SerialSIM.begin(115200, SERIAL_8N1, RX1, TX1);
  Serial.println("Serial GSM Txd is on pin: " + String (TX1));
  Serial.println("Serial GSM Rxd is on pin: " + String (RX1));
  delay(100);

  // Initialize SIM800L driver with an internal buffer of 200 bytes and a reception buffer of 512 bytes, debug disabled
  sim800l = new SIM800L((Stream *)&SerialSIM, SIM800_RST_PIN, 200, 512);

  // Equivalent line with the debug enabled on the Serial
  // sim800l = new SIM800L((Stream *)&Serial1, SIM800_RST_PIN, 200, 512, (Stream *)&Serial);

  // Setup module for GPRS communication
  setupModuleGPRS();
}

void setupModuleGPRS(){
  // Wait until the module is ready to accept AT commands
  while(!sim800l->isReady()) {
    Serial.println(F("Problem to initialize AT command, retry in 1 sec"));
    delay(1000);
  }
  Serial.println(F("Setup Complete!"));

  // Setup APN for GPRS configuration
  bool success = sim800l->setupGPRS(APN);
  while(!success) {
    success = sim800l->setupGPRS(APN);
    delay(5000);
  }
  Serial.println(F("GPRS config OK"));
}

void setupBuzzer(){
  EasyBuzzer.setPin(BuzzerPIN);
  //fastBeep();
  Serial.println("Buzzer activated");
  delay(100);
}

void setupOnBoardLED(){
  pinMode(ONBOARD_LED,OUTPUT);
  delay(100);
  digitalWrite(ONBOARD_LED,HIGH);
  delay(200);
  digitalWrite(ONBOARD_LED,LOW);
}

void trySendSMS(){
  Serial.println("Sending SMS...");
  SerialSIM.println("at");
  delay(2000);
  SerialSIM.println("at+cmgf=1");
  delay(2000); 
  SerialSIM.println("AT+CMGS=\"082277009251\"\r\n");
  delay(2000);
  SerialSIM.println("Perangkat aktif!");
  delay(2000);
  SerialSIM.println((char)26);
  Serial.println("Success...");
}

void sendData(){
  // Establish GPRS connectivity (5 trials)
  bool connected = false;
  for(uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    connected = sim800l->connectGPRS();
  }

  // Check if connected, if not reset the module and setup the config again
  if(connected) {
    Serial.println(F("GPRS connected !"));
  } else {
    Serial.println(F("GPRS not connected !"));
    Serial.println(F("Reset the module."));
    sim800l->reset(); 
    setupModuleGPRS();
    return;
  }
  
  sprintf(dataSend, format, latitude, longitude, rssi); // convert to string based on format URL
  Serial.println(dataSend);

  // Do HTTP GET communication with 10s for the timeout (read)
  uint16_t rc = sim800l->doGet(dataSend, 10000); // send request dataSend
  if(rc == 200) {
    // Success, output the data received on the serial
    Serial.print(F("HTTP GET successful ("));
    Serial.print(sim800l->getDataSizeReceived());
    Serial.println(F(" bytes)"));
    Serial.print(F("Received : "));
    Serial.println(sim800l->getDataReceived());
  } else {
    // Failed...
    Serial.print(F("HTTP GET error "));
    Serial.println(rc);
  }

  // Close GPRS connectivity (5 trials)
  bool disconnected = sim800l->disconnectGPRS();
  for(uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    disconnected = sim800l->disconnectGPRS();
  }
  
  if(disconnected) {
    Serial.println(F("GPRS disconnected !"));
  } else {
    Serial.println(F("GPRS still connected !"));
  }
}

String command;
void trySendData(){
  if (Serial.available() > 0){
    command = Serial.readStringUntil('\n');
    if(command.equals("send")){
      Serial.println("Send Data");
      sendData();
    } else {
      Serial.println("Invalid Command");
    }
  }
}

void fastBeep(){
  EasyBuzzer.beep(
    1000,    // Frequency in hertz(HZ). 
    50,   // On Duration in milliseconds(ms).
    100,  // Off Duration in milliseconds(ms).
    5,      // The number of beeps per cycle.
    500,  // Pause duration.
    5
  );
}

void readGPS(){
  // use unblocking delay
  if(millis() > time_gps + INTERVAL_GPS){
    time_gps = millis();
    if (gps.location.isUpdated() && gps.location.isValid()){
      latitude = gps.location.lat();
      longitude = gps.location.lng();
    } 
  }
}

void printData(){
   // use unblocking delay
   if (millis() > time_print + INTERVAL_PRINT){
      time_print = millis();
      
      // print to serial
      Serial.print("Lat: "); Serial.print(latitude,6);
      Serial.print("\t Lon: "); Serial.print(longitude,6);
      Serial.print("\t RSSI : "); Serial.print(rssi);
      
      // print enter
      Serial.println();
   }
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    // read gps data
    while (SerialGPS.available())
      // encode gps data using tiny gps
      gps.encode(SerialGPS.read());
  } while (millis() - start < ms);
}

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void setupWebServer(){
    Serial.println("Enabled Web Server");
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", "Pet Tracker Device Ver. 1");
    });
    
     // Send a GET request to <IP>/set_rssi?rssi=<value>
    server.on("/set_rssi", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String message;
        if (request->hasParam(PARAM_MESSAGE)) {
            message = request->getParam(PARAM_MESSAGE)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/json",  message );
        
        rssi = message.toInt(); // set message to rssi integer
    });
    
    server.onNotFound(notFound);
    
    server.begin();
}


void loop() {
  // put your main code here, to run repeatedly:

  // get latitutde and longitude from GPS
  readGPS();

  // print data to serial
  printData();

  // gps loop part
  smartDelay(500);
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

  // try send data
  trySendData();
  
  EasyBuzzer.update(); // buzzer update
  delay(100);
}
