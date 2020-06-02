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
#define INTERVAL_RSSI_BUZZ 1000
unsigned long time_rssi_buzz = 0;

// SIM800L Define
#include "SIM800L.h"
#define SIM800_RST_PIN 4
#define RX1 18 // pin D18 RX serial1 for TX sim
#define TX1 19 // pin D19 TX serial1 for RX sim
#define INTERVAL_SEND 2000 // set 2 second to set data

HardwareSerial SerialSIM(1);
SIM800L* sim800l;
unsigned long time_send = 0;


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
int rssi = -100;

void setup() {
  // put your setup code here, to run once:
  setupOnBoardLED();
  
  Serial.begin(115200);
  Serial.println("Start Pet Tracker");
  delay(500);
  
  // Start GPS
  setupGPS();
  delay(500);
  
  // Start SIM800L
  setupSIM800L();
  delay(500);
  
  // Start Buzzer  
  setupBuzzer();
  delay(500);
  
  // Start ESP32 Access Point
  setupAP();
  delay(500);
  
  // send SMS
  // trySendSMS();

  // Start Web Server
  setupWebServer();
  delay(500);
  
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
  checkRSSIBeep();
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

  // see respone data, comment this to save data
  //  if(rc == 200) {
  //    // Success, output the data received on the serial
  //    Serial.print(F("HTTP GET successful ("));
  //    Serial.print(sim800l->getDataSizeReceived());
  //    Serial.println(F(" bytes)"));
  //    Serial.print(F("Received : "));
  //    Serial.println(sim800l->getDataReceived());
  //  } else {
  //    // Failed...
  //    Serial.print(F("HTTP GET error "));
  //    Serial.println(rc);
  //  }

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
    } else if (command.equals("sms")){
      Serial.println("Send SMS");
      trySendSMS();
    } else {
      Serial.println("Invalid Command");
    }
  }
}

void sendDataInterval(){
  if (millis() > time_send+ INTERVAL_SEND){
    time_send = millis();
    if (gps.location.isUpdated() && gps.location.isValid()){
      Serial.println("Sending Data...");
      sendData();
    }
  }
}

int threshold_rssi = -70;
void checkRSSIBeep(){
   if (millis() > time_rssi_buzz+ INTERVAL_RSSI_BUZZ){
     time_rssi_buzz = millis(); // save the millis
     Serial.print("Time before : ");
     Serial.println(time_rssi_buzz);
     // check rssi more than threshold
     if (rssi > threshold_rssi) {
          int estimate_duration = 5000 / rssi;
          estimate_duration = abs(estimate_duration);
          Serial.print("estimate duration : "); Serial.println(estimate_duration);
          time_rssi_buzz -= estimate_duration; // reduce time to faster the millis
          Serial.print("Time after : ");
          Serial.println(time_rssi_buzz);
          
          tone(BuzzerPIN, 550, estimate_duration, 0); //turnOn
     }
   } else {
      noTone(BuzzerPIN,0); // turnOff
   }
} 

void tone(uint8_t pin, unsigned int frequency, unsigned int duration, uint8_t channel)
{
    if (ledcRead(channel)) {
        log_e("Tone channel %d is already in use", ledcRead(channel));
        return;
    }
    ledcAttachPin(pin, channel);
    ledcWriteTone(channel, frequency);

    if(duration){
      delay(duration);
      noTone(pin,0);
    }
}

void noTone(uint8_t pin, uint8_t channel)
{
    ledcDetachPin(pin);
    ledcWrite(channel, 0);
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
        String json_format = "{\"rssi\":" + message + "}";
        request->send(200, "application/json",  json_format ); // return json application
        
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

  // try send data using serial.
  trySendData();

  // interval send data
  sendDataInterval();

  // interval buzzer rssi
  checkRSSIBeep();
  
  EasyBuzzer.update(); // buzzer update
  delay(100);
}
