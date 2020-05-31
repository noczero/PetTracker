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
#define RX1 18 // pin D18 RX serial1 for TX sim
#define TX1 19 // pin D19 TX serial1 for RX sim
HardwareSerial SerialSIM(1);

// Access Point Mode
#include <WiFi.h>              //wifi library for ESp32 to access other functionalities
// Set these to your desired credentials.
const char *Apssid = "PetTracker";     //Give AccessPoint name whatever you like. (this will be Name of your esp32 HOTSPOT)
const char *Appassword = "123456789";         //Password of your Esp32's hotspot,(minimum length 8 required)


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
  trySendSMS();
  
  delay(100);
}

void setupAP(){
  //uint32_t brown_reg_temp = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG); //save WatchDog register
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  
  WiFi.mode(WIFI_AP);                    // Changing ESP32 wifi mode to AccessPoint
  delay(100);

  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, brown_reg_temp); //enable brownout detector
  
  // You can remove the Appassword parameter if you want the hotspot to be open.
  WiFi.softAP(Apssid);      //Starting AccessPoint on given credential
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
  
  
  EasyBuzzer.update(); // buzzer update
  delay(100);
}
