#include "WiFi.h"       
#include <ThingsBoard.h>    
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Max72xxPanel.h>    //  max7219 library
#include <Wire.h>
#include <Adafruit_PN532.h>

// WiFi access point
#define WIFI_AP_NAME        "FD_HotSpot"//SBC
// WiFi password
#define WIFI_PASSWORD       "Dash_TFG"//sbc$2020

// See https://thingsboard.io/docs/getting-started-guides/helloworld/ 
// to understand how to obtain an access token
#define TOKEN               "yDWHnL7crREzVPFVM8Yn"  //KRgn1MQmVZgKjzGdq0e0
// ThingsBoard server instance.
#define THINGSBOARD_SERVER  "demo.thingsboard.io"  //iot.etsisi.upm.es

// Baud rate for debug serial
#define SERIAL_DEBUG_BAUD    115200

// Initialize ThingsBoard client
WiFiClient espClient;
// Initialize ThingsBoard instance
ThingsBoard tb(espClient);
// the Wifi radio's status
int status = WL_IDLE_STATUS;
#define TIMEOUT 30
//----------------------------------------------------------LED MATRIX
#define timeScreen 10
unsigned long nowScreen = millis();
unsigned long lastScreen = 0;
int pinCS = 16; // Attach CS to this pin, DIN to MOSI and CLK to SCK 
Max72xxPanel matrix = Max72xxPanel(pinCS, 1, 1);
boolean start = false;
TaskHandle_t Matrix_LED; //Declaracion de una Tarea para procesamiento paralelo
//----------------------------------------------------------
//----------------------------------------------------------NFC CARD READER
#define PN532_IRQ   (4)
#define PN532_RESET (3)  // Not connected by default on the NFC Shield

const int DELAY_BETWEEN_CARDS = 500;
long timeLastCardRead = 0;
boolean readerDisabled = false;
int irqCurr;
int irqPrev;
int numPerC;

// This example uses the IRQ line, which is available when in I2C mode.
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
//----------------------------------------------------------
//----------------------------------------------------------SCD30 CO2 TEMP HUMEDAD
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h" 
SCD30 airSensor;
//----------------------------------------------------------
//----------------------------------------------------------LOUDNESS SENSOR
#define timeSeconds 3
const int Sound = 34;

const int sampleWindow = 50;        // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
float value;
//----------------------------------------------------------

//----------------------------------------------------------DUST SENSOR
const int dust = 17;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 3000;//sampe 30s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;
//----------------------------------------------------------

void setup() {//-----------------------------------------------------------------------------SETUP-----------------------------------------------------------------------------
  // Initialize serial for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  InitWiFi();

  Serial.begin(115200);

  //----------------------------------------------------------LED MATRIX
  matrix.setIntensity(3); // Set brightness between 0 and 15
  xTaskCreatePinnedToCore(
                    func_Matrix_LED,  // Task function.
                    "LED_Matrix",     // name of task. 
                    10000,            // Stack size of task 
                    NULL,             // parameter of the task 
                    1,                // priority of the task 
                    &Matrix_LED,      // Task handle to keep track of created task 
                    0);               // pin task to core 0               
  delay(500);
  //----------------------------------------------------------
  //----------------------------------------------------------Pin for controlling reconnections
  pinMode (26, INPUT) ;
  //----------------------------------------------------------
  
  while (!Serial);
  delay(3000);
  int powerUP = 15;
  Serial.println();
  Serial.println(String(powerUP)+" Seconds cooldown so that all the sensores");
  Serial.println("are powered up before monitoring begins.");
  delay(1000);
  for(int i = powerUP; i>0; i--){
    Serial.print(i); Serial.print(", ");
    delay(1000);
  }
  Serial.println("0");
  delay(1000);


  //----------------------------------------------------------NFC CARD READER
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    ESP.restart();
  }
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  
  // configure board to read RFID tags
  nfc.SAMConfig();

  numPerC = 0;
  
  startListeningToNFC();
  //----------------------------------------------------------
 
  //----------------------------------------------------------SCD30
  Wire.begin();
  if (airSensor.begin() == false)
  {
    Serial.println("Air sensor not detected. Please check wiring.");
  }
  else Serial.println("SCD30 READY.");
  delay(1000);
  //----------------------------------------------------------
  
  //----------------------------------------------------------LOUDNESS SENSOR
  pinMode (Sound, INPUT) ;  // define Sound sensor as input interface

  Serial.println("Loudness Sensor READY.");
  delay(1000);
  //----------------------------------------------------------
  
  //----------------------------------------------------------DUST SENSOR
  //Serial.begin(115200);
  pinMode(dust,INPUT);
  starttime = millis();//get the current time;
  
  Serial.println("Dust Sensor READY.");
  delay(1000);
  //----------------------------------------------------------
  start = true;
  Serial.println("Monitoring START.");
}


void loop() {//-----------------------------------------------------------------------------LOOP-----------------------------------------------------------------------------
  //delay(1000);
  //esp_deep_sleep_start();
  // Reconnect to WiFi, if needed
  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
    //return;
  }

//if(digitalRead(26)){ //If pin 26 on high -> Tries to reconnect else it doesnt
  // Reconnect to ThingsBoard, if needed
  if (!tb.connected()) {
//    subscribed = false;

    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect to Thingsboard");
      //return;
    }
  }
//}
//else Serial.println("Not trying to connect to Thingsboard");

  duration = pulseIn(dust, LOW);
  lowpulseoccupancy = lowpulseoccupancy+duration;
  
  if ((millis()-starttime) > sampletime_ms){
  //if the sample time == 3s
     //delay(2000);
        Serial.println("----------------------------------------------------------------------------");
    
    //----------------------------------------------------------SCD30
    if (airSensor.dataAvailable()){
  
    Serial.println("SDC30:");
    Serial.print("CO2(ppm): ");
    Serial.print(airSensor.getCO2());
    tb.sendTelemetryFloat("CO2", airSensor.getCO2());

    Serial.print(" Temperature(C): ");
    Serial.print(airSensor.getTemperature(), 1);
    tb.sendTelemetryFloat("Temperature", airSensor.getTemperature());

    Serial.print(" Humidity(%): ");
    Serial.print(airSensor.getHumidity(), 1);
    tb.sendTelemetryFloat("Humidity", airSensor.getHumidity());

    Serial.println();
    }
    //----------------------------------------------------------
    
    //----------------------------------------------------------LOUDNESS SENSOR
    //val = analogRead(Sound);
    value = calculate_sound_in_db();
    tb.sendTelemetryFloat("Decibels", value);
    //----------------------------------------------------------
    
    //----------------------------------------------------------DUST SENSOR

        ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
        concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve

        if(concentration > 0.63){ // prevencion de envio de valores de error
          Serial.print("\nDUST SENSOR\nLow Pulse Occupancy: ");
          Serial.print(lowpulseoccupancy);
          tb.sendTelemetryFloat("Dust Low Pulse Occupancy", lowpulseoccupancy);
          Serial.print(", Dust Ratio: ");
          Serial.print(ratio);
          tb.sendTelemetryFloat("Dust Ratio", ratio);
          Serial.print(", Dust Concentration: ");
          Serial.println(concentration);
          tb.sendTelemetryFloat("Dust Concentration", concentration);
          lowpulseoccupancy = 0;
          starttime = millis();
          Serial.println();
        }
    }//---------------------------------------------------------------------------------------------------Sample time thingy from above
    
    //----------------------------------------------------------NFC CARD READER
    if (readerDisabled) {
    if (millis() - timeLastCardRead > DELAY_BETWEEN_CARDS) {
      readerDisabled = false;
      startListeningToNFC();
    }
  } else {
    irqCurr = digitalRead(PN532_IRQ);

    // When the IRQ is pulled low - the reader has got something for us.
    if (irqCurr == LOW && irqPrev == HIGH) {
       Serial.println("Got NFC IRQ");  
       handleCardDetected(); 
    }
  
    irqPrev = irqCurr;
  }
    
  //----------------------------------------------------------
  tb.loop();
}

//---------------------------------------------------------------------------------------------WIFI
void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network
  int i = 0;
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED && i < TIMEOUT) {
    delay(500);
    Serial.print(".");
    i++;
  }
  if (i == TIMEOUT)
    Serial.println("WiFi Connection Timeout");
  else
    Serial.println("Connected to AP");
}

void reconnect() {
  status = WiFi.status();
  int i = 0;
  if ( status != WL_CONNECTED) {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    Serial.println("Disconnected. Atempting to reconnect");
    while (WiFi.status() != WL_CONNECTED && i < TIMEOUT) {
      delay(500);
      Serial.print(".");
      i++;
    }
    if(i == TIMEOUT)
      Serial.println("Failed to re-connect to AP");
    else
      Serial.println("Re-connected to AP");
  }
}

//---------------------------------------------------------------------------------------------LOUDNESS SENSOR
float calculate_sound_in_db(){ //dB calculations
   unsigned long startMillis= millis();                   // Start of sample window
   float peakToPeak = 0;                                  // peak-to-peak level
 
   unsigned int signalMax = 0;                            //minimum value
   unsigned int signalMin = 1024;                         //maximum value
 
                                                          // collect data for 50 mS
   while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(Sound);                         //get reading from microphone
      if (sample < 1024)                    // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;                           // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;                           // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;                    // max - min = peak-peak amplitude
   //Serial.println(peakToPeak);                                     //write calibrated deciBels
   float db = map(peakToPeak,0,1000,48,120);             //calibrate for deciBels peakToPeak,0,1000,48,120
   Serial.print(db);                                     //write calibrated deciBels
   Serial.println(" dB");                                  //write units
   return db;
}
//---------------------------------------------------------------------------------------------NFC CARD READER
void startListeningToNFC() {
  // Reset our IRQ indicators
  irqPrev = irqCurr = HIGH;
  
  Serial.println("Waiting for an ISO14443A Card ...");
  nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);
}

void handleCardDetected() {
    uint8_t success = false;
    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
    uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

    // read the NFC tag's info
    success = nfc.readDetectedPassiveTargetID(uid, &uidLength);
    Serial.println(success ? "Read successful" : "Read failed (not a card?)");

    if (success) {
      // Display some basic information about the card
      Serial.println("Found an ISO14443A card");
      Serial.print("  UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
      Serial.print("  UID Value: ");
      nfc.PrintHex(uid, uidLength);
      numPerC++;//-----------------------------------------------------------------------------------------------------------------------------AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
      Serial.println(numPerC);
      tb.sendTelemetryFloat("Presence", numPerC);
      if (uidLength == 4)
      {
        // We probably have a Mifare Classic card ... 
        uint32_t cardid = uid[0];
        cardid <<= 8;
        cardid |= uid[1];
        cardid <<= 8;
        cardid |= uid[2];  
        cardid <<= 8;
        cardid |= uid[3]; 
        Serial.print("Seems to be a Mifare Classic card #");
        Serial.println(cardid);
      }
      Serial.println("");

      timeLastCardRead = millis();
    }

    // The reader will be enabled again after DELAY_BETWEEN_CARDS ms will pass.
    readerDisabled = true;
}

//----------------------------------------------------------LED MATRIX
void func_Matrix_LED( void * pvParameters ){ 
  Serial.print("\nMatrix LEDs en el core ");
  Serial.println(xPortGetCoreID());
  while(start == false) delay(1000);
  boolean done = true;
  String stringo = "";
  for(;;){
    //if(presencia == true){
    nowScreen = millis();
    if(done) {
      done = false;
    lastScreen = millis();
    
    temperatureMatrix(); 
    humedadMatrix();
    
    if(airSensor.getCO2() > 1000){
      exclamationBlinkMatrix(3);
      stringo = "Niveles de CO2 altos ";
      StringWarningsMatrix(stringo);
      StringWarningsMatrix("Abrir ventanas ");
    }
    if(sample > 75){ //val variable del sonido
      exclamationBlinkMatrix(1);
      stringo = "Mucho ruido ";
      StringWarningsMatrix(stringo);
    }
    if(concentration > 8000){
      stringo = "Concentracion de polvo alta ";
      StringWarningsMatrix(stringo);
    }
    done = true;
    }
    //}
    delay(1000);
  } 
}

void stopSignMatrix(){
  matrix.fillScreen(0);  
  //matrix.drawLine(x1, y1, x2, y2, 1); Draws a line between two coordinates
  //Draws Stop sign (/)
  matrix.drawLine(1, 1, 6, 6, 15);
  matrix.drawLine(0, 2, 0, 5, 15);
  matrix.drawLine(7, 2, 7, 5, 15);
  matrix.drawLine(2, 0, 5, 0, 15);
  matrix.drawLine(2, 7, 5, 7, 15);
  matrix.drawPixel(6, 1, 2);
  matrix.drawPixel(1, 6, 2);
  matrix.write();
  delay(1000);
}

void exclamationBlinkMatrix(int times){
  for(int i=0; i < times; i++){
    matrix.fillScreen(0);  
    matrix.drawRect(3,0,2,5,15); //matrix.drawRect(x,y,s1,s2,15); x,y position of corner -- s1, s2 size of side -- intensity 1 to 15
    matrix.drawRect(3,6,2,2,15);
    matrix.write();
    delay(500);

    matrix.fillScreen(0);  
    matrix.write();
    delay(500);
  }
}

void temperatureMatrix(){
  int temp = airSensor.getTemperature();
  String my_string = String(temp) + "    ";   // This text will be displayed, a letter a time
  //Draw scrolling text
  int spacer = 1;                            // This will scroll the string
  int width = 5 + spacer;                    // The font width is 5 pixels
  for ( int i = 0 ; i < width * my_string.length() + width - 1 - spacer; i++ ) {
    matrix.fillScreen(0);
    int letter = i / width;
    int x = (matrix.width() - 1) - i % width;
    int y = ((matrix.height() - 8) / 2); 

    while ( x + width - spacer >= 0 && letter >= 0 ) {
      if ( letter < my_string.length() ) {
        matrix.drawChar(x, y, my_string[letter], 1, 0, 1);
      }
      letter--;
      x -= width;
    }
    matrix.drawRect(((width+spacer)*(String(temp).length()+1))-i-1,y,3,3,1); //Degree symbol since it doesn't work
    matrix.drawChar((3+(width+spacer)*(String(temp).length()+1))-i, y, 'C', 7, 0, 1); //So we draw it manually
    matrix.write(); 
    delay(100);
  }
}

void humedadMatrix(){
  int hum = airSensor.getHumidity();
  String my_string = String(hum) + "% ";   // This text will be displayed, a letter a time
  //Draw scrolling text
  int spacer = 1;                            // This will scroll the string
  int width = 5 + spacer;                    // The font width is 5 pixels
  for ( int i = 0 ; i < width * my_string.length() + width - 1 - spacer; i++ ) {
    matrix.fillScreen(0);
    int letter = i / width;
    int x = (matrix.width() - 1) - i % width;
    int y = ((matrix.height() - 8) / 2); 

    while ( x + width - spacer >= 0 && letter >= 0 ) {
      if ( letter < my_string.length() ) {
        matrix.drawChar(x, y, my_string[letter], 1, 0, 1);
      }
      letter--;
      x -= width;
    }
    matrix.write(); 
    delay(100);
  }
}

void StringWarningsMatrix(String my_string){
  int spacer = 1;         // This will scroll the string
  int width = 5 + spacer; // The font width is 5 pixels
  for ( int i = 0 ; i < width * my_string.length() + width - 1 - spacer; i++ ) {
    matrix.fillScreen(0);
    int letter = i / width;
    int x = (matrix.width() - 1) - i % width;
    int y = ((matrix.height() - 8) / 2); //center the text vertically

    while ( x + width - spacer >= 0 && letter >= 0 ) {
      if ( letter < my_string.length() ) {
        matrix.drawChar(x, y, my_string[letter], 1, 0, 1);
      }
      letter--;
      x -= width;
    }
    matrix.write();
    delay(100);
  }
}
