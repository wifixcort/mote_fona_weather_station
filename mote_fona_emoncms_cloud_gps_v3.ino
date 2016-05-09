/*
  Moteino FONA 808 Reporter 

  Ricardo Mena C
  ricardo@crcibernetica.com
  http://crcibernetica.com

  License
  **********************************************************************************
  This program is free software; you can redistribute it 
  and/or modify it under the terms of the GNU General    
  Public License as published by the Free Software       
  Foundation; either version 3 of the License, or        
  (at your option) any later version.                    
                                                        
  This program is distributed in the hope that it will   
  be useful, but WITHOUT ANY WARRANTY; without even the  
  implied warranty of MERCHANTABILITY or FITNESS FOR A   
  PARTICULAR PURPOSE. See the GNU General Public        
  License for more details.                              
                                                        
  You should have received a copy of the GNU General    
  Public License along with this program.
  If not, see <http://www.gnu.org/licenses/>.
                                                        
  Licence can be viewed at                               
  http://www.gnu.org/licenses/gpl-3.0.txt

  Please maintain this license information along with authorship
  and copyright notices in any redistribution of this code
  **********************************************************************************
  */

/*
Conecctions
  Moteino    FONA
  3          RX
  4          TX
  5          Rst
  7          Key
  8          PStat
  3.3v       Vio
  Gnd        Gnd
*/


#include <Moteino.h>
//-----Need to be declared for correct GenSens functioning----
#include <RFM69.h>
#include <SPI.h>
#include <LowPower.h>   //https://github.com/rocketscream/Low-Power

#include <SoftwareSerial.h>

#include "Adafruit_FONA.h"

#include <avr/wdt.h>

#include <EEPROM.h>
template <class T> int EEPROM_writeResets(int ee, const T& value);
template <class T> int EEPROM_readResets(int ee, T& value);

//---------FONA network-------------
#define FONA_RX 4
#define FONA_TX 5
#define FONA_RST 6
#define FONA_KEY 7
#define FONA_PS 8
//-----------------------------------

//----DEBUG PINS----
#define DB_RX 2
#define DB_TX 3
//------------------

#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey"

#define IP_JSON "YOUR_SERVER_IP/input/post.json?node="
#define IP_APIKEY "&apikey=YOUR_EMON_API_KEY&json="

Moteino *mio;

//------------Network identifiers-----------------
uint8_t node_id = 2;   //This node id
//uint8_t gw_id = 1;    //gatewayId
uint8_t network = 215; //Gateway
//------------------------------------------------


//---Paquetes to route----
String pck = "";//Packet to send
String msg = "";//Received packets
String stringEvent = "";//Temporal buffer to receive from RX/TX

//-----------------FONA things--------------------
// this is a large buffer for replies
char replybuffer[255];
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

//unsigned int fona_coun_resets = 0;
//------------------------------------------------

//-----------------Debug port---------------------
//SoftwareSerial db = SoftwareSerial(DB_RX, DB_TX);
//------------------------------------------------

//-----Serial configurations-----
#define serial Serial
//#define serial_db db

#define SERIAL_BAUD   115200
#define DB_BAUD 9600
#define FONA_BAUD 4800
//-------------------------------

//#define DEBUG //uncoment for debuging
//#define DEBUG1 //uncoment for debuging
//#define FREERAM
//#define TEST

#if defined(TEST)
  long interval = 1000;
  unsigned long start =0;
#endif

//Log how many times I reset the Fona
struct fona_resets{
  boolean sended;
  int count;
  int system_resets;
}resets;

void init_fona(){
  fona_off();
  delay(2000);
  fona_on();
  delay(2000);  
  fonaSS.begin(4800);
  
  if(!check_fona()){// See if the FONA is responding
    halt(F("Couldn't find FONA"));
  }//end if
  
  //APN configuration
  fona.setGPRSNetworkSettings(F("kolbi3g"), F(""), F(""));
  #if defined(DEBUG)
  serial.println(F("Waiting 20s.."));
  #endif
  
  delay(20000);//Wait for FONA

  gprs_disable();
  gprs_enable(0);
  fona_gps_on();   
 
}//end init_fona

void setup() {
   
  EEPROM_readResets(0, resets);
  resets.system_resets++;//Log system reset
  EEPROM_writeResets(0, resets);//Save system reset
  
  //---Serial init---
  serial.begin(SERIAL_BAUD);
  #if defined(DEBIG)
  serial_db.begin(DB_BAUD);
  #endif
  //-----------------
  
  pinMode(FONA_KEY, OUTPUT);
  pinMode(FONA_PS, INPUT);
  analogReference(INTERNAL);
  delay(2000);
  #if defined(FREERAM)
    serial_db.print("Free 1 = ");
    serial_db.println(freeRam());
  #endif

  init_fona();//Make Fona configurations
  
  print_IMEI();

  //Check moteino node creation
  mio  = new Moteino(node_id, FREQUENCY, ENCRYPTKEY, network);//node#, freq, encryptKey, gateway, LowPower/HighPower(false/true)
  
  #if defined(DEBUG)
    serial_db.println(F("This is your Proxy")); 
    char buff[50];
    sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
    serial_db.println(buff);
  #endif
  #if defined(TEST)
  interval *= 30;
  #endif
  
  publish_resets();//Publish system resets
  
  wdt_enable(WDTO_8S);
  wdt_reset(); 
}//end setup

void loop(){
 
  #if defined(FREERAM)
    serial_db.print("Free 2 = ");
    serial_db.println(freeRam());
  #endif

  //---------All nodes---------------
  mio->moteino_receive(msg);//Get node messages
  #if defined(DEBUG1)
    if(msg != ""){//Check if message is not empty
      serial_db.println(F("Paquete de un nodo"));
      serial_db.println(msg);
    }
  #endif

  publish_resets();//Post resets in emoncms

  check_weather_msg();
    
  check_moteino_nodes();

  #if defined(FREERAM)
    serial_db.print("Free 5 = ");
    serial_db.println(freeRam());
  #endif

  //----------Test Message----------
  #if defined(TEST)
    unsigned long currentMillis = millis();
    if(currentMillis - start > interval) {
      // save the last time you blinked the LED 
      stringEvent = "293 0 46.6 28.5 0 0 88516 3.77 0.2";
      start = currentMillis; 
    }//end if
  #else 
    serialEvent();//interrupt to get messages from UNO
  #endif
  //--------------------------------
  wdt_reset();  
}//end loop

void check_weather_msg(){

  if(stringEvent != ""){//Check messages from UNO
    //Send messages received from UNO to Emoncms
    #if defined(DEBUG)
    serial_db.println(F("stringEvent received"));
    #endif
    pck = String(node_id);//add node ID
    pck += ' ';
    pck += stringEvent;
    stringEvent = "";//Leave stringEvent blank for new messages
    if(fona_gps_fix() >= 2){
      pck += ' ';
      char gps_loc[80];
      fona_gps_location(gps_loc);
      pck += String(gps_loc);
    }//end if
    else{
      #if defined(DEBUG)
      serial_db.println(F("ERROR GPS"));
      #endif
    }//end if
    secure_url_send(pck);
    //pck="";
    //}//end if
    #if defined(FREERAM)
        serial_db.print("Free = ");
        serial_db.println(freeRam());
    #endif
  }//end if
}

void check_moteino_nodes(){
  if(msg != ""){//Check empty msg
    //Adding RSSI
    msg += ' ';
    msg += mio->moteino_rssi();
    secure_url_send(msg);
    #if defined(DEBUG)
    serial_db.println(msg);
    #endif
    msg = "";
  }//end if
}

void secure_url_send(String &url){
  if(send_url(url) == -1){
    #if defined(DEBUG)
    serial_db.println(F("Error sendind URL"));
    #endif
    if((fona.GPRSstate()==0)||(fona.getNetworkStatus() != 1)){
      #if defined(DEBUG)
        serial_db.println(F("NetworkStatus or GPRS State errors"));
      #endif        
      log_resets();//Save this network error
      wdt_disable();//20s for init_fona needed
      delay(500);
      init_fona();//Reset FONA
      wdt_enable(WDTO_8S);
      wdt_reset();  
    }
  }//end if
}

void halt(const __FlashStringHelper *error) {
  wdt_enable(WDTO_1S);
  wdt_reset();
  #if defined(DEBUG)
    seria_db.println(error);
  #endif
  while (1) {}
}

void log_resets(){
  resets.sended = true;
  resets.count++;
  EEPROM_writeResets(0, resets);
}//end log_resets

void publish_resets(){
  if(resets.sended){
    String fona_rst = "4 "+String(resets.count) + " "+ resets.system_resets;//prepare message
    if(send_url(fona_rst) != -1){//Send ok
      resets.sended = false;//block 
      EEPROM_writeResets(0, resets);//Save resets.sended
    }//end if
  }//end if
}//end publish_resets

void print_IMEI(void){
  // Print SIM card IMEI number.
  #if defined(DEBUG)
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    serial.print("SIM card IMEI: "); serial.println(imei);
  }//end if  
  #endif
}//end print_IMEI

String json_split(String &message, String &node_id){
  String brak = "\{";
  String number = "";
  uint16_t j = 0;
  uint8_t data_num = 1;
  node_id = "";
  for(uint8_t i = 0; i < message.length(); i++){
    if((message[i] == ';')||(message[i] == ' ')){
      j = i+1;
      break;
    }else{
      node_id += message[i];
    }//end if
  }//end for

  for(j; j < message.length(); j++){
    if((message[j] == ';')||(message[j] == ' ')){
      brak += String(data_num) + "\:" + number + "\,";
      data_num++;
      //serial.println(brak);
      number = "";
    }else{
      if(message[j]!= '\r'){
        number += message[j];
      }
    }//end if
    
  }//end for
  brak += String(data_num) + "\:" + number+"\}";
  return brak;
  
}//end json_pck

void serialEvent(){
  //String mjs = "";
  while(serial.available()>0){
    delay(10);
    if(serial.available()> 0){   
      char c = serial.read(); // read the next character.
      if(c == '\n'){break;}
      stringEvent +=c;
    }//end if
  }//end while
  //stringEvent = mjs;
  #if defined(DEBUG)
  if(mjs != ""){
    serial_db.print(F("Incomming = "));
    serial_db.println(stringEvent);  
  }//end if
  #endif
  if(stringEvent.length()<=40){//minimum length
    stringEvent = "";
  }
}//end serialEvent

#if defined(FREERAM)
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}//end freeRam
#endif

// Post data to website
int send_url(String &raw_paq){
  flushSerial();
  uint16_t statuscode;
  int16_t length;
  String node = "";//Store node id
  String json = json_split(raw_paq, node);//split packet into json format and store node id througth reference
  String url = IP_JSON+node+IP_APIKEY;
  
  int data_len = json.length()+1;
  char data[data_len];
  json.toCharArray(data, data_len);
  #if defined(DEBUG)
    serial_db.println(json);
  #endif
  
  int l_url = url.length()+json.length();//strlen(data)
  char c_url[l_url];
  sprintf(c_url, "%s%s", url.c_str(),json.c_str());
  //flushSerial();
  
  #if defined(DEBUG)  
    serial_db.println(c_url);
    serial_db.println(F("****"));
  #endif
       
  if (!fona.HTTP_GET_start(c_url, &statuscode, (uint16_t *)&length)) {
    #if defined(DEBUG)
    serial_db.println("GPRS send failed!");
    #endif
    return -1;
  }else{
    #if defined(DEBUG)
    serial_db.println("GPRS send ok");
    #endif
    #if defined(FREERAM)
        serial_db.print("Free RAM TOP = ");
        serial_db.println(freeRam());
    #endif    
  }
  while (length > 0) {
    while (fona.available()) {
      char c = fona.read();     
    // Serial.write is too slow, we'll write directly to Serial register!
      #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
          loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
          UDR0 = c;
      #else
          #if defined(DEBUG)
          serial_db.write(c);
          #endif
      #endif
      length--;
      if (! length) break;
    }//end while
  }//end while
  #if defined(DEBUG)
    serial_db.println(F("\n****"));
  #endif
  fona.HTTP_GET_end();
}//end send_url

int gprs_enable(int maxtry){
  // turn GPRS on
  wdt_enable(WDTO_2S);
  wdt_reset();
  if (!fona.enableGPRS(true)){
    #if defined(DEBUG)
      serial_db.print(F("Failed to turn on GPRS = "));
      serial_db.println(maxtry);
    #endif
    if(maxtry > 200){
      log_resets();//Log this system reset
      wdt_enable(WDTO_1S);
      wdt_reset();
      while(1){}
    }
    maxtry +=1;
    gprs_enable(maxtry);
  }else{
    #if defined(DEBUG)
      serial_db.println(F("GPRS ON"));
    #endif
  }
  wdt_reset();
  wdt_disable();
}//end gprs_enable

int gprs_disable(){
  // turn GPRS off
  if (!fona.enableGPRS(false)){
    #if defined(DEBUG)
      serial_db.println(F("Failed to turn GPRS off"));
    #endif
  }else{
    #if defined(DEBUG)
      serial.println(F("GPRS OFF"));
    #endif
    return 1;
  }
}//end gprs_disable

void flushSerial() {
  serial.flush();
  #if defined(DEBUG)
  serial_db.flush();
  #endif
}//end flushSerial

int check_fona(){
  // See if the FONA is responding
  if (!fona.begin(fonaSS)) {           // can also try fona.begin(Serial1)
    #if defined(DEBUG)
      serial_db.println(F("Couldn't find FONA"));
    #endif  
    return 0;
  }
  #if defined(DEBUG)
    serial_db.println(F("FONA is OK"));
  #endif
  return 1;  
}//end check_fona

void fona_on(){
  #if defined(DEBUG)
    serial_db.println("Turning on Fona: ");
  #endif
  while(digitalRead(FONA_PS)==LOW){
    digitalWrite(FONA_KEY, LOW);
  }
  digitalWrite(FONA_KEY, HIGH);
  delay(4000);  
}

void fona_off(){
  #if defined(DEBUG)
    serial_db.println("Turning off Fona: ");
  #endif
  while(digitalRead(FONA_PS)==HIGH){
    digitalWrite(FONA_KEY, LOW);
  }
  digitalWrite(FONA_KEY, HIGH);
  delay(4000);
}//end fona_off

int fona_gps_on(void){
  // turn GPS on
  if (!fona.enableGPS(true)){
    return 0;//Failed to turn GPS on
  }//end if
  delay(2500);
  return 1;
}//end fona_gps_on

int fona_gps_off(void){
  delay(2500);
  // turn GPS off
  if (!fona.enableGPS(false)){
    return 0; //Failed to turn GPS off
  }//end if
  return 1;
}//end fona_gps_off

int fona_gps_fix(){
  int8_t stat;
  // check GPS fix
  stat = fona.GPSstatus();
  
  if (stat < 0){
    #if defined(DEBUG)
    serial_db.println(F("Failed to query"));
    #endif
  }//end if
  if (stat == 0){ 
    fona_gps_on();
    #if defined(DEBUG)
    serial_db.println(F("GPS off"));
    #endif
    stat = fona_gps_fix();  
  };
  #if defined(DEBUG)
  if (stat == 1)serial_db.println(F("No fix"));
  if (stat == 2) serial_db.println(F("2D fix"));
  if (stat == 3) serial_db.println(F("3D fix"));
  #endif
  return stat;
}//end fona_gps_fix

int fona_gps_location(char *gpsdata){
  // check for GPS location
  // Latitude & longitude for distance measurement
  float latitude, longitude, speed_kph, heading, altitude;
  bool gpsFix = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
  String dad = "";
  dad += float_to_string(latitude, 4) +" "+ float_to_string(longitude, 4) +" "+ float_to_string(altitude, 4);
  #if defined(DEBUG)
  serial_db.print("->");
  serial_db.println(dad.length());
  serial_db.println(dad);
  #endif
  sprintf(gpsdata, "%s", dad.c_str());
  return gpsFix;
}//end fona_gps_location

String float_to_string(float value, uint8_t places) {
  // this is used to cast digits 
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  //int i;
  float tempfloat = value;
  String float_obj = "";

    // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
  // if this rounding step isn't here, the value  54.321 prints as 54.3209

  // calculate rounding term d:   0.5/pow(10,places)  
  float d = 0.5;
  if (value < 0){
    d *= -1.0;
  }
  // divide by ten for each decimal place
  for (uint8_t i = 0; i < places; i++){
    d/= 10.0;
  }
  // this small addition, combined with truncation will round our values properly 
  tempfloat +=  d;

  // first get value tens to be the large power of ten less than value
  // tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

  if (value < 0){
    tempfloat *= -1.0;
  }
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }
  // write out the negative if needed
  if (value < 0){
    float_obj += "-";
  }//en if
  
  if (tenscount == 0){
    float_obj += String(0, DEC);
  }//en if
  
  for (uint8_t i = 0; i< tenscount; i++) {
    digit = (int) (tempfloat/tens);
    float_obj += String(digit, DEC);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }//en for

  // if no places after decimal, stop now and return
  if (places <= 0){
    return float_obj;
  }//end if

  // otherwise, write the point and continue on
  float_obj += ".";

  // now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
  for (uint8_t i = 0; i < places; i++) {
    tempfloat *= 10.0; 
    digit = (int) tempfloat;
    float_obj += String(digit,DEC);  
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit; 
  }//end for
  return float_obj;
}

template <class T> int EEPROM_writeResets(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(ee++, *p++);
    return i;
}//end EEPROM_writeAnything

template <class T> int EEPROM_readResets(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(ee++);
    return i;
}//end EEPROM_readAnything

