//LIBRARIES
#include <U8g2lib.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <utility/wifi_drv.h>
#include <RTCZero.h>
#include <Timezone.h>    // https://github.com/JChristensen/Timezone

#include "secrets.h"


// setup time zone
// US Central Time Zone (Austin)
TimeChangeRule myDST = {"CDT", Second, Sun, Mar, 2, -300};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"CST", First, Sun, Nov, 2, -360};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

// MACRO DEFINES
#define SW 0 // RotaryEncoder SW Pin
#define DT 1 // RotaryEncoder DT Pin
#define CLK 2 // RotaryEncoder CLK Pin

#define RED 25
#define GREEN 26
#define BLUE 27

// Stuff for WIFI and NTP
int status = WL_IDLE_STATUS;

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen for UDP packets

IPAddress timeServer(162, 159, 200, 123); // pool.ntp.org NTP server

const int NTP_PACKET_SIZE = 48; // NTP timestamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

// DEFINE DISPLAY/CONTROL OBJECTS 
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g_panel(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);
  
// ROTARY ENCODER VARIABLES
int counter = 30; //SAME AS ALTITUDE to start
int currentStateCLK;
int lastStateCLK;
String currentDir ="";

/* Create an rtc object */
RTCZero rtc;

// ************************************************
//  This is our ISR which has the job of responding to interrupt events
bool clicked = false;

int num_clicks=0;
void updateClick(){
  if (digitalRead(SW) == 1){
      clicked = true;
      Serial.print("received click: ");
      Serial.println(num_clicks);
      num_clicks++;
  }
}

enum state{
    FREE = 0x00,
    BUSY = 0x01,
    MEETING = 0x02,
};

uint8_t current_state = MEETING;
unsigned long current_state_duration = 62 * 1000;

char state_string[8] = "READY";

unsigned long state_list[5][3];

void updateEncoder(){
 
  // Read the current state of encoder CLK
  currentStateCLK = digitalRead(CLK);
 
  // If last and current state of CLK are different, then pulse occurred
  // React to only 0->1 state change to avoid double counting
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
 
    // If the DT state is different than the CLK state then
    // the encoder is rotating CW so INCREASE counter by 1
    if (digitalRead(DT) != currentStateCLK) {
      counter ++;
      currentDir ="CW";
 
    } else {
      // Encoder is rotating CCW so DECREASE counter by 1
      counter --;
      currentDir ="CCW";
    }
 
    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.print(counter);
    Serial.println();
  }

    // Remember last CLK state to use on next interrupt...
  lastStateCLK = currentStateCLK;
}
 
int current_color = RED;

void RGB_LED(int red, int green, int blue){
  WiFiDrv::analogWrite(RED, red);
  WiFiDrv::analogWrite(GREEN, green);
  WiFiDrv::analogWrite(BLUE, blue);
}

void print2digits(int number) {
  if (number < 10) {
    Serial.print("0");
  }
  Serial.print(number);
}

// ************************************************
void setup(void) {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for native USB port only
  // }

  u8g_panel.begin();

  WiFiDrv::pinMode(RED, OUTPUT);
  WiFiDrv::pinMode(GREEN, OUTPUT);
  WiFiDrv::pinMode(BLUE, OUTPUT);

  // want led to be red until connection is established
  RGB_LED(25,0,0);

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // want led to be red until connection is established
  RGB_LED(0,0,25);

  Serial.println("Connected to WiFi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  Udp.begin(localPort);
 
  rtc.begin();

  // set the time from a NTP server
  sendNTPpacket(timeServer); // send an NTP packet to a time server

  // wait to see if a reply is available
  delay(1000);
  if (Udp.parsePacket()) {
    Serial.println("packet received");
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;

    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;

    rtc.setEpoch(epoch); // Jan 1, 2016
    
    // print Unix time:
    Serial.println(epoch);

    Serial.print("UTC date and time: ");
    Serial.print(rtc.getYear());
    Serial.print("-");
    print2digits(rtc.getMonth());
    Serial.print("-");
    print2digits(rtc.getDay());
    Serial.print(". ");

    // print the hour, minute and second:
    // ...and time
    print2digits(rtc.getHours());
    Serial.print(":");
    print2digits(rtc.getMinutes());
    Serial.print(":");
    print2digits(rtc.getSeconds());
    Serial.println();
  }

  RGB_LED(0,25,0);

  // Set encoder pins as inputs
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(DT, INPUT);
 
  u8g_panel.setColorIndex(1); // pixel on

  // u8g_panel.println("System Initialization has begin.");

  // Read the initial state of CLK
  lastStateCLK = digitalRead(CLK);
  current_color = RED;

   // Call Interrupt Service Routine (ISR) updateEncoder() when any high/low change
  // is seen on A (CLK) interrupt  (pin 2), or B (DT) interrupt (pin 3)
 
  attachInterrupt(digitalPinToInterrupt(CLK), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW), updateClick, CHANGE);

  // u8g_panel.println("Input configured.");

}
 
// ************************************************
TimeChangeRule *tcr;
char datetime_string[24];

// format and print a time_t value, with a time zone appended.
void makeDateStr()
{
    time_t utc = rtc.getEpoch();
    time_t local = myTZ.toLocal(utc, &tcr);

    char m[4];    // temporary storage for month string (DateStrings.cpp uses shared buffer)
    strncpy(m, monthShortStr(month(local)), 4);
    snprintf(datetime_string, 24, "%s %d-%d %.2d:%.2d:%.2d", 
      dayShortStr(weekday(local)), month(local), day(local), hour(local), minute(local), second(local));
    Serial.println(datetime_string);
}

int getStateStr(uint8_t current_state){

  int skip_to_center = 0;

  if (current_state == MEETING){
    memcpy(state_string, "MEETING", 8);
    skip_to_center = 29;
  }
  else if (current_state == BUSY){
    memcpy(state_string, "BUSY", 5);
    skip_to_center = 44;
  }
  else if (current_state == FREE){
    memcpy(state_string, "FREE", 5);
    skip_to_center = 44;
  }

  return skip_to_center;
}

char duration_string[12];

int getStateDurationStr(){
  
  int num_hours_left = (int)(current_state_duration / 3600000);
  int num_mins_left = (current_state_duration - (num_hours_left * 3600000)) / 60000;
  int num_secs_left = (current_state_duration - (num_hours_left * 3600000) - (num_mins_left) * 60000) / 1000;

  int num_written = 0;
  if(num_hours_left > 0){
      num_written = snprintf(duration_string, 12, "%dh %dm %ds", num_hours_left, num_mins_left, num_secs_left);
  } else if (num_mins_left > 0){
      num_written = snprintf(duration_string, 12, "%dm %ds", num_mins_left, num_secs_left);
  }
  else{
      num_written = snprintf(duration_string, 12, "%ds", num_secs_left);
  }

  int duration_string_to_center = 10;
  if(num_written < 12){
    duration_string_to_center += (int)((12-num_written) * 9 / 2);
  }

  return duration_string_to_center;
}

void loop(void) {

  int begin = millis();

  makeDateStr();
  int state_string_to_center = getStateStr(current_state);
  int duration_string_to_center = getStateDurationStr();

  // picture loop  
  u8g_panel.firstPage();  
  do {
     u8g_panel.setFont(u8g2_font_t0_11_mf);
     u8g_panel.drawStr( 13, 8, datetime_string);
     u8g_panel.setFont(u8g2_font_10x20_mf);
     u8g_panel.drawStr(state_string_to_center, 30, state_string);
     u8g_panel.setFont(u8g2_font_9x18_mf);
     u8g_panel.drawStr(duration_string_to_center, 48, duration_string);
  } while( u8g_panel.nextPage() );

  Serial.print("Current state: ");
  Serial.print(current_state); 
  Serial.print("  state string:");
  Serial.print(state_string);
  Serial.print(" ");
  Serial.println(current_state_duration);

  if (current_state == MEETING){
    RGB_LED(25, 0 , 0);
  } else if (current_state == BUSY){
    RGB_LED(0, 0, 25);
  } else if (current_state == FREE) {
    RGB_LED(0, 25, 0);
  }

  // if (clicked == true){
  //   WiFiDrv::analogWrite(current_color, 0);
  //   current_color = RED + ((current_color-RED+1) % 3);
  //   clicked = false;
  //   Serial.print("New color: ");
  //   Serial.println(current_color);
  // }

  // int color_val = 255 * (counter % 100) / 100;
  // //RGB_LED(0,green_val,0);
  // // unsigned long currMillisec = millis();
  // WiFiDrv::analogWrite(current_color, color_val);

  current_state_duration -= 1000;
  if (current_state_duration == 0){
    if (current_state == MEETING){
      current_state = FREE;
      current_state_duration = 72*1000;
    }
    else if( current_state == FREE){
      current_state = BUSY;
      current_state_duration = 44 * 1000;
    }
    else if( current_state == BUSY){
      current_state = MEETING;
      current_state_duration = 88 * 1000;
    }
  }

  // wait ten seconds before asking for the time again
  Serial.print("delaying for");
  Serial.println(1000 - (millis()-begin));
  delay(1000 - (millis()-begin));
} 

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {
  //Serial.println("1");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  //Serial.println("2");
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  //Serial.println("3");

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  //Serial.println("4");
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  //Serial.println("5");
  Udp.endPacket();
  //Serial.println("6");
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
