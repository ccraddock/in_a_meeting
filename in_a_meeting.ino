
//LIBRARIES

#include <ArduinoBearSSL.h>
#include <EasyNTPClient.h>
#include <RTCZero.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <WiFiNINA.h>
#include <utility/wifi_drv.h>
#include <WiFiUdp.h>

// #include <uICAL.h>

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

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;
EasyNTPClient ntpClient(udp, "pool.ntp.org", 0);

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

unsigned long get_time(){
  return rtc.getEpoch();
}

void calendar_update(){

    Serial.println("Fetching: calendar!");

    WiFiClient client;
    BearSSLClient ssl_client(client);

    ArduinoBearSSL.onGetTime(get_time);

    char get_string[128];
    snprintf(get_string, 128, "GET %s", CALENDAR_ENDPOINT);
    Serial.print("Querying endpoint ");
    Serial.println(CALENDAR_ENDPOINT);

    if(ssl_client.connect(CALENDAR_SERVER, 443)) {
      ssl_client.println(get_string);
      ssl_client.println("Host: www.google.com");
      ssl_client.println("Connection: close");
      ssl_client.println();
    }

    while(ssl_client.available()){
      char c = ssl_client.read();
      Serial.write(c);
    }

    Serial.println("Finished reading.");
    ssl_client.stop();
    // uICAL::Calendar_ptr cal = nullptr;
    // try {
    //     uICAL::istream_Stream istm(https.getStream());
    //     cal = uICAL::Calendar::load(istm);
    // }
    // catch (uICAL::Error ex) {
    //     Serial.println("%s: %s", ex.message.c_str(), "! Failed loading calendar");
    //     stop();
    // }

    // unsigned now = g_ntpClient.getUnixTime();

    // uICAL::DateTime calBegin(now);
    // uICAL::DateTime calEnd(now + 86400);

    // uICAL::CalendarIter_ptr calIt = uICAL::new_ptr<uICAL::CalendarIter>(this->cal, calBegin, calEnd);

    // while (calIt->next()) {
    //     uICAL::CalendarEntry_ptr entry = calIt->current();
    //     Serial.println("Event @ %s -> %s : %s",
    //                     entry->start().as_str().c_str(),
    //                     entry->end().as_str().c_str(),
    //                     entry->summary().c_str());

    // }

    // stop();

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

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  Serial.println("Connected to WiFi");
  printWifiStatus();

  // now that wifi connection is established, 
  // show blue
  RGB_LED(0,0,25);

  Serial.println("\nUpdating time from NTP server ...");
  udp.begin(localPort);
 
  rtc.begin();

  // set the RTC time
  rtc.setEpoch(ntpClient.getUnixTime());

  // now that time is set, go grean
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

bool calendar_init = false;

void loop(void) {

  if (calendar_init == false){
    calendar_update();
    calendar_init = true;
  }
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
