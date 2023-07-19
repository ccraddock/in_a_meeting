# in_a_meeting
Arduino project for a IOT device that indices whether someone is in a meeting or not, based on the contents of a iCal feed retrieved over https.

This is hacked together from the examples distributed with the libraries.

The following libraries are required:
- u8g2lib
- WiFiNINA
- WiFiUdp
- RTCZero
- Timezone: https://github.com/JChristensen/Timezone

This was designed on a Arduino MKR Wifi 1010. 

Put your secrets in secrets.h, but don't add it to this repository!!
